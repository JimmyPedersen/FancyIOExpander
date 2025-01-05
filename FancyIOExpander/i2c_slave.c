#include "i2c_slave.h"
#include "debug.h"

/**
 Section: Global Variables
 */

static struct
{
    volatile uint8_t rxSlaveAddr;
    volatile uint16_t EEPMemAddr;
    volatile I2C_State_t state;
    volatile i2c_write_to_slave_t to_slave_ptr;
    volatile i2c_read_from_slave_t from_slave_ptr;
    volatile uint8_t byteNo;
}i2c;

I2C_State_t i2c_slave_get_state(void)
{
    return i2c.state;
}

uint8_t i2c_slave_get_address(void)
{
    return i2c.rxSlaveAddr;
}

uint8_t i2c_slave_get_byte_no(void)
{
    return i2c.byteNo;
}

bool TestForStopBit()
{
    if(SSP1STATbits.P)
    {
      i2c.state = eIdle;
      dbg_analog(0);
      return true;
    }
    return false;
}

void SlaveRdDevAddrFromBus(void)
{
    i2c.byteNo = 0;
    if(TestForStopBit())
        return;
        
    dbg_analog(4);
    i2c.rxSlaveAddr = I2C1_Read();
    if(!I2C1_IsRead()) //Check for Write
        i2c.state = eAddrH;
}


void MasterR_SlaveW(void)
{   
    if(TestForStopBit())
        return;
    
    // Call callback if defined
    if(i2c.from_slave_ptr != 0)
    {
        uint8_t tmp;
        bool res = (i2c.from_slave_ptr)(&i2c.EEPMemAddr, &tmp);
        i2c.byteNo++;
        
        if(res)
             I2C1_Write(tmp);
        else
            I2C1_SendNack();
        
    }
}

void MasterW_SlaveR(void)
{  
    if(i2c.state == eIdle)
        return;
    
    // Test for stop bit
    if(TestForStopBit())
        return;
    
    // Read byte
    uint8_t rb = I2C1_Read();    
    
    // Is this a address write?
    if(i2c.state == eAddrH)
    {
        dbg_analog(8);
        i2c.EEPMemAddr = (uint16_t)rb<<8;
        i2c.state++;
        return;
    }
    else if(i2c.state == eAddrL)
    {        
        dbg_analog(16);
        i2c.EEPMemAddr |= rb;
        i2c.state++;
        return;        
    }
    else if(i2c.state != eData)
    {
        i2c.state = eIdle;
//        return;
    }    
    
    // Call callback if defined
    if(i2c.to_slave_ptr != 0)
    {
        bool res = (i2c.to_slave_ptr)(&i2c.EEPMemAddr, rb);
        i2c.byteNo++;
        if(!res)
            I2C1_SendNack();
    }
}



void i2c_slave_init(uint8_t slaveAddr, uint8_t maskAddr, i2c_write_to_slave_t to_slave_func, i2c_read_from_slave_t from_slave_func)
{
    i2c.to_slave_ptr = to_slave_func;
    i2c.from_slave_ptr = from_slave_func;
    
    I2C1_Open();
    SSP1ADD = (uint8_t) (slaveAddr << 1);
    SSP1MSK = (uint8_t) (maskAddr << 1);
    SSP1CON3bits.PCIE = 1;  // Enable stop interrupt

    I2C1_SlaveSetAddrIntHandler(SlaveRdDevAddrFromBus);
    I2C1_SlaveSetWriteIntHandler(MasterR_SlaveW);
    I2C1_SlaveSetReadIntHandler(MasterW_SlaveR);
}