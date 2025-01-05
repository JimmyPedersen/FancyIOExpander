#pragma warning disable 520,2053

#include "config.h"
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>
#include "i2c_slave.h"
#include "registers.h"
#include "eeprom.h"
#include "debug.h"
#include "analog.h"

    
typedef union
{
    struct 
    {
        unsigned int cmd : 8;
        unsigned int extra : 4; // Also used for selecting specific pin
        unsigned int portNo : 3;
        unsigned int isCMD : 1;    
    };
    struct
    {
        unsigned char AddrL;
        unsigned char AddrH;
    };
    unsigned int raw;
}address_t;



bool handle_write_cmd(address_t c, uint8_t rb)
{
     // Handle register commands
    if(c.cmd < REGISTERS_MAX_COUNT)
        return WriteReg(c.cmd, c.portNo, rb);
    else
    {
        // Handle other commands
        switch(c.cmd)
        {
            case REGISTERS_MAX_COUNT+0: // 32
                // Write to DACs
                break;
                
            case REGISTERS_MAX_COUNT+1: // 33
                break;
            
            default:
                return false;
        }
    }
    return true;
}


bool handle_read_cmd(address_t c, uint8_t *wb)
{
    static word_arr_t adc_tmp;
    static address_t a;
    
     // Handle register commands
    if(c.cmd < REGISTERS_MAX_COUNT)
        return ReadReg(c.cmd, c.portNo, wb);
    else
    {
        // Handle other commands
        switch(c.cmd)
        {
             // Read ADC value
            case REGISTERS_MAX_COUNT + 0:   // 32
                if(!i2c_slave_get_byte_no())
                    a.raw = c.raw;
                
                // If it's byte 0
                if(!(i2c_slave_get_byte_no() & 1))
                    adc_tmp.word = analog_get_adc_val(a.portNo, a.extra);
                
                *wb = adc_tmp.arr[i2c_slave_get_byte_no() & 1];
                
                if(i2c_slave_get_byte_no() & 1)
                {
                    if(++a.extra >= EXP_PINS_PER_PORT)
                    {
                        a.extra = 0;
                        if(++a.portNo >= EXP_PORTS_CNT)
                            a.portNo = 0;
                    }
                }
                break;
                
            case REGISTERS_MAX_COUNT + 1:  // 33

                break;
            
            default:
                return false;
        }
    }
    return true;
}


bool i2c_write_to_slave_cb(volatile uint16_t *addr, uint8_t rb)
{
    dbg_analog(32);
  
    address_t c;
    c.raw = *addr;
    if(c.isCMD)
    {
        // Check if the requested command exists, if not return false
        if(!handle_write_cmd(c, rb))
            return false;
    }
    else if(*addr < SLAVE_EEPROM_SIZE)
    {
    //    DATAEE_WriteByte(i2c1EEPMemAddr++, I2C1_Read());
        if(eeprom_read_buffer((eeAddr_t)*addr) != rb)
            eeprom_write_buffer((eeAddr_t)*addr, rb);
        
        // Prepare for next byte
        *addr += 1;
        
        // Wrap around in EEPROM area
        if(*addr >= SLAVE_EEPROM_SIZE)
            *addr = 0;   
        
        return true;
    } 

    return false;
}


bool i2c_read_from_slave_cb(volatile uint16_t *addr, uint8_t *wb)
{
    dbg_analog(24);
    
    address_t c;
    c.raw = *addr;
        
    if(c.isCMD)
        return handle_read_cmd(c, wb);
    else if(*addr < SLAVE_EEPROM_SIZE)
    {
        // Wrap around in EEPROM area
        if(*addr >= SLAVE_EEPROM_SIZE)
            *addr = 0;

        // Send data to master
        *wb = eeprom_read_buffer((eeAddr_t)*addr);    //i2c1EEPMemAddr);//42);//
        
        // Next byte
        *addr += 1; 
        
        return true;
    }
    
    return false;   
}


void main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();
    
    dbg_analog(0);
    eeprom_buffer_whole();
    analog_init();
    dbg_analog(32);
    
    dbg_printf("Fancy I2C Extender\r\n");
    
    i2c_slave_init(I2C_SLAVE_ADDRESS, I2C_SLAVE_MASK, &i2c_write_to_slave_cb, &i2c_read_from_slave_cb);
    dbg_analog(0);  
    
    while (1)
    {
        eeprom_handle_writing();
        analog_handler();
    }
}

/**
 End of File
*/