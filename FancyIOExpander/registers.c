#include "registers.h"
#include <assert.h>

#define REGS_CNT_OFFSET(c,o) (uint8_t)((c & 3) <<5 | (o & 31) )
#define REGS_CNT(b)          (b >> 5) 
#define REGS_OFFSET(b)       (b & 31) 

const struct
{
    volatile unsigned char *reg;
    unsigned char portCntOffset;
}Registers[eRegistersCNT] = {
    {   &LATA,     REGS_CNT_OFFSET(CPU_PORTS, 1)    },
    {   &PORTA,    REGS_CNT_OFFSET(CPU_PORTS, 1)    },
    {   &TRISA,    REGS_CNT_OFFSET(CPU_PORTS, 1)    },
    {   &INLVLA,   REGS_CNT_OFFSET(CPU_PORTS, 8)    },
    {   &SLRCONA,  REGS_CNT_OFFSET(CPU_PORTS, 8)    },
    {   &ODCONA,   REGS_CNT_OFFSET(CPU_PORTS, 8)    },
    {   &WPUA,     REGS_CNT_OFFSET(CPU_PORTS, 8)    },
    {   &ANSELA,   REGS_CNT_OFFSET(CPU_PORTS, 8)    },   
    {   &IOCAF,    REGS_CNT_OFFSET(CPU_PORTS, 8)    },
    {   &IOCAN,    REGS_CNT_OFFSET(CPU_PORTS, 8)    },
    {   &IOCAP,    REGS_CNT_OFFSET(CPU_PORTS, 8)    }
};



uint8_t ReadBit(Registers_t regNo, EXP_PORT portNo, EXP_PIN pinNo)
{
    assert(regNo < eRegistersCNT);
    assert(portNo < eEXP_PORT_CNT);
    assert(pinNo < eEXP_PIN_CNT);
    
    const Pin_t *pin = &PINS[portNo][pinNo]; 
//    if(pin->Port >= REGS_CNT(Registers[regNo].portCntOffset))
//        return 0;   
    volatile port_t *reg = Registers[regNo].reg;
    reg += pin->Port * REGS_OFFSET(Registers[regNo].portCntOffset);
    port_t portData = *reg;
    port_t portMask = (port_t)((int)1 << pin->Pin);
    return (portData & portMask) ? 1 : 0;
}

bool ReadReg(Registers_t regNo, EXP_PORT portNo, port_t *dst)
{
    assert(regNo < eRegistersCNT);
    assert(portNo < EXP_PORTS_CNT);
    
    if(regNo >= eRegistersCNT)
        return false;

    if(portNo >= EXP_PORTS_CNT)
        return false;
    
    port_t tmp = 0;   
    for(uint8_t p = EXP_PINS_PER_PORT; p > 0; p--)
    {
        tmp<<=1;
        tmp |= ReadBit(regNo, portNo, p-1);
    }  
    *dst = tmp;
    return true;
}


void WriteBit(Registers_t regNo, EXP_PORT portNo, EXP_PIN pinNo, bool value)
{
    assert(regNo < eRegistersCNT);
    assert(portNo < eEXP_PORT_CNT);
    assert(pinNo < eEXP_PIN_CNT);
    
    const Pin_t *pin = &PINS[portNo][pinNo]; 
//    if(pin->Port >= REGS_CNT(Registers[regNo].portCntOffset))
//        return 0;   
    volatile unsigned char *reg = Registers[regNo].reg;
    reg += pin->Port * REGS_OFFSET(Registers[regNo].portCntOffset);   
      
    port_t portData = *reg;
    port_t portMask = (port_t)((int)1<<pin->Pin);
    if(value)
        portData |= portMask;
    else
        portData &= ~portMask;
    *reg = portData;
}

bool WriteReg(Registers_t regNo, EXP_PORT portNo, port_t value)
{
    assert(regNo < eRegistersCNT);
    assert(portNo < EXP_PORTS_CNT);
    
    if(regNo >= eRegistersCNT)
        return false;
    
    if(portNo >= eEXP_PORT_CNT)
        return false;
      
    for(uint8_t p = 0; p < EXP_PINS_PER_PORT; p++)
    {
        WriteBit(regNo, portNo, p, value & 1);
        value >>= 1;
    } 
    return true;
}
