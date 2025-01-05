/* 
 * File:   registers.h
 * Author: Jimmy
 *
 * Created on November 18, 2024, 6:55 PM
 */

#pragma once

#include "mcc_generated_files/mcc.h"
#include "config.h"

#define REGISTERS_MAX_COUNT 32

typedef enum
{
    eLAT,       // 0
    ePORT,      // 1
    eTRIS,      // 2
    eINLVL,     // 3
    eSLRCON,    // 4
    eODCON,     // 5
    eWPU,       // 6
    eANSEL,     // 7
    eIOCxF,     // 8
    eIOCxN,     // 9
    eIOCxP,     // 00
    eRegistersCNT       
}Registers_t;


uint8_t ReadBit(Registers_t regNo, EXP_PORT portNo, EXP_PIN pinNo);
bool ReadReg(Registers_t regNo, EXP_PORT portNo, port_t *dst);
void WriteBit(Registers_t regNo, EXP_PORT portNo, EXP_PIN pinNo, bool value);
bool WriteReg(Registers_t regNo, EXP_PORT portNo, port_t value);

