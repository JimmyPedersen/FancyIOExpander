/* 
 * File:   main.h
 * Author: Jimmy
 *
 * Created on November 18, 2024, 6:57 PM
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define EXP_PORTS_CNT                   2
#define EXP_PINS_PER_PORT               8
#define EXP_PINS_COUNT                  ( eEXP_PORT_CNT * EXP_PINS_PER_PORT )

#define EXP_PORTS_MAX_CNT               4
#define CPU_PORTS                       3

#define EXP_PORT_FROM_IDX(IDX)          ( IDX / EXP_PINS_PER_PORT )
#define EXP_PIN_FROM_IDX(IDX)           ( IDX & (EXP_PINS_PER_PORT - 1) )
#define IDX_FROM_PORT_PIN(PORT, PIN)    ( (PORT * EXP_PINS_PER_PORT) + PIN )       


#if EXP_PINS_PER_PORT<=8
    typedef uint8_t port_t;
#else
    typedef uint16_t port_t;    
#endif

    
#define I2C_SLAVE_ADDRESS               42
#define I2C_SLAVE_MASK                  127

typedef enum
{
    ePORTA = 0,
    ePORTB = 1,
    ePORTC = 2,
    eCPU_PORT_CNT
} CPU_PORT;

typedef enum
{
    eEXP_PORT_0 = 0,
    eEXP_PORT_1 = 1,
    eEXP_PORT_CNT
} EXP_PORT;


typedef enum
{
    eEXP_PIN_0 = 0,
    eEXP_PIN_1 = 1,
    eEXP_PIN_2 = 2,
    eEXP_PIN_3 = 3,
    eEXP_PIN_4 = 4,
    eEXP_PIN_5 = 5,
    eEXP_PIN_6 = 6,
    eEXP_PIN_7 = 7,
#if EXP_PINS_PER_PORT > 8            
    eEXP_PIN_8 = 8,
    eEXP_PIN_9 = 9,
    eEXP_PIN_10 = 10,
    eEXP_PIN_11 = 11,
    eEXP_PIN_12 = 12,
    eEXP_PIN_13 = 13,
    eEXP_PIN_14 = 14,
    eEXP_PIN_15 = 15,
#endif       
    eEXP_PIN_CNT
} EXP_PIN;


typedef struct {
    union
    {
        struct 
        {    
            unsigned char lb;
            unsigned char hb;         
        };
        unsigned char arr[2];
        unsigned int word;
    };
}word_arr_t;

typedef struct {
    union
    {
        struct 
        {    
            unsigned int Port : 4;
            unsigned int Pin : 4;     
            unsigned int Unused : 3;
            unsigned int AnalogCh : 5;      
        };
        unsigned int Raw;
    };
}Pin_t;

const Pin_t PINS[EXP_PORTS_CNT][EXP_PINS_PER_PORT] =
{
    {
        { ePORTA, 3, 0, 3 },    // P0.0
        { ePORTA, 4, 0, 4  },   // P0.1
        { ePORTA, 5, 0, 5  },   // P0.2
        { ePORTA, 7, 0, 7  },   // P0.3
        { ePORTA, 6, 0, 6  },   // P0.4
        { ePORTC, 0, 0, 16 },   // P0.5
        { ePORTC, 1, 0, 17 },   // P0.6
        { ePORTC, 2, 0, 18 }    // P0.7
    },
    {  
        { ePORTB, 4, 0, 12 },   // P1.0
        { ePORTB, 3, 0, 11 },   // P1.1
        { ePORTB, 2, 0, 10 },   // P1.2
        { ePORTB, 1, 0, 9  },   // P1.3
        { ePORTB, 0, 0, 8  },   // P1.4
        { ePORTC, 7, 0, 23 },   // P1.5
        { ePORTC, 6, 0, 22 },   // P1.6
        { ePORTC, 5, 0, 21 }    // P1.7
    }
};
