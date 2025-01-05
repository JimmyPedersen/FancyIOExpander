#if 0

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();


        
        // Read EEPROM and update data in RAM
//        E2_DATA[i2c1EEPMemAddr] = DATAEE_ReadByte((uint8_t)i2c1EEPMemAddr);
        


/*
        // Handle commands
        uint16_t tmp = i2c1EEPMemAddr - CMD_START_ADDRESS;
        uint8_t cmd = tmp & 0x0FFF;
        tmp>>=12;
        uint8_t portNo = tmp & 3;
        tmp>>=2;
        bool extra = tmp & 1;
*/
 
        
/*        
        
        switch(c->cmd)
        {
            case 0:
                // Read ports (If extra is set it uses the latches current value otherwhise actual pin voltages)
                I2C1_Write(ReadPort(portNo, c->extra));
                break;
                
                
                
            case 1:
                // Read ports (If extra is set it uses the latches current value otherwhise actual pin voltages)
                I2C1_Write(ReadPort(portNo, c->extra));
                break;
                
        }
*/




#endif







#if 0

// i2c_slave.h
#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include "mcc_generated_files/mcc.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// Software version information
#define SW_VERSION_MAJOR    1
#define SW_VERSION_MINOR    0
#define SW_VERSION_PATCH    0

// I2C slave address
#define I2C_SLAVE_ADDR 0x50

// Command definitions
#define CMD_WRITE_DATA      0x01
#define CMD_READ_DATA       0x02
#define CMD_GET_VERSION     0x03

// Buffer size
#define MAX_BUFFER_SIZE 4

// Command state machine states
typedef enum {
    STATE_IDLE,
    STATE_CMD_RECEIVED,
    STATE_DATA_PROCESSING
} cmd_state_t;

// Command structure
typedef struct {
    uint8_t command;
    uint8_t length;
    uint8_t *data;
    bool needsResponse;
    uint8_t responseLength;
} cmd_t;

void I2C_SlaveInit(void);
void I2C_SlaveHandler(void);

#endif

// i2c_slave.c
//#include "i2c_slave.h"
//#include "mcc_generated_files/mcc.h"

// Global variables
static uint8_t dataBuffer[MAX_BUFFER_SIZE];
static uint8_t responseBuffer[MAX_BUFFER_SIZE];
static uint8_t bufferIndex = 0;
static uint8_t responseIndex = 0;
static cmd_state_t currentState = STATE_IDLE;
static cmd_t currentCmd = {0};

// Command handlers
static void HandleWriteData(uint8_t *data, uint8_t length);
static void HandleReadData(void);
static void HandleGetVersion(void);

// Command configuration table
typedef struct {
    uint8_t command;
    uint8_t expectedLength;
    void (*handler)(uint8_t*, uint8_t);
    bool needsResponse;
    uint8_t responseLength;
} cmd_config_t;

static const cmd_config_t cmdConfig[] = {
    {CMD_WRITE_DATA, MAX_BUFFER_SIZE, HandleWriteData, false, 0},
    {CMD_READ_DATA, 0, HandleReadData, true, MAX_BUFFER_SIZE},
    {CMD_GET_VERSION, 0, HandleGetVersion, true, 3}
};




void I2C_SlaveInit(void) {
    // Initialize I2C peripheral using MCC
    I2C1_Initialize();
    
    // Set slave address
    I2C1_Open();
//    I2C1_SlaveSetAddress(I2C_SLAVE_ADDR);
    
    // Clear buffers
    for(uint8_t i = 0; i < MAX_BUFFER_SIZE; i++) {
        dataBuffer[i] = 0;
        responseBuffer[i] = 0;
    }
}

static void HandleWriteData(uint8_t *data, uint8_t length) {
    for(uint8_t i = 0; i < length && i < MAX_BUFFER_SIZE; i++) {
        dataBuffer[i] = data[i];
    }
}

static void HandleReadData(void) {
    for(uint8_t i = 0; i < MAX_BUFFER_SIZE; i++) {
        responseBuffer[i] = dataBuffer[i];
    }
}

static void HandleGetVersion(void) {
    responseBuffer[0] = SW_VERSION_MAJOR;
    responseBuffer[1] = SW_VERSION_MINOR;
    responseBuffer[2] = SW_VERSION_PATCH;
}

static void ProcessCommand(uint8_t command) {
    // Find command in configuration table
    for(uint8_t i = 0; i < sizeof(cmdConfig)/sizeof(cmd_config_t); i++) {
        if(cmdConfig[i].command == command) {
            currentCmd.command = command;
            currentCmd.length = cmdConfig[i].expectedLength;
            currentCmd.needsResponse = cmdConfig[i].needsResponse;
            currentCmd.responseLength = cmdConfig[i].responseLength;
            return;
        }
    }
    // Invalid command handling
    currentCmd.command = 0;
    currentState = STATE_IDLE;
}

void I2C_SlaveHandler(void) {
    if (I2C1_IsRxBufFull()) {
        uint8_t receivedByte = I2C1_ReadByte();
        
        switch(currentState) {
            case STATE_IDLE:
                ProcessCommand(receivedByte);
                if(currentCmd.command != 0) {
                    currentState = STATE_CMD_RECEIVED;
                    bufferIndex = 0;
                    if(currentCmd.length == 0) {
                        currentState = STATE_DATA_PROCESSING;
                    }
                }
                break;
                
            case STATE_CMD_RECEIVED:
                if(bufferIndex < currentCmd.length) {
                    dataBuffer[bufferIndex++] = receivedByte;
                    if(bufferIndex >= currentCmd.length) {
                        currentState = STATE_DATA_PROCESSING;
                    }
                }
                break;
                
            default:
                break;
        }
    }
    
    if(currentState == STATE_DATA_PROCESSING) {
        // Execute command handler
        for(uint8_t i = 0; i < sizeof(cmdConfig)/sizeof(cmd_config_t); i++) {
            if(cmdConfig[i].command == currentCmd.command) {
                if(cmdConfig[i].handler != NULL) {
                    cmdConfig[i].handler(dataBuffer, bufferIndex);
                }
                break;
            }
        }
        responseIndex = 0;
        currentState = STATE_IDLE;
    }
    
    if (I2C1_IsData() && currentCmd.needsResponse) {
        if(responseIndex < currentCmd.responseLength) {
            I2C1_WriteByte(responseBuffer[responseIndex++]);
        }
    }
    
    if (I2C1_IsStop()) {
        currentState = STATE_IDLE;
        bufferIndex = 0;
        responseIndex = 0;
    }
}

// main.c
#include "mcc_generated_files/mcc.h"
//#include "i2c_slave.h"

void main(void) {
    // Initialize the device
    SYSTEM_Initialize();
    
    // Initialize I2C slave
    I2C_SlaveInit();
    
    // Enable interrupts
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    while (1) {
        I2C_SlaveHandler();
        
        // Other application code here
    }
}
#endif


#if 0
/**
 * PIC18F24 GPIO Controller
 * 
 * Features:
 * - Complete GPIO control (input/output, pull-up/down)
 * - PWM configuration (frequency and duty cycle)
 * - Port-wide operations
 * - UART command interface with buffering
 * - I2C slave functionality
 * - Change notification capability
 * 
 * Compiler: XC8
 * Required: MCC Libraries
 */

#include "mcc_generated_files/mcc.h"
#include <string.h>
#include <stdint.h>

// Total number of pins
#define PORT_MAX_PINS 24

// Buffer sizes
#define UART_BUFFER_SIZE 64
#define MAX_COMMAND_LENGTH 32
#define PWM_CHANNELS 4

// Command definitions
#define CMD_SET_GPIO 0x01
#define CMD_READ_GPIO 0x02
#define CMD_SET_PORT 0x03
#define CMD_READ_PORT 0x04
#define CMD_CONFIG_PWM 0x05
#define CMD_SET_PULLUP 0x06
#define CMD_SET_PULLDOWN 0x07
#define CMD_SET_ANALOG 0x08
#define CMD_READ_ANALOG 0x09
#define CMD_CONFIG_CHANGE_NOT 0x0A

// Buffers
volatile uint8_t uartRxBuffer[UART_BUFFER_SIZE];
volatile uint8_t uartTxBuffer[UART_BUFFER_SIZE];
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;
volatile uint8_t txHead = 0;
volatile uint8_t txTail = 0;

// State variables
volatile uint8_t changeNotificationPin = 0xFF; // 0xFF = disabled
struct {
    uint8_t enabled;
    uint8_t frequency;
    uint8_t dutyCycle;
    uint8_t pin;
} pwmChannels[PWM_CHANNELS];

// I2C slave address
#define I2C_SLAVE_ADDR 0x50

// I2C state machine states
typedef enum {
    I2C_STATE_WAIT_CMD,
    I2C_STATE_WAIT_LENGTH,
    I2C_STATE_GET_DATA,
    I2C_STATE_PROCESS
} i2c_state_t;

// I2C communication structure
static struct {
    i2c_state_t state;
    uint8_t cmd;
    uint8_t length;
    uint8_t dataIndex;
    uint8_t data[MAX_COMMAND_LENGTH];
} i2cComm = {I2C_STATE_WAIT_CMD, 0, 0, 0, {0}};


// Function prototypes
void initializeSystem(void);
void processCommand(uint8_t cmd, uint8_t* data, uint8_t length);
void handleUARTReceive(void);

void configurePin(uint8_t pin, uint8_t mode);
void setPWM(uint8_t channel, uint8_t frequency, uint8_t dutyCycle);
void setChangeNotification(uint8_t pin);
void checkPinChanges(void);


/**
 * I2C Write callback handler
 * Generated by MCC, we just need to implement it
 */
void I2C1_WR_ISR(void) {
    uint8_t incoming = I2C1_Read();  // Read the received byte
    
    switch(i2cComm.state) {
        case I2C_STATE_WAIT_CMD:
            i2cComm.cmd = incoming;
            i2cComm.state = I2C_STATE_WAIT_LENGTH;
            break;
            
        case I2C_STATE_WAIT_LENGTH:
            i2cComm.length = incoming;
            i2cComm.dataIndex = 0;
            if(i2cComm.length > 0) {
                i2cComm.state = I2C_STATE_GET_DATA;
            } else {
                i2cComm.state = I2C_STATE_PROCESS;
            }
            break;
            
        case I2C_STATE_GET_DATA:
            if(i2cComm.dataIndex < MAX_COMMAND_LENGTH) {
                i2cComm.data[i2cComm.dataIndex++] = incoming;
                if(i2cComm.dataIndex >= i2cComm.length) {
                    i2cComm.state = I2C_STATE_PROCESS;
                }
            }
            break;
            
        default:
            i2cComm.state = I2C_STATE_WAIT_CMD;
            break;
    }
}

/**
 * I2C Read callback handler
 * Generated by MCC, we just need to implement it
 */
void I2C1_RD_ISR(void) {
    if(txHead > txTail) {
        I2C1_Write(uartTxBuffer[txTail++]);
    } else {
        I2C1_Write(0xFF);  // No data available
    }
}

/**
 * I2C Address callback handler
 * Generated by MCC, we just need to implement it
 */
void I2C1_ADDR_ISR(void) {
    // Address match occurred
    // Reset state if it's a new write operation
    if(!I2C1_IsRead()) {
        i2cComm.state = I2C_STATE_WAIT_CMD;
    }
}

/**
 * Handle I2C receive in main loop
 * Much simpler now using MCC libraries
 */
void handleI2CReceive(void) {
    // Process any complete commands
    if(i2cComm.state == I2C_STATE_PROCESS) {
        processCommand(i2cComm.cmd, i2cComm.data, i2cComm.length);
        i2cComm.state = I2C_STATE_WAIT_CMD;
    }
    
    // MCC handles all the low-level I2C operations
    I2C1_Tasks();
}

/**
 * Initialize I2C slave mode
 * Using MCC generated functions
 */
void initI2CSlave(void) {
    // MCC handles all the I2C configuration
    I2C1_Initialize();
        
    // Set up the callbacks
    I2C1_SlaveSetWriteIntHandler(I2C1_WR_ISR);
    I2C1_SlaveSetReadIntHandler(I2C1_RD_ISR);
    I2C1_SlaveSetAddrIntHandler(I2C1_ADDR_ISR);
}

/**
 * System initialization
 */
void initializeSystem(void) {
    // Initialize the device
    SYSTEM_Initialize();
    
    // Configure I2C slave mode
    I2C1_Open();
    
    // Initialize PWM channels
    for(uint8_t i = 0; i < PWM_CHANNELS; i++) {
        pwmChannels[i].enabled = 0;
    }
    
    // Enable interrupts
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
}


/**
 * GPIO Pin Write function
 * pin: 0-7 for PORTA, 8-15 for PORTB, 16-23 for PORTC
 * value: 0 or 1
 */
void GPIO_PinWrite(uint8_t pin, uint8_t value) {
    uint8_t port = pin / 8;
    uint8_t pin_num = pin % 8;
    
    switch(port) {
        case 0: // PORTA
            if(value) {
                LATA |= (1 << pin_num);
            } else {
                LATA &= ~(1 << pin_num);
            }
            break;
            
        case 1: // PORTB
            if(value) {
                LATB |= (1 << pin_num);
            } else {
                LATB &= ~(1 << pin_num);
            }
            break;
            
        case 2: // PORTC
            if(value) {
                LATC |= (1 << pin_num);
            } else {
                LATC &= ~(1 << pin_num);
            }
            break;
    }
}

/**
 * GPIO Pin Read function
 * pin: 0-7 for PORTA, 8-15 for PORTB, 16-23 for PORTC
 * returns: pin state (0 or 1)
 */
uint8_t GPIO_PinRead(uint8_t pin) {
    uint8_t port = pin / 8;
    uint8_t pin_num = pin % 8;
    uint8_t value = 0;
    
    switch(port) {
        case 0: // PORTA
            value = (PORTA >> pin_num) & 0x01;
            break;
            
        case 1: // PORTB
            value = (PORTB >> pin_num) & 0x01;
            break;
            
        case 2: // PORTC
            value = (PORTC >> pin_num) & 0x01;
            break;
    }
    
    return value;
}

/**
 * Configure GPIO pin direction
 * pin: 0-23
 * isOutput: 1 for output, 0 for input
 */
void GPIO_PinDirection(uint8_t pin, uint8_t isOutput) {
    uint8_t port = pin / 8;
    uint8_t pin_num = pin % 8;
    
    switch(port) {
        case 0: // PORTA
            if(isOutput) {
                TRISA &= ~(1 << pin_num);
            } else {
                TRISA |= (1 << pin_num);
            }
            break;
            
        case 1: // PORTB
            if(isOutput) {
                TRISB &= ~(1 << pin_num);
            } else {
                TRISB |= (1 << pin_num);
            }
            break;
            
        case 2: // PORTC
            if(isOutput) {
                TRISC &= ~(1 << pin_num);
            } else {
                TRISC |= (1 << pin_num);
            }
            break;
    }
}

/**
 * Process received command
 */
void processCommand(uint8_t cmd, uint8_t* data, uint8_t length) {
    switch(cmd) {
        case CMD_SET_GPIO:
            if(length >= 2) {
                uint8_t pin = data[0];
                uint8_t value = data[1];
                // Set individual pin
                if(pin < PORT_MAX_PINS) {
                    GPIO_PinWrite(pin, value);
                }
            }
            break;
            
        case CMD_READ_GPIO:
            if(length >= 1) {
                uint8_t pin = data[0];
                uint8_t value = GPIO_PinRead(pin);
                // Send response via UART
                if(txHead < UART_BUFFER_SIZE - 1) {
                    uartTxBuffer[txHead++] = value;
                }
            }
            break;
            
        case CMD_CONFIG_PWM:
            if(length >= 4) {
                uint8_t channel = data[0];
                uint8_t pin = data[1];
                uint8_t frequency = data[2];
                uint8_t dutyCycle = data[3];
                if(channel < PWM_CHANNELS) {
                    setPWM(channel, frequency, dutyCycle);
                    pwmChannels[channel].pin = pin;
                }
            }
            break;
            
        // Add other command handlers
    }
}

/**
 * Set PWM duty cycle for a specific channel
 * channel: PWM channel number (1-4)
 * dutyCycle: 0-255 (0-100%)
 */
void PWM_LoadDutyValue(uint8_t channel, uint8_t dutyCycle) {
    uint16_t duty = ((uint16_t)dutyCycle * (PR2 + 1)) / 255;
    
    switch(channel) {
        case 1:
            CCPR1L = (uint8_t)(duty >> 2);
            CCP1CON = (uint8_t)((CCP1CON & 0xCF) | ((duty & 0x03) << 4));
            break;
            
        case 2:
            CCPR2L = (uint8_t)(duty >> 2);
            CCP2CON = (uint8_t)((CCP2CON & 0xCF) | ((duty & 0x03) << 4));
            break;
            
        // Add cases for additional PWM channels if available
    }
}

/**
 * Configure PWM on specified channel
 */
void setPWM(uint8_t channel, uint8_t frequency, uint8_t dutyCycle) {
    if(channel >= PWM_CHANNELS) return;
    
    pwmChannels[channel].enabled = 1;
    pwmChannels[channel].frequency = frequency;
    pwmChannels[channel].dutyCycle = dutyCycle;
    
    // Configure PWM using MCC PWM module
    // Specific implementation depends on MCC configuration
    switch(channel) {
        case 0:
            PWM_LoadDutyValue(1, dutyCycle);
            break;
        case 1:
            PWM_LoadDutyValue(2, dutyCycle);
            break;
        // Add other channels
    }
}

/**
 * UART receive interrupt handler
 */
void UART_RX_ISR(void) {
    if(EUSART1_is_rx_ready() && (rxHead < UART_BUFFER_SIZE - 1)) {
        uartRxBuffer[rxHead++] = EUSART1_Read();
    }
}

/**
 * I2C receive interrupt handler
 */
void I2C_RX_ISR(void) {
    uint8_t data = I2C1_Read();
    if(rxHead < UART_BUFFER_SIZE - 1) {
        uartRxBuffer[rxHead++] = data;
    }
}

/**
 * Main program loop
 */
void main(void) {
    initializeSystem();
    
    while(1) {
        // Process UART receive buffer
        if(rxHead != rxTail) {
            uint8_t cmd = uartRxBuffer[rxTail++];
            uint8_t length = uartRxBuffer[rxTail++];
            uint8_t data[MAX_COMMAND_LENGTH];
            
            for(uint8_t i = 0; i < length && rxTail < rxHead; i++) {
                data[i] = uartRxBuffer[rxTail++];
            }
            
            processCommand(cmd, data, length);
        }
        
        // Check for pin changes if enabled
        if(changeNotificationPin != 0xFF) {
            checkPinChanges();
        }
    }
}

/**
 * Check for GPIO pin changes and signal if detected
 */
void checkPinChanges(void) {
    static uint16_t lastPinStates = 0;
    uint16_t currentPinStates = 0;
    
    // Read all GPIO pins
    // Implementation depends on port configuration
    currentPinStates = ((uint16_t)PORTA << 8) | PORTB;
    
    if(currentPinStates != lastPinStates) {
        // Signal change on notification pin
        if(changeNotificationPin != 0xFF) {
            GPIO_PinWrite(changeNotificationPin, 0);
            // Pin will return to Hi-Z after reading status
        }
        lastPinStates = currentPinStates;
    }
}

#endif

#if 0
/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F24Q10
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
#include <stdio.h>
//#include "mcc_generated_files/eusart1.h"
/*
                         Main application
 */
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    
    while (1)
    {
        // Add your application code

//        EUSART1_Write('J');    EUSART1_Write('P');    EUSART1_Write('!');    EUSART1_Write('\r');    EUSART1_Write('\n');
//        printf("Hello from printf!\r\n");       
       __delay_ms(10);

    }
}
#endif