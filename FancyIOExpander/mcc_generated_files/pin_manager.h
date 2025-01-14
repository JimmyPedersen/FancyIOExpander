/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F24Q10
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.36 and above
        MPLAB 	          :  MPLAB X 6.00	
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

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set I2C_ADDRESS aliases
#define I2C_ADDRESS_TRIS                 TRISAbits.TRISA0
#define I2C_ADDRESS_LAT                  LATAbits.LATA0
#define I2C_ADDRESS_PORT                 PORTAbits.RA0
#define I2C_ADDRESS_WPU                  WPUAbits.WPUA0
#define I2C_ADDRESS_OD                   ODCONAbits.ODCA0
#define I2C_ADDRESS_ANS                  ANSELAbits.ANSELA0
#define I2C_ADDRESS_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define I2C_ADDRESS_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define I2C_ADDRESS_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define I2C_ADDRESS_GetValue()           PORTAbits.RA0
#define I2C_ADDRESS_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define I2C_ADDRESS_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define I2C_ADDRESS_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define I2C_ADDRESS_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define I2C_ADDRESS_SetPushPull()        do { ODCONAbits.ODCA0 = 0; } while(0)
#define I2C_ADDRESS_SetOpenDrain()       do { ODCONAbits.ODCA0 = 1; } while(0)
#define I2C_ADDRESS_SetAnalogMode()      do { ANSELAbits.ANSELA0 = 1; } while(0)
#define I2C_ADDRESS_SetDigitalMode()     do { ANSELAbits.ANSELA0 = 0; } while(0)

// get/set IO_RA1 aliases
#define IO_RA1_TRIS                 TRISAbits.TRISA1
#define IO_RA1_LAT                  LATAbits.LATA1
#define IO_RA1_PORT                 PORTAbits.RA1
#define IO_RA1_WPU                  WPUAbits.WPUA1
#define IO_RA1_OD                   ODCONAbits.ODCA1
#define IO_RA1_ANS                  ANSELAbits.ANSELA1
#define IO_RA1_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define IO_RA1_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define IO_RA1_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define IO_RA1_GetValue()           PORTAbits.RA1
#define IO_RA1_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define IO_RA1_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define IO_RA1_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define IO_RA1_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define IO_RA1_SetPushPull()        do { ODCONAbits.ODCA1 = 0; } while(0)
#define IO_RA1_SetOpenDrain()       do { ODCONAbits.ODCA1 = 1; } while(0)
#define IO_RA1_SetAnalogMode()      do { ANSELAbits.ANSELA1 = 1; } while(0)
#define IO_RA1_SetDigitalMode()     do { ANSELAbits.ANSELA1 = 0; } while(0)

// get/set RA2 procedures
#define RA2_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define RA2_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define RA2_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define RA2_GetValue()              PORTAbits.RA2
#define RA2_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define RA2_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define RA2_SetPullup()             do { WPUAbits.WPUA2 = 1; } while(0)
#define RA2_ResetPullup()           do { WPUAbits.WPUA2 = 0; } while(0)
#define RA2_SetAnalogMode()         do { ANSELAbits.ANSELA2 = 1; } while(0)
#define RA2_SetDigitalMode()        do { ANSELAbits.ANSELA2 = 0; } while(0)

// get/set P0_0 aliases
#define P0_0_TRIS                 TRISAbits.TRISA3
#define P0_0_LAT                  LATAbits.LATA3
#define P0_0_PORT                 PORTAbits.RA3
#define P0_0_WPU                  WPUAbits.WPUA3
#define P0_0_OD                   ODCONAbits.ODCA3
#define P0_0_ANS                  ANSELAbits.ANSELA3
#define P0_0_SetHigh()            do { LATAbits.LATA3 = 1; } while(0)
#define P0_0_SetLow()             do { LATAbits.LATA3 = 0; } while(0)
#define P0_0_Toggle()             do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define P0_0_GetValue()           PORTAbits.RA3
#define P0_0_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define P0_0_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define P0_0_SetPullup()          do { WPUAbits.WPUA3 = 1; } while(0)
#define P0_0_ResetPullup()        do { WPUAbits.WPUA3 = 0; } while(0)
#define P0_0_SetPushPull()        do { ODCONAbits.ODCA3 = 0; } while(0)
#define P0_0_SetOpenDrain()       do { ODCONAbits.ODCA3 = 1; } while(0)
#define P0_0_SetAnalogMode()      do { ANSELAbits.ANSELA3 = 1; } while(0)
#define P0_0_SetDigitalMode()     do { ANSELAbits.ANSELA3 = 0; } while(0)

// get/set P0_1 aliases
#define P0_1_TRIS                 TRISAbits.TRISA4
#define P0_1_LAT                  LATAbits.LATA4
#define P0_1_PORT                 PORTAbits.RA4
#define P0_1_WPU                  WPUAbits.WPUA4
#define P0_1_OD                   ODCONAbits.ODCA4
#define P0_1_ANS                  ANSELAbits.ANSELA4
#define P0_1_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define P0_1_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define P0_1_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define P0_1_GetValue()           PORTAbits.RA4
#define P0_1_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define P0_1_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define P0_1_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define P0_1_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define P0_1_SetPushPull()        do { ODCONAbits.ODCA4 = 0; } while(0)
#define P0_1_SetOpenDrain()       do { ODCONAbits.ODCA4 = 1; } while(0)
#define P0_1_SetAnalogMode()      do { ANSELAbits.ANSELA4 = 1; } while(0)
#define P0_1_SetDigitalMode()     do { ANSELAbits.ANSELA4 = 0; } while(0)

// get/set P0_2 aliases
#define P0_2_TRIS                 TRISAbits.TRISA5
#define P0_2_LAT                  LATAbits.LATA5
#define P0_2_PORT                 PORTAbits.RA5
#define P0_2_WPU                  WPUAbits.WPUA5
#define P0_2_OD                   ODCONAbits.ODCA5
#define P0_2_ANS                  ANSELAbits.ANSELA5
#define P0_2_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define P0_2_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define P0_2_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define P0_2_GetValue()           PORTAbits.RA5
#define P0_2_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define P0_2_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define P0_2_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define P0_2_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define P0_2_SetPushPull()        do { ODCONAbits.ODCA5 = 0; } while(0)
#define P0_2_SetOpenDrain()       do { ODCONAbits.ODCA5 = 1; } while(0)
#define P0_2_SetAnalogMode()      do { ANSELAbits.ANSELA5 = 1; } while(0)
#define P0_2_SetDigitalMode()     do { ANSELAbits.ANSELA5 = 0; } while(0)

// get/set P0_4 aliases
#define P0_4_TRIS                 TRISAbits.TRISA6
#define P0_4_LAT                  LATAbits.LATA6
#define P0_4_PORT                 PORTAbits.RA6
#define P0_4_WPU                  WPUAbits.WPUA6
#define P0_4_OD                   ODCONAbits.ODCA6
#define P0_4_ANS                  ANSELAbits.ANSELA6
#define P0_4_SetHigh()            do { LATAbits.LATA6 = 1; } while(0)
#define P0_4_SetLow()             do { LATAbits.LATA6 = 0; } while(0)
#define P0_4_Toggle()             do { LATAbits.LATA6 = ~LATAbits.LATA6; } while(0)
#define P0_4_GetValue()           PORTAbits.RA6
#define P0_4_SetDigitalInput()    do { TRISAbits.TRISA6 = 1; } while(0)
#define P0_4_SetDigitalOutput()   do { TRISAbits.TRISA6 = 0; } while(0)
#define P0_4_SetPullup()          do { WPUAbits.WPUA6 = 1; } while(0)
#define P0_4_ResetPullup()        do { WPUAbits.WPUA6 = 0; } while(0)
#define P0_4_SetPushPull()        do { ODCONAbits.ODCA6 = 0; } while(0)
#define P0_4_SetOpenDrain()       do { ODCONAbits.ODCA6 = 1; } while(0)
#define P0_4_SetAnalogMode()      do { ANSELAbits.ANSELA6 = 1; } while(0)
#define P0_4_SetDigitalMode()     do { ANSELAbits.ANSELA6 = 0; } while(0)

// get/set P0_3 aliases
#define P0_3_TRIS                 TRISAbits.TRISA7
#define P0_3_LAT                  LATAbits.LATA7
#define P0_3_PORT                 PORTAbits.RA7
#define P0_3_WPU                  WPUAbits.WPUA7
#define P0_3_OD                   ODCONAbits.ODCA7
#define P0_3_ANS                  ANSELAbits.ANSELA7
#define P0_3_SetHigh()            do { LATAbits.LATA7 = 1; } while(0)
#define P0_3_SetLow()             do { LATAbits.LATA7 = 0; } while(0)
#define P0_3_Toggle()             do { LATAbits.LATA7 = ~LATAbits.LATA7; } while(0)
#define P0_3_GetValue()           PORTAbits.RA7
#define P0_3_SetDigitalInput()    do { TRISAbits.TRISA7 = 1; } while(0)
#define P0_3_SetDigitalOutput()   do { TRISAbits.TRISA7 = 0; } while(0)
#define P0_3_SetPullup()          do { WPUAbits.WPUA7 = 1; } while(0)
#define P0_3_ResetPullup()        do { WPUAbits.WPUA7 = 0; } while(0)
#define P0_3_SetPushPull()        do { ODCONAbits.ODCA7 = 0; } while(0)
#define P0_3_SetOpenDrain()       do { ODCONAbits.ODCA7 = 1; } while(0)
#define P0_3_SetAnalogMode()      do { ANSELAbits.ANSELA7 = 1; } while(0)
#define P0_3_SetDigitalMode()     do { ANSELAbits.ANSELA7 = 0; } while(0)

// get/set P1_4 aliases
#define P1_4_TRIS                 TRISBbits.TRISB0
#define P1_4_LAT                  LATBbits.LATB0
#define P1_4_PORT                 PORTBbits.RB0
#define P1_4_WPU                  WPUBbits.WPUB0
#define P1_4_OD                   ODCONBbits.ODCB0
#define P1_4_ANS                  ANSELBbits.ANSELB0
#define P1_4_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define P1_4_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define P1_4_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define P1_4_GetValue()           PORTBbits.RB0
#define P1_4_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define P1_4_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define P1_4_SetPullup()          do { WPUBbits.WPUB0 = 1; } while(0)
#define P1_4_ResetPullup()        do { WPUBbits.WPUB0 = 0; } while(0)
#define P1_4_SetPushPull()        do { ODCONBbits.ODCB0 = 0; } while(0)
#define P1_4_SetOpenDrain()       do { ODCONBbits.ODCB0 = 1; } while(0)
#define P1_4_SetAnalogMode()      do { ANSELBbits.ANSELB0 = 1; } while(0)
#define P1_4_SetDigitalMode()     do { ANSELBbits.ANSELB0 = 0; } while(0)

// get/set P1_3 aliases
#define P1_3_TRIS                 TRISBbits.TRISB1
#define P1_3_LAT                  LATBbits.LATB1
#define P1_3_PORT                 PORTBbits.RB1
#define P1_3_WPU                  WPUBbits.WPUB1
#define P1_3_OD                   ODCONBbits.ODCB1
#define P1_3_ANS                  ANSELBbits.ANSELB1
#define P1_3_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define P1_3_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define P1_3_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define P1_3_GetValue()           PORTBbits.RB1
#define P1_3_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define P1_3_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define P1_3_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define P1_3_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define P1_3_SetPushPull()        do { ODCONBbits.ODCB1 = 0; } while(0)
#define P1_3_SetOpenDrain()       do { ODCONBbits.ODCB1 = 1; } while(0)
#define P1_3_SetAnalogMode()      do { ANSELBbits.ANSELB1 = 1; } while(0)
#define P1_3_SetDigitalMode()     do { ANSELBbits.ANSELB1 = 0; } while(0)

// get/set P1_2 aliases
#define P1_2_TRIS                 TRISBbits.TRISB2
#define P1_2_LAT                  LATBbits.LATB2
#define P1_2_PORT                 PORTBbits.RB2
#define P1_2_WPU                  WPUBbits.WPUB2
#define P1_2_OD                   ODCONBbits.ODCB2
#define P1_2_ANS                  ANSELBbits.ANSELB2
#define P1_2_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define P1_2_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define P1_2_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define P1_2_GetValue()           PORTBbits.RB2
#define P1_2_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define P1_2_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define P1_2_SetPullup()          do { WPUBbits.WPUB2 = 1; } while(0)
#define P1_2_ResetPullup()        do { WPUBbits.WPUB2 = 0; } while(0)
#define P1_2_SetPushPull()        do { ODCONBbits.ODCB2 = 0; } while(0)
#define P1_2_SetOpenDrain()       do { ODCONBbits.ODCB2 = 1; } while(0)
#define P1_2_SetAnalogMode()      do { ANSELBbits.ANSELB2 = 1; } while(0)
#define P1_2_SetDigitalMode()     do { ANSELBbits.ANSELB2 = 0; } while(0)

// get/set P1_1 aliases
#define P1_1_TRIS                 TRISBbits.TRISB3
#define P1_1_LAT                  LATBbits.LATB3
#define P1_1_PORT                 PORTBbits.RB3
#define P1_1_WPU                  WPUBbits.WPUB3
#define P1_1_OD                   ODCONBbits.ODCB3
#define P1_1_ANS                  ANSELBbits.ANSELB3
#define P1_1_SetHigh()            do { LATBbits.LATB3 = 1; } while(0)
#define P1_1_SetLow()             do { LATBbits.LATB3 = 0; } while(0)
#define P1_1_Toggle()             do { LATBbits.LATB3 = ~LATBbits.LATB3; } while(0)
#define P1_1_GetValue()           PORTBbits.RB3
#define P1_1_SetDigitalInput()    do { TRISBbits.TRISB3 = 1; } while(0)
#define P1_1_SetDigitalOutput()   do { TRISBbits.TRISB3 = 0; } while(0)
#define P1_1_SetPullup()          do { WPUBbits.WPUB3 = 1; } while(0)
#define P1_1_ResetPullup()        do { WPUBbits.WPUB3 = 0; } while(0)
#define P1_1_SetPushPull()        do { ODCONBbits.ODCB3 = 0; } while(0)
#define P1_1_SetOpenDrain()       do { ODCONBbits.ODCB3 = 1; } while(0)
#define P1_1_SetAnalogMode()      do { ANSELBbits.ANSELB3 = 1; } while(0)
#define P1_1_SetDigitalMode()     do { ANSELBbits.ANSELB3 = 0; } while(0)

// get/set P1_0 aliases
#define P1_0_TRIS                 TRISBbits.TRISB4
#define P1_0_LAT                  LATBbits.LATB4
#define P1_0_PORT                 PORTBbits.RB4
#define P1_0_WPU                  WPUBbits.WPUB4
#define P1_0_OD                   ODCONBbits.ODCB4
#define P1_0_ANS                  ANSELBbits.ANSELB4
#define P1_0_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define P1_0_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define P1_0_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define P1_0_GetValue()           PORTBbits.RB4
#define P1_0_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define P1_0_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define P1_0_SetPullup()          do { WPUBbits.WPUB4 = 1; } while(0)
#define P1_0_ResetPullup()        do { WPUBbits.WPUB4 = 0; } while(0)
#define P1_0_SetPushPull()        do { ODCONBbits.ODCB4 = 0; } while(0)
#define P1_0_SetOpenDrain()       do { ODCONBbits.ODCB4 = 1; } while(0)
#define P1_0_SetAnalogMode()      do { ANSELBbits.ANSELB4 = 1; } while(0)
#define P1_0_SetDigitalMode()     do { ANSELBbits.ANSELB4 = 0; } while(0)

// get/set RB5 procedures
#define RB5_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define RB5_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define RB5_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define RB5_GetValue()              PORTBbits.RB5
#define RB5_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define RB5_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define RB5_SetPullup()             do { WPUBbits.WPUB5 = 1; } while(0)
#define RB5_ResetPullup()           do { WPUBbits.WPUB5 = 0; } while(0)
#define RB5_SetAnalogMode()         do { ANSELBbits.ANSELB5 = 1; } while(0)
#define RB5_SetDigitalMode()        do { ANSELBbits.ANSELB5 = 0; } while(0)

// get/set RB6 procedures
#define RB6_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define RB6_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define RB6_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define RB6_GetValue()              PORTBbits.RB6
#define RB6_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define RB6_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define RB6_SetPullup()             do { WPUBbits.WPUB6 = 1; } while(0)
#define RB6_ResetPullup()           do { WPUBbits.WPUB6 = 0; } while(0)
#define RB6_SetAnalogMode()         do { ANSELBbits.ANSELB6 = 1; } while(0)
#define RB6_SetDigitalMode()        do { ANSELBbits.ANSELB6 = 0; } while(0)

// get/set RB7 procedures
#define RB7_SetHigh()            do { LATBbits.LATB7 = 1; } while(0)
#define RB7_SetLow()             do { LATBbits.LATB7 = 0; } while(0)
#define RB7_Toggle()             do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define RB7_GetValue()              PORTBbits.RB7
#define RB7_SetDigitalInput()    do { TRISBbits.TRISB7 = 1; } while(0)
#define RB7_SetDigitalOutput()   do { TRISBbits.TRISB7 = 0; } while(0)
#define RB7_SetPullup()             do { WPUBbits.WPUB7 = 1; } while(0)
#define RB7_ResetPullup()           do { WPUBbits.WPUB7 = 0; } while(0)
#define RB7_SetAnalogMode()         do { ANSELBbits.ANSELB7 = 1; } while(0)
#define RB7_SetDigitalMode()        do { ANSELBbits.ANSELB7 = 0; } while(0)

// get/set P0_5 aliases
#define P0_5_TRIS                 TRISCbits.TRISC0
#define P0_5_LAT                  LATCbits.LATC0
#define P0_5_PORT                 PORTCbits.RC0
#define P0_5_WPU                  WPUCbits.WPUC0
#define P0_5_OD                   ODCONCbits.ODCC0
#define P0_5_ANS                  ANSELCbits.ANSELC0
#define P0_5_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define P0_5_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define P0_5_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define P0_5_GetValue()           PORTCbits.RC0
#define P0_5_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define P0_5_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define P0_5_SetPullup()          do { WPUCbits.WPUC0 = 1; } while(0)
#define P0_5_ResetPullup()        do { WPUCbits.WPUC0 = 0; } while(0)
#define P0_5_SetPushPull()        do { ODCONCbits.ODCC0 = 0; } while(0)
#define P0_5_SetOpenDrain()       do { ODCONCbits.ODCC0 = 1; } while(0)
#define P0_5_SetAnalogMode()      do { ANSELCbits.ANSELC0 = 1; } while(0)
#define P0_5_SetDigitalMode()     do { ANSELCbits.ANSELC0 = 0; } while(0)

// get/set P0_6 aliases
#define P0_6_TRIS                 TRISCbits.TRISC1
#define P0_6_LAT                  LATCbits.LATC1
#define P0_6_PORT                 PORTCbits.RC1
#define P0_6_WPU                  WPUCbits.WPUC1
#define P0_6_OD                   ODCONCbits.ODCC1
#define P0_6_ANS                  ANSELCbits.ANSELC1
#define P0_6_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define P0_6_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define P0_6_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define P0_6_GetValue()           PORTCbits.RC1
#define P0_6_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define P0_6_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define P0_6_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define P0_6_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define P0_6_SetPushPull()        do { ODCONCbits.ODCC1 = 0; } while(0)
#define P0_6_SetOpenDrain()       do { ODCONCbits.ODCC1 = 1; } while(0)
#define P0_6_SetAnalogMode()      do { ANSELCbits.ANSELC1 = 1; } while(0)
#define P0_6_SetDigitalMode()     do { ANSELCbits.ANSELC1 = 0; } while(0)

// get/set P0_7 aliases
#define P0_7_TRIS                 TRISCbits.TRISC2
#define P0_7_LAT                  LATCbits.LATC2
#define P0_7_PORT                 PORTCbits.RC2
#define P0_7_WPU                  WPUCbits.WPUC2
#define P0_7_OD                   ODCONCbits.ODCC2
#define P0_7_ANS                  ANSELCbits.ANSELC2
#define P0_7_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define P0_7_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define P0_7_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define P0_7_GetValue()           PORTCbits.RC2
#define P0_7_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define P0_7_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define P0_7_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define P0_7_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define P0_7_SetPushPull()        do { ODCONCbits.ODCC2 = 0; } while(0)
#define P0_7_SetOpenDrain()       do { ODCONCbits.ODCC2 = 1; } while(0)
#define P0_7_SetAnalogMode()      do { ANSELCbits.ANSELC2 = 1; } while(0)
#define P0_7_SetDigitalMode()     do { ANSELCbits.ANSELC2 = 0; } while(0)

// get/set RC3 procedures
#define RC3_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define RC3_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define RC3_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define RC3_GetValue()              PORTCbits.RC3
#define RC3_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define RC3_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define RC3_SetPullup()             do { WPUCbits.WPUC3 = 1; } while(0)
#define RC3_ResetPullup()           do { WPUCbits.WPUC3 = 0; } while(0)
#define RC3_SetAnalogMode()         do { ANSELCbits.ANSELC3 = 1; } while(0)
#define RC3_SetDigitalMode()        do { ANSELCbits.ANSELC3 = 0; } while(0)

// get/set RC4 procedures
#define RC4_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define RC4_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define RC4_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define RC4_GetValue()              PORTCbits.RC4
#define RC4_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define RC4_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define RC4_SetPullup()             do { WPUCbits.WPUC4 = 1; } while(0)
#define RC4_ResetPullup()           do { WPUCbits.WPUC4 = 0; } while(0)
#define RC4_SetAnalogMode()         do { ANSELCbits.ANSELC4 = 1; } while(0)
#define RC4_SetDigitalMode()        do { ANSELCbits.ANSELC4 = 0; } while(0)

// get/set P1_7 aliases
#define P1_7_TRIS                 TRISCbits.TRISC5
#define P1_7_LAT                  LATCbits.LATC5
#define P1_7_PORT                 PORTCbits.RC5
#define P1_7_WPU                  WPUCbits.WPUC5
#define P1_7_OD                   ODCONCbits.ODCC5
#define P1_7_ANS                  ANSELCbits.ANSELC5
#define P1_7_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define P1_7_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define P1_7_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define P1_7_GetValue()           PORTCbits.RC5
#define P1_7_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define P1_7_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define P1_7_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define P1_7_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define P1_7_SetPushPull()        do { ODCONCbits.ODCC5 = 0; } while(0)
#define P1_7_SetOpenDrain()       do { ODCONCbits.ODCC5 = 1; } while(0)
#define P1_7_SetAnalogMode()      do { ANSELCbits.ANSELC5 = 1; } while(0)
#define P1_7_SetDigitalMode()     do { ANSELCbits.ANSELC5 = 0; } while(0)

// get/set P1_6 aliases
#define P1_6_TRIS                 TRISCbits.TRISC6
#define P1_6_LAT                  LATCbits.LATC6
#define P1_6_PORT                 PORTCbits.RC6
#define P1_6_WPU                  WPUCbits.WPUC6
#define P1_6_OD                   ODCONCbits.ODCC6
#define P1_6_ANS                  ANSELCbits.ANSELC6
#define P1_6_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define P1_6_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define P1_6_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define P1_6_GetValue()           PORTCbits.RC6
#define P1_6_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define P1_6_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define P1_6_SetPullup()          do { WPUCbits.WPUC6 = 1; } while(0)
#define P1_6_ResetPullup()        do { WPUCbits.WPUC6 = 0; } while(0)
#define P1_6_SetPushPull()        do { ODCONCbits.ODCC6 = 0; } while(0)
#define P1_6_SetOpenDrain()       do { ODCONCbits.ODCC6 = 1; } while(0)
#define P1_6_SetAnalogMode()      do { ANSELCbits.ANSELC6 = 1; } while(0)
#define P1_6_SetDigitalMode()     do { ANSELCbits.ANSELC6 = 0; } while(0)

// get/set P1_5 aliases
#define P1_5_TRIS                 TRISCbits.TRISC7
#define P1_5_LAT                  LATCbits.LATC7
#define P1_5_PORT                 PORTCbits.RC7
#define P1_5_WPU                  WPUCbits.WPUC7
#define P1_5_OD                   ODCONCbits.ODCC7
#define P1_5_ANS                  ANSELCbits.ANSELC7
#define P1_5_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define P1_5_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define P1_5_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define P1_5_GetValue()           PORTCbits.RC7
#define P1_5_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define P1_5_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define P1_5_SetPullup()          do { WPUCbits.WPUC7 = 1; } while(0)
#define P1_5_ResetPullup()        do { WPUCbits.WPUC7 = 0; } while(0)
#define P1_5_SetPushPull()        do { ODCONCbits.ODCC7 = 0; } while(0)
#define P1_5_SetOpenDrain()       do { ODCONCbits.ODCC7 = 1; } while(0)
#define P1_5_SetAnalogMode()      do { ANSELCbits.ANSELC7 = 1; } while(0)
#define P1_5_SetDigitalMode()     do { ANSELCbits.ANSELC7 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/