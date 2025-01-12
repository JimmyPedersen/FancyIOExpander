#include "pwm.h"
#include "mcc_generated_files/mcc.h"
#include "config.h"

/**
 * @brief Configure PWM/CCP output on a specified pin
 * @param port: Port number (0 for PORTA, 1 for PORTB, 2 for PORTC)
 * @param pin: Pin number (0-7)
 * @param module: Module number (1-2 for CCP1-2, 3-8 for PWM1-6, 0 to disable)
 * @return bool: true if successful, false if invalid parameters
 */
bool pwm_configure_pin(uint8_t port, uint8_t pin, uint8_t module) {
    // Validate parameters
    if (pin > 7 || port > 2 || module > 4) {
        return false;
    }

    const Pin_t *phyPin = &PINS[port][pin];
    
    // Base address for PPS registers - points to RA0PPS
    volatile uint8_t* pps_reg = (volatile uint8_t*)&RA0PPS;
    
    // Add offset for the specific PPS register
    pps_reg += (phyPin->Port * 8) + phyPin->Pin;
    
    // Set specific pin as digital output while preserving other pins
    volatile uint8_t* tris_reg = (volatile uint8_t*)&TRISA + phyPin->Port;
    *tris_reg &= ~(1 << phyPin->Pin);
    
    // If module is 0, disable output on the pin
    if (module == 0) {
        *pps_reg = 0x00;
        return true;
    }
    
    // Configure module and set PPS value based on module number
    if (module <= 2) {  // CCP modules
        if(phyPin->Port == ePORTA)
            return false;
        
        // Configure CCP module
        volatile uint8_t* ccp_con = (volatile uint8_t*)&CCP1CON + ((module - 1) * 4);
        *ccp_con = 0x0C;  // PWM mode
        // Map CCP to pin (CCP1 = 0x05, CCP2 = 0x06)
        *pps_reg = 0x05 + (module - 1);
    }
    else {
        if(phyPin->Port == ePORTB)
            return false;
        
        // Configure PWM module
        volatile uint8_t* pwm_con = (volatile uint8_t*)&PWM3CON + ((module-3) * 4);
        *pwm_con = 0x80;  // Enable PWM
        // Map PWM to pin (PWM3 = 0x07, PWM4 = 0x08, etc.)
        *pps_reg = 0x07 + (module-3);
    }
    
    return true;
}

/**
 * @brief Set PWM/CCP duty cycle
 * @param module: Module number (1-2 for CCP1-2, 3-8 for PWM1-6)
 * @param duty_cycle: Duty cycle value (0-1023 for 0-100.0%)
 * @return bool: true if successful, false if invalid parameters
 */
bool pwm_set_duty_cycle(uint8_t module, uint16_t duty_cycle) {
    // Validate parameters
    if (duty_cycle > 1023) {
        
    }
    
    switch(module)
    {
        case 1:
            PWM1_LoadDutyValue(duty_cycle);
            break;
            
        case 2:
            PWM2_LoadDutyValue(duty_cycle);
            break;
            
        case 3:
            PWM3_LoadDutyValue(duty_cycle);
            break;
            
        case 4:
            PWM4_LoadDutyValue(duty_cycle);
            break;
            
        default:
            return false;
            break;
    }    
    return true;
}