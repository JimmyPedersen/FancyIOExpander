/* 
 * File:   pwm.h
 * Author: Jimmy
 *
 * Created on January 12, 2025, 3:59 PM
 */
#pragma once

#ifdef	__cplusplus
extern "C" {
#endif
    #include <stdint.h>
    #include <stdbool.h>
    
    bool pwm_configure_pin(uint8_t port, uint8_t pin, uint8_t pwm_module);
    bool pwm_set_duty_cycle(uint8_t pwm_module, uint16_t duty_cycle);
#ifdef	__cplusplus
}
#endif


