/* 
 * File:   analog.h
 * Author: Jimmy
 *
 * Created on den 24 november 2024, 09:40
 */

#ifndef ANALOG_H
#define	ANALOG_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include "mcc_generated_files/mcc.h"
    #include "config.h"
    
    typedef enum
    {
        eARef_VDD = 0,
        eARef_1024mV,   
        eARef_2048mV,
        eARef_4096mV,
    }eAnalogPosRef_t;
    
    void analog_select_pos_ref_idx(uint8_t idx, eAnalogPosRef_t ref);
    void analog_select_pos_ref(EXP_PORT port, EXP_PIN pin, eAnalogPosRef_t ref);
    uint16_t analog_get_adc_val_idx(uint8_t idx);
    uint16_t analog_get_adc_val(EXP_PORT port, EXP_PIN pin);
    void analog_init(void);
    void analog_handler(void);

#ifdef	__cplusplus
}
#endif

#endif	/* ANALOG_H */

