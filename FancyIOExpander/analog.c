#include "analog.h"
#include <assert.h>


static struct
{
    eAnalogPosRef_t ref[EXP_PINS_COUNT];
    uint16_t adc_val[EXP_PINS_COUNT];        
    uint8_t ch_sampled;
}analog;


void analog_select_pos_ref_idx(uint8_t idx, eAnalogPosRef_t ref)
{
    assert(idx < EXP_PINS_COUNT);
    analog.ref[idx] = ref;
}

void analog_select_pos_ref(EXP_PORT port, EXP_PIN pin, eAnalogPosRef_t ref)
{
    analog_select_pos_ref_idx(IDX_FROM_PORT_PIN(port, pin), ref);
}

uint16_t analog_get_adc_val_idx(uint8_t idx)
{
    assert(idx < EXP_PINS_COUNT);
    return analog.adc_val[idx];
}

uint16_t analog_get_adc_val(EXP_PORT port, EXP_PIN pin)
{
    return analog_get_adc_val_idx(IDX_FROM_PORT_PIN(port, pin));
}


void analog_set_pos_ref(eAnalogPosRef_t ref)
{
    // Get current pos. ref.
    eAnalogPosRef_t current = (eAnalogPosRef_t)(FVRCON & 3);
    
    // Has the ref. changed
    if(ref != current)
    {
        // Set pos. ref.
        FVRCON = (FVRCON & 0xFC) | ref;
        
        // If we want to use VDD as pos. ref. change ADC Ref.
        if((ref & 1) != (current & 1))
            ADREF = (ADREF & 0xFC) | ((ref==eARef_VDD) ? 0 : 3);

        // Wait for FVR to be ready
        while(!FVR_IsOutputReady());
    }
}

void analog_init(void)
{
    // Set all ADC to 2.048 mv ref. at startup
    for(uint8_t i=0; i < EXP_PINS_COUNT; i++)
        analog_select_pos_ref_idx(i, eARef_2048mV);
    
    // Signal we're not ready yet
    analog.ch_sampled = 255;
}


void analog_handler(void)
{
    if(analog.ch_sampled > EXP_PINS_COUNT || ADCC_IsConversionDone())
    {
        // If there was actual data sampled save it now
        if(analog.ch_sampled < EXP_PINS_COUNT && ADCC_IsConversionDone())
            analog.adc_val[(uint8_t)analog.ch_sampled] = ADCC_GetConversionResult();
        
        // Advance to next pin
        if(++analog.ch_sampled >= EXP_PINS_COUNT)
            analog.ch_sampled = 0;

        // Set chosen positive ref.
        analog_set_pos_ref(analog.ref[analog.ch_sampled]);
        
        // Calculate port & pin to get ADC channel
        uint8_t port = EXP_PORT_FROM_IDX(analog.ch_sampled);
        uint8_t pin = EXP_PIN_FROM_IDX(analog.ch_sampled);
        
        // Start ADC conversion
        ADCC_StartConversion( PINS[port][pin].AnalogCh );   
    }
}