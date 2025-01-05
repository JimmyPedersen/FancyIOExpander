/* 
 * File:   debug.h
 * Author: JimmyPedersen
 *
 * Created on den 20 november 2024, 09:15
 */

#ifndef DEBUG_H
#define	DEBUG_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    #include "mcc_generated_files/mcc.h"
    #include <stdio.h>
    #include <stdarg.h>
    #include <assert.h>
    
    
    //Enable/Disable debugging methods
    #define DBG_USE_ANALOG_OUT  1
    #define DBG_USE_PRINTF      1


    #if DBG_USE_ANALOG_OUT > 0
        #define dbg_analog(a) do{ DAC1CON1 = a; }while(0)  // DAC1_SetOutput(a);
    #else
        #define dbg_analog(a) do{}while(0)
    #endif

    #if DBG_USE_PRINTF > 0
        #define dbg_printf(...) do{ printf(__VA_ARGS__); }while(0)
    #else
        #define dbg_printf(fmt, ...) do{}while(0)
    #endif

#ifdef	__cplusplus
}
#endif

#endif	/* DEBUG_H */

