/* 
 * File:   eeprom.h
 * Author: JimmyPedersen
 *
 * Created on den 20 november 2024, 09:09
 */

#ifndef EEPROM_H
#define	EEPROM_H

    #ifdef	__cplusplus
    extern "C" {
    #endif
    
        #include <stdbool.h>
        #include <stdint.h>

        #define SLAVE_EEPROM_SIZE           256

        #if SLAVE_EEPROM_SIZE<=256
            typedef uint8_t eeAddr_t;
        #else
            typedef uint16_t eeAddr_t;    
        #endif

        void eeprom_buffer_whole(void);
        void eeprom_handle_writing(void);
        
//        uint8_t eeprom_read_byte(eeAddr_t addr);
//        void eeprom_write_byte(eeAddr_t addr, uint8_t data);
        
        uint8_t eeprom_read_buffer(eeAddr_t addr);
        void eeprom_write_buffer(eeAddr_t addr, uint8_t data);
    #ifdef	__cplusplus
    }
    #endif

#endif	/* EEPROM_H */

