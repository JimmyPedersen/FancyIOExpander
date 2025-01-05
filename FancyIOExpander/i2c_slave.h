/* 
 * File:   i2c_slave.h
 * Author: JimmyPedersen
 *
 * Created on den 20 november 2024, 09:19
 */

#ifndef I2C_SLAVE_H
    #define	I2C_SLAVE_H

    #ifdef	__cplusplus
    extern "C" {
    #endif
        #include <stdbool.h>
        #include <stdint.h>
        
        
        typedef enum
        {
            eIdle,
            eAddrH,
            eAddrL,
            eData
        } I2C_State_t;

        
        typedef bool (*i2c_write_to_slave_t)(volatile uint16_t *addr, uint8_t rb);
        typedef bool (*i2c_read_from_slave_t)(volatile uint16_t *addr, uint8_t *wb);
                
        I2C_State_t i2c_slave_get_state(void);
        uint8_t i2c_slave_get_address(void);
        uint8_t i2c_slave_get_byte_no(void);
        void i2c_slave_init(uint8_t slaveAddr, uint8_t maskAddr, i2c_write_to_slave_t to_slave_func, i2c_read_from_slave_t from_slave_func);
   
    #ifdef	__cplusplus
    }
    #endif

#endif	/* I2C_SLAVE_H */

