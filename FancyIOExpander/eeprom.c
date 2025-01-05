#include "config.h"
#include "mcc_generated_files/mcc.h"
#include "eeprom.h"
#include "i2c_slave.h"
#include "debug.h"

static volatile uint16_t i2c1EEPMemAddr = 0x00;
static volatile uint8_t E2_DATA[SLAVE_EEPROM_SIZE];
static volatile uint8_t E2_Changes[SLAVE_EEPROM_SIZE/8];


static uint8_t eeprom_read_byte(eeAddr_t addr)
{
    return DATAEE_ReadByte(addr);
}

static void eeprom_write_byte(eeAddr_t addr, uint8_t data)
{
//    dbg_printf("W EE #%u: %u (%u)\r\n", eeAddr, data, E2_DATA[eeAddr]);
    DATAEE_WriteByte(addr, data);
    E2_Changes[addr >> 3] &= ~(1 << (addr & 0x7)); // Signal EEPROM uptodate
}

uint8_t eeprom_read_buffer(eeAddr_t addr)
{
    return E2_DATA[addr];
}

void eeprom_write_buffer(eeAddr_t addr, uint8_t data)
{
    E2_DATA[addr] = data;                       // Update buffer
    E2_Changes[addr >> 3] |= 1 << (addr & 0x7); // Signal EEPROM change
}


void eeprom_buffer_whole(void)
{
    // Read whole EEPROM    
    for(uint16_t b = 0; b < SLAVE_EEPROM_SIZE; b++)
        E2_DATA[b] = eeprom_read_byte((eeAddr_t)b);
}

void eeprom_handle_writing(void)
{
    // Exit if we're busy with I2C commands
    // as EEPROM writes (temporarily) blocks interrupts
    if(i2c_slave_get_state() != eIdle)
        return;
            
    // Iterate over byte flags
    for(uint8_t by = 0;by < (SLAVE_EEPROM_SIZE / 8); by++)
    {
        // Check if there are any changes on these 8 bytes, if not skip
        // to next byte flag
        if(!E2_Changes[by])
            continue;

        // Check each byte in the byte flag
        for(uint8_t bi = 0; E2_Changes[by] && bi < 8; bi++)
        {
            // Exit if we're busy with I2C commands
            // as EEPROM writes (temporarily) blocks interrupts
            if(i2c_slave_get_state() != eIdle)
                return;

            // Calculate bit-mask
            uint8_t bitval = (uint8_t)(1 << bi);

            // Check if a byte has been flagged as changed
            if(E2_Changes[by] & bitval) 
            {
                // Calculate EEPROM address
                eeAddr_t eeAddr = (eeAddr_t)(((uint16_t)by << 3) + bi);

                dbg_analog(31);
                eeprom_write_byte((eeAddr_t)eeAddr, E2_DATA[eeAddr]);
                dbg_analog(0);
            }
        }
    }
}