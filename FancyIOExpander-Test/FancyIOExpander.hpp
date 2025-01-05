#ifndef __FANCY_IO_EXP_H__
#define __FANCY_IO_EXP_H__

//#include <cstdint>
#include <Wire.h>

class FancyIOExp {
public:

    /**
     * Default constructor
     */
    FancyIOExp() : chipAddr_(0) {}

    /**
     * Constructor with chip address
     * @param chipAddr Device address
     */
    explicit FancyIOExp(uint8_t chipAddr) : chipAddr_(chipAddr) {}


    /**
     * Set the chip address
     * @param chipAddr New device address
     */
    void setChipAddress(uint8_t chipAddr) {
        chipAddr_ = chipAddr;
    }

    /**
     * Get the current chip address
     * @return Current device address
     */
    uint8_t getChipAddress() const {
        return chipAddr_;
    }

    /**
     * Write data to expansion board memory
     * @param addr Memory address
     * @param length Number of bytes to write
     * @param data Pointer to data buffer
     */
    void writeMemory(uint16_t addr, uint8_t length, uint8_t *data) {
        xprintf("WR: C:%u, Addr:0x%04X Len:%u Data: ", chipAddr_, addr, length);
        for(int i = 0; i < length; i++) {
            xprintf("%s0x%02X%s", (i == 0 ? "" : ", "), data[i], ((i+1) == length ? "\r\n" : ""));
        }

        Wire.beginTransmission(chipAddr_);
        Wire.write(static_cast<uint8_t>(addr >> 8));
        Wire.write(static_cast<uint8_t>(addr & 255));
        
        while(length--) {
            Wire.write(*data++);
        }

        Wire.endTransmission();
        delay(1);
    }

    /**
     * Write single byte to EEPROM memory
     * @param EEAddr Memory address
     * @param data Byte to write
     */
    void writeEEPROM(uint16_t EEAddr, uint8_t data) {
        writeMemory(EEAddr, 1, &data);
    }

    /**
     * Write byte to EEPROM memory
     * @param EEAddr Memory address
	   * @param length Number of bytes to write
     * @param data Pointer to data buffer
     * @param data Byte to write
     */
    void writeEEPROM(uint16_t EEAddr, uint8_t length, uint8_t *data) {
        writeMemory(EEAddr, length, data);
    }


    /**
     * Write data to expansion board registers
     * @param addr Register address
     * @param length Number of bytes to write
     * @param data Pointer to data buffer
     */
    void writeRegisters(uint16_t addr, uint8_t length, uint8_t *data) {
        writeMemory(addr | 0x8000, length, data);
    }

    /**
     * Write single byte to expansion board register
     * @param addr Register address
     * @param data Byte to write
     */
    void writeRegister(uint16_t addr, uint8_t data) {
        writeMemory(addr | 0x8000, 1, &data);
    }

    /**
     * Read data from expansion board memory
     * @param addr Memory address
     * @param length Number of bytes to read
     * @param data Pointer to data buffer
     * @return Number of bytes actually read
     */
    uint8_t readMemory(uint16_t addr, uint8_t length, uint8_t *data) {
        if(!length) {
            return 0;
        }

        xprintf("RD: C:%u, Addr:0x%04X Len:%u Data: ", chipAddr_, addr, length);

        Wire.beginTransmission(chipAddr_);
        Wire.write(static_cast<uint8_t>(addr >> 8));
        Wire.write(static_cast<uint8_t>(addr & 255));
        Wire.endTransmission();
        delay(1);

        Wire.requestFrom(chipAddr_, length);
        delay(1);

        uint8_t read = 0;
        uint8_t *pDest = data;
        uint8_t left = length;
        
        while(left--) {
            if (!Wire.available()) {
                break;
            }
            *pDest++ = Wire.read();
            read++;
        }

        for(int i = 0; i < read; i++) {
            xprintf("%s0x%02X%s", (i == 0 ? "" : ", "), data[i], ((i+1) == length ? "\n" : ""));
        }

        if(read < length) {
            xprintf("Failed to read all! (%u/%u)\n", read, length);
        }

        return read;
    }

    /**
     * Read single byte from expansion board memory
     * @param EEAddr Memory address
     * @return Read byte value
     */
    uint8_t readEEPROM(uint16_t EEAddr) {
        uint8_t tmp;
        lastReadOk_ = (readMemory(EEAddr, 1, &tmp) == 1);
        return tmp;
    }

    /**
     * Read single byte from expansion board memory
     * @param EEAddr Memory address
	 * @param length Number of bytes to read
     * @param data Pointer to data buffer
     * @return Read byte value
     */
    uint8_t readEEPROM(uint16_t EEAddr, uint8_t length, uint8_t *data) {
       return readMemory(EEAddr, length, data);
    }
	
    /**
     * Read data from expansion board register
     * @param regAddr Register address
     * @param length Number of bytes to read
     * @param data Pointer to data buffer
     * @return Number of bytes actually read
     */
    uint8_t readRegisters(uint16_t regAddr, uint8_t length, uint8_t *data) {
        return readMemory(regAddr | 0x8000, length, data);
    }

    /**
     * Read single byte from expansion board register
     * @param regAddr Register address
     * @return Read register byte value
     */
    uint8_t readRegister(uint16_t regAddr) {
        uint8_t tmp;
        readMemory(regAddr | 0x8000, 1 , &tmp);
        return tmp;
    }

    /**
     * Read ADC value from expansion board
     * @param adcAddr ADC address
     * @return 16-bit ADC value
     */
    uint16_t readADC(uint8_t adcAddr) {
        uint8_t adc_tmp[2] = {0, 0};
        readMemory(0x8020 | ((uint16_t)adcAddr<<8), 2, adc_tmp);
        xprintf("LB:0x%02X\n", adc_tmp[0]);
        xprintf("HB:0x%02X\n", adc_tmp[1]);
        return static_cast<uint16_t>(adc_tmp[1]) << 8 | adc_tmp[0];
    }

    /**
     * Read multiple ADC values from expansion board
     * @param start First ADC to read
     * @param adcs Number of ADCs to read
     * @param dest Pointer to destination buffer for ADC values
     * @return True if all ADCs were read successfully
     */
    bool readADCs(uint8_t start, uint8_t adcs, uint16_t *dest) {
        return readMemory(0x8020 | ((uint16_t)start<<8) , adcs * 2, reinterpret_cast<uint8_t*>(dest)) == 1;
    }

    /**
     * Check if the last read operation was successful
     * @return True if last read succeeded, false otherwise
     */
    bool isLastReadSuccessful() const { 
        return lastReadOk_; 
    }

private:
    /**
     * Printf function
     * @param format printf format
     */
    void xprintf(const char *format, ...)
    {
      char buffer[256];  // or smaller or static &c.
      va_list args;
      va_start(args, format);
      vsprintf(buffer, format, args);
      va_end(args);
      Serial.print(buffer);
    }

    uint8_t chipAddr_;     // Stored chip address
    bool lastReadOk_ = true;  // Status of the last read operation
};

#endif // __FANCY_IO_EXP_H__