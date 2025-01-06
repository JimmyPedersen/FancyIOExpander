// --------------------------------------
// FancyIOExpander tester
// --------------------------------------
#include <Wire.h>
#include "FancyIOExpander.hpp"

FancyIOExp IOExp;
using Regs = FancyIOExp::Registers;
using Ports = FancyIOExp::Ports;

//bool exp_read_ok = true;
 
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

void setup()
{
  Wire.begin();
 
  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}


void loop()
{
  byte error, chipAddr;
  static byte val = 0;
  int nDevices;
 
//  Serial.println("Scanning...");
 
  nDevices = 0;
  //for(chipAddr = 1; chipAddr < 127; chipAddr++ )
  chipAddr=0x2A;
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(chipAddr);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.println("");
      Serial.print("I2C device found at address 0x");
      if (chipAddr<16)
        Serial.print("0");
      Serial.print(chipAddr, HEX);
      Serial.println("  !");
 
      IOExp.setChipAddress(chipAddr);

      if(chipAddr==0x2A)
      {
 //       delay(1000);

        val += 1;

        uint8_t rb[3] = {0,0,0};
        uint8_t wb[3] = {val,(uint8_t)(val+1),(uint8_t)(val+2)};

        static unsigned char Toggle = 0x55;
        uint16_t EEaddress = 0x0005;


        // Read EEPROM contents
        uint8_t read = IOExp.readEEPROM(5, 3, rb);
        xprintf("Read %u bytes!\n", read);

        // Write EEPROM contents
        IOExp.writeEEPROM(5, 3, wb);

        // Read EEPROM contents
        read = IOExp.readEEPROM(5, 3, rb);
        xprintf("Read %u bytes!\n", read);

        if(wb[0] == rb[0] && wb[1] == rb[1] && wb[2] == rb[2])
          Serial.println("Read == Write");
        else
          Serial.println("Read != Write");  


        // PORT0 - Write TRIS
//        IOExp.writeRegister(2, 0); // 0 = All outputs
        IOExp.writeRegister(Regs::TRIS, Ports::Port0, 0x00); // 0 = All outputs

        // PORT0 - Write LAT
//        IOExp.writeRegister(0, Toggle);  // Set all outputs
        IOExp.writeRegister(Regs::LAT, Ports::Port0, Toggle); // Set all outputs

        // Toggle data outputed     
        Toggle = ~Toggle; 



        // PORT1 - TRIS all input
//        IOExp.writeRegister(0x1002, 0xFF);// 0xFF = Set all as inputs
        IOExp.writeRegister(IOExp.TRIS, IOExp.Port1, 0xff);// 0xFF = Set all as inputs

        // PORT1 - WPU all input 
//        IOExp.writeRegister( 0x1006, 0x7F);// 0x7F = Activate all pullups but on the P1.7 pin
        IOExp.writeRegister(IOExp.WPU, IOExp.Port1, 0x7f);// 0x7F = Activate all pullups but on the P1.7 pin

        // PORT1 - ANSEL P1.7
//        IOExp.writeRegister(0x1007, 0x80);// Set P1.7 pin as analog
        IOExp.writeRegister(IOExp.ANSEL, IOExp.Port1, 0x80);// Set P1.7 pin as analog

        // PORT1 - read input
//        read = IOExp.readRegister(0x1001);
        read = IOExp.readRegister(Regs::PORT, Ports::Port1);
        xprintf("Read Port 1: %u  (Inv:%u) Read OK:%u\n", read, 255-read, IOExp.isLastReadSuccessful());


        // PORT1 - read adc P1.7
        uint16_t adc_val = IOExp.readADC(0x17);
        xprintf("P1.7 ADC value:%u\n", adc_val);

        #define NO_OF_ADCS_TO_READ  16
        uint16_t adc_tmp[NO_OF_ADCS_TO_READ] = {0};
        bool read_res = IOExp.readADCs(0, NO_OF_ADCS_TO_READ, adc_tmp);
        xprintf("Read %u ADCs(Res:%u): ", NO_OF_ADCS_TO_READ, read_res);
        for(uint8_t i = 0; i < NO_OF_ADCS_TO_READ; i++)
           xprintf("#%u:%u ", i, adc_tmp[i]);
        Serial.println("");
      }
      nDevices++;
      Serial.println("");
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (chipAddr<16)
        Serial.print("0");
      Serial.println(chipAddr,HEX);
      Serial.println("");
    }    
    
  }
  val++;
  if (nDevices == 0)
  {
//    Serial.println("No I2C devices found");
  }  
  else
  {
    Serial.print("# of devices found:");
    Serial.println(nDevices);
    Serial.println("");
  }
  Serial.print(".");

  delay(1000);           // wait a couple seconds for next scan
}