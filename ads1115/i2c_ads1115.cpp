#include <iostream>
// #include "mraa.h"
#include "mraa/i2c.hpp"
#include "mraa/initio.hpp"

using namespace std;

uint16_t swapWord(uint16_t value) {
    uint16_t res = value;
    return ((res & 0xFF) << 8) | ((res >> 8) & 0xFF);
}

int main() {
    
//    // UPM CODE BEGIN -------------------------------------
//
//    int bus = 0;
//    uint8_t address = 0x48;
//
//    mraa::MraaIo mraaIo;
//    mraa::I2c* i2c;
//
//    if (!(i2c = new mraa::I2c(bus))) {
//        cerr << "I2c init failed\n";
//        return 0;
//    }
//
//    if ((i2c->address(address) != mraa::SUCCESS)) {
//        cerr << "I2C set address failed\n"; 
//        return 0;
//    }
//
//    if (i2c->frequency(mraa::I2C_FAST) != mraa::SUCCESS) {
//        cerr << "Failed to set I2C frequency\n";
//    }
//
//    uint16_t config = 0x0000;
//    // uint16_t ADS_MUX_MASK = 0x7000;
//    // uint16_t ADS_MUX_SINGLE_0 = 0x4000;
//
//    // config = (config & ~ADS_MUX_MASK) | ADS_MUX_SINGLE_0;
//
//    if (i2c->writeWordReg(0x01, swapWord(config)) != mraa::SUCCESS) {
//        cerr << "Failed to write config register\n";
//        return 0;
//    }
//
//    cout << "Success\n";
//
//    // UPM CODE END ----------------------------------------
 
     mraa::I2c i2c(0);
     uint8_t writeBuf[3];
     uint8_t readBuf[2];
 
     // Set the slave to talk to
     if (i2c.address(0x48) != mraa::SUCCESS) {
         cerr << "Failed to set address\n";
             return 0;
     }
 
     writeBuf[0] = 0x01; // Write to config register
     // if (i2c.write(writeBuf, 1) != mraa::SUCCESS) {
     //     cerr << "Failed to write config register (8 bits)\n";
     //     return 0;
     // }
     writeBuf[1] = 0xC3;
     // if (i2c.write(writeBuf, 2) != mraa::SUCCESS) {
     //     cerr << "Failed to write config register (16 bits)\n";
     //     return 0;
     // }
     writeBuf[2] = 0x03;
     if (i2c.write(writeBuf, 3) != mraa::SUCCESS) {
         cerr << "Failed to write config register (24 bits)\n";
         return 0;
     }
 
     readBuf[0] = 0;
     readBuf[1] = 0;
 
     while ((readBuf[0] & 0x80) == 0) {
         i2c.read(readBuf, 2); // Read the config register into readBuf
     }
 
     writeBuf[0] = 0;					// Set pointer register to 0 to read from the conversion register
     if (i2c.write(writeBuf, 1) != mraa::SUCCESS) {
         cerr << "Failed to write to pointer register\n";
     }
     
     i2c.read(readBuf, 2);
 
     int16_t val = readBuf[0] << 8 | readBuf[1];	// Combine the two bytes of readBuf into a single 16 bit result 
     
     printf("A0 Voltage Reading %f (V) \n", (float)val*4.096/32767.0);	// Print the result to terminal, first convert from binary value to mV
 
}
