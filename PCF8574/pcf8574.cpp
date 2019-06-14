// UNTESTED: read from I/O expander

#include <iostream>
#include "mraa/i2c.hpp"
#include <unistd.h>

using namespace std;


int main() {
     mraa::I2c i2c(0);
     uint8_t writeBuf[2];
 
     // Set the slave to talk to
     if (i2c.address(0x71) != mraa::SUCCESS) {
         cerr << "Failed to set address\n";
             return 0;
     }
 
     writeBuf[0] = 0xFF;
     writeBuf[1] = 0xD; // Set P0 to output, P1 to input
     if (i2c.write(writeBuf, 2) != mraa::SUCCESS) {
         cerr << "Failed to set ports\n";
         return 0;
     }

     sleep(1000000);
 
     writeBuf[0] = 0x0;
     writeBuf[1] = 0x1; // high output
     if (i2c.write(writeBuf, 2) != mraa::SUCCESS) {
         cerr << "Failed to write to port (high)\n";
     }

     sleep(1000000);

     writeBuf[1] = 0x0; // low output
     if (i2c.write(writeBuf, 2) != mraa::SUCCESS) {
         cerr << "Failed to write to port (low)\n";
     }

     cout << "Success" << endl;
}
