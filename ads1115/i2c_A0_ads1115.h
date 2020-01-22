// Test: Attempting to Read single ended from A1 channel

#include <iostream>
#include "mraa/i2c.hpp"

using namespace std;

class ADS1115 {
    public:
        ADS1115(): m_i2c(1, true) {
            m_A0 = 0;
        }
        virtual bool pollData()
        {
            uint8_t writeBuf[3];
            uint8_t readBuf[2];

            // Set the slave to talk to
            if (m_i2c.address(0x48) != mraa::SUCCESS) {
                cerr << "Failed to set address\n";
                return 0;
            }

            writeBuf[0] = 0x01; // Write to config register
            writeBuf[1] = 0xD3;
            writeBuf[2] = 0x03;
            if (m_i2c.write(writeBuf, 3) != mraa::SUCCESS) {
                cerr << "Failed to write config register (24 bits)\n";
                return 0;
            }

            readBuf[0] = 0;
            readBuf[1] = 0;

            while ((readBuf[0] & 0x80) == 0) {
                m_i2c.read(readBuf, 2); // Read the config register into readBuf
            }

            writeBuf[0] = 0;					// Set pointer register to 0 to read from the conversion register
            if (m_i2c.write(writeBuf, 1) != mraa::SUCCESS) {
                cerr << "Failed to write to pointer register\n";
            }

            m_i2c.read(readBuf, 2);

            int16_t val = readBuf[0] << 8 | readBuf[1];	// Combine the two bytes of readBuf into a single 16 bit result 
            m_A0 = (float)val*4.096/32767.0;

            printf("A0 Voltage Reading %f (V) \n", (float)val*4.096/32767.0);	// Print the result to terminal, first convert from binary value to mV
        }
        virtual int powerOn()
        {
        }
        virtual int powerOff()
        {
        }
        virtual void printSensorInfo()
        {
        }
        virtual void printValues()
        {
        }
    private:
        mraa::I2c m_i2c;
        float m_A0;
};
