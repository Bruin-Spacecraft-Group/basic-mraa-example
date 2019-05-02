#include <iostream>
#include "mraa.h"
#include "mraa/i2c.hpp" // or "mraa/i2c.h"?
using namespace std;

#define READ_BUFFER_LENGTH 6

// address and id
#define ADXL345_I2C_ADDR 0x53
#define ADXL345_ID 0x00

//control registers
#define ADXL345_OFSX 0x1E
#define ADXL345_OFSY 0x1F
#define ADXL345_OFSZ 0x20
#define ADXL345_TAP_THRESH 0x1D
#define ADXL345_TAP_DUR 0x21
#define ADXL345_TAP_LATENCY 0x22
#define ADXL345_ACT_THRESH 0x24
#define ADXL345_INACT_THRESH 0x25
#define ADXL345_INACT_TIME 0x26
#define ADXL345_INACT_ACT_CTL 0x27
#define ADXL345_FALL_THRESH 0x28
#define ADXL345_FALL_TIME 0x29
#define ADXL345_TAP_AXES 0x2A
#define ADXL345_ACT_TAP_STATUS 0x2B

//interrupt registers
#define ADXL345_INT_ENABLE 0x2E
#define ADXL345_INT_MAP 0x2F
#define ADXL345_INT_SOURCE 0x30

//data registers (read only)
#define ADXL345_XOUT_L 0x32
#define ADXL345_XOUT_H 0x33
#define ADXL345_YOUT_L 0x34
#define ADXL345_YOUT_H 0x35
#define ADXL345_ZOUT_L 0x36
#define ADXL345_ZOUT_H 0x37
#define DATA_REG_SIZE 6

//data and power management
#define ADXL345_BW_RATE 0x2C
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_FIFO_CTL 0x38
#define ADXL345_FIFO_STATUS 0x39

//useful values
#define ADXL345_POWER_ON 0x08
#define ADXL345_AUTO_SLP 0x30
#define ADXL345_STANDBY 0x00

//scales and resolution
#define ADXL345_FULL_RES 0x08
#define ADXL345_10BIT 0x00
#define ADXL345_2G 0x00
#define ADXL345_4G 0x01
#define ADXL345_8G 0x02
#define ADXL345_16G 0x036

int main() {
    uint8_t buffer[READ_BUFFER_LENGTH];

    // Unsure why init with 0
    // Doc says "number of the used i2c bus"
    mraa::I2c i2c(0);

    // Set the slave to talk to
    if (i2c.address(ADXL345_I2C_ADDR) != mraa::SUCCESS) {
        cerr << "Failed to set address\n";
            return 0;
    }

    buffer[0] = ADXL345_POWER_CTL;
    buffer[1] = ADXL345_POWER_ON;
    // Writes two bytes to the i2c bus
    if (i2c.write(buffer, 2) != mraa::SUCCESS) {
        cerr << "Failed to write control register\n";
        return 0;
    }

    buffer[0] = ADXL345_DATA_FORMAT;
    buffer[1] = ADXL345_16G | ADXL345_FULL_RES;
    if (i2c.write(buffer, 2) != mraa::SUCCESS) {
        cerr << "Failed to write mode register\n";
        return 0;
    }

    int16_t rawaccel[3];

    for (int i = 0; i < 3; i++) {
        if (i2c.writeByte(ADXL345_XOUT_L) != mraa::SUCCESS) { // 0x32
            cerr << "Failed to write byte to register\n";
            return 0;
        }
        i2c.read(buffer, DATA_REG_SIZE);
        rawaccel[0] = ((buffer[1] << 8 ) | buffer[0]); // x
        rawaccel[1] = ((buffer[3] << 8 ) | buffer[2]); // y
        rawaccel[2] = ((buffer[5] << 8 ) | buffer[4]); // z
        cout << "raw values: (" << rawaccel[0] << ", " << rawaccel[1] << ", " << rawaccel[2] << ")\n";
        sleep(1);
    }
}
