#include <iostream>
#include "i2c_A0_ads1115.h"

using namespace std;

int main() {
    ADS1115 sensor;
    sensor.pollData();
}
