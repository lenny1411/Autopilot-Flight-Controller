#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_BAROMETER_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_BAROMETER_H

#include <cstdint>
#include <cstddef>
#include "../low-level/I2cDevice.h"
#include "../../structs.h"
#include "devices/LPS22DF/LPS.h"

class Barometer {
public:
    Barometer() {}
    Barometer(I2cDevice *i2c);
    ~Barometer();

    int8_t init();
    int8_t deinit();
    int8_t updateAndGetData(struct altitudeData &values);
private:
    I2cDevice *i2c;
    LPS ps;
    float pressureSlow = 0;
    float refAltitude = 0;
    bool isRefAltitudeUpdated = false;
};

#endif