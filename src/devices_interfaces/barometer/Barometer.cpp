#include "Barometer.h"
#include <cmath>

Barometer::Barometer(I2cDevice *i2c) {
    this->i2c = i2c;
    this->ps = LPS(*i2c);
}

Barometer::~Barometer() {

}

int8_t Barometer::init() {
    struct altitudeData values;
    if (!ps.init())
        return -1;

    ps.enableDefault();
    ps.writeReg(0x10, 0b01000111);

    for (size_t i = 0; i < 800; i++)
    {
        updateAndGetData(values);
        delay_milis((1 / BAROMETER_LOOP_FREQ) * 1000);
    }
    
    refAltitude = values.alt;
    return 0;
}

int8_t Barometer::deinit() {
    return 0;
}

int8_t Barometer::updateAndGetData(struct altitudeData &values) {
    float pressure = ps.readPressureMillibars();
  
    pressureSlow = pressureSlow * 0.985f + pressure * 0.015f; 

    float press_diff = pressureSlow - pressure;

    if (press_diff > 8) press_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (press_diff < -8) press_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    // If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (press_diff > 1 || press_diff < -1) pressureSlow -= press_diff / 6.0;

    values.alt = ps.pressureToAltitudeMeters(pressureSlow) * 100.0f - refAltitude;
    ps.readTemperatureC();

    return 0;
}