//
// Created by lenny on 14/01/24.
//

#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_ATTITUDEMODULE_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_ATTITUDEMODULE_H

#include <cstdint>
#include "../../devices_interfaces/low-level/I2cDevice.h"
#include "../../devices_interfaces/imu/Imu.h"
#include "attitude_estimators/mahony/MahonyAHRS.h"
#include "attitude_estimators/kalman/KalmanFilters.h"
#include "../../config.h"
#include "../../utils/utils.h"
#include "../../msg/MessageManager.h"
#include "attitude_estimators/madgwick/MadgwickAHRS.h"

#define EARTH_GRAVITY 9.81 // m/s/s

class SensorsModule {
public:
    SensorsModule(I2cDevice *i2c);
    int8_t init();
    void run();
private:
    Imu imu;
    Madgwick ahrs;
    enum droneState state;

    // struct altitudeData altitudeValues;
    struct attitudeData attitudeValues;
    struct attitudeConfig config;
    uint64_t timestamp = 0, prevTimestamp = 0;
    float previousAlt = 0;
    uint16_t count = 0;
    float altitudeOffset = 0;
    float headingCorrection = 0;
    
    void getDataFromSensors();
    void computeData();
    void computeOrientation(float ax, float ay, float az, float *roll, float *pitch);
    float computeHeading(int16_t magX, int16_t magY, int16_t magZ, float roll, float pitch);
};

#endif //AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_ATTITUDEMODULE_H
