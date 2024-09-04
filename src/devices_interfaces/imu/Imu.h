//
// Created by lenny on 05/01/24.
//

#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_IMU_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_IMU_H

#include <cstdint>
#include <cstddef>
#include "../low-level/I2cDevice.h"
#include "devices/bmi088/BMI088.h"
#include "devices/ist8310/IST8310.h"
#include "../../structs.h"

class Imu {
public:
    Imu() {}
    Imu(I2cDevice *i2c);
    ~Imu();

    int8_t init();
    int8_t deinit();
    int8_t updateAndGetData(struct attitudeData &values);
    int8_t getMagData(struct attitudeData &values);
    void imuCalibration(uint16_t calibNum);
    void magCalibration(uint16_t time);
private:
    I2cDevice* i2c;
    Bmi088Accel accel;
    Bmi088Gyro gyro;
    IST8310 mag;
    // int16_t magMaxAxisY = 168, magMinAxisY = -157;
    // int16_t magMaxAxisX = 148, magMinAxisX = -177;
    // int16_t magMaxAxisZ = 126, magMinAxisZ = -193;
    // int16_t magMaxAxisY = 172, magMinAxisY = -163;
    // int16_t magMaxAxisX = 153, magMinAxisX = -184;
    // int16_t magMaxAxisZ = 129, magMinAxisZ = -198;

    int16_t magMaxAxisX = 169, magMinAxisX = -154;
    int16_t magMaxAxisY = 150, magMinAxisY = -177;
    int16_t magMaxAxisZ = 140, magMinAxisZ = -189;


    float gyroRateOffsetRoll;
    float gyroRateOffsetPitch;
    float gyroRateOffsetYaw;
    float accRateOffsetRoll;
    float accRateOffsetPitch;
    float accRateOffsetYaw;
    float compass_scale_x, compass_scale_y, compass_scale_z;
    int16_t compass_offset_x, compass_offset_y, compass_offset_z;
};


#endif //AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_IMU_H
