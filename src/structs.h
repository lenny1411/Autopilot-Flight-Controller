//
// Created by lenny on 05/01/24.
//

#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_STRUCTS_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_STRUCTS_H

#include <cstdint>
#include <cstddef>
#include <string>
#include "config.h"

enum droneState {
    DISARMED = 0, MANU, LEVEL, POS_HOLD, NAVIGATION
};

struct attitudeData {
    float gyroRateRoll;
    float gyroRatePitch;
    float gyroRateYaw;
    float accRateRoll;
    float accRatePitch;
    float accRateYaw;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
    float roll;
    float pitch;
    float yaw;
    float offsetRoll;
    float offsetPitch;
    float offsetYaw;
    float heading;
    float param1, param2;
    uint64_t loopPeriod;
};

struct attitudeConfig {
    float offsetRoll = OFFSET_ROLL;
    float offsetPitch = OFFSET_PITCH;
    float offsetYaw = OFFSET_YAW;
    float param1 = PARAM_1, param2 = PARAM_2;
};

struct pidSetpoint {
    float roll;
    float pitch;
    float yawRate;
};

struct pidOutput {
    std::string status;
    int16_t out_roll;
    int16_t out_pitch;
    int16_t out_yaw;
    int16_t out_alt;
    // int16_t outBatteryCompensation;
};

struct positionData {
    std::string status;
    float latitude_home = 0.0f;
    float longitude_home = 0.0f;
    unsigned long TOW;
    bool isPositionFix;
    double vn;
    double ve;
    double vd;
    double lat;
    double lon;
    double alt;
    uint64_t loopPeriod;
};

struct pidConfig {
    // float proll = P_ROLL_PITCH, ppitch = P_ROLL_PITCH, pyaw = P_YAW;
    // float iroll = I_ROLL_PITCH, ipitch = I_ROLL_PITCH, iyaw = I_YAW;
    // float droll = D_ROLL_PITCH, dpitch = D_ROLL_PITCH, dyaw = D_YAW;
    float proll = P_ROLL_PITCH_ANGLE, ppitch = P_ROLL_PITCH_ANGLE, pyaw = P_YAW_ANGLE;
    float iroll = I_ROLL_PITCH_ANGLE, ipitch = I_ROLL_PITCH_ANGLE, iyaw = I_YAW_ANGLE;
    float droll = D_ROLL_PITCH_ANGLE, dpitch = D_ROLL_PITCH_ANGLE, dyaw = D_YAW_ANGLE;
    float pAltitude = P_ALT, iAltitude = I_ALT, dAltitude = D_ALT;
    bool newConfig;
};

struct pidNavigationConfig {
    float pnav = P_NAV, inav = I_NAV, dnav = D_NAV;
};

struct receiverData {
    std::string status;
    uint16_t chan[NUMBER_OF_CHANNELS];
    uint64_t loopPeriod;
};

struct altitudeData {
    std::string status;
    float vertical_speed;
    float alt;
    float press;
    uint64_t loopPeriod;
};

struct motorsData {
    std::string status;
    uint16_t mot[NUMBER_OF_MOTORS];
    float vBat;
    uint64_t loopPeriod;
};

struct navigationSetpoint
{
    double lat;
    double lon;
};

#endif //AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_STRUCTS_H
