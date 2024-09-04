#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_CONTROLLERMODULE_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_CONTROLLERMODULE_H

#include <cstdint>
#include "../../devices_interfaces/motors/Motors.h"
#include "../../devices_interfaces/low-level/AdcDevice.h"
#include "../../msg/MessageManager.h"
#include "../../utils/utils.h"
#include "../../../lib/FastPID/src/FastPID.h"
#include <esp_log.h>

class FlightControllerModule {
public:
    FlightControllerModule();
    int8_t init();
    void run();
private:
    struct pidOutput pidValues;
    struct motorsData motorValues;
    struct attitudeData attitudeValues;
    struct altitudeData altitudeValues;
    struct pidSetpoint anglesSetpoint;
    struct receiverData receiverValues;
    struct motorsData setpoint;
    enum droneState state;
    enum droneState stateTelemtry;

    Motors motors;
    AdcDevice vBatPin;

    FastPID rollPid;
    FastPID pitchPid;
    FastPID yawPid;
    FastPID rateRollPid;
    FastPID ratePitchPid;
    FastPID rateYawPid;
    FastPID altitudePid;

    uint64_t timestamp = 0;
    float yaw = 0, offsetYaw = 0;
    float yawRateSetpoint, rollRateSetpoint, pitchRateSetpoint, altitudeSetpoint;
    bool isThrottleMoved;
    bool isNavPosMode;
    bool isAltHoldleEnable;


    void getSensor();
    void computeRateSetpoints();
    void processDroneState();
    void computeMotorsValues();
    void computeOutputValues();
    void sendValuesToMotors();
};

#endif //AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_CONTROLLERMODULE_H