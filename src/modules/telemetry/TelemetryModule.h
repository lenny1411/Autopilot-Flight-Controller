#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_TELEMETRYRMODULE_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_TELEMETRYRMODULE_H

#include <cstdint>
#include "../../devices_interfaces/telemetry/Telemetry.h"
#include "../../devices_interfaces/telemetry/WirelessInterface.h"
#include "../../msg/MessageManager.h"
#include "../../utils/utils.h"

class TelemetryModule {
public:
    TelemetryModule();
    int8_t init();
    void run();

private:
    Telemetry telemetry;
    // Drone -> Telemetry
    struct receiverData receiverValues;
    struct attitudeData attitudeValues;
    struct positionData positionValues;
    struct altitudeData altitudeValues;
    struct motorsData motorsValues;
    enum droneState state;

    // Telemetry -> Drone
    struct navigationSetpoint navSetpointValues;
    struct motorsData motorsSetpointValues;
    struct attitudeConfig attitudeConfValues;
    struct pidConfig pidConfValues;
    struct pidNavigationConfig pidNavConfValues;
    enum droneState stateConf;
    
    uint64_t timestamp, loopPeriod, count;
    bool isFirstDataReceived = false;

    void sendConfigValues();
    void getDataFromNodes();
    void sendTelemetryValues();
    void processDataFromGroundStation();
    void setDataToNodes();
};

#endif