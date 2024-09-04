//
// Created by lenny on 12/01/24.
//

#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_TELEMETRY_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_TELEMETRY_H

#include <cstdint>
#include <cstddef>
#include "../../structs.h"
#include "devices_interfaces/telemetry/wifi/WifiManager.h"

class Telemetry {
public:
    Telemetry();
    ~Telemetry();

    int8_t init();
    int8_t deinit();

    int8_t sendConfigValues(
        struct attitudeConfig &attitudeConf,
        struct pidConfig &pidConf,
        struct pidNavigationConfig &navConf);


    int8_t sendTelemetryValues(
            struct attitudeData &attitude,
            struct altitudeData &altitude,
            struct positionData &position,
            struct receiverData &receiver,
            struct motorsData &motors,
            enum droneState &state,
            uint64_t timestamp,
            uint64_t loopPeriod);

    bool isGroundStationAvailable();

    void resetRecvBuffer();

    int8_t getConfigData(
            struct attitudeConfig *attitude,
            struct pidConfig *pid,
            struct pidNavigationConfig *navConf,
            struct motorsData *motors,
            struct navigationSetpoint *navSetpoint,
            enum droneState *state);
private:
    WifiManager wifiManager;
};


#endif //AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_TELEMETRY_H
