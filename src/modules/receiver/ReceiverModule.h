//
// Created by lenny on 14/01/24.
//

#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_RECEIVERMODULE_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_RECEIVERMODULE_H

#include <cstdint>
#include "../../devices_interfaces/low-level/UartDevice.h"
#include "../../devices_interfaces/receiver/Receiver.h"
#include "../../msg/MessageManager.h"
#include "../../utils/utils.h"

class ReceiverModule {
public:
    ReceiverModule();
    int8_t init();
    void run();
private:
    Receiver receiver;
    struct receiverData values;
    struct pidSetpoint setpoint;
    enum droneState state;
    uint64_t timestamp = 0;

    void getDataFromNodesAndReceiver();
    float computeAngleFromChannel(uint16_t channel);
    float computeYawRateFromChannel(uint16_t channel);
    void setDataToNodes();
};

#endif //AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_RECEIVERMODULE_H
