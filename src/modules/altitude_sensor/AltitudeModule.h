//
// Created by lenny on 15/01/24.
//

#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_ALTITUDEMODULE_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_ALTITUDEMODULE_H

#include <cstdint>
#include "../../devices_interfaces/lidar/Lidar.h"
#include "../../config.h"
#include "../../msg/MessageManager.h"
#include "../../utils/utils.h"

void altitudeTask(void *args);
class AltitudeModule {
public:
    AltitudeModule();
    int8_t init();
    void run();
private:
    Lidar lidar;
    struct altitudeData values;
    uint64_t timestamp = 0;
    float previousAlt = 0;
};

#endif //AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_ALTITUDEMODULE_H
