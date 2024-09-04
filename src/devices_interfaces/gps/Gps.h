//
// Created by lenny on 06/01/24.
//

#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_GPS_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_GPS_H

#include <cstdint>
#include <cstddef>
#include "structs.h"
#include "devices_interfaces/low-level/UartDevice.h"
// #include "NtripClient/NtripClient.h"

class Gps {
public:
    Gps();
    ~Gps();

    int8_t init();
    int8_t deinit();
    int8_t updateAndGetData(struct positionData &values);
private:
    UartDevice uart;
    // NtripClient ntripClient;

    void sendGpsConfig();
    int8_t NMEA_GGA_decode(char* nmeaFrame, double &lat, double &lon);
};

#endif //AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_GPS_H
