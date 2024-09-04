//
// Created by lenny on 15/04/2024.
//

#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_NTRIPCLIENT_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_NTRIPCLIENT_H

#include <cstdint>
#include <cstddef>
#include <Arduino.h>
#include "WiFi.h"
#include "config.h"

class NtripClient {
public:
    NtripClient();
    ~NtripClient();

    int8_t init();
    int8_t connect();
    void process();
    int getMessageLen();
    uint8_t* getMessage();
private:
    int messageSize = 0;
    uint8_t buf[5000];
    unsigned long timeout = NTRIP_TIMEOUT;

    WiFiClient ntripClient;
};


#endif //AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_NTRIPCLIENT_H
