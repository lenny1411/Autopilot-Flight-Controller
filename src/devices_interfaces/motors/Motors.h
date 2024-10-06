//
// Created by lenny on 28/01/24.
//

#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_MOTORS_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_MOTORS_H

#include <cstdint>
#include <cstddef>
#include "../low-level/PwmDevice.h"
#include "../../structs.h"
#include "../../config.h"

#if NUMBER_OF_MOTORS == 4
const uint16_t pins[] = {ESC1_PIN, ESC3_PIN, ESC6_PIN, ESC4_PIN};
const uint16_t panPin = ESC2_PIN;
const uint16_t tiltPin = ESC5_PIN;
#endif

#if NUMBER_OF_MOTORS == 6
const uint16_t pins[] = {ESC6_PIN, ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN, ESC5_PIN};
#endif

class Motors {
public:
    int8_t init();
    void setMotors(struct motorsData setpoint);
    void setServoPulse(uint16_t panPulseDuration, uint16_t tiltPulseDuration);
    int8_t deinit();
private:
    int8_t initPin(uint8_t motorIndex, uint8_t pin, float pwmFreq);
    void setPulse(uint8_t motorIndex, uint16_t pulseDuration);
    PwmDevice pwmDevices[NUMBER_OF_MOTORS];
    PwmDevice pwmPan;
    PwmDevice pwmTilt;
};


#endif //AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_MOTORS_H
