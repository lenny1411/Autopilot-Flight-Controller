//
// Created by lenny on 28/01/24.
//

#include "Motors.h"

int8_t Motors::init() {
    uint8_t index = 0;
    for(index = 0; index < NUMBER_OF_MOTORS; index++) {
        pwmDevices[index].init();
        pwmDevices[index].initPin(index, pins[index], PWM_FREQ);
        setPulse(index, MIN_THROTTLE_VALUE);
    }
    return 0;
}

void Motors::setMotors(struct motorsData setpoint) {
    uint8_t index = 0;
    for(index = 0; index < NUMBER_OF_MOTORS; index++)
        setPulse(index, setpoint.mot[index]);
}

int8_t Motors::initPin(uint8_t motorIndex, uint8_t pin, float pwmFreq) {
    return pwmDevices[motorIndex].initPin(motorIndex, pin, pwmFreq);
}

void Motors::setPulse(uint8_t motorIndex, uint16_t pulseDuration) {
    pwmDevices[motorIndex].setPin(motorIndex, 0, pulseDuration * 1.638f); // 1.638 for 12 bits and 400 Hz
}

int8_t Motors::deinit() {
    return 0;
}