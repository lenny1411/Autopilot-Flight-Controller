//
// Created by lenny on 28/01/24.
//

#include "Motors.h"
#include "../../utils/utils.h"

int8_t Motors::init() {
    uint8_t index = 0;
    for(index = 0; index < NUMBER_OF_MOTORS; index++) {
        pwmDevices[index].init();
        pwmDevices[index].initPin(index, pins[index], PWM_FREQ);
        setPulse(index, MIN_THROTTLE_VALUE);
    }

#if NUMBER_OF_MOTORS == 4
    pwmPan.init();
    pwmPan.initPin(4, ESC2_PIN, PWM_SERVO_FREQ);

    pwmTilt.init();
    pwmTilt.initPin(5, ESC5_PIN, PWM_SERVO_FREQ);

    pwmPan.setPin(4, ESC2_PIN, MID_CHANNEL_VALUE * 0.2); // 1.352 for 12 bits and 330 Hz
    pwmTilt.setPin(5, ESC5_PIN, MIN_SERVO_VALUE * 0.2); // 1.352 for 12 bits and 330 Hz
#endif
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

void Motors::setServoPulse(uint16_t panPulseDuration, uint16_t tiltPulseDuration) {
#if NUMBER_OF_MOTORS == 4
    panPulseDuration = map_((long)panPulseDuration, MIN_SERVO_VALUE, MAX_SERVO_VALUE, MAX_SERVO_VALUE, MIN_SERVO_VALUE);
    tiltPulseDuration -=  SERVO_TILT_OFFSET;
    pwmPan.setPin(4, ESC2_PIN, panPulseDuration * 0.2); // 0.2 for 12 bits and 50 Hz
    pwmTilt.setPin(5, ESC5_PIN, tiltPulseDuration * 0.2); // 0.2 for 12 bits and 50 Hz
#endif
}

int8_t Motors::deinit() {
    return 0;
}