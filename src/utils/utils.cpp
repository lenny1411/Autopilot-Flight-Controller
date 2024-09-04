//
// Created by lenny on 02/01/24.
//

#include "utils.h"
#include "FreeRTOS.h"
#include "freertos/task.h"

void delay_milis(uint32_t milis) {
    vTaskDelay(milis / portTICK_PERIOD_MS);
}

void wait(uint32_t currentLoopDuration, uint16_t loopFreq) {
    int16_t diff = (1.0f / (float)loopFreq) * 1000 - currentLoopDuration;
    for (int16_t count = 0; count < diff; count++) {
             delay_milis(1);
    }
}

uint64_t get_ms_count() {
    return xTaskGetTickCount();
}

long map_(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float map_(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long constrain_(long value, long min, long max) {
    if(value < min)
        return min;
    else if(value > max)
        return max;
    else
        return value;
}

float constrain_(float value, float min, float max) {
    if(value < min)
        return min;
    else if(value > max)
        return max;
    else
        return value;
}

uint16_t constrain_(uint16_t value, uint16_t min, uint16_t max) {
    if(value < min)
        return min;
    else if(value > max)
        return max;
    else
        return value;
}