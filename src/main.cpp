#include "FreeRTOS.h"
#include "freertos/task.h"
#include "utils/utils.h"
#include "msg/MessageManager.h"
#include "devices_interfaces/low-level/IoDevice.h"
#include "config.h"
#include "structs.h"
#include "app_tasks.h"

IoDevice iOpins;
enum droneState state = DISARMED;

void setup() {
    MessageManager::getInstance().addTopic(SENSOR_TOPIC);
    MessageManager::getInstance().addTopic(SENSOR_CONFIG_TOPIC);
    MessageManager::getInstance().addTopic(ALTITUDE_SENSOR_TOPIC);
    MessageManager::getInstance().addTopic(RECEIVER_TOPIC);
    MessageManager::getInstance().addTopic(PID_CONFIG_TOPIC);
    MessageManager::getInstance().addTopic(POSITION_TOPIC);
    MessageManager::getInstance().addTopic(NAVIGATION_TOPIC);
    MessageManager::getInstance().addTopic(NAVIGATION_CONFIG_TOPIC);
    MessageManager::getInstance().addTopic(ANGLE_SETPOINT_TOPIC);
    MessageManager::getInstance().addTopic(ANGLE_SETPOINT_NAV_TOPIC);
    MessageManager::getInstance().addTopic(MOTOR_TOPIC);
    MessageManager::getInstance().addTopic(MOTOR_SETPOINT_TOPIC);
    MessageManager::getInstance().addTopic(STATE_TOPIC);
    MessageManager::getInstance().addTopic(TELEMETRY_STATE_TOPIC);

    iOpins.init();
    iOpins.setPinMode(LED_PIN, false);


    MessageManager::getInstance().subscribe(STATE_TOPIC, [&](const void * message) -> void {
        state = *(static_cast<const droneState *>(message));
    });

    MessageManager::getInstance().publish(STATE_TOPIC, &state);
    
    xTaskCreatePinnedToCore(
        receiverTask,   /* Function to implement the task */
        "receiver",         /* Name of the task */
        10000,          /* Stack size in words */
        nullptr,           /* Task input parameter */
        2,              /* Priority of the task */
        nullptr,
        1);

    xTaskCreatePinnedToCore(
        sensorsTask,   /* Function to implement the task */
        "sensors",         /* Name of the task */
        10000,          /* Stack size in words */
        nullptr,           /* Task input parameter */
        2,              /* Priority of the task */
        nullptr,
        0);

    xTaskCreatePinnedToCore(
        altitudeTask,   /* Function to implement the task */
        "altitude",         /* Name of the task */
        10000,          /* Stack size in words */
        nullptr,           /* Task input parameter */
        2,              /* Priority of the task */
        nullptr,
        0);

    xTaskCreatePinnedToCore(
        navigationTask,   /* Function to implement the task */
        "navigation",         /* Name of the task */
        10000,          /* Stack size in words */
        nullptr,           /* Task input parameter */
        2,              /* Priority of the task */
        nullptr,
        1);
    
    xTaskCreatePinnedToCore(
        controllerTask,   /* Function to implement the task */
        "controller",         /* Name of the task */
        15000,          /* Stack size in words */
        nullptr,           /* Task input parameter */
        2,              /* Priority of the task */
        nullptr,
        1);

    xTaskCreatePinnedToCore(
        telemetryTask,   /* Function to implement the task */
        "telemetry",         /* Name of the task */
        15000,          /* Stack size in words */
        nullptr,           /* Task input parameter */
        2,              /* Priority of the task */
        nullptr,
        1);
}

void loop() {
    if(state == DISARMED) {
        iOpins.setPin(LED_PIN, false);
    } else if(state == MANU) {
        iOpins.setPin(LED_PIN, true);
        delay_milis(1000);
        iOpins.setPin(LED_PIN, false);
        delay_milis(100); 
    } else if(state == LEVEL) {
        iOpins.setPin(LED_PIN, true);
        delay_milis(100);
        iOpins.setPin(LED_PIN, false);
        delay_milis(100); 

        iOpins.setPin(LED_PIN, true);
        delay_milis(100);
        iOpins.setPin(LED_PIN, false);
        delay_milis(1000); 
    } else if(state == POS_HOLD) {
        iOpins.setPin(LED_PIN, true);
        delay_milis(100);
        iOpins.setPin(LED_PIN, false);
        delay_milis(100); 

        iOpins.setPin(LED_PIN, true);
        delay_milis(100);
        iOpins.setPin(LED_PIN, false);
        delay_milis(100); 

        iOpins.setPin(LED_PIN, true);
        delay_milis(100);
        iOpins.setPin(LED_PIN, false);
        delay_milis(1000); 
    } else if(state == NAVIGATION) {
        iOpins.setPin(LED_PIN, true);
        delay_milis(100);
        iOpins.setPin(LED_PIN, false);
        delay_milis(100); 

        iOpins.setPin(LED_PIN, true);
        delay_milis(100);
        iOpins.setPin(LED_PIN, false);
        delay_milis(100); 

        iOpins.setPin(LED_PIN, true);
        delay_milis(100);
        iOpins.setPin(LED_PIN, false);
        delay_milis(100); 

        iOpins.setPin(LED_PIN, true);
        delay_milis(100);
        iOpins.setPin(LED_PIN, false);
        delay_milis(1000); 
    }
}