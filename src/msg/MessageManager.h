//
// Created by lenny on 14/01/24.
//

#ifndef AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_MESSAGEMANAGER_H
#define AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_MESSAGEMANAGER_H

#include <functional>
#include "FreeRTOS.h"
#include "freertos/semphr.h"
#include <vector>
#include <string>

#define SUBSCRIBER_MAX_COUNT 15
#define TOPIC_MAX_COUNT      15

class MessageManager {
public:
    static MessageManager& getInstance() {
        static MessageManager instance;
        return instance;
    }

    MessageManager();
    bool addTopic(const std::string& topic);
    int8_t subscribe(const std::string& topic, std::function<void(const void *)> callback);
    void publish(const std::string& topic, const void * message);
private:
    uint8_t topic_count;
    SemaphoreHandle_t mutex_handles[TOPIC_MAX_COUNT];
    std::string topics[TOPIC_MAX_COUNT];
    int8_t subscriber_count_per_topic[TOPIC_MAX_COUNT];
    std::function<void(const void *)> callback_subscribers_per_topic[TOPIC_MAX_COUNT][SUBSCRIBER_MAX_COUNT];
};



#endif //AUTOPILOT_FLIGHT_CONTROLLER_SOFTWARE_MESSAGEMANAGER_H
