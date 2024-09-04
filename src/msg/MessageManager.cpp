#include "MessageManager.h"
#include "algorithm"

MessageManager::MessageManager()
{
    topic_count = 0;
}

bool MessageManager::addTopic(const std::string &topic)
{
    if(topic_count >= TOPIC_MAX_COUNT)
        return false;

    topics[this->topic_count] = topic;
    mutex_handles[this->topic_count] = xSemaphoreCreateMutex();

    if (mutex_handles[this->topic_count] == nullptr)
        return false;

    topic_count++;
    return true;
}

int8_t MessageManager::subscribe(const std::string& topic, std::function<void(const void *)> callback) {
    uint8_t topic_idx = std::distance(topics, std::find(topics, topics + topic_count, topic));

    if (topic_idx == topic_count || subscriber_count_per_topic[topic_idx] >= SUBSCRIBER_MAX_COUNT)
        return -1;

    xSemaphoreTake(mutex_handles[topic_idx], portMAX_DELAY);

    callback_subscribers_per_topic[topic_idx][subscriber_count_per_topic[topic_idx]] = callback;
    subscriber_count_per_topic[topic_idx]++;

    xSemaphoreGive(mutex_handles[topic_idx]);

    return subscriber_count_per_topic[topic_idx] - 1;
}

void MessageManager::publish(const std::string& topic, const void * message) {
    uint8_t topic_idx = std::distance(topics, std::find(topics, topics + topic_count, topic));

    if (topic_idx == topic_count)
        return;

    xSemaphoreTake(mutex_handles[topic_idx], portMAX_DELAY);
    uint8_t subscriber_count = subscriber_count_per_topic[topic_idx];

    for(int subscriber_idx = 0; subscriber_idx < subscriber_count; subscriber_idx++)
        callback_subscribers_per_topic[topic_idx][subscriber_idx](message);
    
    xSemaphoreGive(mutex_handles[topic_idx]);
}