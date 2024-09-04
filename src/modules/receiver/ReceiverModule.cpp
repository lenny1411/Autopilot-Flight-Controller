//
// Created by lenny on 14/01/24.
//

#include "ReceiverModule.h"

float computeAngleFromChannel(uint16_t channel) {
    return map_((float)channel,  MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE, -MAX_ANGLE, MAX_ANGLE); // deg
}

float computeYawRateFromChannel(uint16_t channel) {
    return map_((float)channel,   MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE,   -MAX_ANGLE_RATE, MAX_ANGLE_RATE);   // deg/sec
}

ReceiverModule::ReceiverModule() {

}

int8_t ReceiverModule::init()
{
    MessageManager::getInstance().subscribe(STATE_TOPIC, [this](const void* message) -> void {
        state = *(static_cast<const droneState *>(message));
    });

    return receiver.init();
}

void ReceiverModule::run()
{
    timestamp = get_ms_count();

    getDataFromNodesAndReceiver();

    setpoint.roll = computeAngleFromChannel(values.chan[ROLL_CHAN]);
    setpoint.pitch = computeAngleFromChannel(values.chan[PITCH_CHAN]);
    setpoint.yawRate = computeYawRateFromChannel(values.chan[YAW_CHAN]);

    setDataToNodes();

    values.loopPeriod = get_ms_count() - timestamp;
    wait(values.loopPeriod, RECEIVER_LOOP_FREQ);
}

void ReceiverModule::getDataFromNodesAndReceiver()
{
    receiver.updateAndGetData(values);
}

float ReceiverModule::computeAngleFromChannel(uint16_t channel)
{
    return map_((float)channel,  MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE, -MAX_ANGLE, MAX_ANGLE); // deg
}

float ReceiverModule::computeYawRateFromChannel(uint16_t channel)
{
    return map_((float)channel,   MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE,   -MAX_ANGLE_RATE, MAX_ANGLE_RATE);   // deg/sec
}

void ReceiverModule::setDataToNodes()
{
    MessageManager::getInstance().publish(RECEIVER_TOPIC, &values);
    if(state == LEVEL)
         MessageManager::getInstance().publish(ANGLE_SETPOINT_TOPIC, &setpoint);

}


