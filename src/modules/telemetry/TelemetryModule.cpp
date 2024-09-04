#include "TelemetryModule.h"

TelemetryModule::TelemetryModule() {
    timestamp = 0;
    loopPeriod = 0;
    count = 0;
}

int8_t TelemetryModule::init() {
    if(telemetry.init() != 0)
        return -1;

    while (!telemetry.isGroundStationAvailable()); //Wait for first data from ground station
    telemetry.resetRecvBuffer();

    MessageManager::getInstance().subscribe(SENSOR_TOPIC, [this](const void * message) -> void {
        attitudeValues = *(static_cast<const attitudeData *>(message));
    });
    
    MessageManager::getInstance().subscribe(ALTITUDE_SENSOR_TOPIC, [this](const void * message) -> void {
        altitudeValues = *(static_cast<const altitudeData *>(message));
    });

    MessageManager::getInstance().subscribe(RECEIVER_TOPIC, [this](const void * message) -> void {
        receiverValues = *(static_cast<const receiverData *>(message));
    });

    MessageManager::getInstance().subscribe(POSITION_TOPIC, [this](const void * message) -> void {
        positionValues = *(static_cast<const positionData *>(message));
    });

    MessageManager::getInstance().subscribe(MOTOR_TOPIC, [this](const void * message) -> void {
        motorsValues = *(static_cast<const motorsData *>(message));
    });

    MessageManager::getInstance().subscribe(STATE_TOPIC, [this](const void * message) -> void {
        state = *(static_cast<const droneState *>(message));
    });

    sendConfigValues();
    return 0;
}

void TelemetryModule::run() {
    timestamp = get_ms_count();

    sendTelemetryValues();
    processDataFromGroundStation();

    loopPeriod = get_ms_count() - timestamp;

    if(state != DISARMED) {
        count++;
    }

    wait(loopPeriod, TELEMETRY_LOOP_FREQ);
}

void TelemetryModule::sendConfigValues() {
    telemetry.sendConfigValues(
        attitudeConfValues,
        pidConfValues,
        pidNavConfValues
    );
}

void TelemetryModule::sendTelemetryValues() {
    telemetry.sendTelemetryValues(
        attitudeValues, 
        altitudeValues, 
        positionValues,  
        receiverValues, 
        motorsValues, 
        state, 
        count / TELEMETRY_LOOP_FREQ,
        loopPeriod);
}

void TelemetryModule::processDataFromGroundStation() {
    if(telemetry.isGroundStationAvailable()) {
        if(telemetry.getConfigData(
            &attitudeConfValues,
            &pidConfValues,
            &pidNavConfValues,
            &motorsSetpointValues,
            &navSetpointValues,
            &stateConf
        ) == 0) 
        {
            setDataToNodes();
        } else {
            sendConfigValues();
        }
    }
}

void TelemetryModule::setDataToNodes() {
    if(state == droneState::NAVIGATION) {
        MessageManager::getInstance().publish(NAVIGATION_TOPIC, &navSetpointValues);
    } else {
        MessageManager::getInstance().publish(SENSOR_CONFIG_TOPIC, &attitudeConfValues);
        MessageManager::getInstance().publish(PID_CONFIG_TOPIC, &pidConfValues);
        MessageManager::getInstance().publish(MOTOR_SETPOINT_TOPIC, &motorsSetpointValues);
        MessageManager::getInstance().publish(STATE_TOPIC, &stateConf);
    }
}