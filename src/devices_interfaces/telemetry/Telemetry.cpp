//
// Created by lenny on 12/01/24.
//

#include "Telemetry.h"
#include "../../../lib/ArduinoJson/ArduinoJson.h"

Telemetry::Telemetry() {

}

Telemetry::~Telemetry() {

}

int8_t Telemetry::init() {
    return wifiManager.init();
}

int8_t Telemetry::deinit() {
    return wifiManager.deinit();
}

int8_t Telemetry::sendConfigValues(
    struct attitudeConfig &attitudeConf,
    struct pidConfig &pidConf,
    struct pidNavigationConfig &navConf
) {
    StaticJsonDocument<1500> documentTx;

    documentTx["isInitConfig"].set(1);

    documentTx["roll"].set(attitudeConf.offsetRoll);
    documentTx["pitch"].set(attitudeConf.offsetPitch);
    documentTx["yaw"].set(attitudeConf.offsetYaw);

    documentTx["proll"].set(pidConf.proll);
    documentTx["ppitch"].set(pidConf.ppitch);
    documentTx["pyaw"].set(pidConf.pyaw);

    documentTx["iroll"].set(pidConf.iroll);
    documentTx["ipitch"].set(pidConf.ipitch);
    documentTx["iyaw"].set(pidConf.iyaw);;

    documentTx["droll"].set(pidConf.droll);
    documentTx["dpitch"].set(pidConf.dpitch);
    documentTx["dyaw"].set(pidConf.dyaw);

    documentTx["pAlt"].set(pidConf.pAltitude);
    documentTx["iAlt"].set(pidConf.iAltitude);
    documentTx["dAlt"].set(pidConf.dAltitude);

    documentTx["pnav"].set(navConf.pnav);
    documentTx["inav"].set(navConf.inav);
    documentTx["dnav"].set(navConf.dnav);

    documentTx["param1"].set(attitudeConf.param1);
    documentTx["param2"].set(attitudeConf.param2);

    char output[1500];
    serializeJson(documentTx, output);

    wifiManager.sendBytes(output, strlen(output));

    documentTx.clear();
    return 0;
}

int8_t Telemetry::sendTelemetryValues(
    struct attitudeData &attitude, 
    struct altitudeData &altitude, 
    struct positionData &position,
    struct receiverData &receiver, 
    struct motorsData &motors, 
    enum droneState &state, 
    uint64_t timestamp,
    uint64_t loopPeriod
) {
    StaticJsonDocument<1500> documentTx;

    documentTx["roll"].set(attitude.roll);
    documentTx["pitch"].set(attitude.pitch);
    documentTx["yaw"].set(attitude.yaw);

    documentTx["gyroRoll"].set(attitude.gyroRateRoll);
    documentTx["gyroPitch"].set(attitude.gyroRatePitch);
    documentTx["gyroYaw"].set(attitude.gyroRateYaw);

    documentTx["accRoll"].set(attitude.accRateRoll);
    documentTx["accPitch"].set(attitude.accRatePitch);
    documentTx["accYaw"].set(attitude.accRateYaw);

    documentTx["loopTime"].set(attitude.loopPeriod);
    documentTx["timestamp"].set(timestamp);
    documentTx["alt"].set(altitude.alt);
    documentTx["vBat"].set(motors.vBat);

    documentTx["lat"].set(position.lat);
    documentTx["lon"].set(position.lon);

    documentTx["status"].set(state);

    for (int i = 0;i < NUMBER_OF_MOTORS;i++) {
        documentTx["mot"][i] = motors.mot[i];
    }

    for (int i = 0;i < NUMBER_OF_CHANNELS;i++) {
        documentTx["ch"][i] = receiver.chan[i];
    }

    char output[1500];
    serializeJson(documentTx, output);

    wifiManager.sendBytes(output, strlen(output));

    documentTx.clear();
    return 0;
}

bool Telemetry::isGroundStationAvailable() {
    if(wifiManager.dataAvailable() > 0)
        return true;
    return false;
}

void Telemetry::resetRecvBuffer() {
    char recvBuffer[1000];
    wifiManager.readAllBytes(recvBuffer);
}

int8_t Telemetry::getConfigData(
    struct attitudeConfig *attitude,
    struct pidConfig *pid,
    struct pidNavigationConfig *navConf,
    struct motorsData *motors,
    struct navigationSetpoint *navSetpoint,
    enum droneState *state) 
{
    char recvBuffer[1000];
    StaticJsonDocument<1200> documentRx;

    wifiManager.readAllBytes(recvBuffer);

    //ESP_LOGE("DEBUG", "Config received : %s", recvBuffer);

    DeserializationError err = deserializeJson(documentRx, recvBuffer);

    if(err != DeserializationError::Ok)
        return -1;

    *state = (enum droneState)documentRx["status"].as<int>();

    attitude->offsetRoll = documentRx["roll"].as<float>();
    attitude->offsetPitch = documentRx["pitch"].as<float>();
    attitude->offsetYaw = documentRx["yaw"].as<float>();

    pid->proll = documentRx["proll"].as<float>();
    pid->ppitch = documentRx["ppitch"].as<float>();
    pid->pyaw = documentRx["pyaw"].as<float>();

    pid->iroll = documentRx["iroll"].as<float>();
    pid->ipitch = documentRx["ipitch"].as<float>();
    pid->iyaw = documentRx["iyaw"].as<float>();

    pid->droll = documentRx["droll"].as<float>();
    pid->dpitch = documentRx["dpitch"].as<float>();
    pid->dyaw = documentRx["dyaw"].as<float>();

    pid->pAltitude = documentRx["pAlt"].as<float>();
    pid->iAltitude = documentRx["iAlt"].as<float>();
    pid->dAltitude = documentRx["dAlt"].as<float>();

    navConf->pnav = documentRx["pnav"].as<float>();
    navConf->inav = documentRx["inav"].as<float>();
    navConf->dnav = documentRx["dnav"].as<float>();

    navSetpoint->lat = documentRx["lat"].as<float>();
    navSetpoint->lon = documentRx["lon"].as<float>();

    attitude->param1 = documentRx["param1"].as<float>();
    attitude->param2 = documentRx["param2"].as<float>();


    for(int i = 0;i < NUMBER_OF_MOTORS;i++)
        motors->mot[i] = documentRx["mot"][i].as<uint16_t>();

    return 0;
}
