#include "AltitudeModule.h"


AltitudeModule::AltitudeModule()
{
    
}

int8_t AltitudeModule::init()
{
    if(lidar.init() != 0)
        return -1;

    lidar.updateAndGetData(values);
    previousAlt = values.alt;

    return 0;
}

void AltitudeModule::run()
{
    timestamp = get_ms_count();

    if(lidar.updateAndGetData(values) == 0)
        MessageManager::getInstance().publish(ALTITUDE_SENSOR_TOPIC, &values);
        

    values.loopPeriod = get_ms_count() - timestamp;
    wait(values.loopPeriod, ALTITUDE_LOOP_FREQ);
}