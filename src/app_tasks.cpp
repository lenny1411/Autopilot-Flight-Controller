#include "app_tasks.h"
#include "modules/sensors/SensorsModule.h"
#include "modules/altitude_sensor/AltitudeModule.h"
#include "modules/navigation/NavigationModule.h"
#include "modules/receiver/ReceiverModule.h"
#include "modules/telemetry/TelemetryModule.h"
#include "modules/controller/FlightControllerModule.h"


void receiverTask(void *args) {
    ReceiverModule module;
    module.init();
    while(true) {
        module.run();
    }
}

void sensorsTask(void *args) {
    I2cDevice i2c;
    SensorsModule module = SensorsModule(&i2c);
    module.init();
    while(true) {
        module.run();
    }
}

void altitudeTask(void *args) {
    AltitudeModule module = AltitudeModule();
    module.init();
    while(true) {
        module.run();
    }
}

void navigationTask(void *args) {
    NavigationModule module;
    module.init();
    while(true) {
        module.run();
    }
}

void controllerTask(void *args) {
    FlightControllerModule module;
    module.init();
    while(true) {
        module.run();
    }
}

void telemetryTask(void *args) {
    TelemetryModule module;
    module.init();
    while(true) {
        module.run();
    }
}