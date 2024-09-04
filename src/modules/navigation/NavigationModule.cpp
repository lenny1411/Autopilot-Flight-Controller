//
// Created by lenny on 28/01/24.
//

#include "NavigationModule.h"

NavigationModule::NavigationModule()
{
    latitudePid = FastPID(P_NAV, I_NAV, D_NAV, NAVIGATION_LOOP_FREQ, 16, true);
    longitudePid = FastPID(P_NAV, I_NAV, D_NAV, NAVIGATION_LOOP_FREQ, 16, true);
}

int8_t NavigationModule::init()
{
    if(gps.init() != 0)
        return -1;

    MessageManager::getInstance().subscribe(NAVIGATION_CONFIG_TOPIC, [this](const void* message) -> void {
        const pidNavigationConfig config = *(static_cast<const pidNavigationConfig *>(message));

        this->latitudePid.setCoefficients(config.pnav, config.inav, config.dnav, NAVIGATION_LOOP_FREQ);
        this->longitudePid.setCoefficients(config.pnav, config.inav, config.dnav, NAVIGATION_LOOP_FREQ);

        this->latitudePid.clear();
        this->longitudePid.clear();
    });

    MessageManager::getInstance().subscribe(SENSOR_TOPIC, [this](const void * message) -> void {
        anglesValues = *(static_cast<const attitudeData *>(message));
    });

    MessageManager::getInstance().subscribe(STATE_TOPIC, [this](const void* message) -> void {
        state = *(static_cast<const droneState *>(message));
    });

    MessageManager::getInstance().subscribe(NAVIGATION_TOPIC, [this](const void* message) -> void {
        setpoint = *(static_cast<const navigationSetpoint *>(message));
    });

    latitudePid.setOutputRange(-MAX_NAV_ANGLE, MAX_NAV_ANGLE);
    longitudePid.setOutputRange(-MAX_NAV_ANGLE, MAX_NAV_ANGLE);

    return 0;
}

void NavigationModule::run()
{
    timestamp = get_ms_count();

    getDataFromNodesAndGps();
    processDataAndSetToNodes();

    values.loopPeriod = get_ms_count() - timestamp;
    wait(values.loopPeriod, NAVIGATION_LOOP_FREQ);
}

void NavigationModule::getDataFromNodesAndGps()
{
    isValidFix = gps.updateAndGetData(values) == 0;

    if(state == droneState::LEVEL) {
        setpoint.lat = values.lat;
        setpoint.lon = values.lon;
    };
}

void NavigationModule::processDataAndSetToNodes()
{
    if(state == droneState::NAVIGATION || state == droneState::POS_HOLD) {
        // anglesSetpoint.pitch = latitudePid.step(int16_t(setpoint.lat * LAT_LON_PRECISION), int16_t(values.lat * LAT_LON_PRECISION));
        // anglesSetpoint.roll = longitudePid.step(int16_t(setpoint.lon * LAT_LON_PRECISION), int16_t(values.lon * LAT_LON_PRECISION));

        int8_t out_lat = latitudePid.step( int16_t(setpoint.lat * LAT_LON_PRECISION), int16_t(values.lat * LAT_LON_PRECISION));
        int8_t out_lon = longitudePid.step(int16_t(setpoint.lon * LAT_LON_PRECISION), int16_t(values.lon * LAT_LON_PRECISION));

        anglesSetpoint.roll  = ((float)out_lon * cos(anglesValues.heading * 0.017453)) + ((float)out_lat * cos((anglesValues.heading + 90) * 0.017453));
        anglesSetpoint.pitch = ((float)out_lat * cos(anglesValues.heading * 0.017453)) + ((float)out_lon * cos((anglesValues.heading - 90) * 0.017453));

        MessageManager::getInstance().publish(ANGLE_SETPOINT_TOPIC, &anglesSetpoint);
    }
    MessageManager::getInstance().publish(POSITION_TOPIC, &values);
}
