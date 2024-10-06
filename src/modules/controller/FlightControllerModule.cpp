#include "FlightControllerModule.h"

FlightControllerModule::FlightControllerModule() {
    rollPid = FastPID(P_ROLL_PITCH_ANGLE, I_ROLL_PITCH_ANGLE, D_ROLL_PITCH_ANGLE, FLIGHT_CONTROL_LOOP_FREQ);
    pitchPid = FastPID(P_ROLL_PITCH_ANGLE, I_ROLL_PITCH_ANGLE, D_ROLL_PITCH_ANGLE, FLIGHT_CONTROL_LOOP_FREQ);
    yawPid = FastPID(P_YAW_ANGLE, I_YAW_ANGLE, D_YAW_ANGLE, FLIGHT_CONTROL_LOOP_FREQ);

    rateRollPid = FastPID(P_ROLL_PITCH, I_ROLL_PITCH, D_ROLL_PITCH, FLIGHT_CONTROL_LOOP_FREQ);
    ratePitchPid = FastPID(P_ROLL_PITCH, I_ROLL_PITCH, D_ROLL_PITCH, FLIGHT_CONTROL_LOOP_FREQ);
    rateYawPid = FastPID(P_YAW, I_YAW, D_YAW, FLIGHT_CONTROL_LOOP_FREQ);

    altitudePid = FastPID(P_ALT, I_ALT, D_ALT, FLIGHT_CONTROL_LOOP_FREQ);
    offsetYaw = 0;
}

int8_t FlightControllerModule::init() {
    MessageManager::getInstance().subscribe(PID_CONFIG_TOPIC, [this](const void * message) -> void {
        const pidConfig pidConfValues = *(static_cast<const pidConfig *>(message));
#if TELEMETRY_PID_CONFIG == 0
        rateRollPid.setCoefficients(pidConfValues.proll, pidConfValues.iroll, pidConfValues.droll, FLIGHT_CONTROL_LOOP_FREQ);
        ratePitchPid.setCoefficients(pidConfValues.ppitch, pidConfValues.ipitch, pidConfValues.dpitch, FLIGHT_CONTROL_LOOP_FREQ);
        rateYawPid.setCoefficients(pidConfValues.pyaw, pidConfValues.iyaw, pidConfValues.dyaw, FLIGHT_CONTROL_LOOP_FREQ);

        rateRollPid.clear();
        ratePitchPid.clear();
        rateYawPid.clear();
#elif TELEMETRY_PID_CONFIG == 1
        rollPid.setCoefficients(pidConfValues.proll, pidConfValues.iroll, pidConfValues.droll, FLIGHT_CONTROL_LOOP_FREQ);
        pitchPid.setCoefficients(pidConfValues.ppitch, pidConfValues.ipitch, pidConfValues.dpitch, FLIGHT_CONTROL_LOOP_FREQ);
        yawPid.setCoefficients(pidConfValues.pyaw, pidConfValues.iyaw, pidConfValues.dyaw, FLIGHT_CONTROL_LOOP_FREQ);

        rollPid.clear();
        pitchPid.clear();
        yawPid.clear();
#endif
        altitudePid.setCoefficients(pidConfValues.pAltitude, pidConfValues.iAltitude, pidConfValues.dAltitude, FLIGHT_CONTROL_LOOP_FREQ);
        altitudePid.clear();
    });

    MessageManager::getInstance().subscribe(SENSOR_TOPIC, [this](const void * message) -> void {
        attitudeValues = *(static_cast<const attitudeData *>(message));
    });
    
    MessageManager::getInstance().subscribe(ALTITUDE_SENSOR_TOPIC, [this](const void * message) -> void {
        altitudeValues = *(static_cast<const altitudeData *>(message));
    });

    MessageManager::getInstance().subscribe(RECEIVER_TOPIC, [this](const void * message) -> void {
        receiverValues = *(static_cast<const receiverData *>(message));
    });

    MessageManager::getInstance().subscribe(ANGLE_SETPOINT_TOPIC, [this](const void * message) -> void {
        anglesSetpoint = *(static_cast<const pidSetpoint *>(message));
    });

    MessageManager::getInstance().subscribe(ANGLE_SETPOINT_NAV_TOPIC, [this](const void * message) -> void {
        anglesSetpointNav = *(static_cast<const pidSetpoint *>(message));
    });

    MessageManager::getInstance().subscribe(MOTOR_SETPOINT_TOPIC, [this](const void * message) -> void {
        motorValues = *(static_cast<const motorsData *>(message));
    });

    MessageManager::getInstance().subscribe(TELEMETRY_STATE_TOPIC, [this](const void * message) -> void {
        stateTelemtry = *(static_cast<const droneState *>(message));
    });

    rollPid.setOutputRange(-MAX_ANGLE_RATE, MAX_ANGLE_RATE);
    pitchPid.setOutputRange(-MAX_ANGLE_RATE, MAX_ANGLE_RATE);
    yawPid.setOutputRange(-MAX_ANGLE_RATE, MAX_ANGLE_RATE);

    rateRollPid.setOutputRange( -200, 200);
    ratePitchPid.setOutputRange(-200, 200);
    rateYawPid.setOutputRange(  -200, 200);

    altitudePid.setOutputRange( -300, 300);

    if(motors.init() != 0)
        return -1;

    vBatPin.init();
    vBatPin.initPin(VBAT_PIN);

    return 0;
}

void FlightControllerModule::run() {
    timestamp = get_ms_count();

    getSensor();
    computeRateSetpoints();
    computePanTiltSetpoints();
    computeNextDroneState();
    processDroneState();
    computeOutputValues();
    if(state != MANU) computeMotorsValues();
    sendValuesToMotors();

    MessageManager::getInstance().publish(MOTOR_TOPIC, &motorValues);
    MessageManager::getInstance().publish(STATE_TOPIC, &state);

    motorValues.loopPeriod = get_ms_count() - timestamp;
    wait(motorValues.loopPeriod, FLIGHT_CONTROL_LOOP_FREQ);
}

void FlightControllerModule::getSensor() {
    motorValues.vBat = vBatPin.getValue(VBAT_PIN) * (3.3f / 4095.0f) * 13.631f;
}

void FlightControllerModule::computeRateSetpoints() {
    if(state == POS_HOLD || state == NAVIGATION) {
        rollRateSetpoint = rollPid.step(anglesSetpointNav.roll, attitudeValues.roll);
        pitchRateSetpoint = pitchPid.step(anglesSetpointNav.pitch, attitudeValues.pitch);
    } else {
        rollRateSetpoint = rollPid.step(anglesSetpoint.roll, attitudeValues.roll);
        pitchRateSetpoint = pitchPid.step(anglesSetpoint.pitch, attitudeValues.pitch);
    }
}

void FlightControllerModule::computePanTiltSetpoints() {
    if(receiverValues.chan[CAMERA_INIT_POS_CHAN] == MAX_CHANNEL_VALUE && state != LEVEL) {
        if(receiverValues.chan[ROLL_CHAN] > MAX_SERVO_CHAN_THRESHOLD || MIN_CHANNEL_THRESOLD > receiverValues.chan[ROLL_CHAN])
            panPulse = panPulse + (receiverValues.chan[ROLL_CHAN] - MID_SERVO_VALUE) * SERVO_SPEED_COEF;
        if(receiverValues.chan[PITCH_CHAN] > MAX_SERVO_CHAN_THRESHOLD || MIN_CHANNEL_THRESOLD > receiverValues.chan[PITCH_CHAN])
            tiltPulse = tiltPulse + (receiverValues.chan[PITCH_CHAN] - MID_SERVO_VALUE) * SERVO_SPEED_COEF;
    } else {
        panPulse = MID_CHANNEL_VALUE;
        tiltPulse = MIN_SERVO_VALUE;
    }
}

void FlightControllerModule::computeYaw() {
    if(receiverValues.chan[YAW_CHAN] < MAX_YAW_CHAN_THRESHOLD && receiverValues.chan[YAW_CHAN] > MIN_YAW_CHAN_THRESHOLD) {
        yaw = attitudeValues.yaw - offsetYaw;
        // yawRateSetpoint = (-yaw) * 5.0f;
        yawRateSetpoint = yawPid.step(0, yaw);
    } else {
        offsetYaw = attitudeValues.yaw;
        yawRateSetpoint = anglesSetpoint.yawRate;
    }
}

void FlightControllerModule::computeNextDroneState() {
    isThrottleDisabled      = receiverValues.chan[THROTTLE_CHAN] < MIN_THROTTLE_THRESOLD;
    isDisarmedChanTriggered = receiverValues.chan[DISARMED_CHAN] >= MAX_CHANNEL_VALUE;
    isRollChanMoved         = receiverValues.chan[ROLL_CHAN]  > MAX_CHANNEL_THRESOLD || receiverValues.chan[ROLL_CHAN]  < MIN_CHANNEL_THRESOLD;
    isPitchChanMoved        = receiverValues.chan[PITCH_CHAN] > MAX_CHANNEL_THRESOLD || receiverValues.chan[PITCH_CHAN] < MIN_CHANNEL_THRESOLD;
    isCameraControlEnable   = receiverValues.chan[CAMERA_INIT_POS_CHAN] == MAX_CHANNEL_VALUE;
    isPosHoldMode           = receiverValues.chan[MODE_SELECTION_CHAN] == POS_HOLD_CHAN_5_VALUE;
    isNavigationMode        = receiverValues.chan[MODE_SELECTION_CHAN] == NAVIGATION_CHAN_5_VALUE;
    isThrottleMoved         = receiverValues.chan[THROTTLE_CHAN] > MAX_THROTTLE_CHAN_THRESHOLD || receiverValues.chan[THROTTLE_CHAN] < MIN_THROTTLE_CHAN_THRESHOLD;
    isAltHoldleEnable       = isPosHoldMode || isNavigationMode;
}

void FlightControllerModule::processDroneState() {
    if(stateTelemtry == MANU) {
        state = MANU;
    } else if(isThrottleDisabled || isDisarmedChanTriggered || stateTelemtry == DISARMED) {
        rollPid.clear();
        pitchPid.clear();
        yawPid.clear();

        rateRollPid.clear();
        ratePitchPid.clear();
        rateYawPid.clear();

        altitudePid.clear();

        offsetYaw = attitudeValues.yaw;

        state = DISARMED;
    } else if((isRollChanMoved || isPitchChanMoved) && !isCameraControlEnable) {
        computeYaw();

        state = LEVEL;
    } else if(isPosHoldMode) {
        computeYaw();

        state = POS_HOLD;
    } else if(isNavigationMode) {
        computeYaw();

        state = NAVIGATION;
    } else {
        computeYaw();

        state = LEVEL;
    }

    altitudeSetpoint = state == LEVEL ? altitudeValues.alt : altitudeSetpoint;
}

void FlightControllerModule::computeOutputValues() {
    pidValues.out_roll  = rateRollPid.step(rollRateSetpoint, attitudeValues.gyroRateRoll);
    pidValues.out_pitch = ratePitchPid.step(pitchRateSetpoint, attitudeValues.gyroRatePitch);
    pidValues.out_yaw   = rateYawPid.step(yawRateSetpoint, -attitudeValues.gyroRateYaw);
    pidValues.out_alt   = isAltHoldleEnable ? altitudePid.step(altitudeSetpoint, altitudeValues.alt) : 0;
}

void FlightControllerModule::computeMotorsValues() {
#if NUMBER_OF_MOTORS == 4
    motorValues.mot[0] = receiverValues.chan[THROTTLE_CHAN] + pidValues.out_roll - pidValues.out_pitch - pidValues.out_yaw + pidValues.out_alt;
    motorValues.mot[1] = receiverValues.chan[THROTTLE_CHAN] - pidValues.out_roll - pidValues.out_pitch + pidValues.out_yaw + pidValues.out_alt;
    motorValues.mot[2] = receiverValues.chan[THROTTLE_CHAN] + pidValues.out_roll + pidValues.out_pitch + pidValues.out_yaw + pidValues.out_alt;
    motorValues.mot[3] = receiverValues.chan[THROTTLE_CHAN] - pidValues.out_roll + pidValues.out_pitch - pidValues.out_yaw + pidValues.out_alt;
    // motorValues.mot[0] = (isAutoThrottleEnable ? 1500 : receiverValues.chan[THROTTLE_CHAN]) + pidValues.out_roll - pidValues.out_pitch - pidValues.out_yaw + pidValues.out_alt;
    // motorValues.mot[1] = (isAutoThrottleEnable ? 1500 : receiverValues.chan[THROTTLE_CHAN]) - pidValues.out_roll - pidValues.out_pitch + pidValues.out_yaw + pidValues.out_alt;
    // motorValues.mot[2] = (isAutoThrottleEnable ? 1500 : receiverValues.chan[THROTTLE_CHAN]) + pidValues.out_roll + pidValues.out_pitch + pidValues.out_yaw + pidValues.out_alt;
    // motorValues.mot[3] = (isAutoThrottleEnable ? 1500 : receiverValues.chan[THROTTLE_CHAN]) - pidValues.out_roll + pidValues.out_pitch - pidValues.out_yaw + pidValues.out_alt;
#else //NUMBER_OF_MOTORS == 6
    motorValues.mot[0] = receiverValues.chan[THROTTLE_CHAN] + int16_t(pidValues.out_roll != 0 ? pidValues.out_roll * COEFF_ROLL_LEFT_RIGHT : 0)                                                                                    + int16_t(pidValues.out_yaw != 0 ? pidValues.out_yaw * C_YAW : 0) + pidValues.out_alt;
    motorValues.mot[1] = receiverValues.chan[THROTTLE_CHAN] + int16_t(pidValues.out_roll != 0 ? pidValues.out_roll * COEFF_ROLL_FRONT_REAR : 0) - int16_t(pidValues.out_pitch != 0 ? pidValues.out_pitch * C_PITCH_FRONT_REAR : 0) - int16_t(pidValues.out_yaw != 0 ? pidValues.out_yaw * C_YAW : 0) + pidValues.out_alt;
    motorValues.mot[2] = receiverValues.chan[THROTTLE_CHAN] - int16_t(pidValues.out_roll != 0 ? pidValues.out_roll * COEFF_ROLL_FRONT_REAR : 0) - int16_t(pidValues.out_pitch != 0 ? pidValues.out_pitch * C_PITCH_FRONT_REAR : 0) + int16_t(pidValues.out_yaw != 0 ? pidValues.out_yaw * C_YAW : 0) + pidValues.out_alt;
    motorValues.mot[3] = receiverValues.chan[THROTTLE_CHAN] - int16_t(pidValues.out_roll != 0 ? pidValues.out_roll * COEFF_ROLL_LEFT_RIGHT : 0)                                                                                    - int16_t(pidValues.out_yaw != 0 ? pidValues.out_yaw * C_YAW : 0) + pidValues.out_alt;
    motorValues.mot[4] = receiverValues.chan[THROTTLE_CHAN] - int16_t(pidValues.out_roll != 0 ? pidValues.out_roll * COEFF_ROLL_FRONT_REAR : 0) + int16_t(pidValues.out_pitch != 0 ? pidValues.out_pitch * C_PITCH_FRONT_REAR : 0) + int16_t(pidValues.out_yaw != 0 ? pidValues.out_yaw * C_YAW : 0) + pidValues.out_alt;
    motorValues.mot[5] = receiverValues.chan[THROTTLE_CHAN] + int16_t(pidValues.out_roll != 0 ? pidValues.out_roll * COEFF_ROLL_FRONT_REAR : 0) + int16_t(pidValues.out_pitch != 0 ? pidValues.out_pitch * C_PITCH_FRONT_REAR : 0) - int16_t(pidValues.out_yaw != 0 ? pidValues.out_yaw * C_YAW : 0) + pidValues.out_alt;
#endif
}

void FlightControllerModule::sendValuesToMotors() {
    if(state == DISARMED) {
        for (int i = 0; i < NUMBER_OF_MOTORS; i++)
            motorValues.mot[i] = MIN_THROTTLE_VALUE;
    }

    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++)
    {
        motorValues.mot[i] = constrain_(motorValues.mot[i], MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE);
    }

    motors.setMotors(motorValues);

    panPulse = constrain_(panPulse, MIN_SERVO_VALUE, MAX_SERVO_VALUE);
    tiltPulse = constrain_(tiltPulse, MIN_SERVO_VALUE, MID_SERVO_VALUE);

    motors.setServoPulse(panPulse, tiltPulse);
}
