#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include "MotorController.h"
#include "SensorIMU.h"
#include "PIDController.h"

class FlightController {
public:
    FlightController(MotorController* motors, SensorIMU* imu);

    void begin();

    void arm();
    void disarm();

    void setThrottle(float t);   // 0.0 → 1.0
    void setRollInput(float r);  // -1.0 → 1.0 (control desde slider)
    void setPitchInput(float p); // -1.0 → 1.0

    void update(float dt);

public:
    MotorController* motors;
    SensorIMU* imu;

    PIDController rollPID;
    PIDController pitchPID;

    bool armed = false;

    float throttle   = 0.0f;
    float rollInput  = 0.0f;
    float pitchInput = 0.0f;

    void mixAndSend(float rollCorr, float pitchCorr);
};

#endif
