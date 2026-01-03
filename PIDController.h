#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp = 0, float ki = 0, float kd = 0);

    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float minOutput, float maxOutput);

    float compute(float setpoint, float measurement, float dt);

    void reset(); 

private:
    float kp, ki, kd;

    float minOut = -1.0f;
    float maxOut =  1.0f;

    float integrator = 0.0f;
    float prevError = 0.0f;

    float derivFilter = 0.0f;      
    const float derivativeAlpha = 0.7f; 
};
#endif
