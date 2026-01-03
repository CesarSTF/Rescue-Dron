#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd) {}

void PIDController::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PIDController::setOutputLimits(float minOutput, float maxOutput) {
    this->minOut = minOutput;
    this->maxOut = maxOutput;
}

void PIDController::reset() {
    integrator = 0.0f;
    prevError = 0.0f;
    derivFilter = 0.0f;
}

float PIDController::compute(float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    float P = kp * error;

    integrator += error * dt * ki;

    if (integrator > maxOut) integrator = maxOut;
    if (integrator < minOut) integrator = minOut;

    float I = integrator;

    float rawD = (error - prevError) / dt;
    derivFilter = derivativeAlpha * derivFilter + (1.0f - derivativeAlpha) * rawD;

    float D = kd * derivFilter;

    prevError = error;

    float output = P + I + D;

    if (output > maxOut) output = maxOut;
    if (output < minOut) output = minOut;

    return output;
}
