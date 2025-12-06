#include "FlightController.h"

FlightController::FlightController(MotorController* motors, SensorIMU* imu)
    : motors(motors), imu(imu) {}

void FlightController::begin() {
    rollPID.setTunings(1.8f, 0.0f, 0.35f);   // puedes afinarlos después
    pitchPID.setTunings(1.8f, 0.0f, 0.35f);

    rollPID.setOutputLimits(-0.3f, 0.3f);
    pitchPID.setOutputLimits(-0.3f, 0.3f);
}

void FlightController::arm() {
    motors->arm();
    rollPID.reset();
    pitchPID.reset();
    armed = true;
}

void FlightController::disarm() {
    motors->disarm();
    armed = false;
}

// ─────────────────────────────
//      INPUTS DE CONTROL
// ─────────────────────────────
void FlightController::setThrottle(float t) {
    throttle = constrain(t, 0.0f, 1.0f);
}

void FlightController::setRollInput(float r) {
    rollInput = constrain(r, -1.0f, 1.0f);
}

void FlightController::setPitchInput(float p) {
    pitchInput = constrain(p, -1.0f, 1.0f);
}


// ─────────────────────────────
//         LOOP PRINCIPAL
// ─────────────────────────────
void FlightController::update(float dt) {
    imu->update(dt);  

    if (!armed) {
        return;  // No mover motores mientras esté desarmado
    }

    float currentRoll  = imu->getRoll();
    float currentPitch = imu->getPitch();

    float rollCorr  = rollPID.compute(rollInput * 20.0f, currentRoll, dt);
    float pitchCorr = pitchPID.compute(pitchInput * 20.0f, currentPitch, dt);

    mixAndSend(rollCorr, pitchCorr);
}


// ─────────────────────────────
//     MEZCLA QUAD-X REAL  
// ─────────────────────────────
void FlightController::mixAndSend(float rollCorr, float pitchCorr) {

    float m1 = throttle + pitchCorr + rollCorr;  // Front Left  (Motor 1)
    float m2 = throttle + pitchCorr - rollCorr;  // Front Right (Motor 2)
    float m3 = throttle - pitchCorr + rollCorr;  // Rear Left   (Motor 3)
    float m4 = throttle - pitchCorr - rollCorr;  // Rear Right  (Motor 4)

    m1 = constrain(m1, 0.0f, 1.0f);
    m2 = constrain(m2, 0.0f, 1.0f);
    m3 = constrain(m3, 0.0f, 1.0f);
    m4 = constrain(m4, 0.0f, 1.0f);

    motors->setMotorSpeed(1, m1);
    motors->setMotorSpeed(2, m2);
    motors->setMotorSpeed(3, m3);
    motors->setMotorSpeed(4, m4);
}
