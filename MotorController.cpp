#include "MotorController.h"

MotorController::MotorController() {}

void MotorController::begin() {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        ledcAttach(motorPins[i], pwmFreq, pwmResolution);
        
        writePWM(i, 0.0f); // Iniciar apagados
    }

    armed = false;
}

// ---------------------- ARM / DISARM ----------------------

void MotorController::arm() {
    armed = true;
}

void MotorController::disarm() {
    armed = false;
    killAllMotors();
}

bool MotorController::isArmed() const {
    return armed;
}

// ---------------------- CONTROL DE MOTORES ----------------------

void MotorController::setMotorSpeed(uint8_t motor, float power) {
    if (motor < 1 || motor > MOTOR_COUNT) return;

    uint8_t index = motor - 1;

    if (!armed) {
        writePWM(index, 0.0f);
        return;
    }

    power = constrain(power, 0.0f, 1.0f);

    writePWM(index, power);
}

void MotorController::writePWM(uint8_t motorIndex, float power) {
    uint32_t duty = (uint32_t)(power * ((1 << pwmResolution) - 1));    
    ledcWrite(motorPins[motorIndex], duty);
}

void MotorController::killAllMotors() {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        writePWM(i, 0.0f);
    }
}