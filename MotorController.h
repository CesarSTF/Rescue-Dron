#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

class MotorController {
public:
    MotorController();

    void begin();

    void arm();       // Permite activar motores
    void disarm();    // Bloquea motores
    bool isArmed() const;

    void setMotorSpeed(uint8_t motor, float power); // motor: 1–4, power: 0.0–1.0
    void killAllMotors(); // Apaga todo inmediatamente

private:
    static const uint8_t MOTOR_COUNT = 4;

    // Pines físicos
    uint8_t motorPins[MOTOR_COUNT] = {12, 13, 14, 15};

    // Canales PWM LEDC
    uint8_t pwmChannels[MOTOR_COUNT] = {0, 1, 2, 3};

    const uint32_t pwmFreq = 20000; // 20 kHz
    const uint8_t pwmResolution = 10; // 2^10 = 1024 niveles

    bool armed = false;

    void writePWM(uint8_t motorIndex, float power);
};

#endif
