#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <Arduino.h>
#include <Wire.h>

class SensorIMU {
public:
    SensorIMU();
    
    bool begin();          // Inicializa I2C + calibra offsets
    void update(float dt); // Lee sensor + aplica filtro complementario

    float getRoll()  const { return roll;  }
    float getPitch() const { return pitch; }

    float getGyroX() const { return gx; }
    float getGyroY() const { return gy; }
    float getGyroZ() const { return gz; }

private:
    // Datos crudos
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    // Ajuste de offsets del giroscopio
    float gyroOffsetX = 0.0f;
    float gyroOffsetY = 0.0f;
    float gyroOffsetZ = 0.0f;

    // Convertidos a G y °/s
    float ax, ay, az;
    float gx, gy, gz;

    // Ángulos finales
    float roll  = 0.0f;
    float pitch = 0.0f;

    // Constantes de escala del MPU6050
    const float accelScale = 16384.0f; // ±2g
    const float gyroScale  = 131.0f;   // ±250°/s

    // Filtro complementario (mezcla gyro/accel)
    const float alpha = 0.96f;

    // Métodos internos
    void readRaw();
    void processAccel();
    void processGyro(float dt);
};

#endif
