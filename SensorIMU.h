#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <Arduino.h>
#include <Wire.h>

class SensorIMU {
public:
    SensorIMU();
    
    bool begin();          
    void update(float dt); 

    float getRoll()  const { return roll;  }
    float getPitch() const { return pitch; }

    float getGyroX() const { return gx; }
    float getGyroY() const { return gy; }
    float getGyroZ() const { return gz; }

private:
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    float gyroOffsetX = 0.0f;
    float gyroOffsetY = 0.0f;
    float gyroOffsetZ = 0.0f;

    float ax, ay, az;
    float gx, gy, gz;

    float roll  = 0.0f;
    float pitch = 0.0f;

    const float accelScale = 16384.0f; 
    const float gyroScale  = 131.0f;   

    const float alpha = 0.96f;

    void readRaw();
    void processAccel();
    void processGyro(float dt);
};

#endif
