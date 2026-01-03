#include "SensorIMU.h"

#define MPU6050_ADDR 0x68

SensorIMU::SensorIMU() {}

bool SensorIMU::begin() {
    Wire.begin(1, 3);
    
    Wire.setClock(400000);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  
    Wire.write(0);     
    if (Wire.endTransmission() != 0) return false;

    delay(50);

    const int N = 1000;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < N; i++) {
        readRaw();
        gx_sum += gx_raw;
        gy_sum += gy_raw;
        gz_sum += gz_raw;
        delay(1);
    }

    gyroOffsetX = gx_sum / (float)N;
    gyroOffsetY = gy_sum / (float)N;
    gyroOffsetZ = gz_sum / (float)N;

    return true;
}

void SensorIMU::readRaw() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);

    ax_raw = (Wire.read() << 8) | Wire.read();
    ay_raw = (Wire.read() << 8) | Wire.read();
    az_raw = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); 
    gx_raw = (Wire.read() << 8) | Wire.read();
    gy_raw = (Wire.read() << 8) | Wire.read();
    gz_raw = (Wire.read() << 8) | Wire.read();
}

void SensorIMU::processAccel() {
    ax = ax_raw / accelScale;
    ay = ay_raw / accelScale;
    az = az_raw / accelScale;
}

void SensorIMU::processGyro(float dt) {
    gx = (gx_raw - gyroOffsetX) / gyroScale;
    gy = (gy_raw - gyroOffsetY) / gyroScale;
    gz = (gz_raw - gyroOffsetZ) / gyroScale;
}

void SensorIMU::update(float dt) {
    readRaw();
    processAccel();
    processGyro(dt);

    float accel_roll  = atan2(ay, az) * 180.0f / PI;
    float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

    roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * accel_roll;
    pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * accel_pitch;
}