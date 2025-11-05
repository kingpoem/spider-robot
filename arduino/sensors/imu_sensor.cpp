#include "imu_sensor.h"

// MPU6050寄存器地址
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

IMUSensor::IMUSensor() {
  initialized = false;
  sensorType = 0;  // 默认MPU6050
  orientation = {0, 0, 0};
  angularVelocity = {0, 0, 0};
  linearAcceleration = {0, 0, 0};
}

bool IMUSensor::init() {
  Wire.begin();
  delay(100);
  
  // 尝试检测MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    initMPU6050();
    sensorType = 0;
    initialized = true;
    return true;
  }
  
  // 可以添加其他传感器检测
  return false;
}

void IMUSensor::initMPU6050() {
  // 唤醒MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();
  
  delay(100);
}

void IMUSensor::update() {
  if (!initialized) return;
  
  if (sensorType == 0) {
    readMPU6050();
  } else if (sensorType == 1) {
    readBNO085();
  }
  
  // 简单的互补滤波器计算姿态
  // 实际应用中应使用更复杂的滤波器（如卡尔曼滤波）
  static float alpha = 0.98;
  static Vector3D accelFiltered = {0, 0, 0};
  
  accelFiltered.x = alpha * accelFiltered.x + (1 - alpha) * linearAcceleration.x;
  accelFiltered.y = alpha * accelFiltered.y + (1 - alpha) * linearAcceleration.y;
  accelFiltered.z = alpha * accelFiltered.z + (1 - alpha) * linearAcceleration.z;
  
  // 计算姿态（简化版本）
  orientation.roll = atan2(accelFiltered.y, accelFiltered.z) * 180.0 / PI;
  orientation.pitch = atan2(-accelFiltered.x, sqrt(accelFiltered.y*accelFiltered.y + accelFiltered.z*accelFiltered.z)) * 180.0 / PI;
  // yaw需要使用磁力计或GPS
}

void IMUSensor::readMPU6050() {
  // 读取加速度计
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  int16_t accelX = Wire.read() << 8 | Wire.read();
  int16_t accelY = Wire.read() << 8 | Wire.read();
  int16_t accelZ = Wire.read() << 8 | Wire.read();
  
  // 转换为g单位（±2g量程）
  linearAcceleration.x = accelX / 16384.0 * 9.81;
  linearAcceleration.y = accelY / 16384.0 * 9.81;
  linearAcceleration.z = accelZ / 16384.0 * 9.81;
  
  // 读取陀螺仪
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  
  // 转换为度/秒（±250°/s量程）
  angularVelocity.x = gyroX / 131.0;
  angularVelocity.y = gyroY / 131.0;
  angularVelocity.z = gyroZ / 131.0;
}

void IMUSensor::readBNO085() {
  // BNO085实现（需要根据具体传感器库实现）
  // 这里留空，实际使用时需要添加BNO085库
}

Orientation IMUSensor::getOrientation() {
  return orientation;
}

Vector3D IMUSensor::getAngularVelocity() {
  return angularVelocity;
}

Vector3D IMUSensor::getLinearAcceleration() {
  return linearAcceleration;
}

