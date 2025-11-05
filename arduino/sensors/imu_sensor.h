/**
 * IMU传感器接口
 * 支持MPU6050、BNO085等
 */

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "../core/config.h"

class IMUSensor {
public:
  IMUSensor();
  bool init();
  void update();
  Orientation getOrientation();
  Vector3D getAngularVelocity();
  Vector3D getLinearAcceleration();
  
private:
  Orientation orientation;
  Vector3D angularVelocity;
  Vector3D linearAcceleration;
  
  bool initialized;
  int sensorType;  // 0=MPU6050, 1=BNO085
  
  // MPU6050相关
  void initMPU6050();
  void readMPU6050();
  
  // BNO085相关
  void initBNO085();
  void readBNO085();
};

#endif // IMU_SENSOR_H

