/**
 * 力传感器接口
 * 每个足端安装力传感器用于接触检测和力反馈
 */

#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "../core/config.h"

class ForceSensor {
public:
  ForceSensor();
  void init(int legIndex);
  Vector3D readForce();
  bool isContact();
  float getContactForce();
  
private:
  int legIndex;
  int address;
  bool initialized;
  Vector3D lastForce;
  float contactThreshold;
  
  void calibrate();
};

#endif // FORCE_SENSOR_H

