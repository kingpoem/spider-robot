#include "force_sensor.h"

ForceSensor::ForceSensor() {
  initialized = false;
  contactThreshold = 10.0;  // 接触阈值 (N)
  lastForce = {0, 0, 0};
}

void ForceSensor::init(int legIndex) {
  this->legIndex = legIndex;
  this->address = FORCE_SENSOR_ADDRS[legIndex];
  
  // 初始化I2C通信
  Wire.begin();
  delay(10);
  
  // 检查传感器是否存在
  Wire.beginTransmission(address);
  if (Wire.endTransmission() == 0) {
    initialized = true;
    calibrate();
  } else {
    Serial.print("警告: 力传感器 ");
    Serial.print(legIndex);
    Serial.println(" 未找到");
  }
}

void ForceSensor::calibrate() {
  // 校准传感器（读取零位）
  // 实际实现需要根据具体传感器型号
  delay(100);
}

Vector3D ForceSensor::readForce() {
  if (!initialized) {
    return {0, 0, 0};
  }
  
  // 读取力传感器数据（假设使用I2C接口）
  Wire.beginTransmission(address);
  Wire.write(0x00);  // 数据寄存器地址
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6, true);
  
  if (Wire.available() >= 6) {
    int16_t fx = Wire.read() << 8 | Wire.read();
    int16_t fy = Wire.read() << 8 | Wire.read();
    int16_t fz = Wire.read() << 8 | Wire.read();
    
    // 转换为力值（单位：N）
    // 需要根据具体传感器规格调整转换系数
    lastForce.x = fx / 1000.0;
    lastForce.y = fy / 1000.0;
    lastForce.z = fz / 1000.0;
  }
  
  return lastForce;
}

bool ForceSensor::isContact() {
  return getContactForce() > contactThreshold;
}

float ForceSensor::getContactForce() {
  return lastForce.magnitude();
}

