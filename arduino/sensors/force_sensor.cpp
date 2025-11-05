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
  
  // 读取多次采样，计算零位偏移
  const int samples = 100;
  Vector3D sum = {0, 0, 0};
  
  for (int i = 0; i < samples; i++) {
    // 读取原始数据
    Wire.beginTransmission(address);
    Wire.write(0x00);  // 数据寄存器地址
    Wire.endTransmission(false);
    Wire.requestFrom(address, 6, true);
    
    if (Wire.available() >= 6) {
      int16_t fx = Wire.read() << 8 | Wire.read();
      int16_t fy = Wire.read() << 8 | Wire.read();
      int16_t fz = Wire.read() << 8 | Wire.read();
      
      sum.x += fx;
      sum.y += fy;
      sum.z += fz;
    }
    
    delay(10);
  }
  
  // 计算平均零位（实际应用中应保存为偏移量）
  // 这里简化处理，实际应保存偏移并在读取时减去
  lastForce = {0, 0, 0};  // 重置为零，假设传感器已校准
  
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

