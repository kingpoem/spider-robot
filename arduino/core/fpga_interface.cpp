#include "fpga_interface.h"

FPGAInterface::FPGAInterface() {
  spiInitialized = false;
}

void FPGAInterface::init() {
  // 初始化SPI通信
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);  // 10MHz
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  
  pinMode(FPGA_CS_PIN, OUTPUT);
  digitalWrite(FPGA_CS_PIN, HIGH);
  
  delay(100);
  spiInitialized = true;
  
  Serial.println("FPGA接口初始化完成");
}

void FPGAInterface::updateGait() {
  if (!spiInitialized) return;
  
  // 从FPGA读取步态命令
  uint8_t cmd = 0x01;  // 读取步态命令
  sendCommand(cmd, nullptr, 0);
  
  // 接收数据
  uint8_t buffer[sizeof(GaitCommands)];
  receiveData(buffer, sizeof(GaitCommands));
  
  // 解析数据（需要根据实际协议调整）
  memcpy(&gaitCommands, buffer, sizeof(GaitCommands));
}

GaitCommands FPGAInterface::getGaitCommands() {
  return gaitCommands;
}

void FPGAInterface::enableEnergyRecovery(int legIndex) {
  uint8_t cmd = 0x02;  // 能量回收命令
  uint8_t data = legIndex;
  sendCommand(cmd, &data, 1);
}

void FPGAInterface::sendSensorData(RobotState state) {
  uint8_t cmd = 0x03;  // 发送传感器数据
  uint8_t* data = (uint8_t*)&state;
  sendCommand(cmd, data, sizeof(RobotState));
}

void FPGAInterface::sendCommand(uint8_t cmd, uint8_t* data, int dataLen) {
  digitalWrite(FPGA_CS_PIN, LOW);
  delayMicroseconds(10);
  
  SPI.transfer(cmd);
  delayMicroseconds(10);
  
  if (data && dataLen > 0) {
    for (int i = 0; i < dataLen; i++) {
      SPI.transfer(data[i]);
    }
  }
  
  delayMicroseconds(10);
  digitalWrite(FPGA_CS_PIN, HIGH);
}

void FPGAInterface::receiveData(uint8_t* data, int dataLen) {
  digitalWrite(FPGA_CS_PIN, LOW);
  delayMicroseconds(10);
  
  for (int i = 0; i < dataLen; i++) {
    data[i] = SPI.transfer(0x00);
  }
  
  delayMicroseconds(10);
  digitalWrite(FPGA_CS_PIN, HIGH);
}

