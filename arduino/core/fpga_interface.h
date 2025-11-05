/**
 * FPGA接口
 * 用于与FPGA协处理器通信，获取高频率步态计算结果
 */

#ifndef FPGA_INTERFACE_H
#define FPGA_INTERFACE_H

#include <Arduino.h>
#include <SPI.h>
#include "../core/config.h"

class FPGAInterface {
public:
  FPGAInterface();
  void init();
  void updateGait();
  GaitCommands getGaitCommands();
  void enableEnergyRecovery(int legIndex);
  void sendSensorData(RobotState state);
  
private:
  GaitCommands gaitCommands;
  bool spiInitialized;
  
  void spiTransfer(uint8_t* txData, uint8_t* rxData, int length);
  void sendCommand(uint8_t cmd, uint8_t* data, int dataLen);
  void receiveData(uint8_t* data, int dataLen);
};

#endif // FPGA_INTERFACE_H

