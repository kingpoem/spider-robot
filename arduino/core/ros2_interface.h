/**
 * ROS2接口
 * 通过WiFi与ROS2节点通信
 * 使用自定义协议（可以后续改为ROS2串行协议）
 */

#ifndef ROS2_INTERFACE_H
#define ROS2_INTERFACE_H

#include <Arduino.h>
#include <WiFi.h>
#include "../core/config.h"

class ROS2Interface {
public:
  ROS2Interface();
  void init();
  void receiveCommands();
  void publishStatus();
  bool isConnected();
  
  // 接收的命令
  struct ROS2Commands {
    bool enableMovement;
    float targetVelocity[3];  // x, y, angular
    float targetStepLength;
    float targetStepHeight;
    int gaitType;  // 0=tripod, 1=wave, 2=ripple
  } commands;
  
private:
  WiFiClient client;
  bool connected;
  const char* ssid = "YOUR_WIFI_SSID";
  const char* password = "YOUR_WIFI_PASSWORD";
  const char* ros2Host = "192.168.1.100";  // ROS2机器IP
  int ros2Port = 8888;
  
  void connectWiFi();
  void connectToROS2();
  void parseCommand(String data);
  String serializeStatus();
};

#endif // ROS2_INTERFACE_H

