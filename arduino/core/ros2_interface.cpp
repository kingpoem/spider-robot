#include "ros2_interface.h"

ROS2Interface::ROS2Interface() {
  connected = false;
  commands.enableMovement = false;
  commands.targetVelocity[0] = 0;
  commands.targetVelocity[1] = 0;
  commands.targetVelocity[2] = 0;
  commands.targetStepLength = 15.0;
  commands.targetStepHeight = 10.0;
  commands.gaitType = 0;
}

void ROS2Interface::init() {
  // 初始化WiFi
  connectWiFi();
  
  // 连接到ROS2节点
  connectToROS2();
}

void ROS2Interface::connectWiFi() {
  Serial.print("连接WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi已连接, IP地址: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi连接失败");
  }
}

void ROS2Interface::connectToROS2() {
  Serial.print("连接到ROS2节点: ");
  Serial.print(ros2Host);
  Serial.print(":");
  Serial.println(ros2Port);
  
  if (client.connect(ros2Host, ros2Port)) {
    connected = true;
    Serial.println("已连接到ROS2节点");
  } else {
    connected = false;
    Serial.println("连接ROS2节点失败");
  }
}

void ROS2Interface::receiveCommands() {
  if (!connected || !client.connected()) {
    // 尝试重连
    if (WiFi.status() == WL_CONNECTED) {
      connectToROS2();
    }
    return;
  }
  
  // 检查是否有数据
  if (client.available()) {
    String data = client.readStringUntil('\n');
    parseCommand(data);
  }
}

void ROS2Interface::publishStatus() {
  if (!connected || !client.connected()) {
    return;
  }
  
  String status = serializeStatus();
  client.println(status);
}

bool ROS2Interface::isConnected() {
  return connected && client.connected();
}

void ROS2Interface::parseCommand(String data) {
  // 简单的命令解析
  // 格式: "CMD:PARAM1:PARAM2:..."
  
  if (data.startsWith("MOVE")) {
    // MOVE:VX:VY:VANGULAR
    int idx1 = data.indexOf(':');
    int idx2 = data.indexOf(':', idx1 + 1);
    int idx3 = data.indexOf(':', idx2 + 1);
    
    commands.enableMovement = true;
    commands.targetVelocity[0] = data.substring(idx1 + 1, idx2).toFloat();
    commands.targetVelocity[1] = data.substring(idx2 + 1, idx3).toFloat();
    commands.targetVelocity[2] = data.substring(idx3 + 1).toFloat();
  } else if (data.startsWith("STOP")) {
    commands.enableMovement = false;
  } else if (data.startsWith("GAIT")) {
    // GAIT:STEPLEN:STEPHEIGHT:TYPE
    int idx1 = data.indexOf(':');
    int idx2 = data.indexOf(':', idx1 + 1);
    int idx3 = data.indexOf(':', idx2 + 1);
    
    commands.targetStepLength = data.substring(idx1 + 1, idx2).toFloat();
    commands.targetStepHeight = data.substring(idx2 + 1, idx3).toFloat();
    commands.gaitType = data.substring(idx3 + 1).toInt();
  }
}

String ROS2Interface::serializeStatus() {
  // 序列化机器人状态
  // 格式: "STATUS:ROLL:PITCH:YAW:X:Y:Z:..."
  
  extern RobotState robotState;  // 声明外部变量
  
  String status = "STATUS:";
  status += String(robotState.orientation.roll);
  status += ":";
  status += String(robotState.orientation.pitch);
  status += ":";
  status += String(robotState.orientation.yaw);
  
  // 添加更多状态信息...
  
  return status;
}

