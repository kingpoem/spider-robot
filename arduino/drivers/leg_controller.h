/**
 * 腿部控制器
 * 负责控制每条腿的伺服电机和计算逆运动学
 */

#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>
#include "../core/config.h"

class LegController {
public:
  LegController();
  void init();
  void updateJointPositions(float angles[NUM_LEGS][JOINTS_PER_LEG]);
  void executeGait(GaitCommands commands);
  void stopAll();
  void holdPosition();
  Vector3D getFootPosition(int legIndex);
  Vector3D forwardKinematics(int legIndex, float angles[JOINTS_PER_LEG]);
  bool inverseKinematics(int legIndex, Vector3D targetPosition, float outputAngles[JOINTS_PER_LEG]);
  
private:
  Servo servos[NUM_LEGS][JOINTS_PER_LEG];
  float currentAngles[NUM_LEGS][JOINTS_PER_LEG];
  Vector3D legBasePositions[NUM_LEGS];  // 每条腿的基座位置
  
  void setServoAngle(int legIndex, int jointIndex, float angle);
  float constrainAngle(float angle);
  void calculateLegBasePositions();
};

#endif // LEG_CONTROLLER_H

