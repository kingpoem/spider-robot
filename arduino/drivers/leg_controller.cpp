#include "leg_controller.h"

LegController::LegController() {
  // 初始化当前角度
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
      currentAngles[i][j] = 0;
    }
  }
}

void LegController::init() {
  // 初始化所有伺服电机
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
      servos[i][j].attach(SERVO_PINS[i][j]);
      servos[i][j].write(90);  // 初始位置（中点）
      delay(50);
    }
  }
  
  // 计算腿部基座位置
  calculateLegBasePositions();
}

void LegController::calculateLegBasePositions() {
  // 假设机器人中心为原点，腿部均匀分布在一个圆上
  float radius = 150;  // 基座半径 (mm)
  float angleStep = 2 * PI / NUM_LEGS;
  
  for (int i = 0; i < NUM_LEGS; i++) {
    float angle = i * angleStep;
    legBasePositions[i].x = radius * cos(angle);
    legBasePositions[i].y = radius * sin(angle);
    legBasePositions[i].z = 0;
  }
}

void LegController::updateJointPositions(float angles[NUM_LEGS][JOINTS_PER_LEG]) {
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
      currentAngles[i][j] = angles[i][j];
    }
  }
}

void LegController::executeGait(GaitCommands commands) {
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
      setServoAngle(i, j, commands.targetAngles[i][j]);
    }
  }
}

void LegController::setServoAngle(int legIndex, int jointIndex, float angle) {
  angle = constrainAngle(angle);
  int servoAngle = map(angle, JOINT_MIN_ANGLE, JOINT_MAX_ANGLE, 0, 180);
  servos[legIndex][jointIndex].write(servoAngle);
  currentAngles[legIndex][jointIndex] = angle;
}

float LegController::constrainAngle(float angle) {
  if (angle < JOINT_MIN_ANGLE) return JOINT_MIN_ANGLE;
  if (angle > JOINT_MAX_ANGLE) return JOINT_MAX_ANGLE;
  return angle;
}

void LegController::stopAll() {
  // 停止所有运动，保持当前位置
  holdPosition();
}

void LegController::holdPosition() {
  // 保持当前角度
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
      setServoAngle(i, j, currentAngles[i][j]);
    }
  }
}

Vector3D LegController::getFootPosition(int legIndex) {
  return forwardKinematics(legIndex, currentAngles[legIndex]);
}

Vector3D LegController::forwardKinematics(int legIndex, float angles[JOINTS_PER_LEG]) {
  // 正运动学计算
  // 角度转换为弧度
  float theta1 = radians(angles[0]);  // 髋关节
  float theta2 = radians(angles[1]);  // 大腿关节
  float theta3 = radians(angles[2]);  // 小腿关节
  
  // 基座位置
  Vector3D base = legBasePositions[legIndex];
  
  // 计算末端位置
  Vector3D footPos;
  
  // 髋关节旋转
  float x1 = LEG_COXA_LENGTH * cos(theta1);
  float y1 = LEG_COXA_LENGTH * sin(theta1);
  
  // 大腿关节
  float x2 = x1 + LEG_FEMUR_LENGTH * cos(theta1) * cos(theta2);
  float y2 = y1 + LEG_FEMUR_LENGTH * sin(theta1) * cos(theta2);
  float z2 = LEG_FEMUR_LENGTH * sin(theta2);
  
  // 小腿关节
  float theta23 = theta2 + theta3;
  footPos.x = x2 + LEG_TIBIA_LENGTH * cos(theta1) * cos(theta23);
  footPos.y = y2 + LEG_TIBIA_LENGTH * sin(theta1) * cos(theta23);
  footPos.z = z2 + LEG_TIBIA_LENGTH * sin(theta23);
  
  // 加上基座位置
  footPos.x += base.x;
  footPos.y += base.y;
  footPos.z += base.z;
  
  return footPos;
}

bool LegController::inverseKinematics(int legIndex, Vector3D targetPosition, float outputAngles[JOINTS_PER_LEG]) {
  // 逆运动学计算
  Vector3D base = legBasePositions[legIndex];
  
  // 相对位置
  float x = targetPosition.x - base.x;
  float y = targetPosition.y - base.y;
  float z = targetPosition.z - base.z;
  
  // 髋关节角度
  float theta1 = atan2(y, x);
  
  // 投影到髋关节平面
  float r = sqrt(x*x + y*y) - LEG_COXA_LENGTH;
  float d = sqrt(r*r + z*z);
  
  // 检查是否可达
  float maxReach = LEG_FEMUR_LENGTH + LEG_TIBIA_LENGTH;
  if (d > maxReach) {
    return false;  // 目标位置不可达
  }
  
  // 大腿和小腿角度
  float cosTheta3 = (LEG_FEMUR_LENGTH*LEG_FEMUR_LENGTH + LEG_TIBIA_LENGTH*LEG_TIBIA_LENGTH - d*d) 
                    / (2 * LEG_FEMUR_LENGTH * LEG_TIBIA_LENGTH);
  
  if (cosTheta3 < -1 || cosTheta3 > 1) {
    return false;  // 无效角度
  }
  
  float theta3 = acos(cosTheta3);
  
  float alpha = atan2(z, r);
  float beta = atan2(LEG_TIBIA_LENGTH * sin(theta3), LEG_FEMUR_LENGTH + LEG_TIBIA_LENGTH * cos(theta3));
  float theta2 = alpha - beta;
  
  // 转换为度
  outputAngles[0] = degrees(theta1);
  outputAngles[1] = degrees(theta2);
  outputAngles[2] = degrees(theta3);
  
  // 检查角度限制
  for (int i = 0; i < JOINTS_PER_LEG; i++) {
    if (outputAngles[i] < JOINT_MIN_ANGLE || outputAngles[i] > JOINT_MAX_ANGLE) {
      return false;
    }
  }
  
  return true;
}

