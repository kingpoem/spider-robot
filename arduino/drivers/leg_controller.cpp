#include "leg_controller.h"

LegController::LegController() {
  // 初始化当前角度
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
      currentAngles[i][j] = 0;
      lastJointVelocities[i][j] = 0;
    }
    energyRecoveryEnabled[i] = false;
    recoveredEnergy[i] = 0.0;
    lastUpdateTime[i] = 0;
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
      
      // 更新关节速度（用于能量回收计算）
      float dt = 0.01;  // 假设100Hz控制频率
      lastJointVelocities[i][j] = (commands.targetAngles[i][j] - currentAngles[i][j]) / dt;
    }
    
    // 处理能量回收
    if (energyRecoveryEnabled[i]) {
      processEnergyRecovery(i);
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

// 能量回收功能实现
void LegController::enableEnergyRecovery(int legIndex) {
  if (legIndex >= 0 && legIndex < NUM_LEGS) {
    energyRecoveryEnabled[legIndex] = true;
    lastUpdateTime[legIndex] = millis();
  }
}

void LegController::disableEnergyRecovery(int legIndex) {
  if (legIndex >= 0 && legIndex < NUM_LEGS) {
    energyRecoveryEnabled[legIndex] = false;
  }
}

float LegController::getRecoveredEnergy(int legIndex) {
  if (legIndex >= 0 && legIndex < NUM_LEGS) {
    return recoveredEnergy[legIndex];
  }
  return 0.0;
}

float LegController::getTotalRecoveredEnergy() {
  float total = 0.0;
  for (int i = 0; i < NUM_LEGS; i++) {
    total += recoveredEnergy[i];
  }
  return total;
}

void LegController::processEnergyRecovery(int legIndex) {
  if (legIndex < 0 || legIndex >= NUM_LEGS) return;
  
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - lastUpdateTime[legIndex];
  
  if (dt == 0) return;  // 避免除零
  
  float dt_sec = dt / 1000.0;  // 转换为秒
  
  // 计算每条腿的总动能（基于关节速度）
  // 简化模型：假设每个关节有等效转动惯量
  const float jointInertia = 0.01;  // 关节等效转动惯量 (kg·m²)
  const float energyRecoveryEfficiency = 0.3;  // 能量回收效率（30%）
  
  float totalKineticEnergy = 0.0;
  
  for (int j = 0; j < JOINTS_PER_LEG; j++) {
    // 将角度速度转换为弧度/秒
    float angularVelocityRad = radians(lastJointVelocities[legIndex][j]);
    
    // 计算动能: E = 0.5 * I * ω²
    float kineticEnergy = 0.5 * jointInertia * angularVelocityRad * angularVelocityRad;
    totalKineticEnergy += kineticEnergy;
  }
  
  // 计算可回收的能量（考虑效率）
  float recoverableEnergy = totalKineticEnergy * energyRecoveryEfficiency * dt_sec;
  
  // 累加回收的能量
  recoveredEnergy[legIndex] += recoverableEnergy;
  
  // 在实际硬件中，这里会将能量存储到电池或超级电容中
  // 对于Portenta H7，可以通过ADC读取能量回收模块的电压/电流
  
  lastUpdateTime[legIndex] = currentTime;
}

