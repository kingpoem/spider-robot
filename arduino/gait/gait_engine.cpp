#include "gait_engine.h"
#include <math.h>

GaitEngine::GaitEngine() {
  currentGait.stepLength = 15.0;
  currentGait.stepHeight = 10.0;
  currentGait.gaitSpeed = 1.0;
  currentGait.currentPhase = 0;
  
  currentAdjustment.rollCorrection = 0;
  currentAdjustment.pitchCorrection = 0;
  currentAdjustment.heightAdjustment = 0;
}

void GaitEngine::init() {
  // 初始化步态参数
}

void GaitEngine::applyPostureAdjustment(PostureAdjustment adjustment) {
  currentAdjustment = adjustment;
}

void GaitEngine::setGaitParameters(float stepLength, float stepHeight, float speed) {
  currentGait.stepLength = constrain(stepLength, MIN_STEP_LENGTH, MAX_STEP_LENGTH);
  currentGait.stepHeight = constrain(stepHeight, MIN_STEP_HEIGHT, MAX_STEP_HEIGHT);
  currentGait.gaitSpeed = speed;
}

GaitCommands GaitEngine::generateGaitCommands(float phase) {
  GaitCommands commands;
  
  // 根据当前步态类型生成命令
  // 这里使用三脚架步态作为示例
  generateTripodGait(phase, commands);
  
  // 应用姿态调整
  // 这里可以调整目标角度
  
  return commands;
}

void GaitEngine::generateTripodGait(float phase, GaitCommands& commands) {
  // 三脚架步态：三条腿同时抬起，三条腿同时支撑
  // 腿0, 2, 4 为一组
  // 腿1, 3, 5 为一组
  
  float stepLength = currentGait.stepLength / 10.0;  // 转换为cm
  float stepHeight = currentGait.stepHeight / 10.0;
  
  for (int i = 0; i < NUM_LEGS; i++) {
    bool isLiftGroup = (i % 2 == 0);
    float legPhase = phase;
    
    if (!isLiftGroup) {
      legPhase = fmod(phase + 0.5, 1.0);  // 相位偏移0.5
    }
    
    // 计算足端目标位置
    Vector3D targetPos;
    
    if (legPhase < 0.5) {
      // 支撑阶段
      targetPos.x = -stepLength * (1 - 2 * legPhase);
      targetPos.y = 0;
      targetPos.z = 0;
    } else {
      // 摆动阶段
      float swingPhase = (legPhase - 0.5) * 2;
      targetPos.x = stepLength * (1 - 2 * swingPhase);
      targetPos.y = 0;
      targetPos.z = stepHeight * sin(PI * swingPhase);
    }
    
    // 应用姿态调整
    targetPos.z += currentAdjustment.heightAdjustment;
    
    // 这里应该调用逆运动学计算关节角度
    // 为简化，这里直接设置角度
    commands.targetAngles[i][0] = 0;
    commands.targetAngles[i][1] = 45;
    commands.targetAngles[i][2] = -45;
    
    commands.targetVelocities[i][0] = 0;
    commands.targetVelocities[i][1] = 0;
    commands.targetVelocities[i][2] = 0;
  }
}

void GaitEngine::generateWaveGait(float phase, GaitCommands& commands) {
  // 波浪步态实现
  // 每条腿依次抬起
}

void GaitEngine::generateRippleGait(float phase, GaitCommands& commands) {
  // 涟漪步态实现
}

