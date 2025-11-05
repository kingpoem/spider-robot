#include "gait_engine.h"
#include "../drivers/leg_controller.h"
#include <math.h>

GaitEngine::GaitEngine() {
  currentGait.stepLength = 15.0;
  currentGait.stepHeight = 10.0;
  currentGait.gaitSpeed = 1.0;
  currentGait.currentPhase = 0;
  
  currentAdjustment.rollCorrection = 0;
  currentAdjustment.pitchCorrection = 0;
  currentAdjustment.heightAdjustment = 0;
  
  legController = nullptr;
}

void GaitEngine::init() {
}

void GaitEngine::setLegController(LegController* legCtrl) {
  legController = legCtrl;
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
  
  switch (currentGait.currentPhase) {
    case 0:
      generateTripodGait(phase, commands);
      break;
    case 1:
      generateWaveGait(phase, commands);
      break;
    case 2:
      generateRippleGait(phase, commands);
      break;
    default:
      generateTripodGait(phase, commands);
      break;
  }
  
  return commands;
}

void GaitEngine::generateTripodGait(float phase, GaitCommands& commands) {
  float stepLength = currentGait.stepLength / 10.0;
  float stepHeight = currentGait.stepHeight / 10.0;
  
  for (int i = 0; i < NUM_LEGS; i++) {
    bool isLiftGroup = (i % 2 == 0);
    float legPhase = phase;
    
    if (!isLiftGroup) {
      legPhase = fmod(phase + 0.5, 1.0);
    }
    
    Vector3D targetPos;
    
    if (legPhase < 0.5) {
      targetPos.x = -stepLength * (1 - 2 * legPhase);
      targetPos.y = 0;
      targetPos.z = 0;
    } else {
      float swingPhase = (legPhase - 0.5) * 2;
      targetPos.x = stepLength * (1 - 2 * swingPhase);
      targetPos.y = 0;
      targetPos.z = stepHeight * sin(PI * swingPhase);
    }
    
    targetPos.z += currentAdjustment.heightAdjustment;
    
    if (legController != nullptr) {
      float angles[JOINTS_PER_LEG];
      if (legController->inverseKinematics(i, targetPos, angles)) {
        commands.targetAngles[i][0] = angles[0];
        commands.targetAngles[i][1] = angles[1];
        commands.targetAngles[i][2] = angles[2];
      } else {
        commands.targetAngles[i][0] = 0;
        commands.targetAngles[i][1] = 45;
        commands.targetAngles[i][2] = -45;
      }
    } else {
      commands.targetAngles[i][0] = 0;
      commands.targetAngles[i][1] = 45;
      commands.targetAngles[i][2] = -45;
    }
    
    static float lastAngles[NUM_LEGS][JOINTS_PER_LEG] = {0};
    float dt = 0.01;
    commands.targetVelocities[i][0] = (commands.targetAngles[i][0] - lastAngles[i][0]) / dt;
    commands.targetVelocities[i][1] = (commands.targetAngles[i][1] - lastAngles[i][1]) / dt;
    commands.targetVelocities[i][2] = (commands.targetAngles[i][2] - lastAngles[i][2]) / dt;
    
    lastAngles[i][0] = commands.targetAngles[i][0];
    lastAngles[i][1] = commands.targetAngles[i][1];
    lastAngles[i][2] = commands.targetAngles[i][2];
  }
}

void GaitEngine::generateWaveGait(float phase, GaitCommands& commands) {
  float stepLength = currentGait.stepLength / 10.0;
  float stepHeight = currentGait.stepHeight / 10.0;
  float phaseOffset = 1.0 / NUM_LEGS;
  
  for (int i = 0; i < NUM_LEGS; i++) {
    float legPhase = fmod(phase + i * phaseOffset, 1.0);
    Vector3D targetPos;
    
    if (legPhase < 0.5) {
      targetPos.x = -stepLength * (1 - 2 * legPhase);
      targetPos.y = 0;
      targetPos.z = 0;
    } else {
      float swingPhase = (legPhase - 0.5) * 2;
      targetPos.x = stepLength * (1 - 2 * swingPhase);
      targetPos.y = 0;
      targetPos.z = stepHeight * sin(PI * swingPhase);
    }
    
    targetPos.z += currentAdjustment.heightAdjustment;
    
    if (legController != nullptr) {
      float angles[JOINTS_PER_LEG];
      if (legController->inverseKinematics(i, targetPos, angles)) {
        commands.targetAngles[i][0] = angles[0];
        commands.targetAngles[i][1] = angles[1];
        commands.targetAngles[i][2] = angles[2];
      } else {
        commands.targetAngles[i][0] = 0;
        commands.targetAngles[i][1] = 45;
        commands.targetAngles[i][2] = -45;
      }
    } else {
      commands.targetAngles[i][0] = 0;
      commands.targetAngles[i][1] = 45;
      commands.targetAngles[i][2] = -45;
    }
    
    static float lastAngles[NUM_LEGS][JOINTS_PER_LEG] = {0};
    float dt = 0.01;
    commands.targetVelocities[i][0] = (commands.targetAngles[i][0] - lastAngles[i][0]) / dt;
    commands.targetVelocities[i][1] = (commands.targetAngles[i][1] - lastAngles[i][1]) / dt;
    commands.targetVelocities[i][2] = (commands.targetAngles[i][2] - lastAngles[i][2]) / dt;
    
    lastAngles[i][0] = commands.targetAngles[i][0];
    lastAngles[i][1] = commands.targetAngles[i][1];
    lastAngles[i][2] = commands.targetAngles[i][2];
  }
}

void GaitEngine::generateRippleGait(float phase, GaitCommands& commands) {
  float stepLength = currentGait.stepLength / 10.0;
  float stepHeight = currentGait.stepHeight / 10.0;
  float groupPhaseOffset = 1.0 / 3.0;
  
  for (int i = 0; i < NUM_LEGS; i++) {
    int group = i % 2;
    int legInGroup = i / 2;
    float groupPhase = fmod(phase + group * 0.5, 1.0);
    float legPhase = fmod(groupPhase + legInGroup * groupPhaseOffset, 1.0);
    
    Vector3D targetPos;
    
    if (legPhase < 0.5) {
      targetPos.x = -stepLength * (1 - 2 * legPhase);
      targetPos.y = 0;
      targetPos.z = 0;
    } else {
      float swingPhase = (legPhase - 0.5) * 2;
      targetPos.x = stepLength * (1 - 2 * swingPhase);
      targetPos.y = 0;
      targetPos.z = stepHeight * sin(PI * swingPhase);
    }
    
    targetPos.z += currentAdjustment.heightAdjustment;
    
    if (legController != nullptr) {
      float angles[JOINTS_PER_LEG];
      if (legController->inverseKinematics(i, targetPos, angles)) {
        commands.targetAngles[i][0] = angles[0];
        commands.targetAngles[i][1] = angles[1];
        commands.targetAngles[i][2] = angles[2];
      } else {
        commands.targetAngles[i][0] = 0;
        commands.targetAngles[i][1] = 45;
        commands.targetAngles[i][2] = -45;
      }
    } else {
      commands.targetAngles[i][0] = 0;
      commands.targetAngles[i][1] = 45;
      commands.targetAngles[i][2] = -45;
    }
    
    static float lastAngles[NUM_LEGS][JOINTS_PER_LEG] = {0};
    float dt = 0.01;
    commands.targetVelocities[i][0] = (commands.targetAngles[i][0] - lastAngles[i][0]) / dt;
    commands.targetVelocities[i][1] = (commands.targetAngles[i][1] - lastAngles[i][1]) / dt;
    commands.targetVelocities[i][2] = (commands.targetAngles[i][2] - lastAngles[i][2]) / dt;
    
    lastAngles[i][0] = commands.targetAngles[i][0];
    lastAngles[i][1] = commands.targetAngles[i][1];
    lastAngles[i][2] = commands.targetAngles[i][2];
  }
}

