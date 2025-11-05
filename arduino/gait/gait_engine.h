/**
 * 步态引擎
 * 基础步态生成和调整
 */

#ifndef GAIT_ENGINE_H
#define GAIT_ENGINE_H

#include "../core/config.h"

class GaitEngine {
public:
  GaitEngine();
  void init();
  void applyPostureAdjustment(PostureAdjustment adjustment);
  void setGaitParameters(float stepLength, float stepHeight, float speed);
  GaitCommands generateGaitCommands(float phase);
  
private:
  GaitState currentGait;
  PostureAdjustment currentAdjustment;
  
  void generateTripodGait(float phase, GaitCommands& commands);
  void generateWaveGait(float phase, GaitCommands& commands);
  void generateRippleGait(float phase, GaitCommands& commands);
};

#endif // GAIT_ENGINE_H

