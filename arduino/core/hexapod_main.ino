/**
 * 蜘蛛型多足机器人主控制程序
 * 核心功能: 实时控制、传感器读取、与ROS2通信
 */

#include "config.h"
#include "../drivers/leg_controller.h"
#include "../sensors/imu_sensor.h"
#include "../sensors/force_sensor.h"
#include "../gait/gait_engine.h"
#include "ros2_interface.h"

// 全局对象
LegController legController;
IMUSensor imu;
ForceSensor forceSensors[NUM_LEGS];
GaitEngine gaitEngine;
ROS2Interface ros2;

// 状态变量
RobotState robotState;
GaitState gaitState;
bool emergencyStop = false;

void setup() {
  Serial.begin(115200);
  Serial.println("=== 蜘蛛型多足机器人启动 ===");
  
  // 初始化硬件
  initHardware();
  
  // 初始化传感器
  initSensors();
  
  // 初始化腿部控制器
  initLegs();
  
  // 设置步态引擎的腿部控制器引用
  gaitEngine.setLegController(&legController);
  
  // 初始化步态引擎
  gaitEngine.init();
  
  // 初始化ROS2接口
  ros2.init();
  
  // 等待系统就绪
  delay(1000);
  
  Serial.println("系统初始化完成");
}

void loop() {
  // 检查紧急停止
  if (emergencyStop) {
    handleEmergencyStop();
    return;
  }
  
  // 更新传感器数据
  updateSensors();
  
  // 从ROS2接收命令
  ros2.receiveCommands();
  
  // 计算姿态控制
  computePostureControl();
  
  // 生成步态命令（Portenta H7本地计算）
  static float gaitPhase = 0.0;
  gaitPhase += 0.01;  // 假设100Hz控制频率
  if (gaitPhase >= 1.0) gaitPhase -= 1.0;
  
  GaitCommands gaitCommands = gaitEngine.generateGaitCommands(gaitPhase);
  
  // 执行步态
  legController.executeGait(gaitCommands);
  
  // 发送状态到ROS2
  ros2.publishStatus();
  
  // 能量回收处理
  handleEnergyRecovery();
  
  // 控制循环延迟（目标频率100Hz）
  delayMicroseconds(10000);
}

void initHardware() {
  // 初始化I2C总线
  Wire.begin();
  
  // 初始化伺服电机控制引脚
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
      pinMode(SERVO_PINS[i][j], OUTPUT);
    }
  }
  
  // 初始化足端爪控制
  for (int i = 0; i < NUM_LEGS; i++) {
    pinMode(CLAW_PINS[i], OUTPUT);
    pinMode(NAIL_PINS[i], OUTPUT);
  }
}

void initSensors() {
  // 初始化IMU
  if (!imu.init()) {
    Serial.println("警告: IMU初始化失败");
  }
  
  // 初始化力传感器
  for (int i = 0; i < NUM_LEGS; i++) {
    forceSensors[i].init(i);
  }
  
  Serial.println("传感器初始化完成");
}

void initLegs() {
  legController.init();
  Serial.println("腿部控制器初始化完成");
}

void updateSensors() {
  // 更新IMU数据
  imu.update();
  robotState.orientation = imu.getOrientation();
  robotState.angularVelocity = imu.getAngularVelocity();
  robotState.linearAcceleration = imu.getLinearAcceleration();
  
  // 更新力传感器数据
  for (int i = 0; i < NUM_LEGS; i++) {
    robotState.footForces[i] = forceSensors[i].readForce();
    robotState.footContacts[i] = forceSensors[i].isContact();
  }
  
  // 更新腿部位置（从编码器或伺服反馈）
  legController.updateJointPositions(robotState.jointAngles);
}

void computePostureControl() {
  // 基于IMU和力传感器数据进行姿态控制
  // 计算重心偏移
  Vector3D cogOffset = calculateCenterOfGravityOffset();
  
  // 计算所需调整
  PostureAdjustment adjustment = calculatePostureAdjustment(
    robotState.orientation,
    cogOffset,
    robotState.footForces
  );
  
  // 应用调整到步态
  gaitEngine.applyPostureAdjustment(adjustment);
}

// executeGait函数已移至主循环中，使用本地步态引擎计算

void handleEmergencyStop() {
  // 停止所有运动
  legController.stopAll();
  
  // 保持当前位置
  legController.holdPosition();
  
  Serial.println("紧急停止激活");
}

void handleEnergyRecovery() {
  // 基于IMU惯性感知启动能量回收
  // 检测下坡、减速或脚步落地时的能量回收机会
  Vector3D accel = robotState.linearAcceleration;
  Vector3D angularVel = robotState.angularVelocity;
  Orientation orient = robotState.orientation;
  
  // 计算下坡角度（俯仰角）
  float pitchAngle = orient.pitch;
  bool isDownhill = pitchAngle < -5.0;  // 下坡阈值：-5度
  
  // 检测减速（负加速度）
  float deceleration = sqrt(accel.x * accel.x + accel.y * accel.y);
  bool isDecelerating = deceleration > 2.0;  // 减速阈值：2 m/s²
  
  // 检测脚步落地时的冲击
  for (int i = 0; i < NUM_LEGS; i++) {
    bool shouldRecover = false;
    
    // 条件1：下坡且脚接触地面
    if (isDownhill && robotState.footContacts[i]) {
      shouldRecover = true;
    }
    
    // 条件2：减速且脚接触地面且有足够力
    if (isDecelerating && robotState.footContacts[i] && 
        robotState.footForces[i].magnitude() > ENERGY_RECOVERY_THRESHOLD) {
      shouldRecover = true;
    }
    
    // 条件3：脚落地时的垂直冲击（负Z加速度）
    if (accel.z < -3.0 && robotState.footContacts[i]) {
      shouldRecover = true;
    }
    
    if (shouldRecover) {
      // 激活该腿的能量回收
      legController.enableEnergyRecovery(i);
    } else {
      // 关闭能量回收
      legController.disableEnergyRecovery(i);
    }
  }
}

Vector3D calculateCenterOfGravityOffset() {
  Vector3D cog(0, 0, 0);
  float totalForce = 0;
  
  for (int i = 0; i < NUM_LEGS; i++) {
    if (robotState.footContacts[i]) {
      Vector3D footPosition = legController.getFootPosition(i);
      float force = robotState.footForces[i].magnitude();
      cog.x += footPosition.x * force;
      cog.y += footPosition.y * force;
      cog.z += footPosition.z * force;
      totalForce += force;
    }
  }
  
  if (totalForce > 0) {
    cog.x /= totalForce;
    cog.y /= totalForce;
    cog.z /= totalForce;
  }
  
  return cog;
}

PostureAdjustment calculatePostureAdjustment(
  Orientation orientation,
  Vector3D cogOffset,
  Vector3D footForces[NUM_LEGS]
) {
  PostureAdjustment adjustment;
  
  // 计算倾斜角度
  float roll = orientation.roll;
  float pitch = orientation.pitch;
  
  // 计算所需调整
  adjustment.rollCorrection = -roll * POSTURE_KP;
  adjustment.pitchCorrection = -pitch * POSTURE_KP;
  
  // 基于重心偏移调整
  adjustment.heightAdjustment = -cogOffset.z * POSTURE_KP;
  
  // 基于力分布调整
  adjustment.legAdjustments = calculateLegAdjustments(footForces);
  
  return adjustment;
}

float* calculateLegAdjustments(Vector3D footForces[NUM_LEGS]) {
  static float adjustments[NUM_LEGS];
  
  // 计算平均力
  float avgForce = 0;
  for (int i = 0; i < NUM_LEGS; i++) {
    avgForce += footForces[i].magnitude();
  }
  avgForce /= NUM_LEGS;
  
  // 计算每个腿的调整量
  for (int i = 0; i < NUM_LEGS; i++) {
    float forceDiff = footForces[i].magnitude() - avgForce;
    adjustments[i] = -forceDiff * POSTURE_LEG_KP;
  }
  
  return adjustments;
}

