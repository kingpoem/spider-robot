/**
 * 系统配置文件
 * 定义硬件引脚、常量、参数
 */

#ifndef CONFIG_H
#define CONFIG_H

// ========== 硬件配置 ==========
#define NUM_LEGS 6              // 腿部数量
#define JOINTS_PER_LEG 3        // 每条腿的关节数
#define NUM_SERVOS (NUM_LEGS * JOINTS_PER_LEG)

// 伺服电机引脚定义 (腿编号, 关节编号)
// 关节: 0=髋关节, 1=大腿关节, 2=小腿关节
const int SERVO_PINS[NUM_LEGS][JOINTS_PER_LEG] = {
  {2, 3, 4},    // 腿0
  {5, 6, 7},    // 腿1
  {8, 9, 10},   // 腿2
  {11, 12, 13}, // 腿3
  {22, 23, 24}, // 腿4
  {25, 26, 27}  // 腿5
};

// 足端爪控制引脚
const int CLAW_PINS[NUM_LEGS] = {28, 29, 30, 31, 32, 33};
const int NAIL_PINS[NUM_LEGS] = {34, 35, 36, 37, 38, 39};

// FPGA通信引脚
#define FPGA_CS_PIN 53
#define FPGA_IRQ_PIN 52

// IMU引脚
#define IMU_SDA_PIN 20
#define IMU_SCL_PIN 21

// 力传感器引脚 (I2C地址)
const int FORCE_SENSOR_ADDRS[NUM_LEGS] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45};

// ========== 机械参数 ==========
// 腿部尺寸 (mm)
#define LEG_COXA_LENGTH 50      // 髋关节长度
#define LEG_FEMUR_LENGTH 100    // 大腿长度
#define LEG_TIBIA_LENGTH 120    // 小腿长度

// 步态参数
#define MIN_STEP_LENGTH 10      // 最小步幅 (cm)
#define MAX_STEP_LENGTH 30      // 最大步幅 (cm)
#define MIN_STEP_HEIGHT 5       // 最小步高 (cm)
#define MAX_STEP_HEIGHT 20      // 最大步高 (cm)

// 关节限制 (度)
#define JOINT_MIN_ANGLE -90
#define JOINT_MAX_ANGLE 90

// ========== 控制参数 ==========
#define CONTROL_FREQ 100        // 控制频率 (Hz)
#define CONTROL_PERIOD_US (1000000 / CONTROL_FREQ)

// 姿态控制增益
#define POSTURE_KP 0.5          // 姿态比例增益
#define POSTURE_LEG_KP 0.3      // 腿部调整增益

// 能量回收阈值
#define ENERGY_RECOVERY_THRESHOLD 50  // 力阈值 (N)

// ========== 数据结构 ==========
struct Vector3D {
  float x, y, z;
  
  float magnitude() {
    return sqrt(x*x + y*y + z*z);
  }
  
  Vector3D operator+(const Vector3D& other) {
    return {x + other.x, y + other.y, z + other.z};
  }
  
  Vector3D operator-(const Vector3D& other) {
    return {x - other.x, y - other.y, z - other.z};
  }
  
  Vector3D operator*(float scalar) {
    return {x * scalar, y * scalar, z * scalar};
  }
};

struct Orientation {
  float roll;   // 横滚角
  float pitch;  // 俯仰角
  float yaw;    // 偏航角
};

struct RobotState {
  Orientation orientation;
  Vector3D angularVelocity;
  Vector3D linearAcceleration;
  float jointAngles[NUM_LEGS][JOINTS_PER_LEG];
  Vector3D footForces[NUM_LEGS];
  bool footContacts[NUM_LEGS];
  Vector3D footPositions[NUM_LEGS];
};

struct GaitState {
  float stepLength;
  float stepHeight;
  float gaitSpeed;
  int currentPhase;
};

struct PostureAdjustment {
  float rollCorrection;
  float pitchCorrection;
  float heightAdjustment;
  float* legAdjustments;
};

struct GaitCommands {
  float targetAngles[NUM_LEGS][JOINTS_PER_LEG];
  float targetVelocities[NUM_LEGS][JOINTS_PER_LEG];
};

#endif // CONFIG_H

