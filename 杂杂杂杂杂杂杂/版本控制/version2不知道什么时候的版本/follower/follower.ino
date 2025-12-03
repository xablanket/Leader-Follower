
#include "Motors.h"         
#include "PID.h"            
#include "Kinematics.h"  
#include "LineSensors.h"
#include <math.h>
#include "Encoders.h"

// ========== 基础组件 ==========
Motors_c motors;    
Kinematics_c pose; 
LineSensors_c line_sensors;

PID_c distance_pid;  // 用于距离控制的PID（控制前进速度）
PID_c direction_pid; // 用于方向控制的PID（控制转向）

// ========== 速度估计变量 ==========
unsigned long speed_est_ts = 0;
#define SPEED_EST_MS 10
long last_e0 = 0;
long last_e1 = 0;
float speed_right = 0.0f;
float speed_left  = 0.0f;

// ========== Leader跟随参数 ==========
const bool DEBUG_SERIAL = true;

// ========== 方向控制参数（使用所有传感器加权平均）==========
const float SIDE_SENSOR_THRESHOLD = 20.0f; // 传感器检测阈值（相对于背景）

// ========== PID控制参数 ==========
// 距离PID参数（控制前进速度）
const float DISTANCE_KP = 1.2f;         // 比例增益：对距离误差的响应速度
const float DISTANCE_KI = 0.01f;        // 积分增益：消除稳态误差
const float DISTANCE_KD = 0.5f;         // 微分增益：抑制超调

// 方向PID参数（控制转向）- 加权平均法优化
const float DIRECTION_KP = 0.5f;        // 比例增益：降低以适应加权平均（更平滑）
const float DIRECTION_KI = 0.01f;       // 积分增益：稍微提高以消除稳态误差
const float DIRECTION_KD = 0.6f;        // 微分增益：提高以抑制震荡

// ========== 距离控制参数（6-10cm目标距离）==========
// IR信号阈值（这些值需要根据实际测试调整）
// ⚠️ 注意：在某些环境下，Leader会"遮挡"环境光，导致读数下降（负值）
// 因此使用绝对值来判断距离：|IR信号|越大 = 距离越近
const float IR_TARGET_DISTANCE = 25.0f;  // 目标距离的IR信号值（|IR|，约8cm）
const float IR_TOO_FAR = 10.0f;          // 太远时的IR信号变化（绝对值，检测阈值）
const float IR_MAX_DISTANCE = 60.0f;     // 最大允许距离对应的IR值（用于限制）

// 背景IR值（启动时校准）
float background_ir[NUM_SENSORS] = {0};
float background_avg = 0.0f;

// ========== 读取IR信号强度 ==========
// 返回5个线传感器的平均IR强度（用于距离判断）
float getIRSignalStrength() {
  line_sensors.readSensorsADC();
  float total = 0.0f;
  for (int i = 0; i < NUM_SENSORS; i++) {
    total += line_sensors.readings[i];
  }
  return total / NUM_SENSORS;
}

// ========== 获取中间传感器的IR强度（用于距离测量）==========
// 使用中间三个传感器（1,2,3）的平均值，更稳定
float getCenterIRStrength() {
  line_sensors.readSensorsADC();
  float center_avg = (line_sensors.readings[1] + line_sensors.readings[2] + line_sensors.readings[3]) / 3.0f;
  return center_avg;
}

// ========== 使用所有传感器计算方向（加权平均法）==========
// 返回值：负数=leader在左侧，正数=leader在右侧，0=居中
// 类似于巡线的加权平均，计算IR信号的"重心"位置
float getDirectionFromAllSensors() {
  line_sensors.readSensorsADC();
  
  // 计算每个传感器相对于背景的信号变化（绝对值）
  float signals[NUM_SENSORS];
  float total_signal = 0.0f;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    signals[i] = fabs(line_sensors.readings[i] - background_ir[i]);
    total_signal += signals[i];
  }
  
  // 如果总信号太弱，说明没有检测到Leader
  if (total_signal < SIDE_SENSOR_THRESHOLD * 2) {
    return 0.0f;  // 信号太弱，保持直行
  }
  
  // 计算加权位置（重心）
  // 传感器位置权重：0=-2, 1=-1, 2=0, 3=+1, 4=+2
  // 负值=左侧，正值=右侧，0=中间
  float weighted_sum = 0.0f;
  float weights[NUM_SENSORS] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    weighted_sum += signals[i] * weights[i];
  }
  
  // 计算方向误差（归一化）
  // 除以total_signal进行归一化，范围大约在[-2, +2]
  float direction_error = weighted_sum / total_signal;
  
  // 放大误差值，使其更容易控制
  // 乘以50使其范围变为[-100, +100]左右
  return direction_error * 50.0f;
}

// ========== 更新位姿 ==========
void updatePose(){
  static unsigned long pose_ts = 0;
  if ( millis() - pose_ts >= 10 ) { 
    pose_ts = millis(); 
    pose.update(); 
  }
}

// ========== PID跟随Leader（平滑距离和方向控制）==========
void followLeaderImproved() {
  // 控制更新频率（每50ms）
  static unsigned long control_ts = 0;
  if (millis() - control_ts < 50) {
    return;  // 未到更新时间
  }
  control_ts = millis();
  
  // ========== 1. 读取传感器数据 ==========
  // 方向：使用所有5个传感器计算"重心"位置
  float direction_error = getDirectionFromAllSensors();
  
  // 距离：使用中间传感器的平均IR强度
  float center_ir = getCenterIRStrength();
  float ir_signal = center_ir - (background_ir[1] + background_ir[2] + background_ir[3]) / 3.0f;
  
  // ⚠️ 使用绝对值判断距离（因为Leader可能降低而非增强读数）
  float ir_signal_abs = fabs(ir_signal);
  
  // ========== 2. 检查是否检测到Leader ==========
  if (ir_signal_abs < IR_TOO_FAR) {
    // 信号变化太弱，未检测到Leader
    motors.setPWM(0, 0);
    distance_pid.reset();
    direction_pid.reset();
    
    if (DEBUG_SERIAL) {
      static unsigned long debug_ts = 0;
      if (millis() - debug_ts >= 500) {
        debug_ts = millis();
        Serial.println("未检测到Leader - 停止");
        Serial.print("IR信号: "); Serial.print(ir_signal, 2);
        Serial.print(" | |IR|: "); Serial.print(ir_signal_abs, 2);
        Serial.print(" | 阈值: "); Serial.println(IR_TOO_FAR, 2);
      }
    }
    return;
  }
  
  // ========== 3. 距离PID控制（平滑前进速度）==========
  // 目标：保持IR信号在IR_TARGET_DISTANCE
  // ir_signal_abs越大=越近，越小=越远
  // 误差：当前距离 - 目标距离（用IR值表示）
  float distance_error = ir_signal_abs - IR_TARGET_DISTANCE;
  
  // 使用PID计算前进速度调整量
  // 如果太近(distance_error>0)，PID输出负值，减速或后退
  // 如果太远(distance_error<0)，PID输出正值，加速前进
  float forward_speed = distance_pid.update(IR_TARGET_DISTANCE, ir_signal_abs);
  
  // 限制前进速度范围
  forward_speed = constrain(forward_speed, -50, 50);
  
  // 状态判断（用于调试）
  String distance_status = "";
  if (fabs(distance_error) < 5.0f) {
    distance_status = "最佳距离";
  } else if (distance_error > 0) {
    distance_status = "稍近";
  } else {
    distance_status = "稍远";
  }
  
  // ========== 4. 方向PID控制（平滑转向）==========
  // 目标：方向误差为0（Leader在正前方）
  // direction_error: 负=左侧，正=右侧
  float turn_adjustment = 0.0f;
  String direction_status = "";
  
  if (direction_error == 0.0f) {
    // Leader在正前方，不需要转向
    turn_adjustment = 0.0f;
    direction_status = "直行";
    direction_pid.reset();
  } else {
    // 使用PID计算转向调整量
    // 目标是0，当前是direction_error
    // ⚠️ 加负号：direction_error<0(左侧有信号)时，PID产生正输出，取反后变负值→左转
    turn_adjustment = -direction_pid.update(0.0f, direction_error);
    
    // 限制转向范围
    turn_adjustment = constrain(turn_adjustment, -40, 40);
    
    if (direction_error < 0) {
      direction_status = "左转";
    } else {
      direction_status = "右转";
    }
  }
  
  // ========== 5. 转向时适当减速（提高稳定性）==========
  float turn_factor = fabs(turn_adjustment) / 40.0f;  // 0-1
  if (turn_factor > 0.5f) {
    forward_speed *= 0.8f;  // 大幅转向时减速20%
  }
  
  // ========== 6. 计算左右轮PWM ==========
  // turn_adjustment为负时左转（左轮慢，右轮快）
  // turn_adjustment为正时右转（左轮快，右轮慢）
  float left_pwm = forward_speed + turn_adjustment;
  float right_pwm = forward_speed - turn_adjustment;
  
  // 限制PWM范围
  left_pwm = constrain(left_pwm, -60, 60);
  right_pwm = constrain(right_pwm, -60, 60);
  
  // ========== 7. 设置电机 ==========
  motors.setPWM((int)left_pwm, (int)right_pwm);
  
  // ========== 8. 调试输出 ==========
  if (DEBUG_SERIAL) {
    static unsigned long debug_ts = 0;
    if (millis() - debug_ts >= 200) {
      debug_ts = millis();
      
      // 输出所有传感器原始读数
      Serial.print("传感器原始 | ");
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print("S"); Serial.print(i); Serial.print(":");
        Serial.print((int)line_sensors.readings[i]);
        if (i < NUM_SENSORS - 1) Serial.print(" ");
      }
      Serial.println();
      
      // 输出信号变化（方便看哪个传感器检测到Leader）
      Serial.print("信号变化 | ");
      for (int i = 0; i < NUM_SENSORS; i++) {
        float signal_change = fabs(line_sensors.readings[i] - background_ir[i]);
        Serial.print("S"); Serial.print(i); Serial.print(":");
        Serial.print((int)signal_change);
        if (i < NUM_SENSORS - 1) Serial.print(" ");
      }
      Serial.println();
      
      // 输出控制状态
      Serial.print("PID控制 | |IR|: "); Serial.print(ir_signal_abs, 1);
      Serial.print(" | 目标: "); Serial.print(IR_TARGET_DISTANCE, 1);
      Serial.print(" | 距离误差: "); Serial.print(distance_error, 1);
      Serial.print(" | 方向误差: "); Serial.print(direction_error, 1);
      Serial.println();
      
      Serial.print("输出 | 前进: "); Serial.print((int)forward_speed);
      Serial.print(" | 转向: "); Serial.print((int)turn_adjustment);
      Serial.print(" | PWM L/R: "); Serial.print((int)left_pwm);
      Serial.print("/"); Serial.print((int)right_pwm);
      Serial.println();
      
      // 输出状态
      Serial.print("状态 | 距离: "); Serial.print(distance_status);
      Serial.print(" | 方向: "); Serial.print(direction_status);
      Serial.println();
      Serial.println("---");
    }
  }
}

// ========== 校准背景IR值 ==========
void calibrateBackground() {
  Serial.println("\n开始校准背景IR值...");
  Serial.println("请确保Leader尚未启动或距离较远");
  delay(2000);
  
  // 采样10次取平均
  for (int sample = 0; sample < 10; sample++) {
    line_sensors.readSensorsADC();
    for (int i = 0; i < NUM_SENSORS; i++) {
      background_ir[i] += line_sensors.readings[i];
    }
    delay(50);
  }
  
  // 计算平均值
  background_avg = 0.0f;
  for (int i = 0; i < NUM_SENSORS; i++) {
    background_ir[i] /= 10.0f;
    background_avg += background_ir[i];
  }
  background_avg /= NUM_SENSORS;
  
  Serial.println("背景IR校准完成：");
  Serial.print("背景平均值: "); Serial.println(background_avg, 2);
  Serial.print("各传感器背景值: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(background_ir[i], 1); Serial.print(" ");
  }
  Serial.println();
}

// ========== SETUP ==========
void setup() {
  // 初始化串口
  Serial.begin(9600);
  delay(1000);
  Serial.println("\n\n*** Follower - Leader跟随程序 ***");
  Serial.println("模式：原地旋转跟随（黑色桌面优化）");

  // 初始化电机
  motors.initialise();

  // 初始化编码器
  setupEncoder0();
  setupEncoder1();

  // 初始化位姿
  pose.initialise(0, 0, 0);
  
  // 初始化线传感器（用于检测Leader的IR信号）
  line_sensors.initialiseForADC();
  
  // ⚠️ 重要：initialiseForADC()会设置EMIT_PIN为OUTPUT+HIGH
  // 必须在之后重新设置为INPUT来关闭IR发射
  // 根据官方文档：INPUT模式会关闭IR LED
  pinMode(EMIT_PIN, INPUT);  // INPUT = 关闭IR发射，只接收外部IR源
  
  Serial.println("✓ 线传感器已初始化");
  Serial.println("✓ EMIT_PIN已设置为INPUT（关闭IR发射）");
  
  // 校准背景IR值
  calibrateBackground();
  
  // 初始化PID控制器
  Serial.println("\n初始化PID控制器...");
  
  // 距离PID：目标是保持IR_TARGET_DISTANCE的IR信号值
  distance_pid.initialise(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);
  distance_pid.setOutputLimits(-50.0f, 50.0f);  // 限制输出范围
  distance_pid.setOutputFilter(0.7f);           // 平滑输出，0.7=较平滑
  distance_pid.reset();
  
  // 方向PID：目标是方向误差为0
  direction_pid.initialise(DIRECTION_KP, DIRECTION_KI, DIRECTION_KD);
  direction_pid.setOutputLimits(-40.0f, 40.0f); // 限制转向范围
  direction_pid.setOutputFilter(0.6f);          // 平滑转向，0.6=更平滑
  direction_pid.reset();
  
  Serial.println("✓ PID控制器已初始化");
  Serial.print("  - 距离PID: Kp="); Serial.print(DISTANCE_KP, 2);
  Serial.print(", Ki="); Serial.print(DISTANCE_KI, 3);
  Serial.print(", Kd="); Serial.println(DISTANCE_KD, 2);
  Serial.print("  - 方向PID: Kp="); Serial.print(DIRECTION_KP, 2);
  Serial.print(", Ki="); Serial.print(DIRECTION_KI, 3);
  Serial.print(", Kd="); Serial.println(DIRECTION_KD, 2);
  
  Serial.println("\n系统初始化完成");
  Serial.println("========================================");
  Serial.println("控制模式：PID平滑跟随");
  Serial.println("方向控制（改进版）：");
  Serial.println("  - 使用所有5个传感器计算IR\"重心\"");
  Serial.println("  - 类似巡线的加权平均法");
  Serial.println("  - 即使侧着也能自动回正");
  Serial.println("  - PID平滑转向，消除抖动");
  Serial.println("距离控制：");
  Serial.print("  - 目标IR信号值: "); Serial.print(IR_TARGET_DISTANCE, 1);
  Serial.println(" (约6-10cm)");
  Serial.println("  - PID平滑速度控制");
  Serial.println("  - 自动保持目标距离");
  Serial.print("  - 检测阈值: |IR| > "); Serial.println(IR_TOO_FAR, 1);
  Serial.println("========================================");
  Serial.println("💡 PID优势：");
  Serial.println("  ✓ 平滑加速/减速，无突变");
  Serial.println("  ✓ 平滑转向，无左右摇摆");
  Serial.println("  ✓ 自动消除稳态误差");
  Serial.println("  ✓ 响应速度和稳定性平衡");
  Serial.println("========================================");
  Serial.println("Follower已准备好，等待检测Leader信号...\n");
  delay(2000);
}

// ========== LOOP ==========
void loop() {
  // 更新位姿
  updatePose();

  // 执行改进的跟随（边缘传感器方向 + 距离控制）
  followLeaderImproved();
  
  delay(10);
}
