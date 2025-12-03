/*
 * Follower机器人 - 整合版本
 * 功能：
 * 1. 按钮A选择Line Sensor模式
 * 2. 按钮B选择Bump Sensor模式
 * 3. 使用增量式PID进行速度控制和方向纠正
 * 4. 自动保持与Leader 5cm的距离
 */

#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "LineSensors.h"
#include "BumpSensors.h"
#include <math.h>

// ===== 按钮引脚定义 =====
#define BUTTON_A 14  // 3Pi+ 按钮A
#define BUTTON_B 30  // 3Pi+ 按钮B
#define BUTTON_C 17  // 3Pi+ 按钮C (可选)

// ===== IR发射控制引脚 =====
#define EMIT_PIN 11

// ===== 运行模式 =====
enum RunMode {
  MODE_STANDBY,      // 待机模式
  MODE_LINE_SENSOR,  // Line Sensor模式
  MODE_BUMP_SENSOR   // Bump Sensor模式
};

RunMode current_mode = MODE_STANDBY;

// ===== 基础组件 =====
Motors_c motors;
Kinematics_c pose;
LineSensors_c line_sensors;
BumpSensors_c bump_sensors;

// ===== PID控制器 =====
PID_c speed_pid_left;    // 左轮速度PID
PID_c speed_pid_right;   // 右轮速度PID
PID_c turn_pid;          // 旋转控制PID
PID_c distance_pid;      // 距离控制PID
PID_c direction_pid;     // 方向纠正PID

// ===== 距离保持参数 =====
const float TARGET_DISTANCE_CM = 5.0f;   // 目标距离：5cm
const float DISTANCE_TOLERANCE = 1.0f;   // 距离容差：±1cm
const float MIN_SIGNAL_CHANGE = 100.0f;  // 最小信号变化（检测Leader）

// ===== 速度控制参数 =====
const float MAX_FOLLOW_SPEED = 25.0f;    // 最大跟随速度（PWM）
const float TURN_MAX_SPEED = 20.0f;      // 最大旋转速度（PWM）

// ===== Line Sensor模式参数 =====
const float LINE_FOLLOW_SPEED = 30.0f;       // Line跟随速度
const float LINE_TURN_GAIN = 0.3f;           // Line转向增益
const float IR_THRESHOLD = 20.0f;            // IR信号检测阈值
const float LINE_ALIGNMENT_TOLERANCE = 30.0f;  // Line模式对齐容差
const float MIN_SIGNAL_TO_FOLLOW = 250.0f;     // 最小信号强度

// 背景IR值（启动时校准）
float background_ir_line[5] = {0};
float background_avg_line = 0.0f;

// ===== Bump Sensor模式参数 =====
const float ALIGNMENT_TOLERANCE = 200.0f; // Bump对齐容差（微秒）

// ===== 速度估算变量 =====
unsigned long drive_est_ts = 0;
long d_last_e0 = 0, d_last_e1 = 0;
float spdL_cps = 0.0f, spdR_cps = 0.0f;

// ===== 数据记录功能已禁用 =====
// 为了节省内存，数据记录功能已禁用
// #define MAX_RESULTS 30
// #define VARIABLES 8
// float results[MAX_RESULTS][VARIABLES];  // 占用960字节RAM
// int results_index = 0;
// unsigned long record_results_ts = 0;
// unsigned long results_interval_ms = 200;
// bool recording_enabled = false;
// unsigned long experiment_start_time = 0;
// float last_distance_error = 0.0f;
// float last_signal_strength = 0.0f;
// int last_left_pwm = 0;
// int last_right_pwm = 0;

// ===== 辅助函数 =====
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ===== 按钮读取（带防抖）=====
bool readButton(int pin) {
  if (digitalRead(pin) == LOW) {
    delay(50);  // 防抖
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW);  // 等待释放
      delay(50);  // 防抖
      return true;
    }
  }
  return false;
}

// ===== Line模式：读取IR信号强度 =====
float getIRSignalStrength() {
  line_sensors.readSensorsADC();
  float total = 0.0f;
  for (int i = 0; i < NUM_SENSORS; i++) {
    total += line_sensors.readings[i];
  }
  return total / NUM_SENSORS;
}

// ===== Line模式：计算左右IR差值 =====
float getIRBalance() {
  line_sensors.readSensorsADC();
  
  // 左侧：传感器0和1
  // 右侧：传感器3和4
  float left_avg = (line_sensors.readings[0] + line_sensors.readings[1]) / 2.0f;
  float right_avg = (line_sensors.readings[3] + line_sensors.readings[4]) / 2.0f;
  
  return left_avg - right_avg;  // 左侧强度 - 右侧强度
}

// ===== Bump模式：距离估算函数（基于IR信号强度）=====
float estimateDistance(float signal_change) {
  // 根据信号变化估算距离
  // signal_change越大，Leader越近，距离越小
  // 这是一个简化的估算，需要根据实际测试调整
  
  if (signal_change < 100.0f) {
    return 30.0f;  // 太远，返回大距离
  }
  
  // 简单的反比例关系（需要根据实际测试校准）
  // 假设：signal_change = 500us 对应 5cm
  float estimated_cm = 2500.0f / signal_change;
  return constrain(estimated_cm, 2.0f, 30.0f);
}

// ===== 速度估算 =====
void estimateSpeed() {
  unsigned long now = millis();
  if (now - drive_est_ts < 30) return;
  
  unsigned long dt = now - drive_est_ts;
  drive_est_ts = now;
  
  long e0 = count_e0;
  long e1 = count_e1;
  
  long delta_e0 = e0 - d_last_e0;
  long delta_e1 = e1 - d_last_e1;
  
  d_last_e0 = e0;
  d_last_e1 = e1;
  
  // 转换为 counts per second
  spdR_cps = (float)delta_e0 * 1000.0f / (float)dt;
  spdL_cps = (float)delta_e1 * 1000.0f / (float)dt;
}

// ===== 更新位姿 =====
void updatePose() {
  static unsigned long pose_ts = 0;
  if (millis() - pose_ts >= 10) { 
    pose_ts = millis(); 
    pose.update(); 
  }
}

// ===== Line Sensor模式跟随（原始代码逻辑）=====
void followLeaderLineMode() {
  // 读取IR信号
  float ir_balance = getIRBalance();
  float avg_signal = getIRSignalStrength();
  float signal_drop = background_avg_line - avg_signal;
  float signal_diff = fabs(ir_balance);
  
  // 检查是否检测到Leader
  // 降低阈值以适应高背景值环境
  if (signal_drop < 20.0f && signal_diff < IR_THRESHOLD) {
    // 没有检测到Leader，停止
    motors.setPWM(0, 0);
    turn_pid.reset();
    
    static unsigned long debug_ts = 0;
    if (millis() - debug_ts >= 500) {
      debug_ts = millis();
      Serial.println("未检测到Leader - 停止");
      Serial.print("信号: "); Serial.print(avg_signal, 2);
      Serial.print(" | 背景: "); Serial.println(background_avg_line, 2);
    }
    return;
  }
  
  // 控制更新（每50ms）
  static unsigned long control_ts = 0;
  if (millis() - control_ts >= 50) {
    control_ts = millis();
    
    // 1. 计算旋转控制（对齐方向）
    float turn_pwm = 0;
    if (fabs(ir_balance) > LINE_ALIGNMENT_TOLERANCE) {
      // 需要旋转对齐
      turn_pwm = turn_pid.update(0.0f, ir_balance);
      turn_pwm = constrain(turn_pwm, -TURN_MAX_SPEED, TURN_MAX_SPEED);
    } else {
      // 已对齐，重置旋转PID
      turn_pid.reset();
    }
    
    // 2. 前进控制
    float forward_pwm = 0;
    
    // 只要检测到Leader，就持续前进
    // 信号太弱（太远）时才停止
    if (avg_signal > MIN_SIGNAL_TO_FOLLOW) {
      // 检测到Leader，持续跟随
      forward_pwm = LINE_FOLLOW_SPEED;
    } else {
      // 信号太弱，停止前进
      forward_pwm = 0;
    }
    
    // 3. 混合控制：旋转 + 前进
    // 如果需要大幅旋转，减少前进速度
    float turn_factor = fabs(turn_pwm) / TURN_MAX_SPEED;  // 0-1
    forward_pwm *= (1.0 - turn_factor * 0.6);  // 旋转时减速60%
    
    // 计算左右轮PWM
    int left_pwm = (int)(forward_pwm - turn_pwm);
    int right_pwm = (int)(forward_pwm + turn_pwm);
    
    // 限制PWM范围
    left_pwm = constrain(left_pwm, -60, 60);
    right_pwm = constrain(right_pwm, -60, 60);
    
    motors.setPWM(left_pwm, right_pwm);
    
    // 数据记录已禁用
    // last_left_pwm = left_pwm;
    // last_right_pwm = right_pwm;
    
    // 调试输出
    static unsigned long debug_ts = 0;
    if (millis() - debug_ts >= 200) {
      debug_ts = millis();
      
      Serial.print("跟随 | 信号: "); Serial.print(avg_signal, 0);
      Serial.print(" | IR差值: "); Serial.print(ir_balance, 2);
      Serial.print(" | 转向: "); Serial.print((int)turn_pwm);
      Serial.print(" | 前进: "); Serial.print((int)forward_pwm);
      Serial.print(" | L/R: "); Serial.print(left_pwm);
      Serial.print("/"); Serial.print(right_pwm);
      
      // 状态指示
      if (fabs(ir_balance) > LINE_ALIGNMENT_TOLERANCE) {
        Serial.print(" | 调整方向");
      } else {
        Serial.print(" | 已对齐");
      }
      
      if (forward_pwm > 0) {
        Serial.println(" | 跟随中");
      } else {
        Serial.println(" | 太远-停止");
      }
    }
  }
}

// ===== Bump Sensor模式跟随（带距离保持）=====
void followLeaderBumpMode() {
  // 读取bump传感器信号
  float ir_balance = bump_sensors.getBalance();
  float signal_change = bump_sensors.getSignalChange();
  
  // 检查是否检测到Leader
  if (signal_change < MIN_SIGNAL_CHANGE) {
    // 没有检测到Leader，停止
    motors.setPWM(0, 0);
    turn_pid.reset();
    distance_pid.reset();
    
    static unsigned long debug_ts = 0;
    if (millis() - debug_ts >= 500) {
      debug_ts = millis();
      Serial.println("未检测到Leader（Bump模式）- 停止");
      Serial.print("信号变化: "); Serial.print(signal_change, 2);
      Serial.print(" us | 左: "); Serial.print(bump_sensors.readings[0]);
      Serial.print(" us | 右: "); Serial.print(bump_sensors.readings[1]);
      Serial.println(" us");
    }
    return;
  }
  
  // 控制更新（每50ms）
  static unsigned long control_ts = 0;
  if (millis() - control_ts >= 50) {
    control_ts = millis();
    
    // 1. 计算旋转控制（对齐方向）
    float turn_pwm = 0;
    if (fabs(ir_balance) > ALIGNMENT_TOLERANCE) {
      turn_pwm = turn_pid.update(0.0f, ir_balance);
      turn_pwm = constrain(turn_pwm, -TURN_MAX_SPEED, TURN_MAX_SPEED);
    } else {
      turn_pid.reset();
    }
    
    // 2. 计算距离控制（保持5cm）
    float estimated_distance = estimateDistance(signal_change);
    float distance_error = estimated_distance - TARGET_DISTANCE_CM;
    
    float distance_pwm = distance_pid.update(0.0f, distance_error);
    distance_pwm = constrain(distance_pwm, -MAX_FOLLOW_SPEED, MAX_FOLLOW_SPEED);
    
    // 3. 混合控制
    float turn_factor = fabs(turn_pwm) / TURN_MAX_SPEED;
    distance_pwm *= (1.0 - turn_factor * 0.5);  // 转向时减速
    
    // 计算左右轮PWM
    int left_pwm = (int)(distance_pwm - turn_pwm);
    int right_pwm = (int)(distance_pwm + turn_pwm);
    
    left_pwm = constrain(left_pwm, -40, 40);
    right_pwm = constrain(right_pwm, -40, 40);
    
    // 数据记录已禁用
    // last_left_pwm = left_pwm;
    // last_right_pwm = right_pwm;
    // last_distance_error = distance_error;
    // last_signal_strength = signal_change;
    
    motors.setPWM(left_pwm, right_pwm);
    
    // 调试输出
    static unsigned long debug_ts = 0;
    if (millis() - debug_ts >= 200) {
      debug_ts = millis();
      
      Serial.print("Bump跟随 | 信号: "); Serial.print(signal_change, 0);
      Serial.print(" us | 估算距离: "); Serial.print(estimated_distance, 1);
      Serial.print(" cm | 目标: "); Serial.print(TARGET_DISTANCE_CM, 1);
      Serial.print(" cm | 误差: "); Serial.print(distance_error, 1);
      Serial.print(" cm | 左右差: "); Serial.print(ir_balance, 0);
      Serial.print(" us | L/R: "); Serial.print(left_pwm);
      Serial.print("/"); Serial.println(right_pwm);
    }
  }
}

// ===== 校准背景IR值（Line模式）=====
void calibrateBackgroundLine() {
  Serial.println("\n开始校准背景IR值...");
  Serial.println("请确保Leader尚未启动或距离较远");
  delay(2000);
  
  // 采样10次取平均
  for (int sample = 0; sample < 10; sample++) {
    line_sensors.readSensorsADC();
    for (int i = 0; i < NUM_SENSORS; i++) {
      background_ir_line[i] += line_sensors.readings[i];
    }
    delay(50);
  }
  
  // 计算平均值
  background_avg_line = 0.0f;
  for (int i = 0; i < NUM_SENSORS; i++) {
    background_ir_line[i] /= 10.0f;
    background_avg_line += background_ir_line[i];
  }
  background_avg_line /= NUM_SENSORS;
  
  Serial.println("背景IR校准完成：");
  Serial.print("背景平均值: "); Serial.println(background_avg_line, 2);
  Serial.print("各传感器背景值: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(background_ir_line[i], 1); Serial.print(" ");
  }
  Serial.println();
}

// ===== 模式选择界面 =====
void waitForModeSelection() {
  Serial.println("\n========================================");
  Serial.println("*** Follower机器人 - 模式选择 ***");
  Serial.println("========================================");
  Serial.println("请选择运行模式：");
  Serial.println("  按钮A - Line Sensor模式");
  Serial.println("  按钮B - Bump Sensor模式");
  Serial.println("等待按钮输入...");
  
  while (current_mode == MODE_STANDBY) {
    if (readButton(BUTTON_A)) {
      current_mode = MODE_LINE_SENSOR;
      Serial.println("\n✓ 已选择：Line Sensor模式");
    } else if (readButton(BUTTON_B)) {
      current_mode = MODE_BUMP_SENSOR;
      Serial.println("\n✓ 已选择：Bump Sensor模式");
    }
    delay(10);
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n========================================");
  Serial.println("*** Follower机器人 - 整合版本 ***");
  Serial.println("========================================");
  
  // 初始化按钮
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  
  // 初始化电机
  motors.initialise();
  
  // 初始化编码器
  setupEncoder0();
  setupEncoder1();
  
  // 初始化位姿
  pose.initialise(0, 0, 0);
  
  Serial.println("✓ 基础系统初始化完成");
  
  // 等待模式选择
  waitForModeSelection();
  
  // 根据选择的模式初始化传感器和PID
  if (current_mode == MODE_LINE_SENSOR) {
    // Line Sensor模式
    line_sensors.initialiseForADC();
    
    // ⚠️ 重要：initialiseForADC()会设置EMIT_PIN为OUTPUT+HIGH
    // 必须在之后重新设置为INPUT来关闭IR发射
    // 根据官方文档：INPUT模式会关闭IR LED
    pinMode(EMIT_PIN, INPUT);  // INPUT = 关闭IR发射，只接收外部IR源
    
    Serial.println("✓ Line Sensor已初始化");
    Serial.println("✓ EMIT_PIN已设置为INPUT（关闭IR发射）");
    
    // 校准背景IR值
    calibrateBackgroundLine();
    
    // 初始化Line模式PID（使用原始代码的参数）
    turn_pid.initialise(0.30f, 0.002f, 0.3f);  // Line转向PID
    turn_pid.setOutputLimits(-TURN_MAX_SPEED, TURN_MAX_SPEED);
    turn_pid.reset();
    
    Serial.println("✓ Line模式PID已初始化");
    Serial.println("  - 旋转PID: Kp=0.30, Ki=0.002, Kd=0.3");
    Serial.println("  - 跟随速度: " + String(LINE_FOLLOW_SPEED, 0) + " PWM");
    Serial.println("  - 最小信号: " + String(MIN_SIGNAL_TO_FOLLOW, 0) + " (太远停止)");
    
  } else if (current_mode == MODE_BUMP_SENSOR) {
    // Bump Sensor模式
    bump_sensors.initialiseForDigital();
    bump_sensors.calibrateBackground();
    
    Serial.println("✓ Bump Sensor已初始化");
    
    // 初始化Bump模式PID
    turn_pid.initialise(0.008f, 0.00005f, 0.003f);  // Bump转向PID
    turn_pid.setOutputLimits(-TURN_MAX_SPEED, TURN_MAX_SPEED);
    
    distance_pid.initialise(2.5f, 0.04f, 0.3f);  // 距离控制PID
    distance_pid.setOutputLimits(-MAX_FOLLOW_SPEED, MAX_FOLLOW_SPEED);
    
    Serial.println("✓ Bump模式PID已初始化");
    Serial.println("  - 转向PID: Kp=0.008, Ki=0.00005, Kd=0.003");
    Serial.println("  - 距离PID: Kp=2.5, Ki=0.04, Kd=0.3");
  }
  
  Serial.println("\n跟随参数：");
  Serial.println("  - 目标距离: " + String(TARGET_DISTANCE_CM, 1) + " cm");
  Serial.println("  - 距离容差: ±" + String(DISTANCE_TOLERANCE, 1) + " cm");
  Serial.println("  - 最大跟随速度: " + String(MAX_FOLLOW_SPEED, 0) + " PWM");
  Serial.println("  - 最大转向速度: " + String(TURN_MAX_SPEED, 0) + " PWM");
  Serial.println("========================================");
  Serial.println("Follower已准备好，等待检测Leader信号...\n");
  
  // 初始化速度估算
  drive_est_ts = millis();
  d_last_e0 = count_e0;
  d_last_e1 = count_e1;
  
  // 数据记录功能已禁用
  // results_index = 0;
  // record_results_ts = millis();
  // experiment_start_time = millis();
  // recording_enabled = true;
  
  Serial.println("\n注意：数据记录功能已禁用以节省内存");
  
  delay(2000);
}

// ===== 报告数据函数已禁用 =====
// void reportResults() {
//   motors.setPWM(0, 0);
//   Serial.println("数据记录功能已禁用");
//   while(true) { delay(1000); }
// }

// ===== LOOP =====
void loop() {
  // 数据记录功能已禁用
  // if (readButton(BUTTON_C)) {
  //   Serial.println("\n按钮C按下...");
  //   reportResults();
  // }
  
  // 更新位姿
  updatePose();
  
  // 更新速度估算
  estimateSpeed();
  
  // ===== 数据记录已禁用 =====
  // if (recording_enabled && results_index < MAX_RESULTS) {
  //   ...
  // }
  
  // 根据模式执行跟随
  if (current_mode == MODE_LINE_SENSOR) {
    followLeaderLineMode();
  } else if (current_mode == MODE_BUMP_SENSOR) {
    followLeaderBumpMode();
  }
  
  delay(10);
}

