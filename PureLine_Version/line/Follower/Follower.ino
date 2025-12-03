/*
 * ============================================
 * FOLLOWER ROBOT - 纯Line方案 + 数据记录
 * ============================================
 * 
 * 功能：
 * 1. 关闭自己的IR发射（EMIT_PIN = INPUT）
 * 2. 使用5个Line传感器：
 *    - 中间3个[1,2,3] → 距离控制（一直检测）
 *    - 左侧[0]、右侧[4] → 方向控制（检测到才转向）
 * 3. PID控制距离
 * 4. 记录kinematics数据
 * 5. 实验结束后通过串口输出CSV数据
 * 
 * 传感器布局：
 *   [0]  [1]  [2]  [3]  [4]
 *    ↓    ↓    ↓    ↓    ↓
 *   方向 ←  距离控制  → 方向
 * 
 * 方向控制逻辑：
 * - 左侧[0]检测到信号 → Leader在左边 → 左转让车头对准
 * - 右侧[4]检测到信号 → Leader在右边 → 右转让车头对准
 * - 两边都没检测到 → Leader在正前方 → 直行
 * 
 * 日期: 2025-12-01
 */

#include "Encoders.h"
#include "Kinematics.h"
#include "Motors.h"
#include "LineSensors.h"
#include "PID.h"

// ========== 引脚定义 ==========
#define EMIT_PIN 11
#define BUZZ_PIN 6
#define LED_PIN  13
#define LED_RED  17
#define BUTTON_B 30

// ========== 全局对象 ==========
Motors_c motors;
Kinematics_c kin;
LineSensors_c line_sensors;
PID_c distance_pid;

// ========== 数据记录（根据项目要求）==========
// 项目要求记录：
//   1. Kinematics: x, y, theta
//   2. Wheel speed: spdL, spdR (cps)
//   3. Sensor activation: IR_center, L_signal, R_signal
//   4. Control output: speed_cmd, steer_cmd
//
// 内存限制：3Pi+只有2560 bytes！
// 50 samples × 10 floats × 4 bytes = 2000 bytes（安全）
#define MAX_RESULTS 15
#define VARIABLES 10  // x, y, theta, spdL, spdR, IR_center, L_sig, R_sig, speed_cmd, steer_cmd
float results[MAX_RESULTS][VARIABLES];
int results_index = 0;
unsigned long record_ts = 0;
unsigned long experiment_start_ts = 0;
#define RECORD_INTERVAL_MS 400  // 每200ms记录（50样本×200ms=10秒）
#define EXPERIMENT_DURATION_MS 6000  // 10秒实验

// 用于记录的临时变量
float rec_IR_center = 0;
int rec_L_signal = 0;
int rec_R_signal = 0;
float rec_speed_cmd = 0;
float rec_steer_cmd = 0;

// ========== 状态机 ==========
#define STATE_IDLE          0
#define STATE_CALIBRATE     1
#define STATE_WAIT_SIGNAL   2
#define STATE_FOLLOWING     3
#define STATE_FINISHED      4

int robot_state = STATE_IDLE;

// ========== 控制参数 ==========
// 距离控制（基于IR强度）
#define DISTANCE_TARGET   80.0f    // 目标IR值
#define DISTANCE_MIN      50.0f    // 理想距离下限
#define DISTANCE_MAX     120.0f    // 理想距离上限
#define DISTANCE_TOO_CLOSE 150.0f  // 过近阈值

// 距离PID参数
#define DIST_KP 0.5f
#define DIST_KI 0.002f
#define DIST_KD 0.4f

// 方向控制参数（学习自合并版本）
#define STEER_DEADBAND 40      // ★死区：增大到20减少左右晃动
#define STEER_CLAMP 60         // 限幅：最大差值
#define STEER_NORM 100.0f      // 归一化因子
#define STEER_ALPHA 0.5f       // 低通滤波系数
#define K_STEER 120.0f         // 转向增益（同合并版本）
#define MIN_WHEEL_SPEED 5.0f   // 最小轮速，确保两轮都在转

// PWM限制（学习自合并版本）
#define SPEED_MIN   0.0f
#define SPEED_MAX  60.0f       // PWM_MAX = 60（同合并版本）
#define MAX_PWM    60

// 信号检测
#define SIGNAL_THRESHOLD  25.0f
#define SIGNAL_LOST_TIME 200

// ========== 背景值存储 ==========
float background_values[NUM_SENSORS];

// ========== 控制变量 ==========
float speed = 0.0f;
float steer_filtered = 0.0f;  // 低通滤波后的转向值
int line_L_offset = 0;        // 左侧传感器背景
int line_R_offset = 0;        // 右侧传感器背景
unsigned long last_signal_time = 0;
unsigned long last_update_time = 0;

#define UPDATE_INTERVAL 25

// ========== 轮速测量（项目要求）==========
#define SPEED_EST_MS 20
unsigned long speed_est_ts = 0;
long last_e0 = 0, last_e1 = 0;
float spdL_cps = 0.0f;  // 左轮速度 (counts per second)
float spdR_cps = 0.0f;  // 右轮速度 (counts per second)

// ========== 函数声明 ==========
void beep(int duration);
void setupIRReceiver();
void calibrateSensors();
float getCenterIRValue();
float getSteerFromLine();
void updateFollowingControl();
bool hasSignal();
void waitForButton();
void recordData();
void printResults();
void updateWheelSpeed();

// ============================================
// SETUP
// ============================================
void setup() {
  // 引脚初始化
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);   // 红LED常亮（active-low）
  
  // ★★★ 关键：关闭自己的IR发射 ★★★
  setupIRReceiver();
  
  // 串口初始化
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("  FOLLOWER - Pure Line Sensor Version");
  Serial.println("  with Data Recording");
  Serial.println("========================================");
  Serial.println("IR Receiver Mode: ON");
  Serial.println("Own IR LED: OFF (INPUT mode)");
  Serial.println("");
  
  // 初始化各模块
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  kin.initialise(0, 0, 0);
  
  // 初始化Line Sensors
  line_sensors.initialiseForADC();
  
  // 初始化PID（参数参考合并版本）
  distance_pid.initialise(DIST_KP, DIST_KI, DIST_KD);
  distance_pid.setOutputLimits(SPEED_MIN, SPEED_MAX);  // 0-60
  distance_pid.setMaxDelta(6.0f);    // 允许更大的速度变化
  distance_pid.setOutputFilter(0.7f);
  
  // ★ 重要：在所有初始化后重新设置红LED ★
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);   // 红LED常亮（active-low）
  
  // 启动提示
  beep(100);
  delay(200);
  beep(100);
  
  Serial.println("Control Strategy:");
  Serial.println("  - Center [1,2,3] -> Distance (always)");
  Serial.println("  - Left [0] detected -> Left turn");
  Serial.println("  - Right [4] detected -> Right turn");
  Serial.println("  - Neither detected -> Go straight");
  Serial.println("  - Target IR: 80, Duration: 10s");
  Serial.println("");
  Serial.println("Press Button B to start calibration...");
  
  waitForButton();
  
  // 校准传感器
  robot_state = STATE_CALIBRATE;
  calibrateSensors();
  
  // 等待信号
  robot_state = STATE_WAIT_SIGNAL;
  Serial.println("Waiting for Leader signal...");
  
  last_update_time = millis();
  speed_est_ts = millis();
  last_e0 = count_e0;
  last_e1 = count_e1;
  results_index = 0;
}

// ============================================
// LOOP
// ============================================
void loop() {
  // ★ 确保红LED持续点亮 ★
  digitalWrite(LED_RED, LOW);
  
  unsigned long now = millis();
  
  switch (robot_state) {
    
    // ========================================
    // 等待Leader信号
    // ========================================
    case STATE_WAIT_SIGNAL:
      {
        static unsigned long last_check = 0;
        if (now - last_check >= 300) {
          last_check = now;
          
          float center_value = getCenterIRValue();
          
          Serial.print("Waiting... IR=");
          Serial.print(center_value, 1);
          Serial.print(" (threshold=");
          Serial.print(SIGNAL_THRESHOLD, 0);
          Serial.println(")");
          
          // 检测到信号
          if (hasSignal()) {
            robot_state = STATE_FOLLOWING;
            last_signal_time = now;
            experiment_start_ts = now;
            record_ts = now;
            
            beep(200);
            Serial.println("\nLeader detected! Starting to follow...\n");
          }
        }
      }
      break;
    
    // ========================================
    // 跟随Leader
    // ========================================
    case STATE_FOLLOWING:
      // 更新轮速（高频率）
      updateWheelSpeed();
      
      if (now - last_update_time >= UPDATE_INTERVAL) {
        last_update_time = now;
        
        // 更新运动学
        kin.update();
        
        // 获取传感器数据
        float ir_value = getCenterIRValue();
        float steer_value = getSteerFromLine();
        
        // 检查实验时间
        if (now - experiment_start_ts >= EXPERIMENT_DURATION_MS) {
          motors.setPWM(0, 0);
          robot_state = STATE_FINISHED;
          beep(300);
          Serial.println("\nExperiment time finished!");
          break;
        }
        
        // 检查信号
        if (hasSignal()) {
          last_signal_time = now;
          
          // 执行跟随控制
          updateFollowingControl();
          
          // 记录数据
          if (now - record_ts >= RECORD_INTERVAL_MS) {
            record_ts = now;
            recordData();
          }
          
        } else {
          // 信号丢失
          if (now - last_signal_time > SIGNAL_LOST_TIME) {
            motors.setPWM(0, 0);
            robot_state = STATE_FINISHED;
            Serial.println("\nSignal lost! Stopping...");
          }
        }
        
        // 调试输出
        static unsigned long last_debug = 0;
        if (now - last_debug >= 300) {
          last_debug = now;
          
          // 读取当前传感器值用于调试
          line_sensors.readSensorsADC();
          int L_sig = line_L_offset - (int)line_sensors.readings[0];
          int R_sig = line_R_offset - (int)line_sensors.readings[4];
          if (L_sig < 0) L_sig = 0;
          if (R_sig < 0) R_sig = 0;
          
          // 应用死区
          int L_eff = (L_sig < STEER_DEADBAND) ? 0 : L_sig;
          int R_eff = (R_sig < STEER_DEADBAND) ? 0 : R_sig;
          
          Serial.print("IR=");
          Serial.print(ir_value, 1);
          Serial.print(" L=");
          Serial.print(L_eff);
          Serial.print(" R=");
          Serial.print(R_eff);
          Serial.print(" Diff=");
          Serial.print(R_eff - L_eff);
          Serial.print(" | Steer=");
          Serial.print(steer_filtered, 2);
          Serial.print(" Spd=");
          Serial.println(speed, 1);
        }
      }
      break;
    
    // ========================================
    // 实验完成，输出数据
    // ========================================
    case STATE_FINISHED:
      motors.setPWM(0, 0);
      
      printResults();
      
      Serial.println("\nExperiment finished. Reset to run again.");
      delay(5000);
      break;
    
    default:
      break;
  }
}

// ============================================
// 跟随控制（学习自合并版本）
// 关键：两个轮子都要转，只是速度不同！
// ============================================
void updateFollowingControl() {
  // 1. 获取传感器数据
  float ir_value = getCenterIRValue();
  float steer_value = getSteerFromLine();  // 已经滤波过的
  
  // ★保存传感器数据用于记录★
  rec_IR_center = ir_value;
  
  // 2. 距离控制：PID让IR值保持在目标
  speed = distance_pid.update(DISTANCE_TARGET, ir_value);
  
  // 3. 速度限制（确保有基础前进速度）
  if (speed < MIN_WHEEL_SPEED) speed = MIN_WHEEL_SPEED;
  
  // 4. 方向控制：转向量 = 滤波值 × 增益
  // steer_value范围约 [-1, +1]
  // steer_term最大 = 1.0 * 15 = 15 PWM
  float steer_term = steer_value * K_STEER;
  
  // 5. 限制转向量，确保不会让任何一个轮子停下
  // 转向量最大不超过当前速度的一半
  float max_steer = speed * 0.6f;
  if (steer_term > max_steer) steer_term = max_steer;
  if (steer_term < -max_steer) steer_term = -max_steer;
  
  // ★保存控制指令用于记录★
  rec_speed_cmd = speed;
  rec_steer_cmd = steer_term;
  
  // 6. 计算左右轮目标速度
  // 差速转向：想右转 → 左轮快、右轮慢
  // Leader在右边(steer_term > 0) → 需要右转 → 左轮加速、右轮减速
  // Leader在左边(steer_term < 0) → 需要左转 → 右轮加速、左轮减速
  float demand_L = speed + steer_term;  // 右侧检测到时加速
  float demand_R = speed - steer_term;  // 右侧检测到时减速
  
  // 7. 确保两轮都有最小速度（不停转）
  if (demand_L < MIN_WHEEL_SPEED) demand_L = MIN_WHEEL_SPEED;
  if (demand_R < MIN_WHEEL_SPEED) demand_R = MIN_WHEEL_SPEED;
  if (demand_L > MAX_PWM) demand_L = MAX_PWM;
  if (demand_R > MAX_PWM) demand_R = MAX_PWM;
  
  // 8. 发送到电机
  motors.setPWM((int)demand_L, (int)demand_R);
}

// ============================================
// 获取中间传感器的IR值（距离）
// ============================================
float getCenterIRValue() {
  line_sensors.readSensorsADC();
  
  // 计算中间3个传感器的IR强度
  float ir_1 = background_values[1] - line_sensors.readings[1];
  float ir_2 = background_values[2] - line_sensors.readings[2];
  float ir_3 = background_values[3] - line_sensors.readings[3];
  
  // 取最大值避免负值
  if (ir_1 < 0) ir_1 = 0;
  if (ir_2 < 0) ir_2 = 0;
  if (ir_3 < 0) ir_3 = 0;
  
  float ir_avg = (ir_1 + ir_2 + ir_3) / 3.0f;
  return ir_avg;
}

// ============================================
// 获取方向偏差（学习自合并版本）
// 中间[1,2,3]用于距离控制（在getCenterIRValue中）
// 左侧[0]和右侧[4]用于方向控制：
//   - 信号<10视为0（死区）
//   - 左侧[0]有信号 → Leader在左边 → 左转
//   - 右侧[4]有信号 → Leader在右边 → 右转
//   - 两边都没信号 → 直行
// 返回：正值=右转，负值=左转，0=直行
// ============================================
float getSteerFromLine() {
  line_sensors.readSensorsADC();
  
  // 计算左右传感器的信号强度（减去背景）
  int L_signal = line_L_offset - (int)line_sensors.readings[0];  // 左侧[0]
  int R_signal = line_R_offset - (int)line_sensors.readings[4];  // 右侧[4]
  
  // 确保不为负
  if (L_signal < 0) L_signal = 0;
  if (R_signal < 0) R_signal = 0;
  
  // ★保存原始信号用于记录（死区处理前）★
  rec_L_signal = L_signal;
  rec_R_signal = R_signal;
  
  // ★死区处理：信号小于10就设为0★
  if (L_signal < STEER_DEADBAND) L_signal = 0;
  if (R_signal < STEER_DEADBAND) R_signal = 0;
  
  // 计算差值（学习自合并版本）
  // diff = R - L
  // Leader在右边 → R > L → diff > 0 → 右转
  // Leader在左边 → L > R → diff < 0 → 左转
  int diff_raw = R_signal - L_signal;
  
  // 限幅
  if (diff_raw > STEER_CLAMP) diff_raw = STEER_CLAMP;
  if (diff_raw < -STEER_CLAMP) diff_raw = -STEER_CLAMP;
  
  // 归一化到 [-1, +1] 范围
  float diff_norm = (float)diff_raw / STEER_NORM;
  
  // 低通滤波：平滑转向
  steer_filtered = STEER_ALPHA * steer_filtered + (1.0f - STEER_ALPHA) * diff_norm;
  
  return steer_filtered;
}

// ============================================
// 检测是否有信号
// ============================================
bool hasSignal() {
  return (getCenterIRValue() > SIGNAL_THRESHOLD);
}

// ============================================
// 记录数据（完整版，根据项目要求）
// 记录：x, y, theta, spdL, spdR, IR_center, L_sig, R_sig, speed_cmd, steer_cmd
// ============================================
void recordData() {
  if (results_index < MAX_RESULTS) {
    results[results_index][0] = kin.x;            // X坐标 (mm)
    results[results_index][1] = kin.y;            // Y坐标 (mm)
    results[results_index][2] = kin.theta;        // 角度 (rad)
    results[results_index][3] = spdL_cps;         // 左轮速度 (counts/sec)
    results[results_index][4] = spdR_cps;         // 右轮速度 (counts/sec)
    results[results_index][5] = rec_IR_center;    // 中心IR值（距离）
    results[results_index][6] = (float)rec_L_signal;  // 左侧传感器信号
    results[results_index][7] = (float)rec_R_signal;  // 右侧传感器信号
    results[results_index][8] = rec_speed_cmd;    // 速度指令
    results[results_index][9] = rec_steer_cmd;    // 转向指令
    results_index++;
  }
}

// ============================================
// 输出数据（CSV格式，完整版）
// 包含项目要求的所有数据：
//   - Kinematics: x, y, theta
//   - Wheel speed: spdL, spdR
//   - Sensor activation: IR_center, L_signal, R_signal
//   - Control output: speed_cmd, steer_cmd
// ============================================
void printResults() {
  Serial.println("\n========== FOLLOWER DATA (CSV) ==========");
  Serial.println("Sample,X_mm,Y_mm,Theta_rad,SpdL_cps,SpdR_cps,IR_center,L_signal,R_signal,Speed_cmd,Steer_cmd");
  
  for (int i = 0; i < results_index; i++) {
    Serial.print(i);
    Serial.print(",");
    Serial.print(results[i][0], 2);  // X
    Serial.print(",");
    Serial.print(results[i][1], 2);  // Y
    Serial.print(",");
    Serial.print(results[i][2], 4);  // Theta
    Serial.print(",");
    Serial.print(results[i][3], 1);  // SpdL
    Serial.print(",");
    Serial.print(results[i][4], 1);  // SpdR
    Serial.print(",");
    Serial.print(results[i][5], 1);  // IR_center
    Serial.print(",");
    Serial.print((int)results[i][6]); // L_signal
    Serial.print(",");
    Serial.print((int)results[i][7]); // R_signal
    Serial.print(",");
    Serial.print(results[i][8], 1);  // Speed_cmd
    Serial.print(",");
    Serial.println(results[i][9], 2); // Steer_cmd
  }
  
  Serial.println("==========================================");
  Serial.println("Data Description:");
  Serial.println("  X_mm, Y_mm: Position (mm)");
  Serial.println("  Theta_rad: Heading angle (rad)");
  Serial.println("  SpdL_cps, SpdR_cps: Wheel speed (counts/sec)");
  Serial.println("  IR_center: Center sensor signal (distance)");
  Serial.println("  L_signal, R_signal: Left/Right sensor signal (direction)");
  Serial.println("  Speed_cmd: Speed command from PID");
  Serial.println("  Steer_cmd: Steering command");
  Serial.println("");
  Serial.print("Total samples: ");
  Serial.println(results_index);
  Serial.print("Record interval: ");
  Serial.print(RECORD_INTERVAL_MS);
  Serial.println(" ms");
}

// ============================================
// 蜂鸣器
// ============================================
void beep(int duration) {
  analogWrite(BUZZ_PIN, 120);
  delay(duration);
  analogWrite(BUZZ_PIN, 0);
}

// ============================================
// 更新轮速测量（项目要求：wheel speed）
// ============================================
void updateWheelSpeed() {
  unsigned long now = millis();
  if (now - speed_est_ts >= SPEED_EST_MS) {
    unsigned long dt = now - speed_est_ts;
    speed_est_ts = now;
    
    long e0 = count_e0;
    long e1 = count_e1;
    long d0 = e0 - last_e0;
    long d1 = e1 - last_e1;
    last_e0 = e0;
    last_e1 = e1;
    
    float dt_sec = (float)dt / 1000.0f;
    spdR_cps = (float)d0 / dt_sec;  // 右轮（encoder 0）
    spdL_cps = (float)d1 / dt_sec;  // 左轮（encoder 1）
  }
}

// ============================================
// 设置IR接收模式
// ============================================
void setupIRReceiver() {
  pinMode(EMIT_PIN, INPUT);
  digitalWrite(EMIT_PIN, LOW);
}

// ============================================
// 校准传感器（学习自合并版本：多帧平均）
// ============================================
void calibrateSensors() {
  Serial.println("\n===== Sensor Calibration =====");
  Serial.println("Sampling background values...");
  Serial.println("Make sure Leader is NOT active or far away!");
  Serial.println("");
  
  beep(100);
  delay(2000);
  
  // 初始化
  for (int i = 0; i < NUM_SENSORS; i++) {
    background_values[i] = 0.0f;
  }
  long sum_L = 0;
  long sum_R = 0;
  
  // 采样（30帧平均，学习自合并版本）
  Serial.println("Sampling 30 frames...");
  for (int sample = 0; sample < 30; sample++) {
    line_sensors.readSensorsADC();
    
    for (int i = 0; i < NUM_SENSORS; i++) {
      background_values[i] += line_sensors.readings[i];
    }
    sum_L += (long)line_sensors.readings[0];
    sum_R += (long)line_sensors.readings[4];
    
    delay(50);
  }
  
  // 计算平均
  for (int i = 0; i < NUM_SENSORS; i++) {
    background_values[i] /= 30.0f;
  }
  line_L_offset = sum_L / 30;
  line_R_offset = sum_R / 30;
  
  Serial.println("Calibration complete!");
  Serial.println("Background values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("  Sensor[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(background_values[i], 1);
  }
  Serial.println("");
  Serial.print("Steering offsets: L=");
  Serial.print(line_L_offset);
  Serial.print(" R=");
  Serial.println(line_R_offset);
  Serial.println("");
  
  beep(100);
  delay(200);
  beep(100);
}

// ============================================
// 等待按钮
// ============================================
void waitForButton() {
  while (digitalRead(BUTTON_B) == HIGH) {
    delay(10);
  }
  while (digitalRead(BUTTON_B) == LOW) {
    delay(10);
  }
  beep(100);
  delay(200);
}

