/*
 * ============================================
 * FOLLOWER ROBOT - Bump Sensor方案 + 数据记录
 * ============================================
 * 
 * 功能：
 * 1. 关闭自己的IR发射（EMIT_PIN = INPUT）
 * 2. 使用2个Bump传感器：
 *    - BUMP_L + BUMP_R 平均 → 距离控制
 *    - BUMP_R - BUMP_L 差值 → 方向控制
 * 3. 速度映射 + PID控制
 * 4. 记录kinematics数据
 * 5. 实验结束后通过串口输出CSV数据
 * 
 * 传感器布局：
 *   BUMP_L (Pin 4)    BUMP_R (Pin 5)
 *       ↓                  ↓
 *    距离+方向控制
 * 
 * 日期: 2025-12-02
 */

#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"

// ========== 引脚定义 ==========
#define BUMP_L 4
#define BUMP_R 5
#define EMIT_PIN 11
#define BUZZ_PIN 6
#define LED_RED 17

// ========== 全局对象 ==========
Motors_c motors;
Kinematics_c kin;
PID_c pidL, pidR;

// ========== 数据记录 ==========
// 15 samples × 10 floats × 4 bytes = 600 bytes（安全）
#define MAX_RESULTS 15
#define VARIABLES 10  // x, y, theta, spdL, spdR, IR_center, L_sig, R_sig, speed_cmd, steer_cmd
float results[MAX_RESULTS][VARIABLES];
int results_index = 0;
unsigned long record_ts = 0;
unsigned long experiment_start_ts = 0;
#define RECORD_INTERVAL_MS 400  // 每400ms记录（15样本×400ms=6秒）
#define EXPERIMENT_DURATION_MS 6000  // 6秒实验

// 用于记录的临时变量
unsigned long rec_dL = 0;      // 左Bump原始值
unsigned long rec_dR = 0;      // 右Bump原始值
float rec_IR_center = 0;       // 平均距离
float rec_speed_cmd = 0;       // 目标速度
float rec_steer_cmd = 0;       // 转向量

// ========== 状态机 ==========
#define STATE_IDLE          0
#define STATE_WAIT_SIGNAL   1
#define STATE_FOLLOWING     2
#define STATE_FINISHED      3

int robot_state = STATE_IDLE;

// ========== 控制参数 ==========
#define DRIVE_EST_MS 20
#define DRIVE_PID_MS 40
#define DRIVE_PWM_LIMIT 60

float KP_L = 0.04025, KI_L = 0.00005, KD_L = 0;
float KP_R = 0.05000, KI_R = 0.00005, KD_R = 0;

const int kF_L = 16;
const int kF_R = 16;

// 信号检测
#define LOST_TH 3800
#define NEAR_TH 300
#define SIGNAL_THRESHOLD 500  // 开始跟随的阈值

// 碰撞检测
int faceCount = 0;
bool faceCrash = false;

// ========== 运行时变量 ==========
unsigned long ts_est = 0, ts_pid = 0;
long last_e0 = 0, last_e1 = 0;
float spdL = 0, spdR = 0;
unsigned long last_d = 9999;
unsigned long last_signal_time = 0;

// ========== 函数声明 ==========
unsigned long readBump(int pin);
float mapIRtoCS(unsigned long d);
void beep(int duration);
void recordData();
void printResults();
void updateWheelSpeed();
void updateFollowingControl();
bool hasSignal();

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("  FOLLOWER - Bump Sensor Version");
  Serial.println("  with Data Recording");
  Serial.println("========================================");
  
  // 红色LED初始化（第一次）
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);   // 红LED常亮（active-low）
  
  // 初始化电机和编码器
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  kin.initialise(0, 0, 0);
  
  // 初始化PID
  pidL.initialise(KP_L, KI_L, KD_L);
  pidR.initialise(KP_R, KI_R, KD_R);
  
  // ★★★ 关键：关闭自己的IR发射 ★★★
  pinMode(BUMP_L, INPUT);
  pinMode(BUMP_R, INPUT);
  pinMode(EMIT_PIN, INPUT);
  digitalWrite(EMIT_PIN, LOW);
  
  pinMode(BUZZ_PIN, OUTPUT);
  
  // ★ 重要：在所有初始化后重新设置红LED ★
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);   // 红LED常亮（active-low）
  
  Serial.println("IR Receiver Mode: ON");
  Serial.println("Own IR LED: OFF (INPUT mode)");
  Serial.println("");
  
  // 启动提示
  beep(100);
  delay(200);
  beep(100);
  
  Serial.println("Control Strategy:");
  Serial.println("  - Distance: (BUMP_L + BUMP_R) / 2");
  Serial.println("  - Steering: BUMP_R - BUMP_L");
  Serial.println("  - Duration: 6s, Record: 15 samples");
  Serial.println("");
  Serial.println("Waiting for Leader signal...");
  
  // 初始化
  ts_est = ts_pid = millis();
  last_e0 = count_e0;
  last_e1 = count_e1;
  results_index = 0;
  
  robot_state = STATE_WAIT_SIGNAL;
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
          
          unsigned long dL = readBump(BUMP_L);
          unsigned long dR = readBump(BUMP_R);
          unsigned long d = (dL + dR) / 2;
          
          Serial.print("Waiting... L=");
          Serial.print(dL);
          Serial.print(" R=");
          Serial.print(dR);
          Serial.print(" AVG=");
          Serial.print(d);
          Serial.print(" (threshold=");
          Serial.print(SIGNAL_THRESHOLD);
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
      
      // 更新运动学
      kin.update();
      
      // 执行跟随控制
      updateFollowingControl();
      
      // ★★★ 只检查实验时间，不管信号丢失 ★★★
      // 即使丢失信号也继续尝试跟随并记录数据，直到6秒结束
      if (now - experiment_start_ts >= EXPERIMENT_DURATION_MS) {
        motors.setPWM(0, 0);
        robot_state = STATE_FINISHED;
        beep(300);
        Serial.println("\nExperiment time finished!");
        break;
      }
      
      // 信号状态监控（仅用于调试显示，不影响运行）
      if (hasSignal()) {
        last_signal_time = now;  // 有信号，更新时间戳
      }
      
      // 记录数据
      if (now - record_ts >= RECORD_INTERVAL_MS) {
        record_ts = now;
        recordData();
      }
      
      // 调试输出（增强版，显示信号状态）
      static unsigned long last_debug = 0;
      if (now - last_debug >= 300) {
        last_debug = now;
        
        bool has_sig = hasSignal();
        unsigned long time_since_signal = now - last_signal_time;
        
        Serial.print("L=");
        Serial.print(rec_dL);
        Serial.print(" R=");
        Serial.print(rec_dR);
        Serial.print(" AVG=");
        Serial.print(rec_IR_center, 0);
        Serial.print(" | Sig=");
        Serial.print(has_sig ? "YES" : "NO");
        Serial.print(" (");
        Serial.print(time_since_signal);
        Serial.print("ms) | Spd=");
        Serial.print(rec_speed_cmd, 1);
        Serial.print(" Turn=");
        Serial.println(rec_steer_cmd, 2);
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
// 跟随控制（保持Bump_only原有逻辑）
// ============================================
void updateFollowingControl() {
  unsigned long now = millis();
  
  // 读取Bump传感器
  unsigned long dL = readBump(BUMP_L);
  unsigned long dR = readBump(BUMP_R);
  unsigned long d = (dL + dR) / 2;
  
  // ★保存传感器数据用于记录★
  rec_dL = dL;
  rec_dR = dR;
  rec_IR_center = (float)d;
  
  // 碰撞检测
  if (d > LOST_TH && last_d < NEAR_TH) {
    if (fabs(spdL) < 5 && fabs(spdR) < 5) {
      faceCount++;
      if (faceCount >= 5) {
        faceCrash = true;
      }
    }
  } else {
    faceCount = 0;
  }
  
  last_d = d;
  
  // 速度估计
  if (now - ts_est >= DRIVE_EST_MS) {
    ts_est = now;
    
    long e0 = count_e0;
    long e1 = count_e1;
    
    long de0 = e0 - last_e0;
    long de1 = e1 - last_e1;
    
    last_e0 = e0;
    last_e1 = e1;
    
    spdR = de0 / (DRIVE_EST_MS / 1000.0f);
    spdL = de1 / (DRIVE_EST_MS / 1000.0f);
  }
  
  // PID控制
  if (now - ts_pid >= DRIVE_PID_MS) {
    ts_pid = now;
    
    float demand;
    if (faceCrash) demand = 0;
    else demand = mapIRtoCS(d);
    
    // ★保存控制指令★
    rec_speed_cmd = demand;
    
    float uL = pidL.update(demand, spdL);
    float uR = pidR.update(demand, spdR);
    
    float pwmL = kF_L + uL;
    float pwmR = kF_R + uR;
    
    // 转向控制
    float steer = (float)dR - (float)dL;
    float turn = steer * 0.009;
    if (turn > 10) turn = 10;
    if (turn < -10) turn = -10;
    
    // ★保存转向指令★
    rec_steer_cmd = turn;
    
    pwmL -= turn;
    pwmR += turn;
    
    if (pwmL > DRIVE_PWM_LIMIT) pwmL = DRIVE_PWM_LIMIT;
    if (pwmL < -DRIVE_PWM_LIMIT) pwmL = -DRIVE_PWM_LIMIT;
    if (pwmR > DRIVE_PWM_LIMIT) pwmR = DRIVE_PWM_LIMIT;
    if (pwmR < -DRIVE_PWM_LIMIT) pwmR = -DRIVE_PWM_LIMIT;
    
    motors.setPWM((int)pwmL, (int)pwmR);
  }
}

// ============================================
// 读取Bump传感器（保持原逻辑）
// ============================================
unsigned long readBump(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);
  unsigned long t0 = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t0 > 4500) break;
  }
  return micros() - t0;
}

// ============================================
// 距离映射（调整参数防止爆冲）
// ============================================
float mapIRtoCS(unsigned long d) {
  if (d > LOST_TH) return 0;
  if (d < 150) return 0;
  float cs = d * 0.15;        // 改：0.25 → 0.15（降低40%，防止起步爆冲）
  if (cs > 200) cs = 200;     // 改：310 → 220（降低最大速度）
  return cs;
}

// ============================================
// 检测是否有信号
// ============================================
bool hasSignal() {
  unsigned long dL = readBump(BUMP_L);
  unsigned long dR = readBump(BUMP_R);
  unsigned long d = (dL + dR) / 2;
  return (d > SIGNAL_THRESHOLD && d < LOST_TH);
}

// ============================================
// 更新轮速测量
// ============================================
void updateWheelSpeed() {
  unsigned long now = millis();
  if (now - ts_est >= DRIVE_EST_MS) {
    unsigned long dt = now - ts_est;
    ts_est = now;
    
    long e0 = count_e0;
    long e1 = count_e1;
    long d0 = e0 - last_e0;
    long d1 = e1 - last_e1;
    last_e0 = e0;
    last_e1 = e1;
    
    float dt_sec = (float)dt / 1000.0f;
    spdR = (float)d0 / dt_sec;  // 右轮（encoder 0）
    spdL = (float)d1 / dt_sec;  // 左轮（encoder 1）
  }
}

// ============================================
// 记录数据（10变量）
// 记录：x, y, theta, spdL, spdR, IR_center, L_sig, R_sig, speed_cmd, steer_cmd
// ============================================
void recordData() {
  if (results_index < MAX_RESULTS) {
    results[results_index][0] = kin.x;            // X坐标 (mm)
    results[results_index][1] = kin.y;            // Y坐标 (mm)
    results[results_index][2] = kin.theta;        // 角度 (rad)
    results[results_index][3] = spdL;             // 左轮速度 (counts/sec)
    results[results_index][4] = spdR;             // 右轮速度 (counts/sec)
    results[results_index][5] = rec_IR_center;    // 平均距离 (μs)
    results[results_index][6] = (float)rec_dL;    // 左Bump传感器值 (μs)
    results[results_index][7] = (float)rec_dR;    // 右Bump传感器值 (μs)
    results[results_index][8] = rec_speed_cmd;    // 速度指令
    results[results_index][9] = rec_steer_cmd;    // 转向指令
    results_index++;
  }
}

// ============================================
// 输出数据（CSV格式）
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
    Serial.print((int)results[i][5]); // IR_center (平均距离，μs)
  Serial.print(",");
    Serial.print((int)results[i][6]); // L_signal (左Bump，μs)
  Serial.print(",");
    Serial.print((int)results[i][7]); // R_signal (右Bump，μs)
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
  Serial.println("  IR_center: Average Bump distance (microseconds)");
  Serial.println("  L_signal, R_signal: Left/Right Bump sensor (microseconds)");
  Serial.println("  Speed_cmd: Speed command from mapping");
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
