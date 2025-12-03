/*
 * ============================================
 * LEADER ROBOT - 纯Line方案 + 直线运动 + 数据记录
 * ============================================
 * 
 * 功能：
 * 1. 发射Line Sensor IR (EMIT_PIN = HIGH，持续发射)
 * 2. 直线运动450mm
 * 3. 记录kinematics数据（x, y, theta）
 * 4. 运动结束后通过串口输出CSV数据
 * 
 * 运动路线：
 * - Stage 0: 上电后等待1秒（让Follower准备）
 * - Stage 1: 直线前进450mm
 * - Stage 2: 停止，输出数据
 * 
 * 日期: 2025-12-02
 */

#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "LineSensors.h"

// ========== 引脚定义 ==========
#define EMIT_PIN    11
#define BUZZ_PIN    6

// ========== 全局对象 ==========
Motors_c motors;
Kinematics_c kin;
PID_c left_pid;
PID_c right_pid;

// ========== 数据记录 ==========
// 内存限制：3Pi+只有2560 bytes动态内存！
// 50 samples × 3 floats × 4 bytes = 600 bytes（安全范围内）
#define MAX_RESULTS 50
#define VARIABLES 3  // x, y, theta（不存time，用index计算）
float results[MAX_RESULTS][VARIABLES];
int results_index = 0;
unsigned long record_ts = 0;
#define RECORD_INTERVAL_MS 150  // 每150ms记录一次（50样本×150ms=7.5秒）

// ========== 状态机 ==========
#define STATE_WAIT        0
#define STATE_STRAIGHT    1
#define STATE_FINISHED    2
int state = STATE_WAIT;

// ========== 运动参数（学习自合并版本）==========
#define DRIVE_EST_MS    20UL
#define DRIVE_PID_MS    40UL
#define DRIVE_PWM_LIMIT 60       // 改：50→60

const float DEMAND_CS = -300.0f; // 改：-250→-300（更快）
const int kF_L = 16;             // 改：15→16
const int kF_R = 15;             // 改：14→15

float KP_L = 0.04025f, KI_L = 0.0f, KD_L = 0.0f;  // 改：KI=0
float KP_R = 0.07000f, KI_R = 0.0f, KD_R = 0.0f;  // 改：KI=0

// ========== 运行时变量 ==========
unsigned long drive_est_ts = 0, drive_pid_ts = 0;
long d_last_e0 = 0, d_last_e1 = 0;
float spdL_cps = 0.0f, spdR_cps = 0.0f;
float d_mL1 = 0, d_mL2 = 0, d_mR1 = 0, d_mR2 = 0;

float x0 = 0, y0 = 0;
unsigned long state_start_ts = 0;

// ========== 直线运动参数 ==========
#define STRAIGHT_DISTANCE_MM 450.0f  // 直线距离450mm（同直行.txt）

// ========== 辅助函数 ==========
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int iround(float v) {
  return (int)lroundf(v);
}

static inline float wrapPi(float a) {
  while (a > M_PI) a -= 2.0f * M_PI;
  while (a <= -M_PI) a += 2.0f * M_PI;
  return a;
}

// 非阻塞蜂鸣器（学习自合并版本）
unsigned long beep_off_time = 0;

void softBeep(int duration_ms) {
  tone(BUZZ_PIN, 800);
  beep_off_time = millis() + duration_ms;
}

void handleBeep() {
  if (beep_off_time != 0 && millis() >= beep_off_time) {
    noTone(BUZZ_PIN);
    beep_off_time = 0;
  }
}

void beep(int duration) {
  analogWrite(BUZZ_PIN, 120);
  delay(duration);
  analogWrite(BUZZ_PIN, 0);
}

// ========== 记录数据（精简版，节省内存）==========
void recordData() {
  if (results_index < MAX_RESULTS) {
    results[results_index][0] = kin.x;      // X坐标
    results[results_index][1] = kin.y;      // Y坐标
    results[results_index][2] = kin.theta;  // 角度
    results_index++;
  }
}

// ========== 输出数据（CSV格式）==========
void printResults() {
  Serial.println("\n========== LEADER DATA (CSV) ==========");
  Serial.println("Sample,X_mm,Y_mm,Theta_rad");
  
  for (int i = 0; i < results_index; i++) {
    Serial.print(i);
    Serial.print(",");
    Serial.print(results[i][0], 2);  // X
    Serial.print(",");
    Serial.print(results[i][1], 2);  // Y
    Serial.print(",");
    Serial.println(results[i][2], 4);  // Theta
  }
  
  Serial.println("========================================");
  Serial.print("Total samples: ");
  Serial.println(results_index);
  Serial.print("Record interval: ");
  Serial.print(RECORD_INTERVAL_MS);
  Serial.println(" ms");
}

// ========== 速度估计 ==========
void updateSpeedEstimate() {
  unsigned long now = millis();
  if (now - drive_est_ts >= DRIVE_EST_MS) {
    unsigned long dt = now - drive_est_ts;
    drive_est_ts = now;
    
    long e0 = count_e0, e1 = count_e1;
    long d0 = e0 - d_last_e0;
    long d1 = e1 - d_last_e1;
    d_last_e0 = e0;
    d_last_e1 = e1;
    
    spdR_cps = (float)d0 / ((float)dt / 1000.0f);
    spdL_cps = (float)d1 / ((float)dt / 1000.0f);
    
    kin.update();
  }
}

// ========== 直线PID控制 ==========
void driveStraight() {
  unsigned long now = millis();
  if (now - drive_pid_ts >= DRIVE_PID_MS) {
    drive_pid_ts = now;
    
    // 左右轮速度相同
    float demandL = DEMAND_CS;
    float demandR = DEMAND_CS;
    
    // 速度滤波（3点滑动平均）
    float measL = (spdL_cps + d_mL1 + d_mL2) / 3.0f;
    float measR = (spdR_cps + d_mR1 + d_mR2) / 3.0f;
    d_mL2 = d_mL1; d_mL1 = spdL_cps;
    d_mR2 = d_mR1; d_mR1 = spdR_cps;
    
    // PID计算
    float uL = left_pid.update(demandL, measL);
    float uR = right_pid.update(demandR, measR);
    float uLc = clampf(uL, -12.0f, 12.0f);  // 改：±10→±12
    float uRc = clampf(uR, -12.0f, 12.0f);
    
    // 前馈 + PID
    float baseL = (demandL >= 0.0f) ? kF_L : -kF_L;
    float baseR = (demandR >= 0.0f) ? kF_R : -kF_R;
    
    float pwmL = clampf(baseL + uLc, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
    float pwmR = clampf(baseR + uRc, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
    
    motors.setPWM(iround(pwmL), iround(pwmR));
  }
}


// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n========================================");
  Serial.println("  LEADER - Pure Line Sensor Version");
  Serial.println("  with Data Recording");
  Serial.println("========================================");
  
  // 初始化电机和编码器
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  delay(300);
  
  // 初始化运动学（从0度开始，沿X轴正向）
  kin.initialise(0.0f, 0.0f, 0.0f);
  
  // 初始化PID
  left_pid.initialise(KP_L, KI_L, KD_L);
  right_pid.initialise(KP_R, KI_R, KD_R);
  left_pid.reset();
  right_pid.reset();
  
  // ★★★ 关键：开启Line Sensor IR发射 ★★★
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, HIGH);
  Serial.println("Line IR: ON (EMIT_PIN = HIGH)");
  
  // 蜂鸣器
  pinMode(BUZZ_PIN, OUTPUT);
  
  Serial.println("\nMotion Plan (Pure Straight Line):");
  Serial.println("  1. Wait 1 second");
  Serial.println("  2. Straight forward 450mm");
  Serial.println("  3. Stop & output data");
  Serial.println("  Initial angle: 0 deg (along +X axis)");
  Serial.println("");
  
  beep(200);
  
  // 初始化时间戳
  drive_est_ts = drive_pid_ts = record_ts = millis();
  state_start_ts = millis();
  
  state = STATE_WAIT;
  results_index = 0;
}

// ============================================
// LOOP
// ============================================
void loop() {
  unsigned long now = millis();
  
  // 始终更新速度估计
  updateSpeedEstimate();
  
  // 定时记录数据（在运动状态时）
  if (state == STATE_STRAIGHT) {
    if (now - record_ts >= RECORD_INTERVAL_MS) {
      record_ts = now;
      recordData();
    }
  }
  
  // 状态机
  switch (state) {
    
    // ========== 等待1秒 ==========
    case STATE_WAIT:
      motors.setPWM(0, 0);
      
      // 上电后等待1秒自动启动
      if (now - state_start_ts >= 1000) {
        Serial.println("1 second elapsed! Starting straight motion...");
        beep(100);
        delay(100);
        beep(100);
        
        // 记录起始位置
        x0 = kin.x;
        y0 = kin.y;
        
        left_pid.reset();
        right_pid.reset();
        
        // 进入直线状态
        state = STATE_STRAIGHT;
        state_start_ts = now;
        
        Serial.println("Moving forward 450mm...");
      } else {
        // 等待提示
        static unsigned long last_print = 0;
        if (now - last_print >= 500) {
          last_print = now;
          int remaining = 1 - (now - state_start_ts) / 1000;
          Serial.print("Starting in ");
          Serial.print(remaining);
          Serial.println(" second...");
        }
      }
      break;
    
    // ========== 直线运动 ==========
    case STATE_STRAIGHT:
      {
        driveStraight();
        
        // 计算已行进距离
        float dx = kin.x - x0;
        float dy = kin.y - y0;
        float dist = sqrt(dx * dx + dy * dy);
        
        // 调试输出
        static unsigned long last_debug = 0;
        if (now - last_debug >= 200) {
          last_debug = now;
          Serial.print("[STRAIGHT] Dist=");
          Serial.print(dist, 1);
          Serial.print("mm / ");
          Serial.print(STRAIGHT_DISTANCE_MM, 1);
          Serial.print("mm | Speed L=");
          Serial.print(spdL_cps, 1);
          Serial.print(" R=");
          Serial.print(spdR_cps, 1);
          Serial.println(" cps");
        }
        
        // 达到目标距离，运动完成
        if (dist >= STRAIGHT_DISTANCE_MM) {
          Serial.println("Straight motion complete!");
          beep(200);
          
          motors.setPWM(0, 0);
          
          state = STATE_FINISHED;
          state_start_ts = now;
        }
      }
      break;
    
    // ========== 运动完成，输出数据 ==========
    case STATE_FINISHED:
      motors.setPWM(0, 0);
      
      // ★★★ 保持IR发射，让Follower继续跟随停止的Leader ★★★
      // 如果想让Follower停止，可以改成 digitalWrite(EMIT_PIN, LOW);
      
      // 只输出一次数据
      static bool data_printed = false;
      if (!data_printed) {
        // 输出所有记录的数据
        printResults();
        
        Serial.println("\nMotion finished.");
        Serial.println("IR still ON - Follower can continue to follow stopped Leader");
        Serial.println("Reset to run again.");
        
        data_printed = true;
        beep(50);
        delay(200);
        beep(50);
      }
      
      break;
  }
}

