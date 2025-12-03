/*
 * ============================================
 * LEADER ROBOT - 纯Line方案 + 数据记录
 * ============================================
 * 
 * 功能：
 * 1. 发射Line Sensor IR (EMIT_PIN = HIGH)
 * 2. 按预定路线运动（直线 → 弧线）
 * 3. 记录kinematics数据（x, y, theta）
 * 4. 运动结束后通过串口输出CSV数据
 * 
 * 运动路线：
 * - Stage 0: 等待3秒（让Follower准备）
 * - Stage 1: 直线前进（约30cm）
 * - Stage 2: 弧线运动（半径300mm，左转90度）
 * - Stage 3: 停止，输出数据
 * 
 * 日期: 2025-12-01
 */

#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "LineSensors.h"

// ========== 引脚定义 ==========
#define EMIT_PIN    11
#define BUZZ_PIN    6
#define BTN_PIN     14     // 按钮引脚（学习自合并版本）

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
#define STATE_ARC         2
#define STATE_FINISHED    3
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

float x0 = 0, y0 = 0, theta0 = 0;
unsigned long state_start_ts = 0;

// ========== 直线运动参数 ==========
#define STRAIGHT_DISTANCE_MM 300.0f  // 直线距离30cm

// ========== 弧线运动参数（学习自合并版本）==========
#define ARC_RADIUS_MM 511.0f         // 改：300→450（同合并版本）
#define ARC_ANGLE_RAD (M_PI / 3.0f)  // 改：90度→60度（同合并版本）

// 轮距补偿（学习自合并版本）
const float SCALE_L = 0.904f;        // 新增：左轮补偿
const float SCALE_R = 1.00f;         // 新增：右轮补偿

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

// ========== 弧线PID控制（学习自合并版本）==========
void driveArc() {
  unsigned long now = millis();
  if (now - drive_pid_ts >= DRIVE_PID_MS) {
    drive_pid_ts = now;
    
    // 弧线运动：内外轮速度不同（学习自合并版本）
    const float wheel_sep_local = 70.0f;  // 轮距
    float R_L = ARC_RADIUS_MM - wheel_sep_local;  // 内轮
    float R_R = ARC_RADIUS_MM + wheel_sep_local;  // 外轮
    
    float v_center = DEMAND_CS;
    
    // ★新增：轮距补偿（学习自合并版本）★
    float demandL = v_center * (R_L / ARC_RADIUS_MM) * SCALE_L;
    float demandR = v_center * (R_R / ARC_RADIUS_MM) * SCALE_R;
    
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
  
  // 初始化运动学（从30度开始，同30度转弯.txt）
  kin.initialise(0.0f, 0.0f, M_PI/6.0f);  // 改：从30度开始
  
  // 初始化PID
  left_pid.initialise(KP_L, KI_L, KD_L);
  right_pid.initialise(KP_R, KI_R, KD_R);
  left_pid.reset();
  right_pid.reset();
  
  // ★★★ 关键：开启Line Sensor IR发射 ★★★
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, HIGH);
  Serial.println("Line IR: ON (EMIT_PIN = HIGH)");
  
  // 蜂鸣器和按钮
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  
  Serial.println("\nMotion Plan (same as 30度转弯.txt):");
  Serial.println("  1. Wait 1s");
  Serial.println("  2. Arc only (R=450mm, turn 60 deg left)");
  Serial.println("  3. Stop & output data");
  Serial.println("  Initial angle: 30 deg (pi/6)");
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
  if (state == STATE_STRAIGHT || state == STATE_ARC) {
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
      
      if (now - state_start_ts >= 1000) {
        Serial.println("Starting arc motion...");
        beep(100);
        delay(100);
        beep(100);
        
        x0 = kin.x;
        y0 = kin.y;
        theta0 = kin.theta;  // 记录起始角度（30度）
        
        left_pid.reset();
        right_pid.reset();
        
        // 直接进入弧线状态（跳过直线，同30度转弯.txt）
        state = STATE_ARC;
        state_start_ts = now;
      } else {
        static unsigned long last_print = 0;
        if (now - last_print >= 500) {
          last_print = now;
          Serial.println("Starting in 1 second...");
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
          Serial.println("mm");
        }
        
        // 达到目标距离
        if (dist >= STRAIGHT_DISTANCE_MM) {
          Serial.println("Straight phase complete. Starting arc...");
          beep(100);
          
          // 记录弧线起始角度
          theta0 = kin.theta;
          
          left_pid.reset();
          right_pid.reset();
          
          state = STATE_ARC;
          state_start_ts = now;
        }
      }
      break;
    
    // ========== 弧线运动（角度计算同30度转弯.txt）==========
    case STATE_ARC:
      {
        driveArc();
        
        // 角度计算（同30度转弯.txt）
        float dtheta = kin.theta - theta0;
        // wrap到[-π, π]
        while (dtheta > M_PI) dtheta -= 2*M_PI;
        while (dtheta < -M_PI) dtheta += 2*M_PI;
        
        // 调试输出
        static unsigned long last_debug_arc = 0;
        if (now - last_debug_arc >= 200) {
          last_debug_arc = now;
          Serial.print("[ARC] dtheta=");
          Serial.print(dtheta * 180.0f / M_PI, 1);
          Serial.print("deg / target=");
          Serial.print(-60.0f);  // -π/3 = -60度
          Serial.println("deg");
        }
        
        // 达到目标角度：转过-60度（-π/3）停止（同30度转弯.txt）
        if (dtheta <= -M_PI/3.0f) {
          Serial.println("Arc phase complete!");
          beep(200);
          
          motors.setPWM(0, 0);
          state = STATE_FINISHED;
        }
      }
      break;
    
    // ========== 运动完成，输出数据 ==========
    case STATE_FINISHED:
      motors.setPWM(0, 0);
      
      // ★★★ 关闭IR让Follower知道结束了 ★★★
      digitalWrite(EMIT_PIN, LOW);
      Serial.println("IR OFF - Follower will stop due to signal lost");
      
      // 输出所有记录的数据
      printResults();
      
      Serial.println("\nMotion finished. Reset to run again.");
      
      delay(5000);  // 5秒后重新输出
      break;
  }
}

