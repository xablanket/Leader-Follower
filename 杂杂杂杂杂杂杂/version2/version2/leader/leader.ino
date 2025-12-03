#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "LineSensors.h"

Motors_c motors;
Kinematics_c kin;
PID_c left_pid;
PID_c right_pid;

LineSensors_c line_sensors;

// ===== Constants you tuned =====
#define DRIVE_EST_MS 20UL
#define DRIVE_PID_MS 40UL
#define DRIVE_PWM_LIMIT 60

const float DEMAND_CS = -300.0f;   
const int kF_L = 16;
const int kF_R = 15;

float KP_L = 0.04400f, KI_L = 0.00000f, KD_L = 0.0f;                    //左为右 右为左
float KP_R = 0.07000f, KI_R = 0.00000f, KD_R = 0.0f;

// ===== Runtime vars =====
unsigned long drive_est_ts = 0, drive_pid_ts = 0;
long d_last_e0 = 0, d_last_e1 = 0;
float spdL_cps = 0.0f, spdR_cps = 0.0f;
float d_mL1 = 0, d_mL2 = 0, d_mR1 = 0, d_mR2 = 0;

float x0 = 0, y0 = 0;     // 起点记录
bool started = false;

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int iround(float v) {
  return (int)lroundf(v);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n========================================");
  Serial.println("*** Leader机器人 - Leader-Follower项目 ***");
  Serial.println("模式：静止发射IR信号");
  Serial.println("========================================");
  
  motors.initialise();
  
  // ===== 重要：开启IR发射器，让Follower能检测到 =====
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, HIGH);  // 持续发射IR信号
  Serial.println("✓ IR发射器已开启（EMIT_PIN = HIGH）");
  Serial.println("  Follower将能够检测到本机器人");
  Serial.println("✓ Leader保持静止，只发射IR信号");
  Serial.println("  你可以手动转动Leader测试Follower跟随");
  Serial.println("========================================");
}

void loop() {
  // Leader保持静止，只发射IR信号
  // 不执行任何运动控制
  
  motors.setPWM(0, 0);  // 确保电机停止
  
  // 定期输出状态确认IR仍在发射
  static unsigned long debug_ts = 0;
  if (millis() - debug_ts >= 2000) {
    debug_ts = millis();
    Serial.println("Leader静止中 | IR发射器: ON | 等待Follower检测...");
  }
  
  delay(100);
}