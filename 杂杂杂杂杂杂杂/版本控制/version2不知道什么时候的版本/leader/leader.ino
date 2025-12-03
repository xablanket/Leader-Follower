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

// ========== IR时序控制 ==========
enum IR_MODE {
  IR_LINE_SENSOR,    // Line sensor IR发射 (EMIT_PIN = HIGH)
  IR_BUMP_SENSOR     // Bump sensor IR发射 (EMIT_PIN = LOW)
};

IR_MODE current_ir_mode = IR_LINE_SENSOR;
unsigned long ir_switch_ts = 0;

// ========== 时序参数 ==========
// 根据你的需求调整这些值
#define LINE_IR_DURATION 20     // Line sensor IR开启时间 (ms) - 用于方向检测
#define BUMP_IR_DURATION 40     // Bump sensor IR开启时间 (ms) - 用于距离检测
#define IR_SETTLE_TIME 1        // IR切换后稳定时间 (ms)
// 总周期: 60ms，更新率: 16.7Hz

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

// ========== IR时序切换管理 ==========
void manageIRSwitching() {
  unsigned long now = millis();
  unsigned long elapsed = now - ir_switch_ts;
  
  if (current_ir_mode == IR_LINE_SENSOR) {
    // 当前是Line Sensor模式 (EMIT_PIN = HIGH)
    if (elapsed >= LINE_IR_DURATION) {
      // 切换到Bump Sensor模式
      current_ir_mode = IR_BUMP_SENSOR;
      digitalWrite(EMIT_PIN, LOW);   // LOW = 开启Bump IR
      ir_switch_ts = now;
      delayMicroseconds(IR_SETTLE_TIME * 1000);  // 等待IR稳定
    }
  } 
  else if (current_ir_mode == IR_BUMP_SENSOR) {
    // 当前是Bump Sensor模式 (EMIT_PIN = LOW)
    if (elapsed >= BUMP_IR_DURATION) {
      // 切换回Line Sensor模式
      current_ir_mode = IR_LINE_SENSOR;
      digitalWrite(EMIT_PIN, HIGH);  // HIGH = 开启Line IR
      ir_switch_ts = now;
      delayMicroseconds(IR_SETTLE_TIME * 1000);  // 等待IR稳定
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n========================================");
  Serial.println("*** Leader机器人 - 双IR时序系统 ***");
  Serial.println("模式：交替发射Line Sensor和Bump Sensor IR");
  Serial.println("========================================");
  
  motors.initialise();
  
  // ===== 初始化IR时序系统 =====
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, HIGH);  // 开始时开启Line Sensor IR
  current_ir_mode = IR_LINE_SENSOR;
  ir_switch_ts = millis();
  
  Serial.println("✓ 双IR时序系统已初始化");
  Serial.println("IR发射时序：");
  Serial.print("  - Line Sensor IR (HIGH): "); 
  Serial.print(LINE_IR_DURATION); 
  Serial.println(" ms");
  Serial.print("  - Bump Sensor IR (LOW):  "); 
  Serial.print(BUMP_IR_DURATION); 
  Serial.println(" ms");
  Serial.print("  - 切换周期: "); 
  Serial.print(LINE_IR_DURATION + BUMP_IR_DURATION); 
  Serial.println(" ms");
  Serial.print("  - 更新频率: "); 
  Serial.print(1000.0 / (LINE_IR_DURATION + BUMP_IR_DURATION), 1); 
  Serial.println(" Hz");
  Serial.println("");
  Serial.println("📡 Follower可以使用：");
  Serial.println("  ✓ Line Sensor (5个) - 检测方向，车头对齐");
  Serial.println("  ✓ Bump Sensor (2个) - 检测距离，保持间距");
  Serial.println("========================================");
}

void loop() {
  // ========== 优先级1：IR时序管理（最重要）==========
  manageIRSwitching();
  
  // ========== 优先级2：运动控制 ==========
  motors.setPWM(0, 0);  // Leader保持静止
  // 如果需要让Leader移动，在这里添加运动控制代码
  
  // ========== 优先级3：调试输出（每1秒）==========
  static unsigned long debug_ts = 0;
  if (millis() - debug_ts >= 1000) {
    debug_ts = millis();
    
    Serial.print("IR模式: ");
    if (current_ir_mode == IR_LINE_SENSOR) {
      Serial.print("LINE_SENSOR (HIGH)");
    } else {
      Serial.print("BUMP_SENSOR (LOW) ");
    }
    
    // 计算时序占空比
    float line_duty = (float)LINE_IR_DURATION / (LINE_IR_DURATION + BUMP_IR_DURATION) * 100.0f;
    float bump_duty = (float)BUMP_IR_DURATION / (LINE_IR_DURATION + BUMP_IR_DURATION) * 100.0f;
    
    Serial.print(" | 占空比: Line=");
    Serial.print(line_duty, 1);
    Serial.print("% Bump=");
    Serial.print(bump_duty, 1);
    Serial.println("%");
  }
  
  // 非阻塞延时，让出CPU时间
  delayMicroseconds(100);
}