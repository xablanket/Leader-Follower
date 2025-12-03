#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "LineSensors.h"

#define EMIT_PIN 11
#define BUMP_L 4
#define BUMP_R 5

Motors_c motors;
Kinematics_c kin;
PID_c left_pid;
PID_c right_pid;
LineSensors_c line_sensors;

// ========== IR时序控制参数 ==========
enum IR_MODE {
  IR_LINE_SENSOR,    // Line sensor IR发射 (EMIT_PIN = HIGH)
  IR_BUMP_SENSOR     // Bump sensor IR发射 (EMIT_PIN = LOW)
};

IR_MODE current_ir_mode = IR_LINE_SENSOR;
unsigned long ir_switch_ts = 0;

// ========== 时序参数（针对速度PID优化）==========
#define LINE_IR_DURATION 50     // Line sensor IR开启时间 (方向控制)
#define BUMP_IR_DURATION 10     // Bump sensor IR开启时间 (速度PID)
#define IR_SETTLE_TIME 2        // IR切换后稳定时间
// 总周期: 60ms, Bump更新率: 16.7Hz (足够速度PID)

// Bump传感器数据
unsigned long bump_left = 0;
unsigned long bump_right = 0;
unsigned long bump_avg = 0;

#define DRIVE_EST_MS 20UL
#define DRIVE_PID_MS 40UL
#define DRIVE_PWM_LIMIT 60

const float DEMAND_CS = -300.0f;  // 反向逻辑 = 前进
const int kF_L = 16;
const int kF_R = 15;

float KP_L = 0.04080f, KI_L = 0.0f, KD_L = 0.0f;  // KP_L = 右轮
float KP_R = 0.07000f, KI_R = 0.0f, KD_R = 0.0f;  // KP_R = 左轮

unsigned long drive_est_ts = 0, drive_pid_ts = 0;
long d_last_e0 = 0, d_last_e1 = 0;
float spdL_cps = 0.0f, spdR_cps = 0.0f;
float d_mL1 = 0, d_mL2 = 0, d_mR1 = 0, d_mR2 = 0;

float theta0 = 0.0f;
float stage = 0;

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

// ========== Bump传感器读取（elapsed time方法）==========
unsigned long readBump(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);      // 充电
  delayMicroseconds(10);
  pinMode(pin, INPUT);          // 开始放电
  
  unsigned long t0 = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t0 > 4500) break;  // 超时保护
  }
  return micros() - t0;  // 返回放电时间
}

// ========== IR时序切换管理 ==========
void manageIRSwitching() {
  unsigned long now = millis();
  unsigned long elapsed = now - ir_switch_ts;
  
  if (current_ir_mode == IR_LINE_SENSOR) {
    // 当前是Line Sensor模式
    if (elapsed >= LINE_IR_DURATION) {
      // 切换到Bump Sensor模式
      current_ir_mode = IR_BUMP_SENSOR;
      digitalWrite(EMIT_PIN, LOW);   // LOW = Bump IR开启
      ir_switch_ts = now;
      delay(IR_SETTLE_TIME);         // 等待IR稳定
      
      // 立即读取bump传感器
      bump_left = readBump(BUMP_L);
      bump_right = readBump(BUMP_R);
      bump_avg = (bump_left + bump_right) / 2;
    }
  } 
  else if (current_ir_mode == IR_BUMP_SENSOR) {
    // 当前是Bump Sensor模式
    if (elapsed >= BUMP_IR_DURATION) {
      // 切换回Line Sensor模式
      current_ir_mode = IR_LINE_SENSOR;
      digitalWrite(EMIT_PIN, HIGH);  // HIGH = Line IR开启
      ir_switch_ts = now;
      delay(IR_SETTLE_TIME);         // 等待IR稳定
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Leader with Dual IR System ===");
  
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  delay(300);

  kin.initialise(0.0f, 0.0f, 0.0f);
  theta0 = kin.theta;

  // 初始化Bump传感器引脚
  pinMode(BUMP_L, INPUT);
  pinMode(BUMP_R, INPUT);
  
  // 初始化EMIT_PIN，默认开启Line Sensor IR
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, HIGH);  // 开始时Line IR开启
  current_ir_mode = IR_LINE_SENSOR;
  ir_switch_ts = millis();
  
  // 初始化Line传感器
  line_sensors.initialiseForADC();

  // 初始化PID
  left_pid.initialise(KP_L, KI_L, KD_L);
  right_pid.initialise(KP_R, KI_R, KD_R);
  left_pid.reset();
  right_pid.reset();

  drive_est_ts = drive_pid_ts = millis();
  
  Serial.println("✓ 系统初始化完成");
  Serial.println("✓ IR时序切换已启用：");
  Serial.print("  - Line Sensor IR: "); Serial.print(LINE_IR_DURATION); Serial.println(" ms");
  Serial.print("  - Bump Sensor IR: "); Serial.print(BUMP_IR_DURATION); Serial.println(" ms");
  Serial.print("  - 切换周期: "); Serial.print(LINE_IR_DURATION + BUMP_IR_DURATION); Serial.println(" ms");
  Serial.println("========================================\n");
}

void loop() {
  // ========== IR时序管理（优先级最高）==========
  manageIRSwitching();
  
  // ========== 调试输出（每500ms）==========
  static unsigned long debug_ts = 0;
  if (millis() - debug_ts >= 500) {
    debug_ts = millis();
    
    Serial.print("IR模式: ");
    if (current_ir_mode == IR_LINE_SENSOR) {
      Serial.print("LINE_SENSOR");
    } else {
      Serial.print("BUMP_SENSOR");
    }
    Serial.print(" | Bump: L="); Serial.print(bump_left);
    Serial.print(" R="); Serial.print(bump_right);
    Serial.print(" AVG="); Serial.println(bump_avg);
  }
  
  unsigned long now = millis();

  // ========== 速度估计 ==========
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

  // -------- Stage 0: 原地右转30度 --------
  if (stage == 0) {
    float target = M_PI / 6.0f;  // 30度
    float err = wrapPi(target - kin.theta);

    if (fabs(err) < (2.0f * M_PI / 180.0f)) {
      motors.setPWM(0, 0);
      delay(500);
      stage = 1;
      return;
    }

    float turn_pwm = 15;
    motors.setPWM(-turn_pwm, +turn_pwm);  // 原地右转
    return;
  }

  // -------- Stage 1: 暂停 --------
  if (stage == 1) {
    motors.setPWM(0, 0);
    delay(500);
    theta0 = kin.theta;
    stage = 2;
    return;
  }

  // -------- Stage 2: 弧线运动（左转60度后停止）--------
  float dtheta = wrapPi(kin.theta - theta0);

  if (dtheta <= -M_PI / 3.0f) {  // 累计左转60度
    motors.setPWM(0, 0);
    Serial.println("=== 运动完成 ===");
    while (1) {}  // 停止
  }

  // ========== PID弧线控制 ==========
  if (now - drive_pid_ts >= DRIVE_PID_MS) {
    drive_pid_ts = now;

    const float R_arc = 450.0f;  // 弧线半径450mm
    const float R_L = R_arc - wheel_sep;  // 内轮半径
    const float R_R = R_arc + wheel_sep;  // 外轮半径

    float v_center = DEMAND_CS;  // 中心速度

    // ★补偿层★
    const float SCALE_L = 0.904f;  // 右轮（内轮）稍微减速
    const float SCALE_R = 1.00f;   // 左轮（外轮）不变

    float demandL = v_center * (R_L / R_arc) * SCALE_L;   // 右轮（内轮）
    float demandR = v_center * (R_R / R_arc) * SCALE_R;   // 左轮（外轮）

    // 速度滤波（3点移动平均）
    float measL = (spdL_cps + d_mL1 + d_mL2) / 3.0f;
    float measR = (spdR_cps + d_mR1 + d_mR2) / 3.0f;
    d_mL2 = d_mL1; d_mL1 = spdL_cps;
    d_mR2 = d_mR1; d_mR1 = spdR_cps;

    // PID计算
    float uL = left_pid.update(demandL, measL);
    float uR = right_pid.update(demandR, measR);
    float uLc = clampf(uL, -12.0f, 12.0f);
    float uRc = clampf(uR, -12.0f, 12.0f);

    // 前馈 + PID输出
    float baseL = (demandL >= 0.0f) ? kF_L : -kF_L;
    float baseR = (demandR >= 0.0f) ? kF_R : -kF_R;

    float pwmL = clampf(baseL + uLc, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
    float pwmR = clampf(baseR + uRc, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);

    motors.setPWM(iround(pwmL), iround(pwmR));
  }
}
