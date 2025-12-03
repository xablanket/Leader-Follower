#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "LineSensors.h"

#define BUMP_L 4
#define BUMP_R 5

#define BTN_PIN 14
#define BUZZER_PIN 6

Motors_c motors;
Kinematics_c kin;
PID_c pidL;
PID_c pidR;
LineSensors_c line_sensors;

unsigned long bump_base = 0;    // 只用于打印，不参与计算

int line_L_offset = 0;
int line_R_offset = 0;

unsigned long beep_off_time = 0;
unsigned long t0 = 0;

unsigned long ts_est = 0;
unsigned long ts_pid = 0;
float spdL = 0.0f;
float spdR = 0.0f;
long last_e0 = 0;
long last_e1 = 0;

float demand_cs = 0.0f;
unsigned long last_raw = 200;

const float KP_L = 0.04025f;
const float KI_L = 0.00005f;
const float KD_L = 0.0f;

const float KP_R = 0.05025f;
const float KI_R = 0.00005f;
const float KD_R = 0.0f;

const float kF_L = 13.5f;
const float kF_R = 13.5f;
const float PWM_MAX = 60.0f;

// steering 相关
float diff_norm = 0.0f;
float diff_filtered = 0.0f;
float steer_term = 0.0f;
int diff_raw = 0;
float demand_L = 0.0f;
float demand_R = 0.0f; 

const float N_norm = 100.0f;
const int deadband_raw = 10;
const int clamp_raw = 60;
const float alpha = 0.5f;
const float K_steer = 120.0f;

unsigned long readBump(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);
  unsigned long t_start = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t_start > 4500) break;
  }
  return micros() - t_start;
}

void softBeep(int duration_ms) {
  tone(BUZZER_PIN, 800);
  beep_off_time = millis() + duration_ms;
}

void handleBeep() {
  if (beep_off_time != 0 && millis() >= beep_off_time) {
    noTone(BUZZER_PIN);
    beep_off_time = 0;
  }
}

bool isJumpCloseAndShift(unsigned long a, unsigned long b) {
  return (a >= 150 && a <= 200 && b >= 300 && (b - a) >= 80);
}

float mapIRtoCS(unsigned long raw) {
  const float V_MIN = 0.0f;
  const float V_MAX = 250.0f;
  const float C     = 150.0f;
  const float K     = 20.0f;

  if (raw <= 150) return 0.0f;

  float x = ((float)raw - C) / K;
  float s = 1.0f / (1.0f + expf(-x));
  float v = V_MIN + (V_MAX - V_MIN) * s;

  if (v < 0.0f) v = 0.0f;
  if (v > V_MAX) v = V_MAX;
  return v;
}

float mapIRtoCS_withSafety(unsigned long raw, unsigned long last) {
  const float SAFE_SLOW = 40.0f;
  if (isJumpCloseAndShift(last, raw)) return SAFE_SLOW;
  return mapIRtoCS(raw);
}

void setup() {
  Serial.begin(115200);

  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  pidL.initialise(KP_L, KI_L, KD_L);
  pidR.initialise(KP_R, KI_R, KD_R);

  kin.initialise(0.0f, 0.0f, 0.0f);

  pinMode(BUMP_L, INPUT);
  pinMode(BUMP_R, INPUT);

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  softBeep(40);

  // 第一次按钮：bump 标定（单帧，只打印，不参与计算）
  while (digitalRead(BTN_PIN) == HIGH) handleBeep();
  softBeep(40);

  bump_base = readBump(BUMP_L);
  Serial.print("BUMP_BASE:");
  Serial.println(bump_base);

  softBeep(40);
  while (digitalRead(BTN_PIN) == LOW) handleBeep();

  // 第二次按钮：line offset（30 帧平均）
  while (digitalRead(BTN_PIN) == HIGH) handleBeep();
  softBeep(40);

  {
    long sL = 0;
    long sR = 0;
    for (int i = 0; i < 30; i++) {
      line_sensors.readSensorsADC();
      sL += line_sensors.readings[0];
      sR += line_sensors.readings[4];
    }
    line_L_offset = sL / 30;
    line_R_offset = sR / 30;
  }

  Serial.print("LINE_OFF_L:");
  Serial.println(line_L_offset);
  Serial.print("LINE_OFF_R:");
  Serial.println(line_R_offset);

  softBeep(40);
  while (digitalRead(BTN_PIN) == LOW) handleBeep();

  // 第三次按钮：进入主循环
  while (digitalRead(BTN_PIN) == HIGH) handleBeep();
  softBeep(40);

  t0 = millis();

  last_e0 = count_e0;
  last_e1 = count_e1;
  ts_est = millis();
  ts_pid = millis();

  while (digitalRead(BTN_PIN) == LOW) handleBeep();
}

void loop() {
  unsigned long now = millis();
  handleBeep();

  // 20ms 速度估计
  if (now - ts_est >= 20) {
    unsigned long dt = now - ts_est;
    long e0 = count_e0;
    long e1 = count_e1;
    float dt_sec = (float)dt / 1000.0f;
    if (dt_sec > 0.0f) {
      spdR = (float)(e0 - last_e0) / dt_sec;
      spdL = (float)(e1 - last_e1) / dt_sec;
    }
    last_e0 = e0;
    last_e1 = e1;
    ts_est = now;
  }

  // 40ms PID 内环
  if (now - ts_pid >= 40) {
    ts_pid = now;

    float uL = pidL.update(demand_L, spdL);
    float uR = pidR.update(demand_R, spdR);

    float pwmL = kF_L + uL;
    float pwmR = kF_R + uR;

    if (pwmL < 0.0f) pwmL = 0.0f;
    if (pwmR < 0.0f) pwmR = 0.0f;
    if (pwmL > PWM_MAX) pwmL = PWM_MAX;
    if (pwmR > PWM_MAX) pwmR = PWM_MAX;

    motors.setPWM(pwmL, pwmR);
  }

  // 200ms bump/line window
  unsigned long t = (now - t0) % 200;
  bool bump_window = (t < 100);
  static bool last_window = false;

  if (bump_window != last_window) {
    last_window = bump_window;

    // bump window：距离控制 —— 完全恢复原始 FOLLOWER 逻辑（raw 不做差分）
    if (bump_window) {
      unsigned long rawL = readBump(BUMP_L);
      unsigned long rawR = readBump(BUMP_R);
      unsigned long raw = (rawL + rawR) / 2;

      unsigned long prev_raw = last_raw;
      last_raw = raw;

      demand_cs = mapIRtoCS_withSafety(raw, prev_raw);

      // steering 用的 demand_L / demand_R 在 line window 内更新
      // 这里先把两边的目标设置成同一前进速度
      demand_L = demand_cs;
      demand_R = demand_cs;
    }

    // line window：方向控制（line offset + steering）
    else {
      line_sensors.readSensorsADC();

      int L = line_sensors.readings[0] - line_L_offset;
      int R = line_sensors.readings[4] - line_R_offset;

      diff_raw = R - L;

      if (diff_raw > -deadband_raw && diff_raw < deadband_raw) diff_raw = 0;
      if (diff_raw > clamp_raw) diff_raw = clamp_raw;
      if (diff_raw < -clamp_raw) diff_raw = -clamp_raw;

      diff_norm = (float)diff_raw / N_norm;

      diff_filtered = alpha * diff_filtered + (1.0f - alpha) * diff_norm;

      steer_term = K_steer * diff_filtered;

      demand_L = demand_cs - steer_term;
      demand_R = demand_cs + steer_term;
    }
  }
}
