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

unsigned long bump_base = 0;
int line_L_base = 0;
int line_R_base = 0;

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

const float KP_R = 0.07000f;
const float KI_R = 0.00005f;
const float KD_R = 0.0f;

const float kF_L = 16.0f;
const float kF_R = 10.0f;
const float PWM_MAX = 120.0f;

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

bool isJumpCloseAndShift(unsigned long last_raw_val, unsigned long raw) {
  if (last_raw_val >= 150 && last_raw_val <= 200 && raw >= 300 && (raw - last_raw_val) >= 80) return true;
  return false;
}

float mapIRtoCS(unsigned long raw) {
  const float V_MIN = 0.0f;
  const float V_MAX = 250.0f;
  const float C = 230.0f;
  const float K = 20.0f;

  if (raw <= 150) return 0.0f;

  float x = ((float)raw - C) / K;
  float s = 1.0f / (1.0f + expf(-x));
  float v = V_MIN + (V_MAX - V_MIN) * s;

  if (v < 0.0f) v = 0.0f;
  if (v > V_MAX) v = V_MAX;

  return v;
}

float mapIRtoCS_withSafety(unsigned long raw, unsigned long last_raw_val) {
  const float SAFE_SLOW = 40.0f;
  if (isJumpCloseAndShift(last_raw_val, raw)) return SAFE_SLOW;
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

  while (digitalRead(BTN_PIN) == HIGH) handleBeep();
  softBeep(40);
  bump_base = readBump(BUMP_L);
  Serial.print("BUMP_BASE:");
  Serial.println(bump_base);
  softBeep(40);
  while (digitalRead(BTN_PIN) == LOW) handleBeep();

  while (digitalRead(BTN_PIN) == HIGH) handleBeep();
  softBeep(40);
  line_sensors.readSensorsADC();
  line_L_base = line_sensors.readings[0];
  line_R_base = line_sensors.readings[4];
  Serial.print("LINE_BASE_L:");
  Serial.println(line_L_base);
  Serial.print("LINE_BASE_R:");
  Serial.println(line_R_base);
  softBeep(40);
  while (digitalRead(BTN_PIN) == LOW) handleBeep();

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

  if (now - ts_pid >= 40) {
    ts_pid = now;
    float uL = pidL.update(demand_cs, spdL);
    float uR = pidR.update(demand_cs, spdR);
    float pwmL = kF_L + uL;
    float pwmR = kF_R + uR;

    if (pwmL < 0.0f) pwmL = 0.0f;
    if (pwmR < 0.0f) pwmR = 0.0f;
    if (pwmL > PWM_MAX) pwmL = PWM_MAX;
    if (pwmR > PWM_MAX) pwmR = PWM_MAX;

    motors.setPWM(pwmL, pwmR);
  }

  unsigned long t = (now - t0) % 200;
  bool bump_window = (t < 100);
  static bool last_window = false;

  if (bump_window != last_window) {
    last_window = bump_window;
    if (bump_window) {
      unsigned long rawL = readBump(BUMP_L);
      unsigned long rawR = readBump(BUMP_R);
      unsigned long raw = (rawL + rawR) / 2;
      unsigned long prev_raw = last_raw;
      last_raw = raw;
      demand_cs = mapIRtoCS_withSafety(raw, prev_raw);
    }
  }
}
