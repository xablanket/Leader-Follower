#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "LineSensors.h"

#define BUMP_L 4
#define BUMP_R 5

#define BTN_PIN 14
#define BUZZER_PIN 6
#define LED_RED 17

Motors_c motors;
Kinematics_c kin;
PID_c pidL;
PID_c pidR;
LineSensors_c line_sensors;

unsigned long bump_base = 0;

int line_L_offset = 0;
int line_R_offset = 0;

unsigned long beep_off_time = 0;
unsigned long t0 = 0;

#define MAX_RESULTS 15
#define VARIABLES 10
float results[MAX_RESULTS][VARIABLES];
int results_index = 0;
unsigned long record_ts = 0;
unsigned long experiment_start_ts = 0;
#define RECORD_INTERVAL_MS 400
#define EXPERIMENT_DURATION_MS 6000

float rec_IR_center = 0;
int rec_L_signal = 0;
int rec_R_signal = 0;
float rec_speed_cmd = 0;
float rec_steer_cmd = 0;

bool experiment_running = false;
bool experiment_finished = false;

unsigned long ts_est = 0;
unsigned long ts_pid = 0;
float spdL = 0.0f;
float spdR = 0.0f;
long last_e0 = 0;
long last_e1 = 0;

float demand_cs = 0.0f;
unsigned long last_raw = 200;

const float KP_L = 0.03525f;
const float KI_L = 0.00005f;
const float KD_L = 0.0f;

const float KP_R = 0.05025f;
const float KI_R = 0.00005f;
const float KD_R = 0.0f;

const float kF_L = 13.5f;
const float kF_R = 13.5f;
const float PWM_MAX = 60.0f;

float diff_norm = 0.0f;
float diff_filtered = 0.0f;
float steer_term = 0.0f;
int diff_raw = 0;
float demand_L = 0.0f;
float demand_R = 0.0f; 

const float N_norm = 100.0f;
const int deadband_raw = 10;
const int clamp_raw = 80;
const float alpha = 0.4f;
const float K_steer = 240.0f;

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
  const float C     = 180.0f;
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

  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);

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

  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);

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

  while (digitalRead(BTN_PIN) == HIGH) handleBeep();
  softBeep(40);

  t0 = millis();

  last_e0 = count_e0;
  last_e1 = count_e1;
  ts_est = millis();
  ts_pid = millis();

  experiment_start_ts = millis();
  record_ts = millis();
  experiment_running = true;
  experiment_finished = false;
  results_index = 0;

  while (digitalRead(BTN_PIN) == LOW) handleBeep();
}

void loop() {
  digitalWrite(LED_RED, LOW);
  
  unsigned long now = millis();
  handleBeep();

  if (experiment_running && (now - experiment_start_ts >= EXPERIMENT_DURATION_MS)) {
    experiment_running = false;
    experiment_finished = true;
    motors.setPWM(0, 0);
    softBeep(100);
    delay(200);
    softBeep(100);
    printResults();
    Serial.println("\nExperiment finished. Reset to run again.");
  }
  
  if (experiment_finished) {
    return;
  }

  kin.update();

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

      rec_IR_center = (float)raw;
      rec_speed_cmd = demand_cs;

      demand_L = demand_cs;
      demand_R = demand_cs;
    }

    else {
      line_sensors.readSensorsADC();

      int L = line_sensors.readings[0] - line_L_offset;
      int R = line_sensors.readings[4] - line_R_offset;

      rec_L_signal = (L > 0) ? L : 0;
      rec_R_signal = (R > 0) ? R : 0;

      diff_raw = R - L;

      if (diff_raw > -deadband_raw && diff_raw < deadband_raw) diff_raw = 0;
      if (diff_raw > clamp_raw) diff_raw = clamp_raw;
      if (diff_raw < -clamp_raw) diff_raw = -clamp_raw;

      diff_norm = (float)diff_raw / N_norm;

      diff_filtered = alpha * diff_filtered + (1.0f - alpha) * diff_norm;

      steer_term = K_steer * diff_filtered;

      rec_steer_cmd = steer_term;

      demand_L = demand_cs - steer_term;
      demand_R = demand_cs + steer_term;
    }
  }

  if (experiment_running && (now - record_ts >= RECORD_INTERVAL_MS)) {
    record_ts = now;
    recordData();
  }
}

void recordData() {
  if (results_index < MAX_RESULTS) {
    results[results_index][0] = kin.x;
    results[results_index][1] = kin.y;
    results[results_index][2] = kin.theta;
    results[results_index][3] = spdL;
    results[results_index][4] = spdR;
    results[results_index][5] = rec_IR_center;
    results[results_index][6] = (float)rec_L_signal;
    results[results_index][7] = (float)rec_R_signal;
    results[results_index][8] = rec_speed_cmd;
    results[results_index][9] = rec_steer_cmd;
    results_index++;
  }
}

void printResults() {
  Serial.println("\n===========================DATA================");
  Serial.println("Sample,X_mm,Y_mm,Theta_rad,SpdL_cps,SpdR_cps,IR_center,L_signal,R_signal,Speed_cmd,Steer_cmd");
  
  for (int i = 0; i < results_index; i++) {
    Serial.print(i);
    Serial.print(",");
    Serial.print(results[i][0], 2);
    Serial.print(",");
    Serial.print(results[i][1], 2);
    Serial.print(",");
    Serial.print(results[i][2], 4);
    Serial.print(",");
    Serial.print(results[i][3], 1);
    Serial.print(",");
    Serial.print(results[i][4], 1);
    Serial.print(",");
    Serial.print(results[i][5], 1);
    Serial.print(",");
    Serial.print((int)results[i][6]);
    Serial.print(",");
    Serial.print((int)results[i][7]);
    Serial.print(",");
    Serial.print(results[i][8], 1);
    Serial.print(",");
    Serial.println(results[i][9], 2);
  }
}
