// FOLLOWER_LINE.txt
// Based on FOLLOWER_BUMP: 3-stage calibration, 200ms timing,
// bump reads preserved (no print), line window prints L/R/diff.

#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "LineSensors.h"

#define BUMP_L 4
#define BUMP_R 5
#define EMIT_PIN 11

#define BTN_PIN 14
#define BUZZER_PIN 6

Motors_c motors;
Kinematics_c kin;
PID_c pidL, pidR;
LineSensors_c line_sensors;

unsigned long bump_base = 0;
int line_L_base = 0;
int line_R_base = 0;

unsigned long beep_off_time = 0;
unsigned long t0 = 0;

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

void softBeep(int duration_ms) {
  tone(BUZZER_PIN, 800);
  beep_off_time = millis() + duration_ms;
}

void setup() {
  Serial.begin(115200);

  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  pidL.initialise(0.04025, 0.00005, 0);
  pidR.initialise(0.07000, 0.00005, 0);

  kin.initialise(0, 0, 0);

  pinMode(BUMP_L, INPUT);
  pinMode(BUMP_R, INPUT);

  pinMode(EMIT_PIN, INPUT);

  line_sensors.initialiseForADC();

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  softBeep(40);

  while (digitalRead(BTN_PIN) == HIGH) {
    if (beep_off_time && millis() >= beep_off_time) {
      noTone(BUZZER_PIN);
      beep_off_time = 0;
    }
  }
  softBeep(40);
  bump_base = readBump(BUMP_L);
  Serial.print("BUMP_BASE:");
  Serial.println(bump_base);
  softBeep(40);
  while (digitalRead(BTN_PIN) == LOW) {
    if (beep_off_time && millis() >= beep_off_time) {
      noTone(BUZZER_PIN);
      beep_off_time = 0;
    }
  }

  while (digitalRead(BTN_PIN) == HIGH) {
    if (beep_off_time && millis() >= beep_off_time) {
      noTone(BUZZER_PIN);
      beep_off_time = 0;
    }
  }
  softBeep(40);
  line_sensors.readSensorsADC();
  line_L_base = line_sensors.readings[0];
  line_R_base = line_sensors.readings[4];
  Serial.print("LINE_BASE_L:");
  Serial.println(line_L_base);
  Serial.print("LINE_BASE_R:");
  Serial.println(line_R_base);
  softBeep(40);
  while (digitalRead(BTN_PIN) == LOW) {
    if (beep_off_time && millis() >= beep_off_time) {
      noTone(BUZZER_PIN);
      beep_off_time = 0;
    }
  }

  while (digitalRead(BTN_PIN) == HIGH) {
    if (beep_off_time && millis() >= beep_off_time) {
      noTone(BUZZER_PIN);
      beep_off_time = 0;
    }
  }
  softBeep(40);
  t0 = millis();
  while (digitalRead(BTN_PIN) == LOW) {
    if (beep_off_time && millis() >= beep_off_time) {
      noTone(BUZZER_PIN);
      beep_off_time = 0;
    }
  }
}

void loop() {
  unsigned long now = millis();

  if (beep_off_time && now >= beep_off_time) {
    noTone(BUZZER_PIN);
    beep_off_time = 0;
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
      long diff = (long)bump_base - (long)raw;
      if (diff < 0) diff = 0;
      // 不打印 bump，保证控制变量只差在输出
    } else {
      line_sensors.readSensorsADC();
      int left_raw = line_sensors.readings[0];
      int right_raw = line_sensors.readings[4];

      int left = left_raw - line_L_base;
      int right = right_raw - line_R_base;
      int d = right - left;

      Serial.print("L:");
      Serial.print(left);
      Serial.print(", R:");
      Serial.print(right);
      Serial.print(", d:");
      Serial.println(d);
    }
  }
}
