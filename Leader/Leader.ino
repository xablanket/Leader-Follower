#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"

#define EMIT_PIN 11
#define BTN_PIN 14
#define BUZZER_PIN 6

Motors_c motors;
Kinematics_c kin;
PID_c pidL, pidR;

unsigned long ir_ts = 0;
bool ir_bump_mode = true;

unsigned long beep_off_time = 0;

void softBeep(int duration_ms) {
  tone(BUZZER_PIN, 800);
  beep_off_time = millis() + duration_ms;
}

void setup() {
  Serial.begin(115200);

  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  kin.initialise(0,0,0);

  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  softBeep(40);

  while (digitalRead(BTN_PIN) == HIGH) {
    unsigned long now = millis();
    if (now >= beep_off_time && beep_off_time != 0) {
      noTone(BUZZER_PIN);
      beep_off_time = 0;
    }
  }

  softBeep(40);
  ir_ts = millis();
}

void loop() {
  unsigned long now = millis();

  if (now >= beep_off_time && beep_off_time != 0) {
    noTone(BUZZER_PIN);
    beep_off_time = 0;
  }

  if (now - ir_ts >= 100) {
    ir_ts = now;
    ir_bump_mode = !ir_bump_mode;

    if (ir_bump_mode)
      digitalWrite(EMIT_PIN, LOW);
    else
      digitalWrite(EMIT_PIN, HIGH);
  }
}

