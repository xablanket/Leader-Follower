#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
// #include "LineSensors.h"   // 禁用防干扰

#define EMIT_PIN 11
#define BUMP_LEFT_PIN 4
#define BUMP_RIGHT_PIN 5

Motors_c motors;
Kinematics_c kin;

#define DRIVE_PWM_LIMIT 60

// ===== LOST DETECTION =====
int lost_counter = 0;
const int LOST_COUNT_TH = 10;
const float LOST_TH = 3800.0f;

// ===== Loop timing =====
unsigned long ir_ts = 0;

const unsigned long IR_PERIOD = 15;

/////////
const float STOP_TH = 150.0f;
const float SLOW_TH = 450.0f;
const float MID_TH = 1800.0f;
////////

// ===== SPEED =====
const float PWM_SLOW = 10.0f;
const float PWM_MID = 16.0f;
const float PWM_FAST = 20.0f;


unsigned int readBump(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);

  pinMode(pin, INPUT);

  unsigned long t0 = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t0 > 4000) break;
  }

  return (unsigned int)(micros() - t0);
}

void setup() {
  Serial.begin(115200);
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  delay(200);

  kin.initialise(0, 0, 0);

  pinMode(EMIT_PIN, INPUT);
}

void loop() {
  unsigned long now = millis();

  if (now - ir_ts >= IR_PERIOD) {
    ir_ts = now;

    // --- Read bump sensors ---
    unsigned int L = readBump(BUMP_LEFT_PIN);
    unsigned int R = readBump(BUMP_RIGHT_PIN);
    float d = 0.5f * (float(L) + float(R));  // distance indicator

    // --- LOST detection ---
    if (d >= LOST_TH) lost_counter++;
    else lost_counter = 0;

    if (lost_counter >= LOST_COUNT_TH) {
      motors.setPWM(0, 0);
      Serial.println("LOST! Stopping.");
      return;
    }

    // --- Speed control only (NO steering) ---
    float base_pwm;

    if (d <= STOP_TH)
      base_pwm = 0;
    else if (d <= SLOW_TH)
      base_pwm = PWM_SLOW;
    else if (d <= MID_TH)
      base_pwm = PWM_MID;
    else
      base_pwm = PWM_FAST;

    float pwmL = base_pwm;
    float pwmR = base_pwm;

    if (pwmL > DRIVE_PWM_LIMIT) pwmL = DRIVE_PWM_LIMIT;
    if (pwmR > DRIVE_PWM_LIMIT) pwmR = DRIVE_PWM_LIMIT;

    motors.setPWM((int)lround(pwmL), (int)lround(pwmR));

    // Debug print
    Serial.print("L=");
    Serial.print(L);
    Serial.print("  R=");
    Serial.print(R);
    Serial.print("  d=");
    Serial.print(d);
    Serial.print("  base=");
    Serial.println(base_pwm);
  }
}
