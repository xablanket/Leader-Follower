#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "LineSensors.h"

#define EMIT_PIN    11
#define BUZZ_PIN    6

Motors_c motors;
Kinematics_c kin;
PID_c left_pid;
PID_c right_pid;

#define MAX_RESULTS 50
#define VARIABLES 3
float results[MAX_RESULTS][VARIABLES];
int results_index = 0;
unsigned long record_ts = 0;
#define RECORD_INTERVAL_MS 150

#define STATE_WAIT        0
#define STATE_STRAIGHT    1
#define STATE_FINISHED    2
int state = STATE_WAIT;

#define DRIVE_EST_MS    20UL
#define DRIVE_PID_MS    40UL
#define DRIVE_PWM_LIMIT 60

const float DEMAND_CS = -300.0f;
const int kF_L = 16;
const int kF_R = 15;

float KP_L = 0.04025f, KI_L = 0.0f, KD_L = 0.0f;
float KP_R = 0.07000f, KI_R = 0.0f, KD_R = 0.0f;

unsigned long drive_est_ts = 0, drive_pid_ts = 0;
long d_last_e0 = 0, d_last_e1 = 0;
float spdL_cps = 0.0f, spdR_cps = 0.0f;
float d_mL1 = 0, d_mL2 = 0, d_mR1 = 0, d_mR2 = 0;

float x0 = 0, y0 = 0;
unsigned long state_start_ts = 0;

#define STRAIGHT_DISTANCE_MM 450.0f

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

void recordData() {
  if (results_index < MAX_RESULTS) {
    results[results_index][0] = kin.x;
    results[results_index][1] = kin.y;
    results[results_index][2] = kin.theta;
    results_index++;
  }
}

void printResults() {
  Serial.println("\n========== LEADER DATA (CSV) ==========");
  Serial.println("Sample,X_mm,Y_mm,Theta_rad");
  
  for (int i = 0; i < results_index; i++) {
    Serial.print(i);
    Serial.print(",");
    Serial.print(results[i][0], 2);
    Serial.print(",");
    Serial.print(results[i][1], 2);
    Serial.print(",");
    Serial.println(results[i][2], 4);
  }
}

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

void driveStraight() {
  unsigned long now = millis();
  if (now - drive_pid_ts >= DRIVE_PID_MS) {
    drive_pid_ts = now;
    
    float demandL = DEMAND_CS;
    float demandR = DEMAND_CS;
    
    float measL = (spdL_cps + d_mL1 + d_mL2) / 3.0f;
    float measR = (spdR_cps + d_mR1 + d_mR2) / 3.0f;
    d_mL2 = d_mL1; d_mL1 = spdL_cps;
    d_mR2 = d_mR1; d_mR1 = spdR_cps;
    
    float uL = left_pid.update(demandL, measL);
    float uR = right_pid.update(demandR, measR);
    float uLc = clampf(uL, -12.0f, 12.0f);
    float uRc = clampf(uR, -12.0f, 12.0f);
    
    float baseL = (demandL >= 0.0f) ? kF_L : -kF_L;
    float baseR = (demandR >= 0.0f) ? kF_R : -kF_R;
    
    float pwmL = clampf(baseL + uLc, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
    float pwmR = clampf(baseR + uRc, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
    
    motors.setPWM(iround(pwmL), iround(pwmR));
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  delay(300);
  
  kin.initialise(0.0f, 0.0f, 0.0f);
  
  left_pid.initialise(KP_L, KI_L, KD_L);
  right_pid.initialise(KP_R, KI_R, KD_R);
  left_pid.reset();
  right_pid.reset();
  
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, HIGH);
  Serial.println("Line IR: ON (EMIT_PIN = HIGH)");
  
  pinMode(BUZZ_PIN, OUTPUT);
  
  beep(200);
  
  drive_est_ts = drive_pid_ts = record_ts = millis();
  state_start_ts = millis();
  
  state = STATE_WAIT;
  results_index = 0;
}

void loop() {
  unsigned long now = millis();
  
  updateSpeedEstimate();
  
  if (state == STATE_STRAIGHT) {
    if (now - record_ts >= RECORD_INTERVAL_MS) {
      record_ts = now;
      recordData();
    }
  }
  
  switch (state) {
    
    case STATE_WAIT:
      motors.setPWM(0, 0);
      
      if (now - state_start_ts >= 1000) {
        Serial.println("1 second elapsed! Starting straight motion...");
        beep(100);
        delay(100);
        beep(100);
        
        x0 = kin.x;
        y0 = kin.y;
        
        left_pid.reset();
        right_pid.reset();
        
        state = STATE_STRAIGHT;
        state_start_ts = now;
        
        Serial.println("Moving forward 450mm...");
      } else {
        static unsigned long last_print = 0;
        if (now - last_print >= 500) {
          last_print = now;
          int remaining = 1 - (now - state_start_ts) / 1000;
          Serial.print("Starting in ");
          Serial.print(remaining);
          Serial.println(" second...");
        }
      }
      break;
    
    case STATE_STRAIGHT:
      {
        driveStraight();
        
        float dx = kin.x - x0;
        float dy = kin.y - y0;
        float dist = sqrt(dx * dx + dy * dy);
        
        static unsigned long last_debug = 0;
        if (now - last_debug >= 200) {
          last_debug = now;
          Serial.print("[STRAIGHT] Dist=");
          Serial.print(dist, 1);
          Serial.print("mm / ");
          Serial.print(STRAIGHT_DISTANCE_MM, 1);
          Serial.print("mm | Speed L=");
          Serial.print(spdL_cps, 1);
          Serial.print(" R=");
          Serial.print(spdR_cps, 1);
          Serial.println(" cps");
        }
        
        if (dist >= STRAIGHT_DISTANCE_MM) {
          Serial.println("Straight motion complete!");
          beep(200);
          
          motors.setPWM(0, 0);
          
          state = STATE_FINISHED;
          state_start_ts = now;
        }
      }
      break;
    
    case STATE_FINISHED:
      motors.setPWM(0, 0);
      
      static bool data_printed = false;
      if (!data_printed) {
        printResults();
        
        Serial.println("\nMotion finished.");
        Serial.println("IR still ON - Follower can continue to follow stopped Leader");
        Serial.println("Reset to run again.");
        
        data_printed = true;
        beep(50);
        delay(200);
        beep(50);
      }
      
      break;
  }
}

