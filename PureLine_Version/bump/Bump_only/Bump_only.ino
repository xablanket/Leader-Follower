#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"

#define BUMP_L 4
#define BUMP_R 5
#define EMIT_PIN 11
#define BUZZ_PIN 6
#define LED_RED 17

Motors_c motors;
Kinematics_c kin;
PID_c pidL, pidR;

#define MAX_RESULTS 15
#define VARIABLES 10
float results[MAX_RESULTS][VARIABLES];
int results_index = 0;
unsigned long record_ts = 0;
unsigned long experiment_start_ts = 0;
#define RECORD_INTERVAL_MS 400
#define EXPERIMENT_DURATION_MS 6000

unsigned long rec_dL = 0;
unsigned long rec_dR = 0;
float rec_IR_center = 0;
float rec_speed_cmd = 0;
float rec_steer_cmd = 0;

#define STATE_IDLE          0
#define STATE_WAIT_SIGNAL   1
#define STATE_FOLLOWING     2
#define STATE_FINISHED      3

int robot_state = STATE_IDLE;

#define DRIVE_EST_MS 20
#define DRIVE_PID_MS 40
#define DRIVE_PWM_LIMIT 60

float KP_L = 0.04025, KI_L = 0.00005, KD_L = 0;
float KP_R = 0.05000, KI_R = 0.00005, KD_R = 0;

const int kF_L = 16;
const int kF_R = 16;

#define LOST_TH 3800
#define NEAR_TH 300
#define SIGNAL_THRESHOLD 500

int faceCount = 0;
bool faceCrash = false;

unsigned long ts_est = 0, ts_pid = 0;
long last_e0 = 0, last_e1 = 0;
float spdL = 0, spdR = 0;
unsigned long last_d = 9999;
unsigned long last_signal_time = 0;

unsigned long readBump(int pin);
float mapIRtoCS(unsigned long d);
void beep(int duration);
void recordData();
void printResults();
void updateWheelSpeed();
void updateFollowingControl();
bool hasSignal();

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);
  
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  kin.initialise(0, 0, 0);
  
  pidL.initialise(KP_L, KI_L, KD_L);
  pidR.initialise(KP_R, KI_R, KD_R);
  
  pinMode(BUMP_L, INPUT);
  pinMode(BUMP_R, INPUT);
  pinMode(EMIT_PIN, INPUT);
  digitalWrite(EMIT_PIN, LOW);
  
  pinMode(BUZZ_PIN, OUTPUT);
  
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);
  
  beep(100);
  delay(200);
  beep(100);
  
  ts_est = ts_pid = millis();
  last_e0 = count_e0;
  last_e1 = count_e1;
  results_index = 0;
  
  robot_state = STATE_WAIT_SIGNAL;
}

void loop() {
  digitalWrite(LED_RED, LOW);
  
  unsigned long now = millis();
  
  switch (robot_state) {
    
    case STATE_WAIT_SIGNAL:
      {
        static unsigned long last_check = 0;
        if (now - last_check >= 300) {
          last_check = now;
          
          unsigned long dL = readBump(BUMP_L);
          unsigned long dR = readBump(BUMP_R);
          unsigned long d = (dL + dR) / 2;
          
          Serial.print("Waiting... L=");
          Serial.print(dL);
          Serial.print(" R=");
          Serial.print(dR);
          Serial.print(" AVG=");
          Serial.print(d);
          Serial.print(" (threshold=");
          Serial.print(SIGNAL_THRESHOLD);
          Serial.println(")");
          
          if (hasSignal()) {
            robot_state = STATE_FOLLOWING;
            last_signal_time = now;
            experiment_start_ts = now;
            record_ts = now;
            
            beep(200);
            Serial.println("\nLeader detected! Starting to follow...\n");
          }
        }
      }
      break;
    
    case STATE_FOLLOWING:
      updateWheelSpeed();
      
      kin.update();
      
      updateFollowingControl();
      
      if (now - experiment_start_ts >= EXPERIMENT_DURATION_MS) {
        motors.setPWM(0, 0);
        robot_state = STATE_FINISHED;
        beep(300);
        Serial.println("\nExperiment time finished!");
        break;
      }
      
      if (hasSignal()) {
        last_signal_time = now;
      }
      
      if (now - record_ts >= RECORD_INTERVAL_MS) {
        record_ts = now;
        recordData();
      }
      
      static unsigned long last_debug = 0;
      if (now - last_debug >= 300) {
        last_debug = now;
        
        bool has_sig = hasSignal();
        unsigned long time_since_signal = now - last_signal_time;
        
        Serial.print("L=");
        Serial.print(rec_dL);
        Serial.print(" R=");
        Serial.print(rec_dR);
        Serial.print(" AVG=");
        Serial.print(rec_IR_center, 0);
        Serial.print(" | Sig=");
        Serial.print(has_sig ? "YES" : "NO");
        Serial.print(" (");
        Serial.print(time_since_signal);
        Serial.print("ms) | Spd=");
        Serial.print(rec_speed_cmd, 1);
        Serial.print(" Turn=");
        Serial.println(rec_steer_cmd, 2);
      }
      break;
    
    case STATE_FINISHED:
      motors.setPWM(0, 0);
      
      printResults();
      
      Serial.println("\nExperiment finished. Reset to run again.");
      delay(5000);
      break;
    
    default:
      break;
  }
}

void updateFollowingControl() {
  unsigned long now = millis();
  
  unsigned long dL = readBump(BUMP_L);
  unsigned long dR = readBump(BUMP_R);
  unsigned long d = (dL + dR) / 2;
  
  rec_dL = dL;
  rec_dR = dR;
  rec_IR_center = (float)d;
  
  if (d > LOST_TH && last_d < NEAR_TH) {
    if (fabs(spdL) < 5 && fabs(spdR) < 5) {
      faceCount++;
      if (faceCount >= 5) {
        faceCrash = true;
      }
    }
  } else {
    faceCount = 0;
  }
  
  last_d = d;
  
  if (now - ts_est >= DRIVE_EST_MS) {
    ts_est = now;
    
    long e0 = count_e0;
    long e1 = count_e1;
    
    long de0 = e0 - last_e0;
    long de1 = e1 - last_e1;
    
    last_e0 = e0;
    last_e1 = e1;
    
    spdR = de0 / (DRIVE_EST_MS / 1000.0f);
    spdL = de1 / (DRIVE_EST_MS / 1000.0f);
  }
  
  if (now - ts_pid >= DRIVE_PID_MS) {
    ts_pid = now;
    
    float demand;
    if (faceCrash) demand = 0;
    else demand = mapIRtoCS(d);
    
    rec_speed_cmd = demand;
    
    float uL = pidL.update(demand, spdL);
    float uR = pidR.update(demand, spdR);
    
    float pwmL = kF_L + uL;
    float pwmR = kF_R + uR;
    
    float steer = (float)dR - (float)dL;
    float turn = steer * 0.009;
    if (turn > 10) turn = 10;
    if (turn < -10) turn = -10;
    
    rec_steer_cmd = turn;
    
    pwmL -= turn;
    pwmR += turn;
    
    if (pwmL > DRIVE_PWM_LIMIT) pwmL = DRIVE_PWM_LIMIT;
    if (pwmL < -DRIVE_PWM_LIMIT) pwmL = -DRIVE_PWM_LIMIT;
    if (pwmR > DRIVE_PWM_LIMIT) pwmR = DRIVE_PWM_LIMIT;
    if (pwmR < -DRIVE_PWM_LIMIT) pwmR = -DRIVE_PWM_LIMIT;
    
    motors.setPWM((int)pwmL, (int)pwmR);
  }
}

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

float mapIRtoCS(unsigned long d) {
  if (d > LOST_TH) return 0;
  if (d < 150) return 0;
  float cs = d * 0.15;
  if (cs > 200) cs = 200;
  return cs;
}

bool hasSignal() {
  unsigned long dL = readBump(BUMP_L);
  unsigned long dR = readBump(BUMP_R);
  unsigned long d = (dL + dR) / 2;
  return (d > SIGNAL_THRESHOLD && d < LOST_TH);
}

void updateWheelSpeed() {
  unsigned long now = millis();
  if (now - ts_est >= DRIVE_EST_MS) {
    unsigned long dt = now - ts_est;
    ts_est = now;
    
    long e0 = count_e0;
    long e1 = count_e1;
    long d0 = e0 - last_e0;
    long d1 = e1 - last_e1;
    last_e0 = e0;
    last_e1 = e1;
    
    float dt_sec = (float)dt / 1000.0f;
    spdR = (float)d0 / dt_sec;
    spdL = (float)d1 / dt_sec;
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
    results[results_index][6] = (float)rec_dL;
    results[results_index][7] = (float)rec_dR;
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
    Serial.print((int)results[i][5]);
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

void beep(int duration) {
  analogWrite(BUZZ_PIN, 120);
  delay(duration);
  analogWrite(BUZZ_PIN, 0);
}
