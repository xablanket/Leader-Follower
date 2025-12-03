#include "Encoders.h"
#include "Kinematics.h"
#include "Motors.h"
#include "LineSensors.h"
#include "PID.h"

#define EMIT_PIN 11
#define BUZZ_PIN 6
#define LED_PIN  13
#define LED_RED  17
#define BUTTON_B 30

Motors_c motors;
Kinematics_c kin;
LineSensors_c line_sensors;
PID_c distance_pid;

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

#define STATE_IDLE          0
#define STATE_CALIBRATE     1
#define STATE_WAIT_SIGNAL   2
#define STATE_FOLLOWING     3
#define STATE_FINISHED      4

int robot_state = STATE_IDLE;

#define DISTANCE_TARGET   80.0f
#define DISTANCE_MIN      50.0f
#define DISTANCE_MAX     120.0f
#define DISTANCE_TOO_CLOSE 150.0f

#define DIST_KP 0.5f
#define DIST_KI 0.002f
#define DIST_KD 0.4f

#define STEER_DEADBAND 40
#define STEER_CLAMP 60
#define STEER_NORM 100.0f
#define STEER_ALPHA 0.5f
#define K_STEER 120.0f
#define MIN_WHEEL_SPEED 5.0f

#define SPEED_MIN   0.0f
#define SPEED_MAX  60.0f
#define MAX_PWM    60

#define SIGNAL_THRESHOLD  25.0f
#define SIGNAL_LOST_TIME 200

float background_values[NUM_SENSORS];

float speed = 0.0f;
float steer_filtered = 0.0f;
int line_L_offset = 0;
int line_R_offset = 0;
unsigned long last_signal_time = 0;
unsigned long last_update_time = 0;

#define UPDATE_INTERVAL 25

#define SPEED_EST_MS 20
unsigned long speed_est_ts = 0;
long last_e0 = 0, last_e1 = 0;
float spdL_cps = 0.0f;
float spdR_cps = 0.0f;

void beep(int duration);
void setupIRReceiver();
void calibrateSensors();
float getCenterIRValue();
float getSteerFromLine();
void updateFollowingControl();
bool hasSignal();
void waitForButton();
void recordData();
void printResults();
void updateWheelSpeed();

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);
  
  setupIRReceiver();
  
  Serial.begin(115200);
  delay(1000);
  
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  kin.initialise(0, 0, 0);
  
  line_sensors.initialiseForADC();
  
  distance_pid.initialise(DIST_KP, DIST_KI, DIST_KD);
  distance_pid.setOutputLimits(SPEED_MIN, SPEED_MAX);
  distance_pid.setMaxDelta(6.0f);
  distance_pid.setOutputFilter(0.7f);
  
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);
  
  beep(100);
  delay(200);
  beep(100);
  
  waitForButton();
  
  robot_state = STATE_CALIBRATE;
  calibrateSensors();
  
  robot_state = STATE_WAIT_SIGNAL;
  Serial.println("Waiting for Leader signal...");
  
  last_update_time = millis();
  speed_est_ts = millis();
  last_e0 = count_e0;
  last_e1 = count_e1;
  results_index = 0;
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
          
          float center_value = getCenterIRValue();
          
          Serial.print("Waiting... IR=");
          Serial.print(center_value, 1);
          Serial.print(" (threshold=");
          Serial.print(SIGNAL_THRESHOLD, 0);
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
      
      if (now - last_update_time >= UPDATE_INTERVAL) {
        last_update_time = now;
        
        kin.update();
        
        float ir_value = getCenterIRValue();
        float steer_value = getSteerFromLine();
        
        if (now - experiment_start_ts >= EXPERIMENT_DURATION_MS) {
          motors.setPWM(0, 0);
          robot_state = STATE_FINISHED;
          beep(300);
          Serial.println("\nExperiment time finished!");
          break;
        }
        
        if (hasSignal()) {
          last_signal_time = now;
          
          updateFollowingControl();
          
          if (now - record_ts >= RECORD_INTERVAL_MS) {
            record_ts = now;
            recordData();
          }
          
        } else {
          if (now - last_signal_time > SIGNAL_LOST_TIME) {
            motors.setPWM(0, 0);
            robot_state = STATE_FINISHED;
            Serial.println("\nSignal lost! Stopping...");
          }
        }
        
        static unsigned long last_debug = 0;
        if (now - last_debug >= 300) {
          last_debug = now;
          
          line_sensors.readSensorsADC();
          int L_sig = line_L_offset - (int)line_sensors.readings[0];
          int R_sig = line_R_offset - (int)line_sensors.readings[4];
          if (L_sig < 0) L_sig = 0;
          if (R_sig < 0) R_sig = 0;
          
          int L_eff = (L_sig < STEER_DEADBAND) ? 0 : L_sig;
          int R_eff = (R_sig < STEER_DEADBAND) ? 0 : R_sig;
          
          Serial.print("IR=");
          Serial.print(ir_value, 1);
          Serial.print(" L=");
          Serial.print(L_eff);
          Serial.print(" R=");
          Serial.print(R_eff);
          Serial.print(" Diff=");
          Serial.print(R_eff - L_eff);
          Serial.print(" | Steer=");
          Serial.print(steer_filtered, 2);
          Serial.print(" Spd=");
          Serial.println(speed, 1);
        }
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
  float ir_value = getCenterIRValue();
  float steer_value = getSteerFromLine();
  
  rec_IR_center = ir_value;
  
  speed = distance_pid.update(DISTANCE_TARGET, ir_value);
  
  if (speed < MIN_WHEEL_SPEED) speed = MIN_WHEEL_SPEED;
  
  float steer_term = steer_value * K_STEER;
  
  float max_steer = speed * 0.6f;
  if (steer_term > max_steer) steer_term = max_steer;
  if (steer_term < -max_steer) steer_term = -max_steer;
  
  rec_speed_cmd = speed;
  rec_steer_cmd = steer_term;
  
  float demand_L = speed + steer_term;
  float demand_R = speed - steer_term;
  
  if (demand_L < MIN_WHEEL_SPEED) demand_L = MIN_WHEEL_SPEED;
  if (demand_R < MIN_WHEEL_SPEED) demand_R = MIN_WHEEL_SPEED;
  if (demand_L > MAX_PWM) demand_L = MAX_PWM;
  if (demand_R > MAX_PWM) demand_R = MAX_PWM;
  
  motors.setPWM((int)demand_L, (int)demand_R);
}

float getCenterIRValue() {
  line_sensors.readSensorsADC();
  
  float ir_1 = background_values[1] - line_sensors.readings[1];
  float ir_2 = background_values[2] - line_sensors.readings[2];
  float ir_3 = background_values[3] - line_sensors.readings[3];
  
  if (ir_1 < 0) ir_1 = 0;
  if (ir_2 < 0) ir_2 = 0;
  if (ir_3 < 0) ir_3 = 0;
  
  float ir_avg = (ir_1 + ir_2 + ir_3) / 3.0f;
  return ir_avg;
}

float getSteerFromLine() {
  line_sensors.readSensorsADC();
  
  int L_signal = line_L_offset - (int)line_sensors.readings[0];
  int R_signal = line_R_offset - (int)line_sensors.readings[4];
  
  if (L_signal < 0) L_signal = 0;
  if (R_signal < 0) R_signal = 0;
  
  rec_L_signal = L_signal;
  rec_R_signal = R_signal;
  
  if (L_signal < STEER_DEADBAND) L_signal = 0;
  if (R_signal < STEER_DEADBAND) R_signal = 0;
  
  int diff_raw = R_signal - L_signal;
  
  if (diff_raw > STEER_CLAMP) diff_raw = STEER_CLAMP;
  if (diff_raw < -STEER_CLAMP) diff_raw = -STEER_CLAMP;
  
  float diff_norm = (float)diff_raw / STEER_NORM;
  
  steer_filtered = STEER_ALPHA * steer_filtered + (1.0f - STEER_ALPHA) * diff_norm;
  
  return steer_filtered;
}

bool hasSignal() {
  return (getCenterIRValue() > SIGNAL_THRESHOLD);
}

void recordData() {
  if (results_index < MAX_RESULTS) {
    results[results_index][0] = kin.x;
    results[results_index][1] = kin.y;
    results[results_index][2] = kin.theta;
    results[results_index][3] = spdL_cps;
    results[results_index][4] = spdR_cps;
    results[results_index][5] = rec_IR_center;
    results[results_index][6] = (float)rec_L_signal;
    results[results_index][7] = (float)rec_R_signal;
    results[results_index][8] = rec_speed_cmd;
    results[results_index][9] = rec_steer_cmd;
    results_index++;
  }
}

void printResults() {
  Serial.println("\n========== FOLLOWER DATA (CSV) ==========");
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
  
  Serial.println("==========================================");
  Serial.print("Total samples: ");
  Serial.println(results_index);
  Serial.print("Record interval: ");
  Serial.print(RECORD_INTERVAL_MS);
  Serial.println(" ms");
}

void beep(int duration) {
  analogWrite(BUZZ_PIN, 120);
  delay(duration);
  analogWrite(BUZZ_PIN, 0);
}

void updateWheelSpeed() {
  unsigned long now = millis();
  if (now - speed_est_ts >= SPEED_EST_MS) {
    unsigned long dt = now - speed_est_ts;
    speed_est_ts = now;
    
    long e0 = count_e0;
    long e1 = count_e1;
    long d0 = e0 - last_e0;
    long d1 = e1 - last_e1;
    last_e0 = e0;
    last_e1 = e1;
    
    float dt_sec = (float)dt / 1000.0f;
    spdR_cps = (float)d0 / dt_sec;
    spdL_cps = (float)d1 / dt_sec;
  }
}

void setupIRReceiver() {
  pinMode(EMIT_PIN, INPUT);
  digitalWrite(EMIT_PIN, LOW);
}

void calibrateSensors() {
  Serial.println("\n===== Sensor Calibration =====");
  Serial.println("Sampling background values...");
  Serial.println("Make sure Leader is NOT active or far away!");
  Serial.println("");
  
  beep(100);
  delay(2000);
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    background_values[i] = 0.0f;
  }
  long sum_L = 0;
  long sum_R = 0;
  
  Serial.println("Sampling 30 frames...");
  for (int sample = 0; sample < 30; sample++) {
    line_sensors.readSensorsADC();
    
    for (int i = 0; i < NUM_SENSORS; i++) {
      background_values[i] += line_sensors.readings[i];
    }
    sum_L += (long)line_sensors.readings[0];
    sum_R += (long)line_sensors.readings[4];
    
    delay(50);
  }
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    background_values[i] /= 30.0f;
  }
  line_L_offset = sum_L / 30;
  line_R_offset = sum_R / 30;
  
  Serial.println("Calibration complete!");
  Serial.println("Background values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("  Sensor[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(background_values[i], 1);
  }
  Serial.println("");
  Serial.print("Steering offsets: L=");
  Serial.print(line_L_offset);
  Serial.print(" R=");
  Serial.println(line_R_offset);
  Serial.println("");
  
  beep(100);
  delay(200);
  beep(100);
}

void waitForButton() {
  while (digitalRead(BUTTON_B) == HIGH) {
    delay(10);
  }
  while (digitalRead(BUTTON_B) == LOW) {
    delay(10);
  }
  beep(100);
  delay(200);
}

