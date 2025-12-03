// Line Sensor Data Collection Only (No Following)
// 只使用中间3个传感器 (line_1, line_2, line_3) 用于直线距离测量
// 去掉最左(line_0)和最右(line_4)，减少干扰
// Collects 50 samples every 30ms after button press
// Outputs CSV: distance_cm,sample_id,line_1,line_2,line_3,center_avg

#include <Arduino.h>

// ===== Pin definitions =====
#define EMIT_PIN 11       // IR emitter control (INPUT = off, to receive Leader IR)
#define BUTTON_PIN 14     // Button A (A0 digital)

// Line sensor pins - 只使用中间3个
const int CENTER_SENSOR_PINS[3] = { A0, A2, A3 };  // 中间3个传感器
const int NUM_CENTER_SENSORS = 3;

// ===== User settings =====
const int RECORD_SAMPLES = 50;
const int RECORD_INTERVAL = 30;   // ms

// ===== Runtime vars =====
int distance_cm = 0;
bool record_request = false;
unsigned long record_ts = 0;
int record_count = 0;

// ===== Background calibration =====
float background_values[NUM_CENTER_SENSORS];
bool calibrated = false;

// ===== Line sensor reading function (只读中间3个) =====
void readCenterSensorsADC(float readings[]) {
  // Set EMIT_PIN to INPUT to receive external IR (from Leader)
  pinMode(EMIT_PIN, INPUT);
  digitalWrite(EMIT_PIN, LOW);
  
  for (int i = 0; i < NUM_CENTER_SENSORS; i++) {
    pinMode(CENTER_SENSOR_PINS[i], INPUT_PULLUP);
    readings[i] = (float)analogRead(CENTER_SENSOR_PINS[i]);
  }
}

// ===== Calibrate background values =====
void calibrateBackground() {
  Serial.println("=== Calibrating Background (Center 3 Sensors Only) ===");
  Serial.println("请确保Leader未启动（或距离很远）...");
  delay(2000);
  
  // Initialize
  for (int i = 0; i < NUM_CENTER_SENSORS; i++) {
    background_values[i] = 0.0f;
  }
  
  // Sample for 2 seconds
  int sample_count = 0;
  unsigned long start = millis();
  float temp_readings[NUM_CENTER_SENSORS];
  
  while (millis() - start < 2000) {
    readCenterSensorsADC(temp_readings);
    for (int i = 0; i < NUM_CENTER_SENSORS; i++) {
      background_values[i] += temp_readings[i];
    }
    sample_count++;
    delay(10);
  }
  
  // Calculate average
  for (int i = 0; i < NUM_CENTER_SENSORS; i++) {
    background_values[i] /= sample_count;
  }
  
  Serial.println("Background calibrated!");
  Serial.print("Samples: ");
  Serial.println(sample_count);
  Serial.println("Background values (Center 3 sensors):");
  for (int i = 0; i < NUM_CENTER_SENSORS; i++) {
    Serial.print("  Sensor[");
    Serial.print(i + 1);  // 显示为 1, 2, 3
    Serial.print("]: ");
    Serial.println(background_values[i], 1);
  }
  Serial.println();
  
  calibrated = true;
}

void setup() {
  Serial.begin(115200);
  
  // Setup IR receiver mode (turn off own IR LED)
  pinMode(EMIT_PIN, INPUT);
  digitalWrite(EMIT_PIN, LOW);
  
  // Setup button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  Serial.println("=== Line Sensor Data Record Mode ===");
  Serial.println("功能：测试line sensor作为IR接收器的距离测量性能");
  Serial.println("策略：只使用中间3个传感器，专注直线距离测量");
  Serial.println("      (去掉最左和最右，减少干扰)");
  Serial.println();
  
  // Calibrate background
  calibrateBackground();
  
  Serial.println("准备就绪！");
  Serial.println("使用方法:");
  Serial.println("  1. 在Serial Monitor输入距离(cm)");
  Serial.println("  2. 启动Leader，放置在指定距离");
  Serial.println("  3. 按按钮开始采集");
  Serial.println();
}

void loop() {
  // ----- Read serial and set distance -----
  if (Serial.available() > 0) {
    int d = Serial.parseInt();
    if (d > 0 && d < 200) {
      distance_cm = d;
      Serial.print("[Distance Set To] ");
      Serial.println(distance_cm);
    }
  }
  
  // ----- Start recording when button pressed -----
  if (!record_request && digitalRead(BUTTON_PIN) == LOW) {
    delay(20);
    if (digitalRead(BUTTON_PIN) == LOW) {
      record_request = true;
      record_count = 0;
      record_ts = millis();
      
      Serial.println("===== DATA RECORD START =====");
      Serial.println("distance_cm,sample_id,line_1,line_2,line_3,center_avg");
    }
  }
  
  if (!record_request) return;
  
  // ----- Sample timing -----
  if (millis() - record_ts < RECORD_INTERVAL) return;
  record_ts += RECORD_INTERVAL;
  
  // ----- Read center 3 line sensors (as IR receivers) -----
  float readings[NUM_CENTER_SENSORS];
  readCenterSensorsADC(readings);
  
  // Calculate IR signal strength (background - current, like follower.ino)
  float ir_values[NUM_CENTER_SENSORS];
  float sum = 0;
  
  for (int i = 0; i < NUM_CENTER_SENSORS; i++) {
    ir_values[i] = background_values[i] - readings[i];
    if (ir_values[i] < 0) ir_values[i] = 0;  // Clip negative
    sum += ir_values[i];
  }
  
  float center_avg = sum / NUM_CENTER_SENSORS;
  
  // ----- Output CSV line -----
  // Format: distance_cm,sample_id,line_1,line_2,line_3,center_avg
  Serial.print(distance_cm);
  Serial.print(",");
  Serial.print(record_count);
  Serial.print(",");
  for (int i = 0; i < NUM_CENTER_SENSORS; i++) {
    Serial.print((int)ir_values[i]);
    Serial.print(",");
  }
  Serial.println((int)center_avg);
  
  record_count++;
  
  if (record_count >= RECORD_SAMPLES) {
    record_request = false;
    Serial.println("===== DATA RECORD END =====");
  }
}
