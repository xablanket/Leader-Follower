// Follower Hybrid: Bump(距离) + Line(方向)

// === Includes ===
#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "LineSensors.h"  // 添加Line传感器

#define BUMP_L 4
#define BUMP_R 5
#define EMIT_PIN 11
#define NUM_SENSORS 5

Motors_c motors;
Kinematics_c kin;
PID_c pidL, pidR;
LineSensors_c line_sensors;  // Line传感器对象

#define DRIVE_EST_MS 20
#define DRIVE_PID_MS 40
#define DRIVE_PWM_LIMIT 60

float KP_L = 0.04025, KI_L = 0.00005, KD_L = 0;
float KP_R = 0.07000, KI_R = 0.00005, KD_R = 0;

const int kF_L = 16;
const int kF_R = 18;

// Bump距离控制阈值
#define LOST_TH 3800   // 太远
#define NEAR_TH 300    // 太近

// Line方向控制参数
float line_background[NUM_SENSORS] = {0};  // 背景校准
const float LINE_THRESHOLD = 20.0f;        // Line检测阈值

int faceCount = 0;
bool faceCrash = false;

unsigned long ts_est = 0, ts_pid = 0;
long last_e0 = 0, last_e1 = 0;
float spdL = 0, spdR = 0;

unsigned long last_d = 9999;

// ========== Bump读取（距离控制）==========
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

// ========== Line加权平均（方向控制）==========
// 返回：负值=Leader在左，正值=Leader在右，0=正前方
float getDirectionFromLine() {
  line_sensors.readSensorsADC();
  
  // 计算每个传感器相对背景的信号变化
  float signals[NUM_SENSORS];
  float total_signal = 0.0f;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    signals[i] = fabs(line_sensors.readings[i] - line_background[i]);
    total_signal += signals[i];
  }
  
  // 信号太弱，无法判断方向
  if (total_signal < LINE_THRESHOLD * 2) {
    return 0.0f;
  }
  
  // 加权平均：位置权重 -2, -1, 0, +1, +2
  float weighted_sum = 0.0f;
  float weights[NUM_SENSORS] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    weighted_sum += signals[i] * weights[i];
  }
  
  // 归一化方向误差，范围约[-2, +2]
  float direction_error = weighted_sum / total_signal;
  
  return direction_error * 100.0f;  // 放大便于控制
}

// ========== 距离映射（保持原逻辑）==========
float mapIRtoCS(unsigned long d) {
  if (d > LOST_TH) return 0;
  if (d < 150) return 0;
  float cs = d * 0.18;
  if (cs > 300) cs = 300;
  return cs;
}

// ========== 校准Line背景 ==========
void calibrateLineBackground() {
  Serial.println("校准Line传感器背景...");
  delay(1000);
  
  for (int sample = 0; sample < 10; sample++) {
    line_sensors.readSensorsADC();
    for (int i = 0; i < NUM_SENSORS; i++) {
      line_background[i] += line_sensors.readings[i];
    }
    delay(50);
  }
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    line_background[i] /= 10.0f;
  }
  
  Serial.println("Line背景校准完成");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Hybrid Follower: Bump(距离) + Line(方向) ===");

  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  pidL.initialise(KP_L, KI_L, KD_L);
  pidR.initialise(KP_R, KI_R, KD_R);

  kin.initialise(0, 0, 0);

  // 初始化Bump传感器
  pinMode(BUMP_L, INPUT);
  pinMode(BUMP_R, INPUT);
  pinMode(EMIT_PIN, INPUT);  // 关闭自己的IR，只接收Leader的
  
  // 初始化Line传感器
  line_sensors.initialiseForADC();
  pinMode(EMIT_PIN, INPUT);  // 再次确认关闭IR发射
  
  // 校准Line背景
  calibrateLineBackground();

  ts_est = ts_pid = millis();
}

void loop() {
  unsigned long now = millis();

  // ========== 读取Bump（距离）==========
  unsigned long dL = readBump(BUMP_L);
  unsigned long dR = readBump(BUMP_R);
  unsigned long d = (dL + dR) / 2;

  // ========== 读取Line（方向）==========
  float direction_error = getDirectionFromLine();

  // 调试输出
  static unsigned long debug_ts = 0;
  if (now - debug_ts >= 200) {
    debug_ts = now;
    Serial.print("Bump: L="); Serial.print(dL);
    Serial.print(" R="); Serial.print(dR);
    Serial.print(" AVG="); Serial.print(d);
    Serial.print(" | Line方向="); Serial.print(direction_error, 1);
    Serial.println();
  }

  // 碰撞检测
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

  // 速度估计
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

    kin.update();
  }

  // PID控制
  if (now - ts_pid >= DRIVE_PID_MS) {
    ts_pid = now;

    // ========== 距离控制（用Bump）==========
    float demand;
    if (faceCrash) {
      demand = 0;
    } else {
      demand = mapIRtoCS(d);
    }

    float uL = pidL.update(demand, spdL);
    float uR = pidR.update(demand, spdR);

    float pwmL = kF_L + uL;
    float pwmR = kF_R + uR;

    // ========== 方向控制（用Line代替Bump左右差）==========
    // 原来：float steer = (float)dR - (float)dL;
    // 现在：用Line的方向误差
    float steer = direction_error;  // 负=左转，正=右转
    
    float turn = steer * 0.002;  // 调整系数（Line范围更大，降低增益）
    if (turn > 10) turn = 10;
    if (turn < -10) turn = -10;

    pwmL -= turn;
    pwmR += turn;

    // PWM限幅
    if (pwmL > DRIVE_PWM_LIMIT) pwmL = DRIVE_PWM_LIMIT;
    if (pwmL < -DRIVE_PWM_LIMIT) pwmL = -DRIVE_PWM_LIMIT;
    if (pwmR > DRIVE_PWM_LIMIT) pwmR = DRIVE_PWM_LIMIT;
    if (pwmR < -DRIVE_PWM_LIMIT) pwmR = -DRIVE_PWM_LIMIT;

    motors.setPWM((int)pwmL, (int)pwmR);
  }
}