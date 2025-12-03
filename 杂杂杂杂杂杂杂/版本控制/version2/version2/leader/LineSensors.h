#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#include <Arduino.h>

// ---- 必须在类之前定义好这些常量/引脚 ----
#define NUM_SENSORS 5

// 3Pi+ 32U4：DN1..DN5 = A11, A0, A2, A3, A4（左→右）
static const int sensor_pins[NUM_SENSORS] = { A11, A0, A2, A3, A4 };

// IR 发射控制
#define EMIT_PIN 11
// -----------------------------------------

class LineSensors_c {
public:
  float readings[NUM_SENSORS];   // 原始 ADC (0..1023)
  float minimum[NUM_SENSORS];    // 标定最小（白面）
  float maximum[NUM_SENSORS];    // 标定最大（黑面）
  float scaling[NUM_SENSORS];    // 1/(max-min)
  float calibrated[NUM_SENSORS]; // 归一化 0..1（按伪代码：x=(raw-min)/range）

  // 必须与类名一致，且保持空体
  LineSensors_c() {
    // leave this empty
  }

  // 初始化：打开 IR 发射 + 传感器为 INPUT_PULLUP
  void initialiseForADC() {
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, HIGH);
    for (int i = 0; i < NUM_SENSORS; i++) {
      pinMode(sensor_pins[i], INPUT_PULLUP);
    }
    analogReference(DEFAULT);
  }

  // 读取5路ADC到 readings[]
  void readSensorsADC() {
    initialiseForADC(); // 按课程模板保留
    for (int i = 0; i < NUM_SENSORS; i++) {
      (void)analogRead(sensor_pins[i]);   // 丢首读
      delayMicroseconds(40);
      readings[i] = (float)analogRead(sensor_pins[i]);
    }
  }

  // 归一化（Algorithm 2）：Calibrated = (Readings - Min) / (Max - Min)
  // 这里保持伪代码极性：白≈0，黑≈1；如果想白=1黑=0，上层做 1.0 - calibrated[i]
  void calcCalibratedADC() {
    readSensorsADC();
    for (int i = 0; i < NUM_SENSORS; i++) {
      float range = maximum[i] - minimum[i];
      if (range < 1.0f) range = 1.0f; // 防止除0
      scaling[i] = 1.0f / range;
      float x = (readings[i] - minimum[i]) * scaling[i];
      if (x < 0) x = 0;
      if (x > 1) x = 1;
      calibrated[i] = x; // 白≈0，黑≈1
    }
  }

  // 是否任意一只“踩黑”
  bool onLine(float thr = 0.60f, bool whiteIsOne = false) {
    for (int i = 0; i < NUM_SENSORS; ++i) {
      float v = calibrated[i];               // 白≈0 黑≈1
      float blackness = whiteIsOne ? (1.0f - v) : v;
      if (blackness >= thr) return true;
    }
    return false;
  }

  // 返回“最黑”的传感器索引；若都不够黑返回 -1
  int dominantSensor(float thr = 0.60f, bool whiteIsOne = false) {
    int best = -1; float bestVal = thr;
    for (int i = 0; i < NUM_SENSORS; ++i) {
      float v = calibrated[i];
      float blackness = whiteIsOne ? (1.0f - v) : v;
      if (blackness > bestVal) { bestVal = blackness; best = i; }
    }
    return best;
  }
};

#endif // _LINESENSORS_H
