#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#include <Arduino.h>

#define NUM_SENSORS 5

static const int sensor_pins[NUM_SENSORS] = { A11, A0, A2, A3, A4 };

#define EMIT_PIN 11

class LineSensors_c {
public:
  float readings[NUM_SENSORS];
  float minimum[NUM_SENSORS];
  float maximum[NUM_SENSORS];
  float scaling[NUM_SENSORS];
  float calibrated[NUM_SENSORS];

  LineSensors_c() {
  }

  void initialiseForADC() {
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, HIGH);
    for (int i = 0; i < NUM_SENSORS; i++) {
      pinMode(sensor_pins[i], INPUT_PULLUP);
    }
    analogReference(DEFAULT);
  }

  void readSensorsADC() {
    initialiseForADC();
    for (int i = 0; i < NUM_SENSORS; i++) {
      (void)analogRead(sensor_pins[i]);
      delayMicroseconds(40);
      readings[i] = (float)analogRead(sensor_pins[i]);
    }
  }

  void calcCalibratedADC() {
    readSensorsADC();
    for (int i = 0; i < NUM_SENSORS; i++) {
      float range = maximum[i] - minimum[i];
      if (range < 1.0f) range = 1.0f;
      scaling[i] = 1.0f / range;
      float x = (readings[i] - minimum[i]) * scaling[i];
      if (x < 0) x = 0;
      if (x > 1) x = 1;
      calibrated[i] = x;
    }
  }

  bool onLine(float thr = 0.60f, bool whiteIsOne = false) {
    for (int i = 0; i < NUM_SENSORS; ++i) {
      float v = calibrated[i];
      float blackness = whiteIsOne ? (1.0f - v) : v;
      if (blackness >= thr) return true;
    }
    return false;
  }

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

#endif
