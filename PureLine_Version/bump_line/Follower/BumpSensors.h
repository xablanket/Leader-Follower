#ifndef _BUMPSENSORS_H
#define _BUMPSENSORS_H

#include <Arduino.h>

#define BUMP_LEFT_PIN A6
#define NUM_BUMP_SENSORS 1

#define EMIT_PIN 11

class BumpSensors_c {
  
  public:
    float reading;
    float background;
    
    static const int FILTER_SIZE = 5;
    float filter_buffer[FILTER_SIZE];
    int filter_index;
    
    BumpSensors_c() {
      reading = 0.0;
      background = 0.0;
      
      for (int i = 0; i < FILTER_SIZE; i++) {
        filter_buffer[i] = 0.0;
      }
      filter_index = 0;
    }
    
    void initialiseForADC() {
      pinMode(BUMP_LEFT_PIN, INPUT);
      
      pinMode(EMIT_PIN, INPUT);
      
      analogReference(DEFAULT);
    }
    
    void readSensorsADC() {
      (void)analogRead(BUMP_LEFT_PIN);
      delayMicroseconds(100);
      
      float raw_value = (float)analogRead(BUMP_LEFT_PIN);
      
      reading = movingAverage(raw_value);
    }
    
    float movingAverage(float new_value) {
      filter_buffer[filter_index] = new_value;
      filter_index = (filter_index + 1) % FILTER_SIZE;
      
      float sum = 0.0;
      for (int i = 0; i < FILTER_SIZE; i++) {
        sum += filter_buffer[i];
      }
      return sum / FILTER_SIZE;
    }
    
    void calibrateBackground(int samples = 10) {
      delay(2000);
      
      float sum = 0.0;
      
      for (int i = 0; i < samples; i++) {
        (void)analogRead(BUMP_LEFT_PIN);
        delayMicroseconds(100);
        sum += (float)analogRead(BUMP_LEFT_PIN);
        delay(50);
      }
      
      background = sum / samples;
    }
    
    float getSignal() {
      readSensorsADC();
      return reading - background;
    }
    
    float getRawReading() {
      readSensorsADC();
      return reading;
    }
    
    bool hasSignal(float threshold = 30.0f) {
      float signal = getSignal();
      return fabs(signal) > threshold;
    }
};

#endif

