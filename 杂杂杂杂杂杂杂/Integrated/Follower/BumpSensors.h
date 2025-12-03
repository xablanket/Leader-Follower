#ifndef _BUMPSENSORS_H
#define _BUMPSENSORS_H

#include <Arduino.h>

// ========== Bump传感器配置（Digital模式）==========
#define BUMP_LEFT_PIN   4    // PD4 - 左侧bump传感器
#define BUMP_RIGHT_PIN  5    // PC6 - 右侧bump传感器
#define NUM_BUMP_SENSORS 2

// IR发射控制引脚
#define EMIT_PIN 11

// Digital模式参数
#define TIMEOUT_US 3000      // 超时时间（微秒）
#define MAX_VALID_TIME 2500  // 有效读数的最大时间

class BumpSensors_c {
  
  public:
    // Digital读数（时间，单位：微秒）
    unsigned long readings[NUM_BUMP_SENSORS];  // [0]=左, [1]=右
    
    // 背景校准值
    unsigned long background[NUM_BUMP_SENSORS];
    
    // 构造函数
    BumpSensors_c() {
      for (int i = 0; i < NUM_BUMP_SENSORS; i++) {
        readings[i] = 0;
        background[i] = 0;
      }
    }
    
    // ========== 初始化bump传感器（Digital模式）==========
    void initialiseForDigital() {
      // 设置bump传感器引脚
      pinMode(BUMP_LEFT_PIN, INPUT);
      pinMode(BUMP_RIGHT_PIN, INPUT);
      
      // 配置IR发射控制引脚
      // 根据官方文档：INPUT = 关闭IR发射，用于接收外部IR信号
      pinMode(EMIT_PIN, INPUT);
      
      Serial.println("✓ Bump传感器已初始化（Digital模式）");
      Serial.println("  - EMIT_PIN: INPUT（接收外部IR）");
    }
    
    // ========== 读取单个传感器（Digital模式）==========
    unsigned long readSensorDigital(int pin) {
      // 1. 充电：设置为OUTPUT HIGH
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
      delayMicroseconds(10);  // 充电时间
      
      // 2. 测量放电时间：设置为INPUT，等待变为LOW
      pinMode(pin, INPUT);
      unsigned long start_time = micros();
      unsigned long elapsed = 0;
      
      // 等待引脚变为LOW，或超时
      while (digitalRead(pin) == HIGH && elapsed < TIMEOUT_US) {
        elapsed = micros() - start_time;
      }
      
      return elapsed;
    }
    
    // ========== 读取两个bump传感器 ==========
    void readSensors() {
      readings[0] = readSensorDigital(BUMP_LEFT_PIN);
      readings[1] = readSensorDigital(BUMP_RIGHT_PIN);
    }
    
    // ========== 校准背景值 ==========
    void calibrateBackground(int samples = 10) {
      Serial.println("\n开始校准bump传感器背景值...");
      Serial.println("请确保Leader尚未启动或距离较远");
      delay(2000);
      
      unsigned long sum_left = 0;
      unsigned long sum_right = 0;
      
      for (int i = 0; i < samples; i++) {
        readSensors();
        sum_left += readings[0];
        sum_right += readings[1];
        delay(50);
      }
      
      background[0] = sum_left / samples;
      background[1] = sum_right / samples;
      
      Serial.println("Bump传感器背景校准完成：");
      Serial.print("  左bump背景值: "); Serial.print(background[0]); Serial.println(" us");
      Serial.print("  右bump背景值: "); Serial.print(background[1]); Serial.println(" us");
    }
    
    // ========== 获取左右差值（减去背景）==========
    float getBalance() {
      readSensors();
      
      // 计算相对于背景的变化
      long left_signal = (long)readings[0] - (long)background[0];
      long right_signal = (long)readings[1] - (long)background[1];
      
      // 返回差值（正值=Leader在右，负值=Leader在左）
      // Digital模式：时间短=信号强=Leader近
      return (float)(left_signal - right_signal);
    }
    
    // ========== 获取平均信号强度 ==========
    float getAverageSignal() {
      readSensors();
      return (float)(readings[0] + readings[1]) / 2.0f;
    }
    
    // ========== 获取信号变化量（相对于背景）==========
    float getSignalChange() {
      readSensors();
      float avg_background = (float)(background[0] + background[1]) / 2.0f;
      float avg_current = (float)(readings[0] + readings[1]) / 2.0f;
      
      // Digital模式：时间减少=信号增强
      return avg_background - avg_current;
    }
    
    // ========== 检测是否有Leader信号 ==========
    bool hasSignal(float threshold = 100.0f) {
      float signal_change = getSignalChange();
      return signal_change > threshold;  // 时间减少 > 阈值 = 检测到Leader
    }
};

#endif

