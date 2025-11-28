#ifndef _BUMPSENSORS_H
#define _BUMPSENSORS_H

#include <Arduino.h>

// ========== Bump传感器配置 ==========
// 3Pi+ 32U4的bump传感器引脚
// ⚠️ 根据官方文档，只有左bump（A6）支持ADC读取
// 因此只使用左bump传感器
#define BUMP_LEFT_PIN A6   // 左侧bump传感器（唯一支持ADC的）
#define NUM_BUMP_SENSORS 1 // 只使用1个传感器

// IR发射控制引脚
#define EMIT_PIN 11        // PB7 - IR LED控制

class BumpSensors_c {
  
  public:
    // ADC原始读数（只用左bump）
    float reading;        // 当前读数
    float background;     // 背景校准值
    
    // 滤波缓冲区（移动平均）
    static const int FILTER_SIZE = 5;
    float filter_buffer[FILTER_SIZE];
    int filter_index;
    
    // 构造函数
    BumpSensors_c() {
      reading = 0.0;
      background = 0.0;
      
      // 初始化滤波缓冲区
      for (int i = 0; i < FILTER_SIZE; i++) {
        filter_buffer[i] = 0.0;
      }
      filter_index = 0;
    }
    
    // ========== 初始化bump传感器（ADC模式）==========
    void initialiseForADC() {
      // 设置左bump传感器引脚为输入（A6支持ADC）
      pinMode(BUMP_LEFT_PIN, INPUT);
      
      // 配置IR发射控制引脚 - 设置为INPUT关闭IR LED
      // 根据官方文档：INPUT模式会关闭IR LED，只接收外部IR源
      pinMode(EMIT_PIN, INPUT);  // ✓ INPUT = 关闭IR发射，接收外部IR
      
      // 设置ADC参考电压
      analogReference(DEFAULT);
    }
    
    // ========== 读取bump传感器ADC值 ==========
    void readSensorsADC() {
      // 只读取左bump传感器（A6，唯一支持ADC的）
      
      // 丢弃第一次读数（ADC预热）
      (void)analogRead(BUMP_LEFT_PIN);
      delayMicroseconds(100);
      
      // 读取实际值
      float raw_value = (float)analogRead(BUMP_LEFT_PIN);
      
      // 应用移动平均滤波
      reading = movingAverage(raw_value);
    }
    
    // ========== 移动平均滤波 ==========
    float movingAverage(float new_value) {
      filter_buffer[filter_index] = new_value;
      filter_index = (filter_index + 1) % FILTER_SIZE;
      
      float sum = 0.0;
      for (int i = 0; i < FILTER_SIZE; i++) {
        sum += filter_buffer[i];
      }
      return sum / FILTER_SIZE;
    }
    
    // ========== 校准背景值 ==========
    void calibrateBackground(int samples = 10) {
      Serial.println("开始校准bump传感器背景值...");
      Serial.println("请确保Leader尚未启动或距离较远");
      delay(2000);
      
      float sum = 0.0;
      
      for (int i = 0; i < samples; i++) {
        (void)analogRead(BUMP_LEFT_PIN);
        delayMicroseconds(100);
        sum += (float)analogRead(BUMP_LEFT_PIN);
        delay(50);
      }
      
      background = sum / samples;
      
      Serial.println("Bump传感器背景校准完成：");
      Serial.print("  左bump（A6）背景值: "); Serial.println(background, 2);
    }
    
    // ========== 获取信号强度（减去背景）==========
    float getSignal() {
      readSensorsADC();
      // 返回相对于背景的信号变化
      return reading - background;
    }
    
    // ========== 获取原始读数 ==========
    float getRawReading() {
      readSensorsADC();
      return reading;
    }
    
    // ========== 检测是否有Leader信号 ==========
    bool hasSignal(float threshold = 30.0f) {
      float signal = getSignal();
      return fabs(signal) > threshold;
    }
};

#endif

