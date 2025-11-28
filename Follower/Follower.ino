/*
 * ============================================
 * FOLLOWER ROBOT - 正着走 + 接收IR
 * ============================================
 * 
 * 核心理念（Hypothesis 3）：
 * 1. Bump sensor (ADC) → 纵向控制 (前后距离) → base_speed
 * 2. Line sensor (ADC) → 横向控制 (左右方向) → turn_output
 * 3. 最终控制：
 *    left_pwm  = base_speed - turn_output
 *    right_pwm = base_speed + turn_output
 * 
 * 传感器配置：
 * - Bump: Digital input (检测Leader的IR信号强度)
 * - Line: ADC input (检测Leader的IR横向分布)
 * 
 * 作者: Hypothesis 3 Team
 * 日期: 2025
 */

#include "Encoders.h"
#include "Kinematics.h"
#include "Motors.h"
#include "LineSensors.h"
#include "PID.h"
#include "lcd.h"

// ========== 引脚定义 ==========
#define BUMP_L 4         // 左Bump传感器引脚
#define BUMP_R 5         // 右Bump传感器引脚
#define EMIT_PIN 11      // IR发射控制引脚
#define BUZZ_PIN 6       // 蜂鸣器引脚
#define LED_PIN  13      // 板载LED

// ========== 全局对象 ==========
Motors_c motors;
Kinematics_c kinematics;
LineSensors_c line_sensors;
LCD_c lcd(0, 1, 14, 17, 13, 30);  // LCD对象

// ========== Bump传感器背景值 ==========
unsigned long bump_background_L = 0;
unsigned long bump_background_R = 0;

// 两个独立的PID控制器
PID_c bump_pid;   // 用于纵向控制 (前后距离)
PID_c line_pid;   // 用于横向控制 (左右方向)

// ========== 状态机 ==========
#define STATE_INITIAL       0
#define STATE_CALIBRATE     1
#define STATE_WAIT_SIGNAL   2
#define STATE_FOLLOWING     3
#define STATE_LOST_SIGNAL   4
#define STATE_FINISHED      5

int robot_state = STATE_INITIAL;

// ========== PID控制参数 ==========
// Bump PID (纵向 - 前后距离控制)
#define BUMP_KP 0.8f
#define BUMP_KI 0.002f
#define BUMP_KD 0.5f
#define BUMP_TARGET 500.0f  // 目标信号强度（elapsed time差值，单位us）

// Line PID (横向 - 左右方向控制)
#define LINE_KP 1.5f
#define LINE_KI 0.001f
#define LINE_KD 0.3f
#define LINE_TARGET 0.0f    // 目标：居中（偏差=0）

// ========== 控制输出 ==========
float base_speed = 0.0f;    // 由bump控制的基础速度
float turn_output = 0.0f;   // 由line控制的转向量

// ========== PWM限制 ==========
#define BASE_SPEED_MIN 20.0f
#define BASE_SPEED_MAX 60.0f
#define TURN_OUTPUT_MAX 40.0f

// ========== 信号检测参数 ==========
#define SIGNAL_THRESHOLD 100.0f   // 最小信号阈值（elapsed time变化，单位us）
#define SIGNAL_LOST_TIME 1000     // 信号丢失超时 (ms)

unsigned long last_signal_time = 0;

// ========== 实验时间 ==========
unsigned long experiment_start_time = 0;
#define EXPERIMENT_DURATION 20000  // 20秒

// ========== 更新周期 ==========
unsigned long last_update_time = 0;
#define UPDATE_INTERVAL 30  // 30ms

// ========== 前向声明 ==========
void beep(int duration);
void setupIRReceiver();
void calibrateBumpSensors();
void calibrateLineSensors();
float getLineError();
float getBumpSignal();
void updateFollowingControl();
unsigned long readBump(int pin);

// ============================================
// Bump传感器读取函数（Digital Input方式）
// ============================================
unsigned long readBump(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);      // 充电电容
  delayMicroseconds(10);
  pinMode(pin, INPUT);          // 开始放电
  
  unsigned long t0 = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t0 > 4500) break;  // 超时保护
  }
  return micros() - t0;  // 返回放电时间（us）
}

// ============================================
// SETUP - 初始化
// ============================================
void setup() {
  // 串口初始化
  Serial.begin(9600);
  delay(500);
  Serial.println("=================================");
  Serial.println("FOLLOWER ROBOT - Hypothesis 3");
  Serial.println("=================================");

  // 引脚初始化
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  
  // 初始化各模块
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  kinematics.initialise(0, 0, 0);
  
  // 初始化IR接收器（关键！）
  setupIRReceiver();
  
  // 初始化传感器
  line_sensors.initialiseForADC();
  
  // 初始化Bump传感器引脚（Digital Input模式）
  pinMode(BUMP_L, INPUT);
  pinMode(BUMP_R, INPUT);
  
  // 初始化LCD
  lcd.init();
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("Follower Ready");
  
  // 初始化PID控制器
  bump_pid.initialise(BUMP_KP, BUMP_KI, BUMP_KD);
  bump_pid.setOutputLimits(BASE_SPEED_MIN, BASE_SPEED_MAX);
  
  line_pid.initialise(LINE_KP, LINE_KI, LINE_KD);
  line_pid.setOutputLimits(-TURN_OUTPUT_MAX, TURN_OUTPUT_MAX);
  
  // 启动提示音
  beep(100);
  delay(200);
  beep(100);
  
  Serial.println("Follower初始化完成！");
  Serial.println("");
  Serial.println("核心控制逻辑：");
  Serial.println("  Bump → base_speed (前后距离)");
  Serial.println("  Line → turn_output (左右方向)");
  Serial.println("  left_pwm  = base_speed - turn_output");
  Serial.println("  right_pwm = base_speed + turn_output");
  Serial.println("");
  
  // 校准传感器
  robot_state = STATE_CALIBRATE;
  calibrateBumpSensors();
  calibrateLineSensors();
  
  // 等待Leader信号
  robot_state = STATE_WAIT_SIGNAL;
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("Wait Leader...");
  Serial.println("等待检测到Leader信号...");
  
  last_update_time = millis();
}

// ============================================
// LOOP - 主循环
// ============================================
void loop() {
  
  switch(robot_state) {
    
    // ========================================
    // 等待Leader信号
    // ========================================
    case STATE_WAIT_SIGNAL:
      {
        float signal = getBumpSignal();
        
        // 显示当前信号强度
        static unsigned long last_print = 0;
        if (millis() - last_print >= 300) {
          last_print = millis();
          Serial.print("等待信号... 当前强度: ");
          Serial.println(signal, 2);
        }
        
        // 检测到足够强的信号
        if (signal > SIGNAL_THRESHOLD) {
          robot_state = STATE_FOLLOWING;
          experiment_start_time = millis();
          last_signal_time = millis();
          
          lcd.clear();
          lcd.gotoXY(0, 0);
          lcd.print("Following!");
          beep(200);
          
          Serial.println("");
          Serial.println("检测到Leader！开始跟随...");
          Serial.println("");
        }
      }
      break;
      
    // ========================================
    // 跟随Leader
    // ========================================
    case STATE_FOLLOWING:
      if (millis() - last_update_time >= UPDATE_INTERVAL) {
        last_update_time = millis();
        
        // 更新运动学
        kinematics.update();
        
        // 执行跟随控制（核心逻辑）
        updateFollowingControl();
        
        // 检查信号状态
        float signal = getBumpSignal();
        if (signal > SIGNAL_THRESHOLD) {
          last_signal_time = millis();
        } else {
          // 信号丢失检查
          if (millis() - last_signal_time > SIGNAL_LOST_TIME) {
            robot_state = STATE_LOST_SIGNAL;
            motors.setPWM(0, 0);
            Serial.println("警告：丢失Leader信号！");
          }
        }
        
        // 定期输出状态（调试用）
        static unsigned long last_debug_print = 0;
        if (millis() - last_debug_print >= 500) {
          last_debug_print = millis();
          
          Serial.print("Bump=");
          Serial.print(signal, 2);
          Serial.print(" | LineErr=");
          Serial.print(getLineError(), 2);
          Serial.print(" | Base=");
          Serial.print(base_speed, 2);
          Serial.print(" | Turn=");
          Serial.print(turn_output, 2);
          Serial.print(" | X=");
          Serial.print(kinematics.x, 2);
          Serial.print(" Y=");
          Serial.println(kinematics.y, 2);
        }
        
        // 检查实验时间
        if (millis() - experiment_start_time >= EXPERIMENT_DURATION) {
          robot_state = STATE_FINISHED;
          motors.setPWM(0, 0);
          beep(500);
          Serial.println("实验完成！");
          lcd.clear();
          lcd.gotoXY(0, 0);
          lcd.print("Finished!");
        }
      }
      break;
      
    // ========================================
    // 丢失信号
    // ========================================
    case STATE_LOST_SIGNAL:
      motors.setPWM(0, 0);
      lcd.clear();
      lcd.gotoXY(0, 0);
      lcd.print("Signal Lost!");
      
      // 尝试重新寻找
      delay(500);
      float signal = getBumpSignal();
      if (signal > SIGNAL_THRESHOLD) {
        robot_state = STATE_FOLLOWING;
        last_signal_time = millis();
        Serial.println("重新检测到Leader信号！");
      }
      break;
      
    // ========================================
    // 实验结束
    // ========================================
    case STATE_FINISHED:
      motors.setPWM(0, 0);
      lcd.clear();
      lcd.gotoXY(0, 0);
      lcd.print("Finished!");
      
      // 输出最终位置
      static bool printed_final = false;
      if (!printed_final) {
        printed_final = true;
        Serial.println("");
        Serial.println("===== 最终位置 =====");
        Serial.print("X: "); Serial.println(kinematics.x, 2);
        Serial.print("Y: "); Serial.println(kinematics.y, 2);
        Serial.print("θ: "); Serial.println(kinematics.theta, 2);
        Serial.println("===================");
      }
      
      delay(1000);
      break;
      
    default:
      break;
  }
}

// ============================================
// 核心控制函数 - 跟随控制
// ============================================
void updateFollowingControl() {
  // 1. 获取Bump信号 (纵向 - 距离)
  float bump_signal = getBumpSignal();
  
  // 2. 用Bump PID计算base_speed
  //    信号强 = 距离近 → 减速
  //    信号弱 = 距离远 → 加速
  base_speed = bump_pid.update(BUMP_TARGET, bump_signal);
  
  // 3. 获取Line误差 (横向 - 方向)
  float line_error = getLineError();
  
  // 4. 用Line PID计算turn_output
  //    正误差 = Leader偏右 → 右转
  //    负误差 = Leader偏左 → 左转
  turn_output = line_pid.update(LINE_TARGET, line_error);
  
  // 5. 计算最终PWM（核心公式！）
  float left_pwm  = base_speed - turn_output;
  float right_pwm = base_speed + turn_output;
  
  // 6. 限制PWM范围
  left_pwm  = constrain(left_pwm,  -MAX_PWM, MAX_PWM);
  right_pwm = constrain(right_pwm, -MAX_PWM, MAX_PWM);
  
  // 7. 发送到电机
  motors.setPWM(left_pwm, right_pwm);
}

// ============================================
// 获取Bump信号强度
// ============================================
float getBumpSignal() {
  // 读取左右Bump传感器（elapsed time方式）
  unsigned long time_L = readBump(BUMP_L);
  unsigned long time_R = readBump(BUMP_R);
  
  // 计算相对于背景的变化（背景 - 当前时间）
  // Leader的IR会让放电时间变短，所以信号 = 背景时间 - 当前时间
  // 信号为正 = 检测到Leader
  long signal_L = (long)bump_background_L - (long)time_L;
  long signal_R = (long)bump_background_R - (long)time_R;
  
  // 取两个传感器的平均值
  float signal = (signal_L + signal_R) / 2.0f;
  
  return signal;
}

// ============================================
// 获取Line横向偏差
// ============================================
float getLineError() {
  // 读取line sensors
  line_sensors.calcCalibratedADC();
  
  // 计算加权位置偏差
  // 传感器位置权重: [-2, -1, 0, +1, +2]
  // 负值=左, 正值=右
  
  float weighted_sum = 0.0f;
  float total_activation = 0.0f;
  
  const float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    float activation = line_sensors.calibrated[i];
    weighted_sum += weights[i] * activation;
    total_activation += activation;
  }
  
  // 避免除零
  if (total_activation < 0.1f) {
    return 0.0f;  // 没有信号，返回0
  }
  
  // 归一化偏差
  float error = weighted_sum / total_activation;
  
  return error;
}

// ============================================
// 蜂鸣器提示音
// ============================================
void beep(int duration) {
  analogWrite(BUZZ_PIN, 120);
  delay(duration);
  analogWrite(BUZZ_PIN, 0);
}

// ============================================
// 设置IR接收器（关键函数）
// ============================================
void setupIRReceiver() {
  // 设置EMIT_PIN为INPUT，关闭自己的IR发射
  // 这样才能接收Leader发射的IR信号
  pinMode(EMIT_PIN, INPUT);  // INPUT = 关闭IR发射
  
  Serial.println("IR接收器已启动！");
  Serial.println("本机IR LED已关闭，只接收外部IR...");
}

// ============================================
// 校准Bump传感器
// ============================================
void calibrateBumpSensors() {
  Serial.println("");
  Serial.println("===== Bump传感器校准 =====");
  Serial.println("请确保Leader尚未启动或距离较远！");
  Serial.println("正在采集背景值...");
  
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("Cal Bump...");
  
  beep(100);
  delay(2000);
  
  // 采集背景值（多次采样求平均）
  int samples = 20;
  
  bump_background_L = 0;
  bump_background_R = 0;
  
  for (int i = 0; i < samples; i++) {
    bump_background_L += readBump(BUMP_L);
    bump_background_R += readBump(BUMP_R);
    delay(50);
  }
  
  bump_background_L /= samples;
  bump_background_R /= samples;
  
  Serial.println("Bump传感器背景值：");
  Serial.print("  左Bump: ");
  Serial.print(bump_background_L);
  Serial.println(" us");
  Serial.print("  右Bump: ");
  Serial.print(bump_background_R);
  Serial.println(" us");
  Serial.println("Bump校准完成！");
  Serial.println("");
  
  beep(100);
  delay(500);
}

// ============================================
// 校准Line传感器
// ============================================
void calibrateLineSensors() {
  Serial.println("===== Line传感器校准 =====");
  Serial.println("请确保Leader的IR能照到Line sensors！");
  Serial.println("采集中...");
  
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("Cal Line...");
  
  beep(100);
  delay(1000);
  
  // 初始化最小最大值
  for (int i = 0; i < NUM_SENSORS; i++) {
    line_sensors.minimum[i] = 1023.0f;
    line_sensors.maximum[i] = 0.0f;
  }
  
  // 采集校准数据（手动移动robot让line sensors扫过Leader）
  unsigned long cal_start = millis();
  while (millis() - cal_start < 3000) {
    line_sensors.readSensorsADC();
    
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (line_sensors.readings[i] < line_sensors.minimum[i]) {
        line_sensors.minimum[i] = line_sensors.readings[i];
      }
      if (line_sensors.readings[i] > line_sensors.maximum[i]) {
        line_sensors.maximum[i] = line_sensors.readings[i];
      }
    }
    
    delay(10);
  }
  
  Serial.println("Line传感器校准完成！");
  Serial.print("Sensor ranges: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("[");
    Serial.print(line_sensors.minimum[i], 0);
    Serial.print("-");
    Serial.print(line_sensors.maximum[i], 0);
    Serial.print("] ");
  }
  Serial.println("");
  Serial.println("");
  
  beep(100);
  delay(500);
}

