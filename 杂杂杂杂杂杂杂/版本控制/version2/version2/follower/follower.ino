/*
 * ============================================
 * FOLLOWER ROBOT - 直行跟随（简化版）
 * ============================================
 * 
 * 核心理念：
 * 1. 只使用中间传感器(sensor[1,2,3])来测量距离
 * 2. 通过PID控制保持IR值在目标范围（6-12）
 * 3. 只做直行，左右轮相同PWM
 * 
 * 传感器布局：
 *   [0]  [1]  [2]  [3]  [4]
 *        ↑    ↑    ↑
 *        使用这三个传感器的平均值
 * 
 * 控制逻辑：
 *   - 距离误差 = 目标值(9) - 当前中间传感器平均值
 *   - speed = PID(距离误差)
 *   - left_pwm  = speed
 *   - right_pwm = speed
 * 
 * 作者: 根据用户需求定制
 * 日期: 2025-11-27
 */

#include "Encoders.h"
#include "Kinematics.h"
#include "Motors.h"
#include "LineSensors.h"
#include "PID.h"

// ========== 引脚定义 ==========
#define EMIT_PIN 11      // IR发射控制引脚（设为INPUT以关闭，接收Leader IR）
#define BUZZ_PIN 6       // 蜂鸣器引脚
#define LED_PIN  13      // 板载LED
#define BUTTON_B 30      // 按钮B（启动按钮）

// ========== 全局对象 ==========
Motors_c motors;
Kinematics_c kinematics;
LineSensors_c line_sensors;

// ========== PID控制器 ==========
PID_c distance_pid;   // 距离控制PID（只用一个）

// ========== 状态机 ==========
#define STATE_IDLE          0
#define STATE_CALIBRATE     1
#define STATE_WAIT_SIGNAL   2
#define STATE_FOLLOWING     3
#define STATE_LOST_SIGNAL   4

int robot_state = STATE_IDLE;

// ========== 控制参数 ==========
// 距离控制参数（使用原始ADC差值，不缩放）
#define DISTANCE_TARGET 70.0f     // 目标IR差值（降低目标，在实际检测范围内）
#define DISTANCE_MIN 40.0f        // 理想距离范围下限
#define DISTANCE_MAX 100.0f       // 理想距离范围上限
#define DISTANCE_TOO_CLOSE 130.0f // 过近阈值

// PID增益 - 距离控制（优化平滑减速，快速起步）
#define DIST_KP 0.7f      // 比例增益（提高响应速度）
#define DIST_KI 0.004f    // 积分增益（累积误差补偿）
#define DIST_KD 0.6f      // 微分增益（增加阻尼，防止急刹和振荡）

// PWM限制（只前进，不后退）
#define SPEED_MIN 0.0f    // 最小速度（停止，不后退）
#define SPEED_MAX 30.0f   // 最大速度

// 信号检测参数（提高阈值，避免环境光波动误触发）
#define SIGNAL_THRESHOLD 30.0f   // 最小信号阈值（原始ADC差值，提高到30避免误触发）
#define SIGNAL_LOST_TIME 2000    // 信号丢失超时(ms)

// ========== 背景值存储 ==========
float background_values[NUM_SENSORS];  // 存储各传感器的环境背景值

// ========== 控制变量 ==========
float speed = 0.0f;               // 前进速度（左右轮相同）
unsigned long last_signal_time = 0;
unsigned long last_update_time = 0;

#define UPDATE_INTERVAL 25  // 更新周期(ms)

// ========== 函数声明 ==========
void beep(int duration);
void setupIRReceiver();
void calibrateSensors();
float getCenterIRValue();
void updateFollowingControl();
bool hasSignal();
void waitForButton();

// ============================================
// SETUP - 初始化
// ============================================
void setup() {
  // 引脚初始化（最先初始化）
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(BUTTON_B, INPUT_PULLUP);
  
  // 关闭自己的IR发射，只接收Leader的IR（在Serial之前）
  setupIRReceiver();
  
  // 串口初始化（在IR设置之后）
  Serial.begin(115200);
  delay(1000);  // 延长等待时间确保串口稳定
  
  Serial.println("=================================");
  Serial.println("FOLLOWER ROBOT - IR Tracking");
  Serial.println("=================================");
  Serial.println("IR接收模式已启动！");
  Serial.println("本机IR LED已关闭(INPUT+LOW)");
  Serial.println("");
  
  // 初始化各模块
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  kinematics.initialise(0, 0, 0);
  
  // 初始化Line Sensors（用于接收IR信号）
  line_sensors.initialiseForADC();
  
  // 初始化PID控制器
  distance_pid.initialise(DIST_KP, DIST_KI, DIST_KD);
  distance_pid.setOutputLimits(SPEED_MIN, SPEED_MAX);
  
  // ✅ 平滑控制的关键设置
  distance_pid.setMaxDelta(5.0f);      // 限制速度每次最大变化5 PWM（起速更快，减速仍平滑）
  distance_pid.setOutputFilter(0.7f);  // 输出滤波0.7（稍微快一点响应）
  distance_pid.setMinEffectiveOutput(3.0f, 10.0f);  // 电机死区：<3→0, 3-10→10
  
  // 启动提示音
  beep(100);
  delay(200);
  beep(100);
  
  Serial.println("初始化完成！");
  Serial.println("");
  Serial.println("控制策略（简化版 - 只前进直行）：");
  Serial.println("  - 中间传感器[1,2,3]平均值 → 距离控制");
  Serial.println("  - 目标IR差值: 70 (原始ADC)");
  Serial.println("  - 理想距离：IR差值 40-100");
  Serial.println("  - 信号检测阈值: 30 (避免环境光误触发)");
  Serial.println("  - 过近时：减速到0（停止），不后退");
  Serial.println("  - 过远时：加速前进");
  Serial.println("  - 速度范围：0-30 PWM（只前进）");
  Serial.println("");
  
  Serial.println("请按Button B开始校准...");
  waitForButton();
  
  // 校准传感器
  robot_state = STATE_CALIBRATE;
  calibrateSensors();
  
  // 等待检测到Leader信号
  robot_state = STATE_WAIT_SIGNAL;
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
        // 定期读取并显示传感器值
        static unsigned long last_check = 0;
        if (millis() - last_check >= 300) {
          last_check = millis();
          
          float center_value = getCenterIRValue();
          
          Serial.print("等待信号... IR值: ");
          Serial.print(center_value, 1);
          
          // 显示中间3个传感器的IR强度差值（调试用）
          line_sensors.readSensorsADC();
          Serial.print(" | 中间Diff[");
          Serial.print(background_values[1] - line_sensors.readings[1], 0);
          Serial.print(",");
          Serial.print(background_values[2] - line_sensors.readings[2], 0);
          Serial.print(",");
          Serial.print(background_values[3] - line_sensors.readings[3], 0);
          Serial.print("] (需要>");
          Serial.print(SIGNAL_THRESHOLD, 0);
          Serial.println(")");
          
          // 检测到信号
          if (hasSignal()) {
            robot_state = STATE_FOLLOWING;
            last_signal_time = millis();
            
            beep(200);
            
            Serial.println("");
            Serial.println("检测到Leader！开始跟随...");
            Serial.println("");
          }
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
        
        // 检查信号状态
        if (hasSignal()) {
          last_signal_time = millis();
          
          // 执行跟随控制
          updateFollowingControl();
          
        } else {
          // 信号丢失检查
          if (millis() - last_signal_time > SIGNAL_LOST_TIME) {
            robot_state = STATE_LOST_SIGNAL;
            motors.setPWM(0, 0);
            
            Serial.println("");
            Serial.println("警告：丢失Leader信号！");
            Serial.println("");
          }
        }
        
        // 定期输出状态（调试用）
        static unsigned long last_debug = 0;
        if (millis() - last_debug >= 300) {  // 改为300ms更新快一点
          last_debug = millis();
          
          float center_ir = getCenterIRValue();
          float error = DISTANCE_TARGET - center_ir;  // 计算误差
          
          Serial.print("IR=");
          Serial.print(center_ir, 1);
          Serial.print(" | Err=");
          Serial.print(error, 1);
          Serial.print(" | Speed=");
          Serial.print(speed, 1);
          Serial.print(" | PWM(L=");
          Serial.print(speed, 0);
          Serial.print(",R=");
          Serial.print(speed, 0);
          Serial.print(")");
          
          // 显示各传感器的IR信号强度（差值）
          Serial.print(" | Diff[");
          Serial.print(background_values[1] - line_sensors.readings[1], 0);
          Serial.print(",");
          Serial.print(background_values[2] - line_sensors.readings[2], 0);
          Serial.print(",");
          Serial.print(background_values[3] - line_sensors.readings[3], 0);
          Serial.print("]");
          
          // 距离状态提示
          if (center_ir > DISTANCE_TOO_CLOSE) {
            Serial.print(" [太近-停止]");
          } else if (center_ir > DISTANCE_MAX) {
            Serial.print(" [靠近-减速]");
          } else if (center_ir < DISTANCE_MIN) {
            Serial.print(" [太远-加速]");
          } else {
            Serial.print(" [合适]");
          }
          
          Serial.println();
        }
      }
      break;
      
    // ========================================
    // 丢失信号 - 尝试重新寻找
    // ========================================
    case STATE_LOST_SIGNAL:
      {
        motors.setPWM(0, 0);
        
        static unsigned long lost_start = millis();
        
        // 尝试重新寻找（停留3秒）
        if (millis() - lost_start < 3000) {
          if (hasSignal()) {
            robot_state = STATE_FOLLOWING;
            last_signal_time = millis();
            lost_start = millis();
            
            beep(100);
            
            Serial.println("重新检测到Leader！");
          }
        } else {
          // 超时，返回等待状态
          robot_state = STATE_WAIT_SIGNAL;
          lost_start = millis();
          
          Serial.println("返回等待状态...");
        }
      }
      break;
      
    default:
      break;
  }
}

// ============================================
// 核心控制函数 - 直行跟随控制（只前进）
// ============================================
void updateFollowingControl() {
  // 1. 获取中间传感器的IR值（距离指示）
  float center_ir = getCenterIRValue();
  
  // 2. 距离控制：使用PID让IR值保持在目标值
  //    IR值大 = 距离近 → 减速（但不后退）
  //    IR值小 = 距离远 → 加速
  //    PID: error = 目标值 - 当前值
  //    距离远(IR小) → error正 → 速度增加
  //    距离近(IR大) → error负 → 速度减小（限制为≥0）
  speed = distance_pid.update(DISTANCE_TARGET, center_ir);
  
  // 调试：显示PID原始输出（在限制之前）
  static unsigned long last_pid_debug = 0;
  if (millis() - last_pid_debug >= 300) {
    last_pid_debug = millis();
    float error = DISTANCE_TARGET - center_ir;
    Serial.print("[PID Debug] Raw_Speed=");
    Serial.print(speed, 2);
    Serial.print(" Error=");
    Serial.print(error, 1);
    Serial.print(" P=");
    Serial.print(distance_pid.p_term, 2);
    Serial.print(" I=");
    Serial.print(distance_pid.i_term, 2);
    Serial.print(" D=");
    Serial.println(distance_pid.d_term, 2);
  }
  
  // 3. 软性速度限制：负值时平滑衰减，不是直接截断（避免急刹）
  //    PID + MaxDelta + 滤波已经提供平滑，这里只做简单限制
  if (speed < 0) {
    speed = 0;  // 直接限制为0，依赖PID的平滑机制
  }
  
  // 4. 左右轮使用相同PWM，只做直行
  float left_pwm  = speed;
  float right_pwm = speed;
  
  // 5. PWM限幅
  left_pwm  = constrain(left_pwm,  0, MAX_PWM);
  right_pwm = constrain(right_pwm, 0, MAX_PWM);
  
  // 6. 发送到电机
  motors.setPWM(left_pwm, right_pwm);
}

// ============================================
// 获取中间传感器的平均IR值（直接用原始ADC差值）
// ============================================
float getCenterIRValue() {
  // 读取传感器原始ADC值
  line_sensors.readSensorsADC();
  
  // 计算中间3个传感器的IR强度（当前值 - 背景值）
  // 背景值减法：Leader的IR会让读数变小，所以用背景值-当前值
  float ir_1 = background_values[1] - line_sensors.readings[1];
  float ir_2 = background_values[2] - line_sensors.readings[2];
  float ir_3 = background_values[3] - line_sensors.readings[3];
  
  // 计算平均值（直接使用原始差值，不缩放）
  float ir_avg = (ir_1 + ir_2 + ir_3) / 3.0f;
  
  // 限制最小值为0（避免负值）
  if (ir_avg < 0) ir_avg = 0;
  
  return ir_avg;
}

// ============================================
// 检测是否有有效信号
// ============================================
bool hasSignal() {
  float center_ir = getCenterIRValue();
  return (center_ir > SIGNAL_THRESHOLD);
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
  // 设置EMIT_PIN为INPUT，并拉低电压到LOW
  // 这样才能关闭自己的IR发射，接收Leader发射的IR信号
  pinMode(EMIT_PIN, INPUT);
  digitalWrite(EMIT_PIN, LOW);  // 确保电压为LOW
  
  // 注意：此时Serial可能还未初始化，所以不在这里打印
}

// ============================================
// 校准传感器（简化版 - 只采集背景值）
// ============================================
void calibrateSensors() {
  Serial.println("");
  Serial.println("===== 传感器校准（简化版）=====");
  Serial.println("采集环境背景值...");
  Serial.println("请确保Leader未启动或距离很远！");
  Serial.println("");
  
  beep(100);
  delay(2000);  // 给2秒准备时间
  
  // 初始化背景值为0
  for (int i = 0; i < NUM_SENSORS; i++) {
    background_values[i] = 0.0f;
  }
  
  // 采集背景值（多次采样求平均，3秒）
  Serial.println("开始采样...");
  int sample_count = 0;
  unsigned long cal_start = millis();
  
  while (millis() - cal_start < 3000) {
    line_sensors.readSensorsADC();
    
    for (int i = 0; i < NUM_SENSORS; i++) {
      background_values[i] += line_sensors.readings[i];
    }
    sample_count++;
    delay(10);
  }
  
  // 计算平均值
  for (int i = 0; i < NUM_SENSORS; i++) {
    background_values[i] /= sample_count;
  }
  
  Serial.println("校准完成！");
  Serial.print("采样次数: ");
  Serial.println(sample_count);
  Serial.println("环境背景值：");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("  Sensor[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(background_values[i], 1);
  }
  Serial.println("");
  Serial.println("现在可以启动Leader并开始跟随！");
  Serial.println("");
  
  beep(100);
  delay(200);
  beep(100);
  delay(500);
}

// ============================================
// 等待按钮按下
// ============================================
void waitForButton() {
  while (digitalRead(BUTTON_B) == HIGH) {
    delay(10);
  }
  
  // 等待释放
  while (digitalRead(BUTTON_B) == LOW) {
    delay(10);
  }
  
  beep(100);
  delay(200);
}

