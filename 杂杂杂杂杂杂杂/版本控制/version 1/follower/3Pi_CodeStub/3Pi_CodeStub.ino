
#include "Motors.h"         
#include "PID.h"            
#include "Kinematics.h"  
#include "LineSensors.h"
#include <math.h>
#include "Encoders.h"

// ========== 基础组件 ==========
Motors_c motors;    
Kinematics_c pose; 
LineSensors_c line_sensors;

PID_c turn_pid;  // 用于旋转控制的PID

// ========== 速度估计变量 ==========
unsigned long speed_est_ts = 0;
#define SPEED_EST_MS 10
long last_e0 = 0;
long last_e1 = 0;
float speed_right = 0.0f;
float speed_left  = 0.0f;

// ========== Leader跟随参数 ==========
const bool DEBUG_SERIAL = true;
const float TURN_BASE_SPEED = 20.0f;    // 原地旋转基础速度（PWM）
const float TURN_MAX_SPEED = 40.0f;     // 最大旋转速度

// ⚠️ 黑色桌面会吸收IR光，导致信号很弱！
// 需要大幅降低阈值，并使用差值而非绝对值来检测
const float IR_THRESHOLD = 20.0f;       // IR信号检测阈值（降低以适应黑色桌面）
const float ALIGNMENT_TOLERANCE = 30.0f; // 对齐容差（降低以提高灵敏度）

// 背景IR值（启动时校准）
float background_ir[NUM_SENSORS] = {0};
float background_avg = 0.0f;

// ========== 读取IR信号强度 ==========
// 返回5个线传感器的平均IR强度
float getIRSignalStrength() {
  line_sensors.readSensorsADC();
  float total = 0.0f;
  for (int i = 0; i < NUM_SENSORS; i++) {
    total += line_sensors.readings[i];
  }
  return total / NUM_SENSORS;
}

// ========== 计算左右IR差值 ==========
// ⚠️ 重要：在黑色桌面上，Leader会"遮挡"IR反射
// 所以读数是：高=没遮挡（远离Leader），低=被遮挡（靠近Leader）
// 正值表示Leader在左侧，负值表示在右侧（与白色桌面相反！）
float getIRBalance() {
  line_sensors.readSensorsADC();
  
  // 使用多个传感器的加权平均
  // 左侧：传感器0和1
  // 右侧：传感器3和4
  float left_avg = (line_sensors.readings[0] + line_sensors.readings[1]) / 2.0f;
  float right_avg = (line_sensors.readings[3] + line_sensors.readings[4]) / 2.0f;
  
  // ⚠️ 黑色桌面：低值=Leader在那一侧
  // 所以用 左侧 - 右侧（与白色桌面相反）
  return left_avg - right_avg;  // 左侧强度 - 右侧强度
}

// ========== 更新位姿 ==========
void updatePose(){
  static unsigned long pose_ts = 0;
  if ( millis() - pose_ts >= 10 ) { 
    pose_ts = millis(); 
    pose.update(); 
  }
}

// ========== 原地旋转跟随Leader（带PID控制）==========
void followLeaderInPlace() {
  // 读取IR信号差值
  float ir_balance = getIRBalance();
  
  // 检查是否检测到Leader
  // 在黑色桌面上，Leader会"遮挡"IR反射，导致信号下降
  // 所以我们检测：平均信号是否明显低于背景值
  float avg_signal = getIRSignalStrength();
  float signal_drop = background_avg - avg_signal;  // 信号下降量
  float signal_diff = fabs(ir_balance);  // 左右差值的绝对值
  
  // 如果信号下降不明显，说明没有Leader
  if (signal_drop < 50.0f && signal_diff < IR_THRESHOLD) {
    // 没有检测到Leader
    motors.setPWM(0, 0);
    turn_pid.reset();  // 重置PID
    if (DEBUG_SERIAL) {
      static unsigned long debug_ts = 0;
      if (millis() - debug_ts >= 500) {
        debug_ts = millis();
        Serial.println("未检测到Leader（信号无明显变化）");
        Serial.print("当前信号: "); Serial.print(avg_signal, 2);
        Serial.print(" | 背景值: "); Serial.print(background_avg, 2);
        Serial.print(" | 下降: "); Serial.print(signal_drop, 2);
        Serial.print(" | 差值: "); Serial.println(signal_diff, 2);
        Serial.println("提示：");
        Serial.println("  1. 将Leader靠近Follower（10-20cm）");
        Serial.println("  2. 确保Leader的IR发射器开启");
        Serial.println("  3. 移除两个机器人的外壳");
      }
    }
    return;
  }
  
  // 根据IR差值计算旋转方向和速度
  if (fabs(ir_balance) < ALIGNMENT_TOLERANCE) {
    // 已对齐，停止旋转
    motors.setPWM(0, 0);
    turn_pid.reset();  // 重置PID
    if (DEBUG_SERIAL) {
      static unsigned long debug_ts = 0;
      if (millis() - debug_ts >= 500) {
        debug_ts = millis();
        Serial.println("========== 已对齐Leader！==========");
        Serial.print("IR差值: "); Serial.println(ir_balance, 2);
        Serial.print("平均信号: "); Serial.println(avg_signal, 2);
      }
    }
  } else {
    // 需要旋转对齐 - 使用PID控制
    // 目标：让ir_balance趋近于0
    // 测量值：当前的ir_balance
    // 输出：旋转PWM
    
    // PID更新（每50ms）
    static unsigned long pid_ts = 0;
    if (millis() - pid_ts >= 50) {
      pid_ts = millis();
      
      // PID控制：目标是0（对齐），当前测量值是ir_balance
      float turn_pwm = turn_pid.update(0.0f, ir_balance);
      
      // 限制PWM范围
      turn_pwm = constrain(turn_pwm, -TURN_MAX_SPEED, TURN_MAX_SPEED);
      
      // 原地旋转：左右轮反向
      // 如果ir_balance为正（Leader在右），turn_pwm为负，需要右转
      // 右转：左轮前进(+)，右轮后退(-)
      int left_pwm = -(int)turn_pwm;
      int right_pwm = (int)turn_pwm;
      
      motors.setPWM(left_pwm, right_pwm);
      
      if (DEBUG_SERIAL) {
        static unsigned long debug_ts = 0;
        if (millis() - debug_ts >= 200) {
          debug_ts = millis();
          Serial.print("跟随中 | IR差值: "); Serial.print(ir_balance, 2);
          Serial.print(" | 信号下降: "); Serial.print(signal_drop, 2);
          Serial.print(" | PID输出: "); Serial.print(turn_pwm, 2);
          Serial.print(" | PWM L/R: "); Serial.print(left_pwm); 
          Serial.print("/"); Serial.println(right_pwm);
          
          // 打印各传感器读数（标注哪个被遮挡）
          Serial.print("传感器[0-4]: ");
          for (int i = 0; i < NUM_SENSORS; i++) {
            Serial.print(line_sensors.readings[i], 0);
            // 标注明显低于背景的传感器（被Leader遮挡）
            if (background_ir[i] - line_sensors.readings[i] > 100) {
              Serial.print("↓ ");  // 被遮挡
            } else {
              Serial.print(" ");
            }
          }
          Serial.println();
        }
      }
    }
  }
}

// ========== 校准背景IR值 ==========
void calibrateBackground() {
  Serial.println("\n开始校准背景IR值...");
  Serial.println("请确保Leader尚未启动或距离较远");
  delay(2000);
  
  // 采样10次取平均
  for (int sample = 0; sample < 10; sample++) {
    line_sensors.readSensorsADC();
    for (int i = 0; i < NUM_SENSORS; i++) {
      background_ir[i] += line_sensors.readings[i];
    }
    delay(50);
  }
  
  // 计算平均值
  background_avg = 0.0f;
  for (int i = 0; i < NUM_SENSORS; i++) {
    background_ir[i] /= 10.0f;
    background_avg += background_ir[i];
  }
  background_avg /= NUM_SENSORS;
  
  Serial.println("背景IR校准完成：");
  Serial.print("背景平均值: "); Serial.println(background_avg, 2);
  Serial.print("各传感器背景值: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(background_ir[i], 1); Serial.print(" ");
  }
  Serial.println();
}

// ========== SETUP ==========
void setup() {
  // 初始化串口
  Serial.begin(9600);
  delay(1000);
  Serial.println("\n\n*** Follower - Leader跟随程序 ***");
  Serial.println("模式：原地旋转跟随（黑色桌面优化）");

  // 初始化电机
  motors.initialise();

  // 初始化编码器
  setupEncoder0();
  setupEncoder1();

  // 初始化位姿
  pose.initialise(0, 0, 0);
  
  // 初始化线传感器（用于检测Leader的IR信号）
  // 注意：不打开EMIT_PIN，因为Follower只接收，不发射
  line_sensors.initialiseForADC();
  
  // 关闭Follower自己的IR发射器
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);  // 关闭发射，只接收Leader的IR
  
  // 校准背景IR值
  calibrateBackground();
  
  // 初始化旋转PID控制器
  // Kp: 比例增益 - 控制响应速度
  // Ki: 积分增益 - 消除稳态误差
  // Kd: 微分增益 - 减少超调
  // 黑色桌面下需要更高的增益来应对弱信号
  turn_pid.initialise(0.30f, 0.002f, 0.3f);  // 更激进的PID参数
  turn_pid.reset();

  Serial.println("系统初始化完成");
  Serial.println("PID参数: Kp=0.30, Ki=0.002, Kd=0.3（黑色桌面优化）");
  Serial.println("⚠️  注意：黑色桌面会吸收IR光");
  Serial.println("建议：将Leader和Follower靠近（10-30cm）");
  Serial.println("Follower已准备好，等待检测Leader信号...");
  Serial.println("========================================");
  delay(2000);
}

// ========== LOOP ==========
void loop() {
  // 更新位姿
  updatePose();

  // 执行原地旋转跟随
  followLeaderInPlace();
  
  delay(10);
}
