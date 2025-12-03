
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

// ========== 跟随移动参数 ==========
const float FOLLOW_SPEED = 30.0f;       // 跟随前进速度（PWM）
const float MIN_SIGNAL_TO_FOLLOW = 250.0f;  // 最小信号强度（太远就不跟了）

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

// ========== 跟随Leader（简化版：旋转对齐 + 持续跟随）==========
void followLeaderSimple() {
  // 读取IR信号
  float ir_balance = getIRBalance();
  float avg_signal = getIRSignalStrength();
  float signal_drop = background_avg - avg_signal;
  float signal_diff = fabs(ir_balance);
  
  // 检查是否检测到Leader
  // 降低阈值以适应高背景值环境
  if (signal_drop < 20.0f && signal_diff < IR_THRESHOLD) {
    // 没有检测到Leader，停止
    motors.setPWM(0, 0);
    turn_pid.reset();
    
    if (DEBUG_SERIAL) {
      static unsigned long debug_ts = 0;
      if (millis() - debug_ts >= 500) {
        debug_ts = millis();
        Serial.println("未检测到Leader - 停止");
        Serial.print("信号: "); Serial.print(avg_signal, 2);
        Serial.print(" | 背景: "); Serial.println(background_avg, 2);
      }
    }
    return;
  }
  
  // 控制更新（每50ms）
  static unsigned long control_ts = 0;
  if (millis() - control_ts >= 50) {
    control_ts = millis();
    
    // ========== 1. 计算旋转控制（对齐方向）==========
    float turn_pwm = 0;
    if (fabs(ir_balance) > ALIGNMENT_TOLERANCE) {
      // 需要旋转对齐
      turn_pwm = turn_pid.update(0.0f, ir_balance);
      turn_pwm = constrain(turn_pwm, -TURN_MAX_SPEED, TURN_MAX_SPEED);
    } else {
      // 已对齐，重置旋转PID
      turn_pid.reset();
    }
    
    // ========== 2. 简单的前进控制 ==========
    float forward_pwm = 0;
    
    // 只要检测到Leader，就持续前进
    // 信号太弱（太远）时才停止
    if (avg_signal > MIN_SIGNAL_TO_FOLLOW) {
      // 检测到Leader，持续跟随
      forward_pwm = FOLLOW_SPEED;
    } else {
      // 信号太弱，停止前进
      forward_pwm = 0;
    }
    
    // ========== 3. 混合控制：旋转 + 前进 ==========
    // 如果需要大幅旋转，减少前进速度
    float turn_factor = fabs(turn_pwm) / TURN_MAX_SPEED;  // 0-1
    forward_pwm *= (1.0 - turn_factor * 0.6);  // 旋转时减速60%
    
    // 计算左右轮PWM
    int left_pwm = (int)(forward_pwm - turn_pwm);
    int right_pwm = (int)(forward_pwm + turn_pwm);
    
    // 限制PWM范围
    left_pwm = constrain(left_pwm, -60, 60);
    right_pwm = constrain(right_pwm, -60, 60);
    
    motors.setPWM(left_pwm, right_pwm);
    
    // ========== 4. 调试输出 ==========
    if (DEBUG_SERIAL) {
      static unsigned long debug_ts = 0;
      if (millis() - debug_ts >= 200) {
        debug_ts = millis();
        
        Serial.print("跟随 | 信号: "); Serial.print(avg_signal, 0);
        Serial.print(" | IR差值: "); Serial.print(ir_balance, 2);
        Serial.print(" | 转向: "); Serial.print((int)turn_pwm);
        Serial.print(" | 前进: "); Serial.print((int)forward_pwm);
        Serial.print(" | L/R: "); Serial.print(left_pwm);
        Serial.print("/"); Serial.print(right_pwm);
        
        // 状态指示
        if (fabs(ir_balance) > ALIGNMENT_TOLERANCE) {
          Serial.print(" | 调整方向");
        } else {
          Serial.print(" | 已对齐");
        }
        
        if (forward_pwm > 0) {
          Serial.println(" | 跟随中");
        } else {
          Serial.println(" | 太远-停止");
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
  line_sensors.initialiseForADC();
  
  // ⚠️ 重要：initialiseForADC()会设置EMIT_PIN为OUTPUT+HIGH
  // 必须在之后重新设置为INPUT来关闭IR发射
  // 根据官方文档：INPUT模式会关闭IR LED
  pinMode(EMIT_PIN, INPUT);  // INPUT = 关闭IR发射，只接收外部IR源
  
  Serial.println("✓ 线传感器已初始化");
  Serial.println("✓ EMIT_PIN已设置为INPUT（关闭IR发射）");
  
  // 校准背景IR值
  calibrateBackground();
  
  // 初始化旋转PID控制器
  turn_pid.initialise(0.30f, 0.002f, 0.3f);
  turn_pid.reset();

  Serial.println("\n系统初始化完成");
  Serial.println("========================================");
  Serial.println("控制模式：简单跟随（旋转对齐 + 持续前进）");
  Serial.println("PID参数：");
  Serial.println("  - 旋转PID: Kp=0.30, Ki=0.002, Kd=0.3");
  Serial.println("跟随参数：");
  Serial.println("  - 跟随速度: 30 PWM");
  Serial.println("  - 最小信号: 250 (太远停止)");
  Serial.println("⚠️  注意：");
  Serial.println("  - Follower会持续跟随Leader");
  Serial.println("  - 不会保持固定距离");
  Serial.println("  - 需要手动控制Leader避免碰撞");
  Serial.println("========================================");
  Serial.println("Follower已准备好，等待检测Leader信号...\n");
  delay(2000);
}

// ========== LOOP ==========
void loop() {
  // 更新位姿
  updatePose();

  // 执行简单跟随（旋转对齐 + 持续前进）
  followLeaderSimple();
  
  delay(10);
}
