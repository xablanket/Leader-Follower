/*
 * Leader机器人 - 整合版本
 * 功能：
 * 1. 按钮A选择Line Sensor模式
 * 2. 按钮B选择Bump Sensor模式
 * 3. 使用增量式PID进行速度控制和方向纠正
 * 4. 发射IR信号供Follower检测
 */

#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "LineSensors.h"

// ===== 按钮引脚定义 =====
#define BUTTON_A 14  // 3Pi+ 按钮A
#define BUTTON_B 30  // 3Pi+ 按钮B
#define BUTTON_C 17  // 3Pi+ 按钮C (可选)

// ===== IR发射控制引脚 =====
#define EMIT_PIN 11

// ===== 运行模式 =====
enum RunMode {
  MODE_STANDBY,      // 待机模式
  MODE_LINE_SENSOR,  // Line Sensor模式
  MODE_BUMP_SENSOR   // Bump Sensor模式
};

RunMode current_mode = MODE_STANDBY;

// ===== 基础组件 =====
Motors_c motors;
Kinematics_c kin;
LineSensors_c line_sensors;

// ===== PID控制器 =====
PID_c speed_pid_left;   // 左轮速度PID
PID_c speed_pid_right;  // 右轮速度PID
PID_c direction_pid;    // 方向纠正PID

// ===== 速度控制参数 =====
#define DRIVE_EST_MS 30UL   // 速度估算周期
#define DRIVE_PID_MS 60UL   // PID更新周期
#define DRIVE_PWM_LIMIT 40  // PWM上限

// 目标速度（counts per second）
const float DEMAND_SPEED = 100.0f;   // 目标速度

// 速度PID参数（增量式）
float KP_SPEED = 0.15f, KI_SPEED = 0.008f, KD_SPEED = 0.05f;

// 方向纠正PID参数（增量式）
float KP_DIR = 0.5f, KI_DIR = 0.01f, KD_DIR = 0.1f;

// ===== 运动状态变量 =====
unsigned long drive_est_ts = 0, drive_pid_ts = 0;
long d_last_e0 = 0, d_last_e1 = 0;
float spdL_cps = 0.0f, spdR_cps = 0.0f;

// 速度平滑滤波
float spdL_filtered = 0.0f;
float spdR_filtered = 0.0f;
const float SPEED_FILTER_ALPHA = 0.3f;

// PWM记录（用于数据记录）
float last_pwm_L = 0.0f;
float last_pwm_R = 0.0f;

// ===== 往返运动参数 =====
#define MOVE_DURATION 3000   // 移动持续时间（毫秒）
#define NUM_ROUNDS 5         // 往返回合数

// 运动状态
enum MotionState {
  MOVING_BACKWARD,  // 后退中
  MOVING_FORWARD,   // 前进中
  COMPLETED,        // 完成
  REPORTING_RESULTS // 报告结果
};

MotionState motion_state = MOVING_BACKWARD;
unsigned long motion_start_time = 0;
int current_round = 0;

// ===== 数据记录功能 =====
// 注意：为了节省内存，数据记录功能已禁用
// 如果需要数据记录，请使用SD卡或实时串口传输
// #define MAX_RESULTS 30
// #define VARIABLES 7
// float results[MAX_RESULTS][VARIABLES];  // 占用840字节RAM
// int results_index = 0;
// unsigned long record_results_ts = 0;
// unsigned long results_interval_ms = 0;
// bool recording_enabled = false;

// ===== 辅助函数 =====
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int iround(float v) {
  return (int)lroundf(v);
}

// ===== 按钮读取（带防抖）=====
bool readButton(int pin) {
  if (digitalRead(pin) == LOW) {
    delay(50);  // 防抖
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW);  // 等待释放
      delay(50);  // 防抖
      return true;
    }
  }
  return false;
}

// ===== 速度估算 =====
void estimateSpeed() {
  unsigned long now = millis();
  if (now - drive_est_ts < DRIVE_EST_MS) return;
  
  unsigned long dt = now - drive_est_ts;
  drive_est_ts = now;
  
  long e0 = count_e0;
  long e1 = count_e1;
  
  long delta_e0 = e0 - d_last_e0;
  long delta_e1 = e1 - d_last_e1;
  
  d_last_e0 = e0;
  d_last_e1 = e1;
  
  // 转换为 counts per second
  // encoder0对应右轮，encoder1对应左轮
  float spdR_raw = (float)delta_e0 * 1000.0f / (float)dt;
  float spdL_raw = (float)delta_e1 * 1000.0f / (float)dt;
  
  // 低通滤波
  spdR_filtered = spdR_filtered * (1.0f - SPEED_FILTER_ALPHA) + spdR_raw * SPEED_FILTER_ALPHA;
  spdL_filtered = spdL_filtered * (1.0f - SPEED_FILTER_ALPHA) + spdL_raw * SPEED_FILTER_ALPHA;
  
  spdR_cps = spdR_filtered;
  spdL_cps = spdL_filtered;
}

// ===== PID速度控制（带方向纠正）=====
void driveWithPID(float demand_L, float demand_R, float direction_correction = 0.0f) {
  unsigned long now = millis();
  if (now - drive_pid_ts < DRIVE_PID_MS) return;
  drive_pid_ts = now;
  
  // 速度PID输出
  float pid_L = speed_pid_left.update(demand_L, spdL_cps);
  float pid_R = speed_pid_right.update(demand_R, spdR_cps);
  
  // 应用方向纠正（如果有）
  pid_L -= direction_correction;
  pid_R += direction_correction;
  
  // 限制PWM
  pid_L = clampf(pid_L, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
  pid_R = clampf(pid_R, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
  
  // 记录PWM值（用于数据记录）
  last_pwm_L = pid_L;
  last_pwm_R = pid_R;
  
  motors.setPWM(iround(pid_L), iround(pid_R));
}

// ===== 模式选择界面 =====
void waitForModeSelection() {
  Serial.println("\n========================================");
  Serial.println("*** Leader机器人 - 模式选择 ***");
  Serial.println("========================================");
  Serial.println("请选择运行模式：");
  Serial.println("  按钮A - Line Sensor模式");
  Serial.println("  按钮B - Bump Sensor模式");
  Serial.println("等待按钮输入...");
  
  while (current_mode == MODE_STANDBY) {
    if (readButton(BUTTON_A)) {
      current_mode = MODE_LINE_SENSOR;
      Serial.println("\n✓ 已选择：Line Sensor模式");
    } else if (readButton(BUTTON_B)) {
      current_mode = MODE_BUMP_SENSOR;
      Serial.println("\n✓ 已选择：Bump Sensor模式");
    }
    delay(10);
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n========================================");
  Serial.println("*** Leader机器人 - 整合版本 ***");
  Serial.println("========================================");
  
  // 初始化按钮
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  
  // 初始化电机
  motors.initialise();
  
  // 初始化编码器
  setupEncoder0();
  setupEncoder1();
  
  // 初始化运动学
  kin.initialise(0, 0, 0);
  
  // 初始化Line Sensors（可选，根据模式使用）
  line_sensors.initialiseForADC();
  
  // 初始化速度PID控制器（增量式）
  speed_pid_left.initialise(KP_SPEED, KI_SPEED, KD_SPEED);
  speed_pid_right.initialise(KP_SPEED, KI_SPEED, KD_SPEED);
  speed_pid_left.setOutputLimits(-DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
  speed_pid_right.setOutputLimits(-DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
  speed_pid_left.setMaxDelta(5.0f);  // 限制输出变化率
  speed_pid_right.setMaxDelta(5.0f);
  
  // 初始化方向纠正PID（增量式）
  direction_pid.initialise(KP_DIR, KI_DIR, KD_DIR);
  direction_pid.setOutputLimits(-15.0f, 15.0f);  // 方向纠正范围
  
  Serial.println("✓ 系统初始化完成");
  Serial.println("✓ 使用增量式PID控制器");
  Serial.print("  - 速度PID: Kp="); Serial.print(KP_SPEED, 3);
  Serial.print(", Ki="); Serial.print(KI_SPEED, 4);
  Serial.print(", Kd="); Serial.println(KD_SPEED, 3);
  Serial.print("  - 方向PID: Kp="); Serial.print(KP_DIR, 3);
  Serial.print(", Ki="); Serial.print(KI_DIR, 4);
  Serial.print(", Kd="); Serial.println(KD_DIR, 3);
  
  // 等待模式选择
  waitForModeSelection();
  
  // 根据选择的模式配置IR发射
  if (current_mode == MODE_LINE_SENSOR) {
    // Line Sensor模式：开启IR发射（供Follower检测）
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, HIGH);  // HIGH = 持续发射IR信号
    Serial.println("✓ IR发射器：开启（Line Sensor模式 - OUTPUT + HIGH）");
    Serial.println("  Follower将能够检测到本机器人");
  } else if (current_mode == MODE_BUMP_SENSOR) {
    // Bump Sensor模式：开启IR发射（供Follower检测）
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, LOW);  // LOW = 发射IR（Bump模式）
    Serial.println("✓ IR发射器：开启（Bump Sensor模式 - OUTPUT + LOW）");
  }
  
  Serial.println("\n运动参数：");
  Serial.print("  - 目标速度: "); Serial.print(DEMAND_SPEED); Serial.println(" counts/s");
  Serial.print("  - PWM上限: ±"); Serial.println(DRIVE_PWM_LIMIT);
  Serial.print("  - 运动模式: 后退"); Serial.print(MOVE_DURATION/1000);
  Serial.print("秒 → 前进"); Serial.print(MOVE_DURATION/1000); Serial.println("秒");
  Serial.print("  - 回合数: "); Serial.println(NUM_ROUNDS);
  Serial.println("========================================");
  Serial.println("准备开始运动...\n");
  
  delay(1000);
  
  // 重置PID和状态
  speed_pid_left.reset();
  speed_pid_right.reset();
  direction_pid.reset();
  
  unsigned long now = millis();
  drive_est_ts = now;
  drive_pid_ts = now;
  d_last_e0 = count_e0;
  d_last_e1 = count_e1;
  spdL_cps = 0.0f;
  spdR_cps = 0.0f;
  spdL_filtered = 0.0f;
  spdR_filtered = 0.0f;
  
  // 数据记录功能已禁用以节省内存
  // results_index = 0;
  // results_interval_ms = (MOVE_DURATION * NUM_ROUNDS * 2) / MAX_RESULTS;
  // record_results_ts = millis();
  // recording_enabled = true;
  
  // Serial.println("数据记录已启用:");
  // Serial.print("  - 记录间隔: "); Serial.print(results_interval_ms); Serial.println(" ms");
  // Serial.print("  - 最大记录数: "); Serial.println(MAX_RESULTS);
  // Serial.println("  - 记录变量: 时间,X,Y,Theta,速度L,速度R,PWM");
  
  // 开始第一个回合
  motion_start_time = millis();
  Serial.print("回合 1/"); Serial.print(NUM_ROUNDS); Serial.println(" - 开始后退");
}

// ===== LOOP =====
void loop() {
  unsigned long current_time = millis();
  unsigned long elapsed = current_time - motion_start_time;
  
  // 更新速度估算
  estimateSpeed();
  
  // 更新运动学
  static unsigned long kin_ts = 0;
  if (current_time - kin_ts >= 10) {
    kin_ts = current_time;
    kin.update();
  }
  
  // ===== 数据记录已禁用 =====
  // if (recording_enabled && results_index < MAX_RESULTS) {
  //   if (current_time - record_results_ts >= results_interval_ms) {
  //     record_results_ts = current_time;
  //     
  //     results[results_index][0] = (float)(current_time - motion_start_time) / 1000.0f;
  //     results[results_index][1] = kin.x;
  //     results[results_index][2] = kin.y;
  //     results[results_index][3] = kin.theta;
  //     results[results_index][4] = spdL_cps;
  //     results[results_index][5] = spdR_cps;
  //     results[results_index][6] = last_pwm_L;
  //     
  //     results_index++;
  //   }
  // }
  
  // 根据状态设置目标速度
  float demand_L = 0.0f;
  float demand_R = 0.0f;
  
  switch (motion_state) {
    case MOVING_BACKWARD:
      // 后退：负速度
      demand_L = -DEMAND_SPEED;
      demand_R = -DEMAND_SPEED;
      
      if (elapsed >= MOVE_DURATION) {
        // 后退完成，切换到前进
        speed_pid_left.reset();
        speed_pid_right.reset();
        direction_pid.reset();
        
        motion_state = MOVING_FORWARD;
        motion_start_time = current_time;
        Serial.print("回合 "); Serial.print(current_round + 1);
        Serial.print("/"); Serial.print(NUM_ROUNDS); Serial.println(" - 开始前进");
      }
      break;
      
    case MOVING_FORWARD:
      // 前进：正速度
      demand_L = DEMAND_SPEED;
      demand_R = DEMAND_SPEED;
      
      if (elapsed >= MOVE_DURATION) {
        // 前进完成
        current_round++;
        
        if (current_round >= NUM_ROUNDS) {
          // 所有回合完成
          motion_state = COMPLETED;
          // recording_enabled = false;  // 数据记录已禁用
          speed_pid_left.reset();
          speed_pid_right.reset();
          direction_pid.reset();
          Serial.println("\n========================================");
          Serial.println("✓ 所有回合完成！Leader停止");
          Serial.println("========================================");
        } else {
          // 开始下一个回合
          speed_pid_left.reset();
          speed_pid_right.reset();
          direction_pid.reset();
          
          motion_state = MOVING_BACKWARD;
          motion_start_time = current_time;
          Serial.print("\n回合 "); Serial.print(current_round + 1);
          Serial.print("/"); Serial.print(NUM_ROUNDS); Serial.println(" - 开始后退");
        }
      }
      break;
      
    case COMPLETED:
      // 保持停止状态（已报告完数据）
      demand_L = 0.0f;
      demand_R = 0.0f;
      motors.setPWM(0, 0);
      
      // 每5秒输出一次状态
      static unsigned long status_ts = 0;
      if (current_time - status_ts >= 5000) {
        status_ts = current_time;
        Serial.println("Leader已完成所有运动 | IR发射器: ON");
      }
      return;  // 跳过PID控制
      
    // 数据报告功能已禁用
    // case REPORTING_RESULTS:
    //   motors.setPWM(0, 0);
    //   Serial.println("\n========================================");
    //   Serial.println("*** Leader数据记录报告 ***");
    //   Serial.println("========================================");
    //   motion_state = COMPLETED;
    //   return;
  }
  
  // 执行PID速度控制（带方向纠正）
  // 方向纠正：目标是左右轮速度相同，误差 = spdL - spdR
  float speed_diff = spdL_cps - spdR_cps;
  float direction_correction = direction_pid.update(0.0f, speed_diff);
  
  driveWithPID(demand_L, demand_R, direction_correction);
  
  // 调试输出（每300ms）
  static unsigned long debug_ts = 0;
  if (current_time - debug_ts >= 300) {
    debug_ts = current_time;
    Serial.print("状态: ");
    Serial.print(motion_state == MOVING_BACKWARD ? "后退" : "前进");
    Serial.print(" | 目标: ");
    Serial.print(demand_L, 1); Serial.print("/"); Serial.print(demand_R, 1);
    Serial.print(" | 实际: ");
    Serial.print(spdL_cps, 1); Serial.print("/"); Serial.print(spdR_cps, 1);
    Serial.print(" | 速度差: ");
    Serial.print(speed_diff, 1);
    Serial.print(" | 方向纠正: ");
    Serial.print(direction_correction, 1);
    Serial.print(" | 剩余: ");
    Serial.print((MOVE_DURATION - elapsed) / 1000.0f, 1);
    Serial.println("秒");
  }
  
  delay(10);
}

