/*
 * ====================================================================
 * Follower IR检测测试程序
 * ====================================================================
 * 
 * 用途：检测Leader发射的IR类型（Line IR 还是 Bump IR）
 * 
 * 使用方法：
 * 1. 上传此程序到Follower
 * 2. 启动Leader（确保时序系统运行）
 * 3. 打开串口监视器（9600波特率）
 * 4. 观察输出，验证Leader的IR时序是否正常
 * 
 * ====================================================================
 */

#include "LineSensors.h"

// Bump传感器引脚
#define BUMP_L 4
#define BUMP_R 5
#define EMIT_PIN 11

LineSensors_c line_sensors;

// 背景值
float line_background[NUM_SENSORS] = {0};
unsigned long bump_background_L = 0;
unsigned long bump_background_R = 0;

// ========== Bump传感器读取（elapsed time方法）==========
unsigned long readBump(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);      // 充电
  delayMicroseconds(10);
  pinMode(pin, INPUT);          // 开始放电
  
  unsigned long t0 = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t0 > 4500) break;  // 超时保护
  }
  return micros() - t0;  // 返回放电时间
}

// ========== 校准背景值 ==========
void calibrateBackground() {
  Serial.println("\n开始校准背景值...");
  Serial.println("请确保Leader尚未启动或距离很远");
  delay(3000);
  
  // 校准Line传感器
  for (int sample = 0; sample < 10; sample++) {
    line_sensors.readSensorsADC();
    for (int i = 0; i < NUM_SENSORS; i++) {
      line_background[i] += line_sensors.readings[i];
    }
    delay(50);
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    line_background[i] /= 10.0f;
  }
  
  // 校准Bump传感器
  for (int sample = 0; sample < 10; sample++) {
    bump_background_L += readBump(BUMP_L);
    bump_background_R += readBump(BUMP_R);
    delay(50);
  }
  bump_background_L /= 10;
  bump_background_R /= 10;
  
  Serial.println("✓ 背景校准完成！");
  Serial.println("\n━━━ Line Sensor背景值 ━━━");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("  S"); Serial.print(i); Serial.print(": ");
    Serial.println(line_background[i], 1);
  }
  Serial.println("\n━━━ Bump Sensor背景值 ━━━");
  Serial.print("  左Bump: "); Serial.println(bump_background_L);
  Serial.print("  右Bump: "); Serial.println(bump_background_R);
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("\n\n╔═══════════════════════════════════════════════╗");
  Serial.println("║   Follower IR检测测试程序                     ║");
  Serial.println("║   实时监测Leader发射的IR类型                  ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  
  // 初始化Line传感器
  line_sensors.initialiseForADC();
  pinMode(EMIT_PIN, INPUT);  // 关闭自己的IR发射
  
  // 初始化Bump传感器
  pinMode(BUMP_L, INPUT);
  pinMode(BUMP_R, INPUT);
  
  Serial.println("\n✓ 传感器已初始化");
  Serial.println("  - Line Sensors: 5个（下向）");
  Serial.println("  - Bump Sensors: 2个（前向）");
  Serial.println("  - Follower IR: 关闭（INPUT模式）");
  
  // 校准背景
  calibrateBackground();
  
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("准备开始检测！");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("\n请启动Leader，确保时序系统运行...\n");
  
  delay(2000);
}

void loop() {
  // ========== 读取Line传感器 ==========
  line_sensors.readSensorsADC();
  
  float line_signals[NUM_SENSORS];
  float line_total = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    line_signals[i] = fabs(line_sensors.readings[i] - line_background[i]);
    line_total += line_signals[i];
  }
  float line_avg = line_total / NUM_SENSORS;
  
  // ========== 读取Bump传感器 ==========
  unsigned long bump_L = readBump(BUMP_L);
  unsigned long bump_R = readBump(BUMP_R);
  
  long bump_signal_L = (long)bump_L - (long)bump_background_L;
  long bump_signal_R = (long)bump_R - (long)bump_background_R;
  float bump_avg = (abs(bump_signal_L) + abs(bump_signal_R)) / 2.0f;
  
  // ========== 判断当前IR类型 ==========
  String ir_type = "未知";
  String status_icon = "⚪";
  
  // Line IR强度通常会显著增加，Bump IR时间会显著减少
  bool line_active = line_avg > 30.0f;      // Line传感器有显著信号
  bool bump_active = bump_avg > 200.0f;     // Bump传感器时间变化显著
  
  if (line_active && !bump_active) {
    ir_type = "LINE_SENSOR (HIGH)";
    status_icon = "🟢";
  } else if (!line_active && bump_active) {
    ir_type = "BUMP_SENSOR (LOW) ";
    status_icon = "🔵";
  } else if (line_active && bump_active) {
    ir_type = "两种都检测到(?)  ";
    status_icon = "🟡";
  } else {
    ir_type = "无信号/太远      ";
    status_icon = "⚪";
  }
  
  // ========== 输出检测结果 ==========
  Serial.println("┌────────────────────────────────────────────────────────┐");
  
  // 当前检测到的IR类型
  Serial.print("│ "); Serial.print(status_icon); 
  Serial.print(" 当前IR: "); Serial.print(ir_type);
  Serial.println();
  
  Serial.println("├────────────────────────────────────────────────────────┤");
  
  // Line传感器详细信息
  Serial.println("│ 📊 Line Sensors (下向，用于方向)");
  Serial.print("│    原始: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print((int)line_sensors.readings[i]);
    if (i < NUM_SENSORS - 1) Serial.print(" | ");
  }
  Serial.println();
  
  Serial.print("│    增强: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print((int)line_signals[i]);
    if (i < NUM_SENSORS - 1) Serial.print(" | ");
  }
  Serial.println();
  
  Serial.print("│    平均增强: "); Serial.print(line_avg, 1);
  if (line_active) {
    Serial.print(" ✓ [有信号]");
  } else {
    Serial.print(" - [无信号]");
  }
  Serial.println();
  
  Serial.println("├────────────────────────────────────────────────────────┤");
  
  // Bump传感器详细信息
  Serial.println("│ 📊 Bump Sensors (前向，用于距离)");
  Serial.print("│    左Bump: "); Serial.print(bump_L);
  Serial.print(" (变化: "); Serial.print(bump_signal_L); Serial.print(")");
  Serial.println();
  
  Serial.print("│    右Bump: "); Serial.print(bump_R);
  Serial.print(" (变化: "); Serial.print(bump_signal_R); Serial.print(")");
  Serial.println();
  
  Serial.print("│    平均变化: "); Serial.print(bump_avg, 1);
  if (bump_active) {
    Serial.print(" ✓ [有信号]");
  } else {
    Serial.print(" - [无信号]");
  }
  Serial.println();
  
  Serial.println("└────────────────────────────────────────────────────────┘");
  Serial.println();
  
  delay(100);  // 每100ms更新一次
}

/*
 * ====================================================================
 * 输出说明
 * ====================================================================
 * 
 * 🟢 LINE_SENSOR (HIGH)：
 *    - Line传感器有强信号（平均增强 > 30）
 *    - Bump传感器无显著变化
 *    → Leader正在发射Line IR
 * 
 * 🔵 BUMP_SENSOR (LOW)：
 *    - Bump传感器时间变化大（平均变化 > 200）
 *    - Line传感器无显著信号
 *    → Leader正在发射Bump IR
 * 
 * 🟡 两种都检测到：
 *    - 可能在切换瞬间
 *    - 或者阈值设置需要调整
 * 
 * ⚪ 无信号/太远：
 *    - Leader未启动
 *    - 或者距离太远
 *    - 或者时序系统未运行
 * 
 * ====================================================================
 * 
 * 如何验证时序正常？
 * 
 * 1. 应该看到 🟢 和 🔵 交替出现
 * 2. 🟢 出现频率应该更高（因为Line占比更大）
 * 3. 如果一直是 🟢 或 🔵，说明Leader时序卡住了
 * 4. 如果一直是 ⚪，说明Leader未发射IR或距离太远
 * 
 * ====================================================================
 */

