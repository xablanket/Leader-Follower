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

// ========== 显示模式 ==========
// 改成 true 显示详细信息，false 显示简洁信息
const bool DETAILED_MODE = false;

// ========== 诊断模式 ==========
// 改成 true 显示原始数值，帮助调整阈值
const bool DIAGNOSTIC_MODE = true;

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
  Serial.print("  - 显示模式: ");
  if (DIAGNOSTIC_MODE) {
    Serial.println("🔍 诊断模式（调试用）");
    Serial.println("    → 显示原始数值和阈值");
    Serial.println("    → 用于调整检测阈值");
  } else if (DETAILED_MODE) {
    Serial.println("详细模式");
  } else {
    Serial.println("简洁模式（推荐）");
  }
  Serial.println("    → 修改代码第19-20行切换模式");
  
  // 校准背景
  calibrateBackground();
  
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("准备开始检测！");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("\n请启动Leader，确保时序系统运行...");
  
  if (DIAGNOSTIC_MODE) {
    Serial.println("\n🔍 诊断模式说明：");
    Serial.println("格式：[时间] Line:值(阈值>X) | Bump:值(阈值>Y) → 状态");
    Serial.println();
    Serial.println("如何调整检测距离和阈值：");
    Serial.println("1. 观察Line和Bump的实际数值范围");
    Serial.println("2. 检测距离太短（经常NONE）：");
    Serial.println("   → 降低阈值：Line改成15-20，Bump改成60-80");
    Serial.println("3. 误检测太多（一直LINE或BOTH）：");
    Serial.println("   → 提高阈值：Line改成50-70，Bump改成150-200");
    Serial.println("4. 调整位置：第182行(Line)和第183行(Bump)");
    Serial.println("5. 正常应该看到：LINE和BUMP交替出现");
    Serial.println();
  } else if (!DETAILED_MODE) {
    Serial.println("\n输出格式说明（简洁模式）：");
    Serial.println("[时间] 图标 类型 | Line值 | Bump值 | 状态");
    Serial.println();
    Serial.println("图标：🟢=Line IR  🔵=Bump IR  ⚪=无信号");
    Serial.println("类型：LINE=Line发射  BUMP=Bump发射");
    Serial.println("状态：L✓=Line有信号  B✓=Bump有信号");
    Serial.println();
    Serial.println("正常时序应看到 🟢 和 🔵 交替出现");
  }
  Serial.println();
  
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
  
  // ========== 检测阈值（控制检测距离）==========
  // ⚠️ 阈值越低 = 检测距离越远（但容易误检测）
  // ⚠️ 阈值越高 = 检测距离越近（但更准确）
  // 
  // 调整建议：
  // - 检测距离太短 → 降低阈值（比如Line改成20，Bump改成80）
  // - 误检测太多（一直显示LINE/BOTH） → 提高阈值（比如Line改成70，Bump改成200）
  
  bool line_active = line_avg > 25.0f;      // Line传感器阈值（降低=检测距离更远）
  bool bump_active = bump_avg > 100.0f;     // Bump传感器阈值（降低=检测距离更远）
  
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
  if (DIAGNOSTIC_MODE) {
    // ===== 诊断模式 - 显示原始数值和阈值 =====
    unsigned long time_s = millis() / 1000;
    
    Serial.print("[");
    if (time_s < 10) Serial.print("0");
    Serial.print(time_s);
    Serial.print("s] ");
    
    // 显示Line原始数据
    Serial.print("Line:");
    Serial.print(line_avg, 1);
    Serial.print("(阈值>");
    Serial.print(25.0, 0);  // Line阈值（与第182行保持一致）
    Serial.print(")");
    if (line_active) Serial.print("✓");
    else Serial.print("-");
    
    Serial.print(" | Bump:");
    Serial.print(bump_avg, 1);
    Serial.print("(阈值>");
    Serial.print(100.0, 0);  // Bump阈值（与第183行保持一致）
    Serial.print(")");
    if (bump_active) Serial.print("✓");
    else Serial.print("-");
    
    Serial.print(" → ");
    Serial.print(status_icon);
    Serial.print(" ");
    if (line_active && !bump_active) Serial.print("LINE");
    else if (!line_active && bump_active) Serial.print("BUMP");
    else if (line_active && bump_active) Serial.print("BOTH");
    else Serial.print("NONE");
    
    // 显示5个Line传感器的详细值
    Serial.print(" [");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print((int)line_signals[i]);
      if (i < NUM_SENSORS - 1) Serial.print(",");
    }
    Serial.print("]");
    
    Serial.println();
    
  } else if (DETAILED_MODE) {
    // ===== 详细模式 =====
    Serial.println("┌───────────────────────────────────────────────┐");
    Serial.print("│ "); Serial.print(status_icon); 
    Serial.print(" "); Serial.print(ir_type);
    Serial.println();
    Serial.println("├───────────────────────────────────────────────┤");
    
    // Line传感器
    Serial.print("│ Line: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print((int)line_signals[i]);
      if (i < NUM_SENSORS - 1) Serial.print("|");
    }
    Serial.print(" avg:"); Serial.print(line_avg, 1);
    if (line_active) Serial.print(" ✓");
    Serial.println();
    
    // Bump传感器
    Serial.print("│ Bump: L="); Serial.print(bump_L);
    Serial.print(" R="); Serial.print(bump_R);
    Serial.print(" avg:"); Serial.print(bump_avg, 1);
    if (bump_active) Serial.print(" ✓");
    Serial.println();
    
    Serial.println("└───────────────────────────────────────────────┘");
    Serial.println();
    
  } else {
    // ===== 简洁模式 =====
    // 格式：[时间] 图标 类型 | Line值 | Bump值 | 状态
    unsigned long time_s = millis() / 1000;
    
    Serial.print("["); 
    if (time_s < 10) Serial.print("0");
    Serial.print(time_s); 
    Serial.print("s] ");
    
    Serial.print(status_icon);
    Serial.print(" ");
    
    // 简化IR类型显示
    if (line_active && !bump_active) {
      Serial.print("LINE ");
    } else if (!line_active && bump_active) {
      Serial.print("BUMP ");
    } else if (line_active && bump_active) {
      Serial.print("BOTH ");
    } else {
      Serial.print("NONE ");
    }
    
    Serial.print("| Line:");
    if (line_avg < 10) Serial.print(" ");
    Serial.print(line_avg, 1);
    
    Serial.print(" | Bump:");
    if (bump_avg < 10) Serial.print("  ");
    else if (bump_avg < 100) Serial.print(" ");
    Serial.print(bump_avg, 1);
    
    Serial.print(" | ");
    Serial.print(line_active ? "L✓" : "L-");
    Serial.print(" ");
    Serial.print(bump_active ? "B✓" : "B-");
    
    Serial.println();
  }
  
  delay(100);  // 每100ms更新一次
}

