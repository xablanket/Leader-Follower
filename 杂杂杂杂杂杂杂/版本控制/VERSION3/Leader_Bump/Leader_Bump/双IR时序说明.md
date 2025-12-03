# Leader 双IR传感器时序控制系统

## 🎯 问题与解决方案

### 问题
Leader有两种IR传感器，但它们**共用一个EMIT_PIN (Pin 11)**：
- **Line Sensor IR**：`EMIT_PIN = HIGH` 时发射
- **Bump Sensor IR**：`EMIT_PIN = LOW` 时发射

**无法同时工作！**

### 解决方案
使用**时间片轮转（Time-Division Multiplexing, TDM）**：
```
周期1 (80ms): HIGH → Line Sensor IR工作 → Follower用Line Sensor检测
周期2 (20ms): LOW  → Bump Sensor IR工作 → 读取Bump数据
重复...
```

总周期 = 100ms，Line Sensor占用80%时间（足够Follower检测）

---

## 📊 工作时序图

```
时间线: 0----80ms----100ms----180ms----200ms---->

EMIT_PIN: [   HIGH   ][LOW][   HIGH   ][LOW]...
          └─ Line IR ─┘└─┘ └─ Line IR ─┘└─┘
                      Bump              Bump

Follower: [  可以检测  ][×][  可以检测  ][×]...
          └─ 80ms足够跟随控制 ─┘

Bump读取:              [√]              [√]
                     20ms               20ms
```

**关键点：**
- Line IR占80%时间 → Follower有足够时间检测和跟随
- Bump IR占20%时间 → 足够读取bump传感器数据
- 每100ms完整循环一次

---

## 🔧 代码实现

### 1. 时序参数定义
```cpp
#define LINE_IR_DURATION 80     // Line sensor IR开启时间 (ms)
#define BUMP_IR_DURATION 20     // Bump sensor IR开启时间 (ms)
#define IR_SETTLE_TIME 2        // IR切换后稳定时间 (ms)
```

**可调整：**
- 如果Follower跟随不稳定 → 增大 `LINE_IR_DURATION` (比如90ms)
- 如果Bump数据不够 → 增大 `BUMP_IR_DURATION` (比如30ms)
- 总周期建议 ≤ 150ms，保持响应速度

### 2. IR模式枚举
```cpp
enum IR_MODE {
  IR_LINE_SENSOR,    // Line sensor IR发射
  IR_BUMP_SENSOR     // Bump sensor IR发射
};
```

### 3. 时序管理函数
```cpp
void manageIRSwitching() {
  // 检查当前模式的持续时间
  // 如果达到切换时间：
  //   1. 切换EMIT_PIN电平
  //   2. 等待IR稳定 (2ms)
  //   3. 如果切换到Bump模式，立即读取数据
  //   4. 更新时间戳
}
```

### 4. Loop中调用
```cpp
void loop() {
  manageIRSwitching();  // 每次循环都检查是否需要切换
  
  // 你的其他代码...
  // 运动控制、PID等
}
```

---

## 📈 时序性能分析

### Follower检测能力
```
Line IR工作时间: 80ms / 100ms = 80%
Follower控制周期: 通常 50ms
```
- ✅ 80ms >> 50ms，足够进行多次检测和控制
- ✅ 20ms的中断不会影响跟随稳定性（PID有惯性）

### Bump数据更新率
```
更新频率: 1000ms / 100ms = 10 Hz
每秒更新: 10次
```
- ✅ 10Hz足够用于障碍检测
- ✅ 如果需要更高频率，减少`LINE_IR_DURATION`

---

## 🔍 调试输出

启动时会显示：
```
=== Leader with Dual IR System ===
✓ 系统初始化完成
✓ IR时序切换已启用：
  - Line Sensor IR: 80 ms
  - Bump Sensor IR: 20 ms
  - 切换周期: 100 ms
========================================
```

运行时每500ms输出一次：
```
IR模式: LINE_SENSOR | Bump: L=1250 R=1320 AVG=1285
IR模式: BUMP_SENSOR | Bump: L=1248 R=1318 AVG=1283
...
```

**读数含义：**
- `L` / `R`：左右bump传感器的放电时间（微秒）
- `AVG`：平均值
- 数值越大 = 距离越远
- 数值越小 = 距离越近（或有障碍物）

---

## ⚙️ 使用建议

### 场景1：主要用于Follower跟随
**推荐配置：**
```cpp
#define LINE_IR_DURATION 90     // 增加到90ms
#define BUMP_IR_DURATION 10     // 减少到10ms
```
- Follower有更多时间检测
- Bump只做基本的障碍检测

### 场景2：需要精确的Bump检测
**推荐配置：**
```cpp
#define LINE_IR_DURATION 70     // 减少到70ms
#define BUMP_IR_DURATION 30     // 增加到30ms
```
- Bump数据更新更频繁
- Follower仍然有足够时间跟随

### 场景3：平衡配置（默认）
**当前配置：**
```cpp
#define LINE_IR_DURATION 80     // 80ms
#define BUMP_IR_DURATION 20     // 20ms
```
- 两者都有合理的工作时间

---

## 🎮 集成到运动控制

### 方法A：只发射IR，不主动使用Bump
```cpp
void loop() {
  manageIRSwitching();  // 只负责切换IR
  
  // 正常的运动控制
  // Follower会检测到Line IR信号
  // Bump数据被读取但不用于控制
}
```
**用途：**让Follower能检测到Leader，Bump数据作为监控

### 方法B：使用Bump数据进行避障
```cpp
void loop() {
  manageIRSwitching();
  
  // 检查bump数据
  if (bump_avg < 500) {
    // 检测到前方障碍，停止
    motors.setPWM(0, 0);
    return;
  }
  
  // 正常运动控制
  // ...
}
```
**用途：**Leader自己用Bump做避障

### 方法C：数据记录
```cpp
void loop() {
  manageIRSwitching();
  
  // 记录bump数据到SD卡或串口
  // 用于后期分析
  
  // 正常运动控制
  // ...
}
```

---

## 🚨 注意事项

### 1. 时序冲突
⚠️ **不要在loop中再次操作EMIT_PIN！**
```cpp
// ❌ 错误：会破坏时序
digitalWrite(EMIT_PIN, HIGH);

// ✅ 正确：让manageIRSwitching()自动管理
```

### 2. 延迟操作
⚠️ **避免在loop中使用大延迟**
```cpp
// ❌ 错误：会阻塞时序切换
delay(500);

// ✅ 正确：使用非阻塞定时
static unsigned long ts = 0;
if (millis() - ts >= 500) {
  ts = millis();
  // 做事情
}
```

### 3. 读取Line Sensor
⚠️ **只在Line IR模式下读取Line Sensor**
```cpp
if (current_ir_mode == IR_LINE_SENSOR) {
  line_sensors.readSensorsADC();
  // 处理数据
}
```

### 4. IR稳定时间
当前设置2ms的`IR_SETTLE_TIME`，如果数据不稳定可以增加：
```cpp
#define IR_SETTLE_TIME 5  // 增加到5ms
```

---

## 📝 技术细节

### 为什么Line IR需要更长时间？
1. **Follower需要连续检测**：PID控制通常50ms一次，需要持续的IR信号
2. **Line Sensor ADC读取**：需要时间充电/放电
3. **5个传感器**：需要逐个读取

### 为什么Bump IR时间短也够用？
1. **只读取2个传感器**：快速完成
2. **数据变化慢**：障碍物位置不会快速改变
3. **10Hz更新率**：对障碍检测足够

### 如何验证时序正确？
1. 观察串口输出，应该看到模式切换
2. Follower应该能正常跟随（说明Line IR在工作）
3. Bump数据应该有合理的读数（说明Bump IR在工作）

---

## 🎯 测试步骤

### 第1步：基础测试
1. 上传代码到Leader
2. 观察串口输出
3. 确认IR模式正常切换

### 第2步：Follower测试
1. 启动Follower
2. 确认Follower能检测到Leader
3. 测试跟随功能

### 第3步：Bump测试
1. 观察Bump数据
2. 手动接近Leader前方
3. 确认Bump读数变化（数值应该减小）

### 第4步：联合测试
1. Follower跟随Leader运动
2. 同时监控Bump数据
3. 确认两者都正常工作

---

## 💡 进阶优化

### 优化1：自适应时序
根据Follower距离动态调整时序：
```cpp
if (bump_avg < 1000) {
  // Follower很近，减少Bump检测
  LINE_IR_DURATION = 90;
  BUMP_IR_DURATION = 10;
} else {
  // 正常距离
  LINE_IR_DURATION = 80;
  BUMP_IR_DURATION = 20;
}
```

### 优化2：智能模式
只在需要时切换到Bump模式：
```cpp
if (need_bump_detection) {
  // 启用Bump检测
} else {
  // 一直保持Line IR模式
  digitalWrite(EMIT_PIN, HIGH);
}
```

### 优化3：数据缓冲
保存多个Bump读数，计算平均值：
```cpp
float bump_buffer[5];
int bump_idx = 0;

// 在读取Bump时
bump_buffer[bump_idx] = bump_avg;
bump_idx = (bump_idx + 1) % 5;

// 使用平均值
float avg = 0;
for (int i = 0; i < 5; i++) avg += bump_buffer[i];
avg /= 5.0;
```

---

## 🎉 总结

✅ **实现了双IR系统共存**
- Line IR: 80ms (80%)
- Bump IR: 20ms (20%)

✅ **不影响Follower跟随**
- 80ms足够连续检测
- PID控制有惯性

✅ **Bump数据可用**
- 10Hz更新率
- 可用于避障或监控

✅ **灵活可调**
- 时序参数可调整
- 支持多种使用场景

现在你的Leader可以**同时为Follower提供IR信号**，又能**自己使用Bump传感器**了！🚀


