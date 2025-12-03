# Leader 双IR时序系统说明

## 🎯 系统目的

让Leader能够同时支持：
1. **Line Sensor跟随**（Follower用5个line sensor检测方向）
2. **Bump Sensor跟随**（Follower用2个bump sensor检测距离）

通过**时分复用**技术，交替发射两种IR信号。

---

## 📊 工作原理

### 时序图

```
时间轴: →→→→→→→→→→→→→→→→→→→→→→→→→→→
        |←  50ms  →|← 10ms →|←  50ms  →|← 10ms →|

EMIT_PIN: HIGH──────LOW─────HIGH──────LOW─────HIGH
          
模式:     Line Sensor  Bump   Line Sensor  Bump   Line
          IR发射     IR发射   IR发射     IR发射   ...

Follower:
  Line     ✓检测到      X       ✓检测到      X      ✓
  Bump       X       ✓检测到     X       ✓检测到   X
```

### 关键参数

```cpp
#define LINE_IR_DURATION 50     // Line sensor持续时间 (ms)
#define BUMP_IR_DURATION 10     // Bump sensor持续时间 (ms)
#define IR_SETTLE_TIME 1        // 切换稳定时间 (ms)
```

**总周期**: 60ms  
**更新频率**: 16.7 Hz

---

## 🔧 EMIT_PIN控制逻辑

| EMIT_PIN状态 | 发射的IR | Follower使用传感器 | 用途 |
|-------------|---------|-------------------|------|
| **HIGH** | Line Sensor IR | 5个Line Sensors | 方向检测、车头对齐 |
| **LOW** | Bump Sensor IR | 2个Bump Sensors | 距离测量、保持间距 |

⚠️ **重要**：这是3Pi+的特殊设计
- HIGH ≠ "关闭"，而是开启Line Sensor的IR LED
- LOW ≠ "关闭"，而是开启Bump Sensor的IR LED
- 两种IR LED是**独立**的，不能同时开启

---

## 💡 时序占空比

### 当前配置（50ms + 10ms）

- **Line Sensor IR占空比**: 50/60 = **83.3%**
- **Bump Sensor IR占空比**: 10/60 = **16.7%**

**设计理由**：
1. Line sensor需要更长时间用于方向检测（5个传感器，数据处理复杂）
2. Bump sensor读取快速（elapsed time方法，10ms足够）
3. 方向控制比距离控制更重要（优先保证不跑偏）

### 如果需要调整

```cpp
// 场景1：更重视距离控制
#define LINE_IR_DURATION 40     // 减少Line时间
#define BUMP_IR_DURATION 20     // 增加Bump时间
// 结果：Line=66.7%, Bump=33.3%

// 场景2：完全平衡
#define LINE_IR_DURATION 30
#define BUMP_IR_DURATION 30
// 结果：Line=50%, Bump=50%

// 场景3：只用Bump（高精度距离）
#define LINE_IR_DURATION 10
#define BUMP_IR_DURATION 50
// 结果：Line=16.7%, Bump=83.3%
```

---

## 🚀 使用方法

### 1. 上传Leader代码

直接上传 `version2/leader/leader.ino` 到Leader机器人。

### 2. 查看串口输出

```
========================================
*** Leader机器人 - 双IR时序系统 ***
模式：交替发射Line Sensor和Bump Sensor IR
========================================
✓ 双IR时序系统已初始化
IR发射时序：
  - Line Sensor IR (HIGH): 50 ms
  - Bump Sensor IR (LOW):  10 ms
  - 切换周期: 60 ms
  - 更新频率: 16.7 Hz

📡 Follower可以使用：
  ✓ Line Sensor (5个) - 检测方向，车头对齐
  ✓ Bump Sensor (2个) - 检测距离，保持间距
========================================
IR模式: LINE_SENSOR (HIGH) | 占空比: Line=83.3% Bump=16.7%
```

### 3. 配置Follower

Follower需要相应修改：

#### 选项A：只用Line Sensor（当前方案）
```cpp
// 你的follower.ino已经实现了
// 使用5个line sensor做方向和距离控制
// 不需要修改
```

#### 选项B：同时用Line + Bump
```cpp
// 需要添加：
// 1. Bump sensor读取代码
// 2. 时序同步逻辑
// 3. 传感器融合算法
```

---

## 📈 性能分析

### 优势

✅ **兼容性强**：支持Line和Bump两种跟随方式  
✅ **灵活切换**：可以随时调整时序参数  
✅ **非阻塞**：不影响Leader的运动控制  
✅ **低延迟**：16.7Hz更新率足够实时控制  

### 考虑因素

⚠️ **Follower需要同步**：
- Follower的sensor读取要在对应的IR发射周期内
- 如果Follower读取太慢，可能读到错误的IR信号

⚠️ **时序抖动**：
- Arduino的`millis()`精度为1ms
- 实际切换时间可能有±1-2ms误差
- 对于50ms/10ms周期，误差可接受

---

## 🔍 调试技巧

### 1. 观察IR切换

串口输出会显示当前模式：
```
IR模式: LINE_SENSOR (HIGH) | 占空比: Line=83.3% Bump=16.7%
IR模式: BUMP_SENSOR (LOW)  | 占空比: Line=83.3% Bump=16.7%
```

如果输出卡在一个模式，说明时序系统有问题。

### 2. 用示波器测量

- 在EMIT_PIN（Pin 11）上接示波器
- 应该看到周期性的高/低电平切换
- HIGH持续50ms，LOW持续10ms

### 3. Follower测试

**测试Line Sensor**：
- Follower应该能检测到方向
- 大部分时间（83%）能读到Line IR

**测试Bump Sensor**：
- Follower用bump sensor时，只有16.7%时间有信号
- 需要多次采样平均

---

## 🎛️ 高级配置

### 自适应时序（未来改进）

可以根据Follower的需求动态调整：

```cpp
// 伪代码
if (follower_distance_error > threshold) {
  // 距离误差大，增加Bump时间
  LINE_IR_DURATION = 30;
  BUMP_IR_DURATION = 30;
} else {
  // 距离稳定，增加Line时间用于精确对齐
  LINE_IR_DURATION = 50;
  BUMP_IR_DURATION = 10;
}
```

### 三模式时序（支持多个Follower）

```cpp
enum IR_MODE {
  IR_LINE_SENSOR,    // 40ms
  IR_BUMP_SENSOR,    // 10ms
  IR_BROADCAST       // 10ms - 发送ID信息
};
```

---

## ⚙️ 代码结构

### 核心函数

```cpp
void manageIRSwitching() {
  // 1. 检查当前模式持续时间
  // 2. 达到时间后切换模式
  // 3. 设置EMIT_PIN电平
  // 4. 记录切换时间
}
```

### 在loop()中调用

```cpp
void loop() {
  manageIRSwitching();  // 优先级最高
  // ... 其他控制代码
}
```

**重要**：时序管理放在loop()最前面，确保及时切换。

---

## 🐛 常见问题

### Q1: Follower检测不到信号
**检查**：
- Leader的IR时序是否正常切换？（看串口输出）
- Follower的sensor读取时机是否正确？
- EMIT_PIN连接是否正常？

### Q2: Follower方向控制不稳定
**原因**：Line IR占空比太低  
**解决**：增加`LINE_IR_DURATION`到70-80ms

### Q3: Follower距离控制不稳定
**原因**：Bump IR占空比太低  
**解决**：增加`BUMP_IR_DURATION`到20-30ms

### Q4: 为什么不让两种IR同时开启？
**答**：3Pi+硬件限制，两种IR LED共用控制，不能同时开启。

---

## 📚 参考资料

- **3Pi+ 文档**：[Pololu 3Pi+ IR系统](https://www.pololu.com/docs/0J83)
- **时分复用原理**：类似于通信中的TDM（Time Division Multiplexing）
- **你同学的实现**：`Leader_Bump.ino`（参考了他的时序设计）

---

## ✅ 测试检查清单

使用这个清单确保系统正常：

- [ ] Leader上传代码成功
- [ ] 串口输出显示时序系统初始化
- [ ] 观察到"IR模式"在LINE_SENSOR和BUMP_SENSOR之间切换
- [ ] 占空比显示正确（Line=83.3%, Bump=16.7%）
- [ ] Follower能检测到Line Sensor信号（方向控制工作）
- [ ] （可选）Follower能检测到Bump Sensor信号（距离控制工作）

---

## 🎉 总结

这个双IR时序系统让你的Leader变得更强大：

1. **兼容两种跟随方式**：Line sensor和Bump sensor
2. **简单高效**：只需60行代码
3. **灵活可调**：参数容易修改
4. **性能稳定**：16.7Hz更新率足够

你现在可以：
- 保持使用Line sensor跟随（当前follower.ino）
- 未来升级到Bump sensor跟随
- 或者结合两者，实现最优跟随效果

祝测试顺利！🤖✨


