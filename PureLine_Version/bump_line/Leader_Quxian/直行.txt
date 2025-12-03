#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"

#define EMIT_PIN    11
#define BTN_PIN     14
#define BUZZER_PIN  6

// ===== Objects =====
Motors_c      motors;
Kinematics_c  kin;
PID_c         left_pid;
PID_c         right_pid;

// ===== IR timing (Leader 200 ms backbone) =====
unsigned long ir_ts        = 0;
bool          ir_bump_mode = true;

// ===== Beep (non-blocking) =====
unsigned long beep_off_time = 0;

void softBeep(int duration_ms) {
  tone(BUZZER_PIN, 800);
  beep_off_time = millis() + duration_ms;
}

// ===== Straight-path control constants =====
#define DRIVE_EST_MS    20UL
#define DRIVE_PID_MS    40UL
#define DRIVE_PWM_LIMIT 60

const float DEMAND_CS = -260.0f;   // 前进方向的目标 counts/s（你的反向逻辑）
const int   kF_L      = 16;
const int   kF_R      = 15;

float KP_L = 0.04020f, KI_L = 0.00000f, KD_L = 0.0f;  // KP_L 实际对应右轮
float KP_R = 0.07000f, KI_R = 0.00000f, KD_R = 0.0f;  // KP_R 实际对应左轮

// ===== Runtime vars for straight control =====
unsigned long drive_est_ts = 0, drive_pid_ts = 0;
long          d_last_e0    = 0, d_last_e1    = 0;
float         spdL_cps     = 0.0f, spdR_cps   = 0.0f;
float         d_mL1        = 0.0f, d_mL2      = 0.0f;
float         d_mR1        = 0.0f, d_mR2      = 0.0f;

float         x0 = 0.0f, y0 = 0.0f;  // 起点记录
bool          straight_done = false; // 走完 450 mm 后只停轮，不停 IR 时序

// ===== helpers =====
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int iround(float v) {
  return (int)lroundf(v);
}

void setup() {
  Serial.begin(115200);

  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  // 初始位姿：朝 +X 方向
  kin.initialise(0.0f, 0.0f, 0.0f);

  // IR 发射引脚
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);

  // 按钮与蜂鸣器
  pinMode(BTN_PIN,    INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  // 上电提示音
  softBeep(40);

  // 等待按钮按下作为“系统起点”，方便与 Follower 第三次按钮对齐
  while (digitalRead(BTN_PIN) == HIGH) {
    unsigned long now = millis();
    if (now >= beep_off_time && beep_off_time != 0) {
      noTone(BUZZER_PIN);
      beep_off_time = 0;
    }
  }

  // 按下确认音
  softBeep(40);

  // 现在才正式启动 200ms IR 时序与直行控制时钟
  unsigned long now = millis();
  ir_ts        = now;
  drive_est_ts = now;
  drive_pid_ts = now;

  // 记录直行起点
  x0 = kin.x;
  y0 = kin.y;

  // PID 初始化
  left_pid.initialise(KP_L, KI_L, KD_L);
  right_pid.initialise(KP_R, KI_R, KD_R);
  left_pid.reset();
  right_pid.reset();
}

void loop() {
  unsigned long now = millis();

  // ===== 非阻塞蜂鸣器关断 =====
  if (now >= beep_off_time && beep_off_time != 0) {
    noTone(BUZZER_PIN);
    beep_off_time = 0;
  }

  // ===== 200 ms IR 时序：0–100ms bump, 100–200ms line =====
  if (now - ir_ts >= 100UL) {
    ir_ts = now;
    ir_bump_mode = !ir_bump_mode;

    if (ir_bump_mode) {
      // bump window：关闭发射，由 Follower 读取 bumpIR
      digitalWrite(EMIT_PIN, LOW);
    } else {
      // line window：打开发射，由 Follower 读取 lineIR
      digitalWrite(EMIT_PIN, HIGH);
    }
  }

  // ===== 20 ms 速度估计 + 里程计 =====
  if (now - drive_est_ts >= DRIVE_EST_MS) {
    unsigned long dt = now - drive_est_ts;
    drive_est_ts = now;

    long e0 = count_e0;
    long e1 = count_e1;
    long d0 = e0 - d_last_e0;
    long d1 = e1 - d_last_e1;
    d_last_e0 = e0;
    d_last_e1 = e1;

    float dt_sec = (float)dt / 1000.0f;

    // 注意你当前符号约定：e0→右轮, e1→左轮
    spdR_cps = (float)d0 / dt_sec;
    spdL_cps = (float)d1 / dt_sec;

    // 更新里程计
    kin.update();
  }

  // ===== 路径：直行 450 mm 后停车（但继续发 IR） =====
  if (!straight_done) {
    float dx   = kin.x - x0;
    float dy   = kin.y - y0;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist >= 450.0f) {
      // 路径结束：停轮子但保持 IR 继续闪烁
      motors.setPWM(0, 0);
      straight_done = true;
    }
  }

  // ===== 40 ms PID 内环：只有在尚未走完路径时才更新 =====
  if (!straight_done && (now - drive_pid_ts >= DRIVE_PID_MS)) {
    drive_pid_ts = now;

    float demand = DEMAND_CS;  // -300 cps，当前系统的前进方向

    // 三点均值滤波
    float measL = (spdL_cps + d_mL1 + d_mL2) / 3.0f;
    float measR = (spdR_cps + d_mR1 + d_mR2) / 3.0f;

    d_mL2 = d_mL1;
    d_mL1 = spdL_cps;
    d_mR2 = d_mR1;
    d_mR1 = spdR_cps;

    // PID 更新
    float uL = left_pid.update(demand, measL);
    float uR = right_pid.update(demand, measR);

    // 小范围限幅，防止一下子推爆 PWM
    float uLc = clampf(uL, -12.0f, 12.0f);
    float uRc = clampf(uR, -12.0f, 12.0f);

    float sgn = (demand >= 0.0f) ? 1.0f : -1.0f;

    float pwmL = clampf(sgn * kF_L + uLc, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
    float pwmR = clampf(sgn * kF_R + uRc, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);

    motors.setPWM(iround(pwmL), iround(pwmR));
  }
}
