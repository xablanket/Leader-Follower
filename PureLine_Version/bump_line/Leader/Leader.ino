
#include "Encoders.h"
#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"

#define EMIT_PIN    11
#define BTN_PIN     14
#define BUZZER_PIN  6

Motors_c     motors;
Kinematics_c kin;
PID_c        left_pid;
PID_c        right_pid;

unsigned long ir_ts = 0;
bool          ir_bump_mode = true;
unsigned long beep_off_time = 0;

void softBeep(int duration_ms) {
  tone(BUZZER_PIN, 800);
  beep_off_time = millis() + duration_ms;
}

#define DRIVE_EST_MS    20UL
#define DRIVE_PID_MS    40UL
#define DRIVE_PWM_LIMIT 60
const float DEMAND_CS = -300.0f;
const int   kF_L      = 16;
const int   kF_R      = 15;

float KP_L = 0.04025f, KI_L = 0.00000f, KD_L = 0.0f;
float KP_R = 0.07000f, KI_R = 0.00000f, KD_R = 0.0f;

unsigned long drive_est_ts = 0, drive_pid_ts = 0;
long          d_last_e0    = 0, d_last_e1    = 0;
float         spdL_cps     = 0.0f, spdR_cps   = 0.0f;
float         d_mL1        = 0.0f, d_mL2      = 0.0f;
float         d_mR1        = 0.0f, d_mR2      = 0.0f;

float theta0 = 0.0f;
int   arc_state = 1;

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

  kin.initialise(0.0f, 0.0f, M_PI/6.0f);
  theta0 = kin.theta;

  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  softBeep(40);
  while (digitalRead(BTN_PIN) == HIGH) {
    unsigned long now = millis();
    if (now >= beep_off_time && beep_off_time != 0) {
      noTone(BUZZER_PIN);
      beep_off_time = 0;
    }
  }
  softBeep(40);

  unsigned long now = millis();
  ir_ts        = now;
  drive_est_ts = now;
  drive_pid_ts = now;

  left_pid.initialise(KP_L, KI_L, KD_L);
  right_pid.initialise(KP_R, KI_R, KD_R);
  left_pid.reset();
  right_pid.reset();
}

void loop() {
  unsigned long now = millis();

  if (now >= beep_off_time && beep_off_time != 0) {
    noTone(BUZZER_PIN);
    beep_off_time = 0;
  }

  if (now - ir_ts >= 100UL) {
    ir_ts = now;
    ir_bump_mode = !ir_bump_mode;
    digitalWrite(EMIT_PIN, ir_bump_mode ? LOW : HIGH);
  }

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

    spdR_cps = (float)d0 / dt_sec;
    spdL_cps = (float)d1 / dt_sec;

    kin.update();
  }

  if (arc_state == 1) {
    float dtheta = kin.theta - theta0;
    while (dtheta >  M_PI) dtheta -= 2*M_PI;
    while (dtheta < -M_PI) dtheta += 2*M_PI;

    if (dtheta <= -M_PI/3.0f) {
      motors.setPWM(0,0);
      arc_state = 4;
    }
  }

  if (arc_state == 4) {
    return;
  }

  if (now - drive_pid_ts >= DRIVE_PID_MS) {
    drive_pid_ts = now;

    const float wheel_sep = 70.0f;
    const float R_arc     = 450.0f;
    const float R_L = R_arc - wheel_sep;
    const float R_R = R_arc + wheel_sep;

    const float SCALE_L = 0.904f;
    const float SCALE_R = 1.00f;

    float v_center = DEMAND_CS;

    float demandL = v_center * (R_L / R_arc) * SCALE_L;
    float demandR = v_center * (R_R / R_arc) * SCALE_R;

    float measL = (spdL_cps + d_mL1 + d_mL2) / 3.0f;
    float measR = (spdR_cps + d_mR1 + d_mR2) / 3.0f;
    d_mL2 = d_mL1; d_mL1 = spdL_cps;
    d_mR2 = d_mR1; d_mR1 = spdR_cps;

    float uL = left_pid.update(demandL, measL);
    float uR = right_pid.update(demandR, measR);

    float uLc = clampf(uL, -12.0f, 12.0f);
    float uRc = clampf(uR, -12.0f, 12.0f);

    float baseL = (demandL >= 0.0f) ? kF_L : -kF_L;
    float baseR = (demandR >= 0.0f) ? kF_R : -kF_R;

    float pwmL = clampf(baseL + uLc, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
    float pwmR = clampf(baseR + uRc, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);

    motors.setPWM(iround(pwmL), iround(pwmR));
  }
}
