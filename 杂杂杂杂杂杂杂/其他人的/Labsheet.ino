// Labsheet4_Final_PWM_Foraging.ino
// Foraging map traversal (6-point), smooth-turn, PWM control, non-blocking
// Added: return-to-home turn-to-initial + 4s wait, clean v6_next_tx/ty assignment, no duplicate setTravel.

#define BUZZER_PIN 6
#define LED_BUILTIN 13

#include "Motors.h"
#include "Encoders.h"
#include "Kinematics.h"

Motors_c motors;
Kinematics_c pose;

#define SPEED_EST_MS 50
unsigned long speed_est_ts;
long last_e0, last_e1;
int MEAS_SIGN_L = +1;
int MEAS_SIGN_R = +1;
float speed_e0_raw = 0, speed_e0_smooth = 0;
float speed_e1_raw = 0, speed_e1_smooth = 0;
float tau_speed = 0.30f;

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
static float wrapPi(float a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a <= -M_PI) a += 2 * M_PI;
  return a;
}
static float angDiff(float tgt, float cur) {
  return wrapPi(tgt - cur);
}

enum { MODE_HOLD = 0,
       MODE_TRAVEL = 1,
       MODE_TURN = 2 };
static uint8_t g_mode = MODE_TRAVEL;

static float g_travel_tx = 0, g_travel_ty = 0;
static bool g_travel_active = false;
static uint8_t g_travel_ok_cnt = 0;
static unsigned long no_reverse_until = 0;

// ---------------- 运动参数 ----------------
#define TRAVEL_V_GAIN_PWM 0.18f
#define TRAVEL_V_MAX_PWM 35
#define TRAVEL_V_MAX_SLOW 25
#define TRAVEL_SLOW_DIST_MM 100.0f
#define TRAVEL_W_GAIN_PWM 18.0f
#define TRAVEL_CORR_MAX_PWM 14
#define START_NOREV_MS 600
#define TRAVEL_STOP_DIST_MM 18.0f
#define TRAVEL_CONSEC_OK 4
#define PWM_MIN_FWD 18

// ===== Non-blocking Beeper =====
static bool g_beeping = false;
static unsigned long g_beep_until = 0;
static inline void Beep_now(unsigned ms) {
  analogWrite(BUZZER_PIN, 128);
  g_beeping = true;
  g_beep_until = millis() + ms;
}
void Beep_update() {
  if (g_beeping && (long)(millis() - g_beep_until) >= 0) {
    analogWrite(BUZZER_PIN, 0);
    g_beeping = false;
  }
}

// ===== Turn behaviour =====
#define TURN_KP 18.0f
#define TURN_P_MIN_PWM 18
#define TURN_P_MAX_PWM 45
#define TURN_DONE_DEG 8.0f
#define TURN_TIMEOUT_MS 2500
static float g_target_angle = 0.0f;
static bool g_turn_active = false;
static unsigned long turn_start_ts = 0;

void setTurn(float tgt_angle) {
  g_target_angle = wrapPi(tgt_angle);
  g_turn_active = true;
  turn_start_ts = millis();
}
bool checkTurn() {
  if (!g_turn_active) return true;
  float e = angDiff(g_target_angle, pose.theta);
  float ae = fabsf(e);
  if (ae < (TURN_DONE_DEG * M_PI / 180.0f)) {
    motors.setPWM(0, 0);
    g_turn_active = false;
    return true;
  }
  float pwmf = clampf(TURN_KP * ae, TURN_P_MIN_PWM, TURN_P_MAX_PWM);
  int pwm = (int)(pwmf + 0.5f);
  int dir = (e > 0) ? -1 : +1;
  motors.setPWM(dir * pwm, -dir * pwm);
  if ((long)(millis() - turn_start_ts) > TURN_TIMEOUT_MS) {
    motors.setPWM(0, 0);
    g_turn_active = false;
    return true;
  }
  return false;
}

// ===== Straight PWM drive =====
static void driveStraightPWM(int pwmL, int pwmR) {
  pwmL = clampf(pwmL, -100, 100);
  pwmR = clampf(pwmR, -100, 100);
  if (pwmL > 0 && pwmL < PWM_MIN_FWD) pwmL = PWM_MIN_FWD;
  if (pwmR > 0 && pwmR < PWM_MIN_FWD) pwmR = PWM_MIN_FWD;
  motors.setPWM(pwmL, pwmR);
}

// ===== Travel =====
void setTravel(float tx, float ty) {
  g_travel_tx = tx;
  g_travel_ty = ty;
  g_travel_active = true;
  g_travel_ok_cnt = 0;
  no_reverse_until = millis() + START_NOREV_MS;
}
bool checkTravel() {
  if (!g_travel_active) return true;
  float x = pose.x, y = pose.y, th = pose.theta;
  float dx = g_travel_tx - x, dy = g_travel_ty - y;
  float rho = sqrtf(dx * dx + dy * dy);
  float bearing = atan2f(dy, dx);
  float e_heading = angDiff(bearing, th);
  float v_cmd_pwm_f = TRAVEL_V_GAIN_PWM * rho;
  float v_cap = (rho <= TRAVEL_SLOW_DIST_MM) ? TRAVEL_V_MAX_SLOW : TRAVEL_V_MAX_PWM;
  if (v_cmd_pwm_f > v_cap) v_cmd_pwm_f = v_cap;
  int pwmF = (int)(v_cmd_pwm_f + 0.5f);
  float corr_pwm_f = TRAVEL_W_GAIN_PWM * e_heading;
  float w_scale = clampf(rho / TRAVEL_SLOW_DIST_MM, 0.0f, 1.0f);
  if (w_scale < 0.50f) w_scale = 0.50f;
  corr_pwm_f *= w_scale;
  float corr_rel_cap = pwmF * 0.25f;
  if (fabsf(corr_pwm_f) > corr_rel_cap)
    corr_pwm_f = (corr_pwm_f > 0 ? 1 : -1) * corr_rel_cap;
  corr_pwm_f = clampf(corr_pwm_f, -TRAVEL_CORR_MAX_PWM, TRAVEL_CORR_MAX_PWM);
  int dPWM = (int)(corr_pwm_f + 0.5f);
  int pwmL = pwmF - dPWM;
  int pwmR = pwmF + dPWM;
  if ((long)(millis() - no_reverse_until) < 0) {
    if (pwmL < 0) pwmL = 0;
    if (pwmR < 0) pwmR = 0;
  }
  if (pwmF > 0) {
    if (pwmL < 0) pwmL = 0;
    if (pwmR < 0) pwmR = 0;
  }
  driveStraightPWM(pwmL, pwmR);
  if (rho <= TRAVEL_STOP_DIST_MM) {
    if (++g_travel_ok_cnt >= TRAVEL_CONSEC_OK) {
      motors.setPWM(0, 0);
      g_travel_active = false;
      g_travel_ok_cnt = 0;
      return true;
    }
  } else g_travel_ok_cnt = 0;
  return false;
}

// ===== Waypoint FSM =====
#define NUM_SITES 6
float site_xy[NUM_SITES][2] = { { 60, 250 }, { 260, 200 }, { 410, 260 }, { 406, 115 }, { 360, 0 }, { 150, 90 } };
float v6_next_tx = 0.0f, v6_next_ty = 0.0f;
uint8_t site_idx = 0;
enum { V6_GOTO = 0,
       V6_BEEP = 1,
       V6_TURN = 2,
       V6_RETURN_TURN = 3,
       V6_DONE_WAIT = 4,
       V6_DONE = 5 };
uint8_t v6_state = V6_GOTO;
unsigned long v6_ts = 0;
float next_bearing = 0;
bool v6_returning_home = false;
unsigned long v6_done_ts = 0;

void visitSixSitesTick() {
  switch (v6_state) {
    case V6_GOTO:
      if (checkTravel()) {
        if (v6_returning_home) {
          setTurn(M_PI / 2 - 8.0f * M_PI / 180.0f);
          v6_state = V6_RETURN_TURN;
        } else {
          Beep_now(200);
          v6_state = V6_BEEP;
          v6_ts = millis();
        }
      }
      break;

    case V6_BEEP:
      if (!g_beeping || (long)(millis() - v6_ts) > 250) {
        if (site_idx < NUM_SITES - 1) {
          float cx = pose.x, cy = pose.y;
          float dx = site_xy[site_idx + 1][0] - cx;
          float dy = site_xy[site_idx + 1][1] - cy;
          next_bearing = atan2f(dy, dx);
          setTurn(next_bearing);
          v6_state = V6_TURN;
        } else {
          v6_next_tx = 0.0f;
          v6_next_ty = 0.0f;
          float cx = pose.x, cy = pose.y;
          next_bearing = atan2f(v6_next_ty - cy, v6_next_tx - cx);
          v6_returning_home = true;
          setTurn(next_bearing);
          v6_state = V6_TURN;
        }
      }
      break;

    case V6_TURN:
      if (checkTurn()) {
        if (!v6_returning_home) {
          site_idx++;
          if (site_idx < NUM_SITES) {
            setTravel(site_xy[site_idx][0], site_xy[site_idx][1]);
            v6_state = V6_GOTO;
          }
        } else {
          setTravel(v6_next_tx, v6_next_ty);
          v6_state = V6_GOTO;
        }
      }
      break;

    case V6_RETURN_TURN:
      if (checkTurn()) {
        motors.setPWM(0, 0);
        Beep_now(300);
        v6_done_ts = millis();
        v6_state = V6_DONE_WAIT;
      }
      break;

    case V6_DONE_WAIT:
      if ((long)(millis() - v6_done_ts) >= 4000) {
        v6_returning_home = false;
        site_idx = 0;
        pose.initialise(0, 0, M_PI / 2);
        setTravel(site_xy[0][0], site_xy[0][1]);
        v6_state = V6_GOTO;
      }
      break;

    case V6_DONE:
      motors.setPWM(0, 0);
      break;
  }
}

// ===== Setup / Loop =====
unsigned long pid_ts, serial_ts;

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, M_PI / 2);
  Serial.begin(9600);
  last_e1 = count_e1;
  last_e0 = count_e0;
  speed_e1_raw = speed_e1_smooth = 0.0f;
  speed_e0_raw = speed_e0_smooth = 0.0f;
  speed_est_ts = pid_ts = serial_ts = millis();
  delay(300);
  setTurn(M_PI / 2);
  Serial.println(" *** READY (PWM smooth-turn 6-site foraging) *** ");
  setTravel(site_xy[0][0], site_xy[0][1]);
  g_mode = MODE_TRAVEL;
}

void loop() {
  unsigned long t = millis();
  if (t - speed_est_ts >= SPEED_EST_MS) {
    unsigned long elapsed = t - speed_est_ts;
    speed_est_ts = t;
    long de0 = count_e0 - last_e0;
    last_e0 = count_e0;
    long de1 = count_e1 - last_e1;
    last_e1 = count_e1;
    speed_e0_raw = (MEAS_SIGN_R * (float)de0) / (float)elapsed;
    speed_e1_raw = (MEAS_SIGN_L * (float)de1) / (float)elapsed;
    float dt_s = (float)elapsed * 0.001f;
    float alpha_dyn = dt_s / (tau_speed + dt_s);
    speed_e0_smooth = alpha_dyn * speed_e0_raw + (1 - alpha_dyn) * speed_e0_smooth;
    speed_e1_smooth = alpha_dyn * speed_e1_raw + (1 - alpha_dyn) * speed_e1_smooth;
  }

  if (t - pid_ts >= 20) {
    pid_ts = t;
    pose.update();
    visitSixSitesTick();
    Beep_update();
    if (t - serial_ts >= 500) {
      serial_ts = t;
      float theta_deg = pose.theta * 180.0f / M_PI;
      float dx = g_travel_tx - pose.x, dy = g_travel_ty - pose.y;
      float rho = sqrtf(dx * dx + dy * dy);
      float bearing = atan2f(dy, dx);
      float e_head_deg = angDiff(bearing, pose.theta) * 180.0f / M_PI;
      Serial.print("site=");
      Serial.print(site_idx);
      Serial.print(" mode=");
      Serial.print(v6_state);
      Serial.print(" | theta=");
      Serial.print(theta_deg, 1);
      Serial.print(" | dist=");
      Serial.print(rho, 1);
      Serial.print(" mm | e(head)=");
      Serial.print(e_head_deg, 1);
      Serial.print(" deg | vR=");
      Serial.print(speed_e0_smooth, 3);
      Serial.print(" | vL=");
      Serial.println(speed_e1_smooth, 3);
    }
  }
}
