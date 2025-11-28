#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15
#define MAX_PWM 120.0
#define FWD LOW
#define REV HIGH

class Motors_c {
public:
  Motors_c() {}

  void initialise() {
    pinMode(L_PWM, OUTPUT);
    pinMode(L_DIR, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(R_DIR, OUTPUT);
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
  }

  void setPWM(float left_pwr, float right_pwr) {

    int Lp = constrain((int)fabs(left_pwr), 0, 255);
    int Rp = constrain((int)fabs(right_pwr), 0, 255);

    digitalWrite(L_DIR, (left_pwr < 0) ? REV : FWD);
    analogWrite(L_PWM, Lp);

    digitalWrite(R_DIR, (right_pwr < 0) ? REV : FWD);
    analogWrite(R_PWM, Rp);
  }
};  // ← 正确关闭类定义

#endif