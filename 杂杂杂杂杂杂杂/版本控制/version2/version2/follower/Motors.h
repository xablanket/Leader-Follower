
#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15

#define L_FWD LOW
#define L_REV HIGH
#define R_FWD LOW
#define R_REV HIGH

#define RIGHT_SCALE 0.978f

#define MAX_PWM 180.0

class Motors_c {

  public:

    Motors_c() {
    }

    void initialise() {

      pinMode( L_PWM , OUTPUT );
      pinMode( L_DIR , OUTPUT );
      pinMode( R_PWM , OUTPUT );
      pinMode( R_DIR , OUTPUT );

      digitalWrite( L_DIR, L_FWD );
      digitalWrite( R_DIR, R_FWD );

      analogWrite( L_PWM , 0 );
      analogWrite( R_PWM , 0 );

    }

    void setPWM( float left_pwr, float right_pwr ) {

      if ( left_pwr < 0 ) {
        digitalWrite( L_DIR, L_REV );
      } else {
        digitalWrite( L_DIR, L_FWD );
      }

      if ( right_pwr < 0 ) {
        digitalWrite( R_DIR, R_REV );
      } else {
        digitalWrite( R_DIR, R_FWD );
      }

      if ( left_pwr < 0 ) left_pwr = -left_pwr;
      if ( left_pwr > 255.0 ) left_pwr = 255.0;
      if ( left_pwr > MAX_PWM ) left_pwr = MAX_PWM;
      int left_pwm_val = (int)left_pwr;

      if ( right_pwr < 0 ) right_pwr = -right_pwr;
      right_pwr = right_pwr * RIGHT_SCALE;
      if ( right_pwr > 255.0 ) right_pwr = 255.0;
      if ( right_pwr > MAX_PWM ) right_pwr = MAX_PWM;
      int right_pwm_val = (int)right_pwr;

      analogWrite( L_PWM, left_pwm_val );
      analogWrite( R_PWM, right_pwm_val );

      return;

    }

};

#endif
