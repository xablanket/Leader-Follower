
#ifndef _LCD_H
#define _LCD_H

#include <PololuHD44780.h>

class LCD_c: public PololuHD44780 {

  public:
    LCD_c( uint8_t rs, uint8_t e, uint8_t db4, uint8_t db5, uint8_t db6, uint8_t db7 ): PololuHD44780(rs, e, db4, db5, db6, db7) {

    }

    void startStopwatch() {
      end_ts = millis() + max_ms;
      display_ts = millis();
    }

    void setMaxMinutes( unsigned long minutes ) {
      max_ms = minutes * 60 * 1000;
    }

    void reset() {
      disableUSB();
      this->clear();
      enableUSB();
    } 

    bool timeRemaining() {
      unsigned long now;
      now = millis();

      if (now - display_ts > 1000 ) {
        display_ts = millis();

        this->clear();

        if ( now < end_ts ) {
          unsigned long dt = end_ts - now;
          dt /= 1000;

          this->gotoXY(0, 1);
          this->print(dt);
          return true;

        } else {
          this->gotoXY(0, 1);
          this->print("- Done -");
          return false;
        }
      }
    }
private:
  unsigned long end_ts;
  unsigned long display_ts;
  unsigned long max_ms = 120000;

  uint8_t savedUDIEN;
  uint8_t savedUENUM;
  uint8_t savedUEIENX0;
  void disableUSB() {
      savedUDIEN = UDIEN;
      UDIEN = 0;

      savedUENUM = UENUM;
      UENUM = 0;

      savedUEIENX0 = UEIENX;
      UEIENX = 0;
    }
    void enableUSB() {
      UENUM = 0;
      UEIENX = savedUEIENX0;

      UENUM = savedUENUM;

      UDIEN = savedUDIEN;
    }
};

#endif
