
#ifndef _LCD_H
#define _LCD_H

#include <PololuHD44780.h>

class LCD_c: public PololuHD44780 {

  public:
    LCD_c( uint8_t rs, uint8_t e, uint8_t db4, uint8_t db5, uint8_t db6, uint8_t db7 ): PololuHD44780(rs, e, db4, db5, db6, db7), _rs(rs), _e(e), _d4(db4), _d5(db5), _d6(db6), _d7(db7) {}

    LCD_c(): PololuHD44780( 0, 1, 14, 17, 13, 30 ), _rs(0), _e(1), _d4(14), _d5(17), _d6(13), _d7(30) {}

    void init(){
      pinMode(_rs, OUTPUT); pinMode(_e, OUTPUT);
      pinMode(_d4, OUTPUT); pinMode(_d5, OUTPUT); pinMode(_d6, OUTPUT); pinMode(_d7, OUTPUT);
      digitalWrite(_rs, LOW); digitalWrite(_e, LOW);
      delay(50);
      writeNibble(0x03); delay(5);
      writeNibble(0x03); delay(5);
      writeNibble(0x03); delay(5);
      writeNibble(0x02); delay(5);
      sendCmd(0x28);
      sendCmd(0x0C);
      sendCmd(0x06);
      sendCmd(0x01);
      delay(2);
      this->clear();
      this->gotoXY(0,0);
      this->print("READY");
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

    bool timeRemaining(uint16_t pushed) {
      unsigned long now;
      now = millis();

      if (now - display_ts > 1000 ) {
        display_ts = millis();

        this->clear();

        if ( now < end_ts ) {
          unsigned long dt = end_ts - now;
          dt /= 1000;

          this->gotoXY(0, 0);
          this->print(dt);
          this->gotoXY(0, 1);
          this->print(pushed);
          return true;

        } else {
          this->gotoXY(0, 0);
          this->print("0");
          this->gotoXY(0, 1);
          this->print(pushed);
          return false;
        }
      }
    }
private:
    uint8_t _rs,_e,_d4,_d5,_d6,_d7;

    inline void pulseEnable(){
      digitalWrite(_e, LOW); delayMicroseconds(1);
      digitalWrite(_e, HIGH); delayMicroseconds(1);
      digitalWrite(_e, LOW); delayMicroseconds(100);
    }
    inline void writeNibble(uint8_t nib){
      digitalWrite(_d4, (nib>>0)&0x01);
      digitalWrite(_d5, (nib>>1)&0x01);
      digitalWrite(_d6, (nib>>2)&0x01);
      digitalWrite(_d7, (nib>>3)&0x01);
      pulseEnable();
    }
    inline void sendCmd(uint8_t cmd){
      digitalWrite(_rs, LOW);
      writeNibble((cmd>>4)&0x0F);
      writeNibble(cmd & 0x0F);
    }
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
