#ifndef OLED_H
#define OLED_H

#include <Arduino.h>
#include <PololuOLED.h>

class OLED_c : public PololuSH1106 {
public:
  OLED_c(uint8_t clk, uint8_t mos, uint8_t res, uint8_t dc, uint8_t cs)
    : PololuSH1106(clk, mos, res, dc, cs) {}

  void setMaxMinutes(unsigned long minutes) {
    max_ms_ = minutes * 60UL * 1000UL;
  }

  void startStopwatch() {
    start_ts_ = millis();
    end_ts_ = start_ts_ + max_ms_;
    last_draw_ts_ = 0;
    first_draw_done_ = false;
  }

  bool timeRemaining(bool forceDraw = false) {
    unsigned long now = millis();
    if (max_ms_ == 0) return false;

    unsigned long rem_ms = (now < end_ts_) ? (end_ts_ - now) : 0UL;
    unsigned long rem_s = rem_ms / 1000UL;

    if (forceDraw || !first_draw_done_ || (now - last_draw_ts_) >= 3000UL) {
      drawCountdown(rem_s);
      last_draw_ts_ = now;
      first_draw_done_ = true;
    }

    return (rem_ms > 0);
  }

private:
  void drawCountdown(unsigned long rem_s) {
    unsigned long mm = rem_s / 60UL;
    unsigned long ss = rem_s % 60UL;
    char buf[9];
    snprintf(buf, sizeof(buf), "%02lu:%02lu", mm, ss);

    this->gotoXY(0, 0);
    this->print(F("T-      "));
    this->gotoXY(0, 1);
    this->print(buf);
    this->print(F("  "));
  }

  unsigned long max_ms_ = 240000UL;
  unsigned long start_ts_ = 0;
  unsigned long end_ts_ = 0;
  unsigned long last_draw_ts_ = 0;
  bool first_draw_done_ = false;
};

#endif
