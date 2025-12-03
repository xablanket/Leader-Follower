#ifndef OLED_H
#define OLED_H

#include <Arduino.h>
#include <PololuOLED.h>

/*
 * Low-power OLED wrapper (compatible with the original interface).
 * - Class name and constructor signature unchanged: OLED_c(clk, mos, res, dc, cs)
 * - Public API kept: setMaxMinutes(), startStopwatch(), timeRemaining()
 * - Non-blocking: no delays inside
 * - Minimal draw: refresh at most once every 3000 ms, fixed "T-" and "MM:SS"
 * - Does not touch buzzer or any robot control
 */
class OLED_c : public PololuSH1106 {
public:
  // Keep the same ctor signature/ordering expected by the course header
  OLED_c(uint8_t clk, uint8_t mos, uint8_t res, uint8_t dc, uint8_t cs)
    : PololuSH1106(clk, mos, res, dc, cs) {}

  // Set the total countdown length (minutes)
  void setMaxMinutes(unsigned long minutes) {
    max_ms_ = minutes * 60UL * 1000UL;
  }

  // Start (or restart) the stopwatch / countdown
  void startStopwatch() {
    start_ts_ = millis();
    end_ts_ = start_ts_ + max_ms_;
    last_draw_ts_ = 0;  // force first draw
    first_draw_done_ = false;
  }

  // Update countdown and (optionally) refresh the screen.
  // Returns true while there is remaining time, false when it has elapsed.
  bool timeRemaining(bool forceDraw = false) {
    unsigned long now = millis();
    if (max_ms_ == 0) return false;

    // Compute remaining ms (clamped to zero)
    unsigned long rem_ms = (now < end_ts_) ? (end_ts_ - now) : 0UL;
    unsigned long rem_s = rem_ms / 1000UL;

    // Only redraw if 3s have passed (or first draw / forced)
    if (forceDraw || !first_draw_done_ || (now - last_draw_ts_) >= 3000UL) {
      drawCountdown(rem_s);
      last_draw_ts_ = now;
      first_draw_done_ = true;
    }

    return (rem_ms > 0);
  }

private:
  void drawCountdown(unsigned long rem_s) {
    // Prepare MM:SS
    unsigned long mm = rem_s / 60UL;
    unsigned long ss = rem_s % 60UL;
    char buf[9];
    snprintf(buf, sizeof(buf), "%02lu:%02lu", mm, ss);

    // Minimal overwrite of two fixed rows (avoid clear() for lower power)
    this->gotoXY(0, 0);
    this->print(F("T-      "));  // label + spaces to wipe leftovers
    this->gotoXY(0, 1);
    this->print(buf);
    this->print(F("  "));  // pad to clear tail
  }

  // State
  unsigned long max_ms_ = 240000UL;  // default 4 minutes
  unsigned long start_ts_ = 0;
  unsigned long end_ts_ = 0;
  unsigned long last_draw_ts_ = 0;
  bool first_draw_done_ = false;
};

#endif  // OLED_H
