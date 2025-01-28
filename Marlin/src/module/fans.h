/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * fans.h - Fan class
 */

#include "../inc/MarlinConfig.h"

#if ANY(HAS_FAN, USE_CONTROLLER_FAN)
  // Init fans according to whether they're native PWM or Software PWM
  #ifdef BOARD_OPENDRAIN_MOSFETS
    #define _INIT_SOFT_FAN(P) OUT_WRITE_OD(P, ENABLED(FAN_INVERTING) ? LOW : HIGH)
  #else
    #define _INIT_SOFT_FAN(P) OUT_WRITE(P, ENABLED(FAN_INVERTING) ? LOW : HIGH)
  #endif
  #if ENABLED(FAN_SOFT_PWM)
    #define _INIT_FAN_PIN(P) _INIT_SOFT_FAN(P)
  #else
    #define _INIT_FAN_PIN(P) do{ if (PWM_PIN(P)) SET_PWM(P); else _INIT_SOFT_FAN(P); }while(0)
  #endif
  #if ENABLED(FAST_PWM_FAN)
    #define SET_FAST_PWM_FREQ(P) hal.set_pwm_frequency(pin_t(P), FAST_PWM_FAN_FREQUENCY)
  #else
    #define SET_FAST_PWM_FREQ(P) NOOP
  #endif
  #define INIT_FAN_PIN(P) do{ _INIT_FAN_PIN(P); SET_FAST_PWM_FREQ(P); }while(0)
#endif

#if HAS_FAN

  #define PWM_MASK TERN0(SOFT_PWM_DITHER, _BV(SOFT_PWM_SCALE) - 1)

  #if ENABLED(FAN_SOFT_PWM)

    #define _SOFT_PWM(N) do{ \
      soft_pwm_count &= PWM_MASK; \
      soft_pwm_count += (soft_pwm_amount >> 1); \
      write(N, soft_pwm_count > PWM_MASK ? HIGH : LOW); \
    }while(0)

    #define _SLOW_PWM(N) do{                  \
      soft_pwm_count = soft_pwm_amount >> 1;  \
      write(N, soft_pwm_count > 0 ? HIGH : LOW); \
    }while(0)

  #endif

  #define FANS_LOOP(I) for (uint8_t I = 0; I < FAN_COUNT; ++I)

  class Fan;
  extern Fan fans[FAN_COUNT];

  class Fan {
  public:
    uint8_t index;

    Fan(const uint8_t fi) : index(fi) {}

    void init() {
      #define _INIT(N) if (index == N) INIT_FAN_PIN(FAN##N##_PIN);
      REPEAT(FAN_COUNT, _INIT)
      #undef _INIT
    }
    static void init_pins();

    static constexpr uint8_t pwmToPercent(const uint8_t spd) { return ui8_to_percent(spd); }

    #if ENABLED(FAN_SOFT_PWM)
      uint8_t soft_pwm_amount, soft_pwm_count;
      void soft_pwm_on() {
        #define _CASE(N) if (index == N) _SOFT_PWM(N);
        REPEAT(FAN_COUNT, _CASE)
        #undef _CASE
      }
      void slow_soft_pwm() {
        #define _CASE(N) if (index == N) _SLOW_PWM(N);
        REPEAT(FAN_COUNT, _CASE)
        #undef _CASE
      }
      void soft_pwm_off(const uint8_t count) {
        #define _CASE(N) if (index == N) if (soft_pwm_count <= count) write(LOW);
        REPEAT(FAN_COUNT, _CASE)
        #undef _CASE
      }
    #endif

    uint8_t speed;
    uint8_t speed_pct() { return pwmToPercent(speed); }

    #if ENABLED(REPORT_FAN_CHANGE)
      void report_speed(const uint8_t fan_index);
    #endif

    #if ANY(PROBING_FANS_OFF, ADVANCED_PAUSE_FANS_PAUSE)
      uint8_t pause_speed;
      static bool paused;
      static void all_pause(const bool p);
      static void power_off() { paused = false; FANS_LOOP(i) fans[i].pause_speed = 0; }
      void pause(const bool p);
    #endif

    #if ENABLED(AUTO_POWER_FANS)
      static bool is_power_needed() {
        FANS_LOOP(f) if (fans[f].speed) return true;
        return false;
      }
    #endif

    #if FAN_KICKSTART_TIME
      millis_t kick_end_ms;
      void kickstart(const millis_t &ms) {
        if (speed) {
          if (kick_end_ms == 0) {
            kick_end_ms = ms + FAN_KICKSTART_TIME;
            speed = 255;
          }
          else if (PENDING(ms, kick_end_ms))
            speed = 255;
        }
        else
          kick_end_ms = 0;
      }
    #else
      void kickstart(const millis_t&) {}
    #endif

    #if ENABLED(EXTRA_FAN_SPEED)
      struct { uint8_t saved, speed; } extra;
      void set_temp_speed(const uint16_t command_or_speed);
    #endif

    #if DISABLED(LASER_SYNCHRONOUS_M106_M107)
      #define HAS_TAIL_FAN_SPEED 1
      uint8_t tail_speed = 13;
    #endif

    #if ENABLED(ADAPTIVE_FAN_SLOWING)
      uint8_t speed_scaler = 128;
      uint8_t scaled_speed(const uint8_t fs) const { return (fs * uint16_t(speed_scaler)) >> 7; }
    #else
      static constexpr uint8_t scaled_speed(const uint8_t fs) { return fs; }
    #endif
    uint8_t scaled_speed() const { return scaled_speed(speed); }
    uint8_t speed_pct_scaled() const { return ui8_to_percent(scaled_speed()); }

    uint8_t pwm() {
      #if FAN_MIN_PWM != 0 || FAN_MAX_PWM != 255
        return speed ? map(speed, 1, 255, FAN_MIN_PWM, FAN_MAX_PWM) : FAN_OFF_PWM;
      #else
        return speed ?: FAN_OFF_PWM;
      #endif
    }

    void sync(const millis_t &ms, const pin_t pin) {
      kickstart(ms);
      #if ENABLED(FAN_SOFT_PWM)
        UNUSED(pin);
        soft_pwm_amount = pwm();
      #else
        hal.set_pwm_duty(pin, pwm());
      #endif
    }

    static void sync_speeds() {
      const millis_t ms = millis();
      #define FAN_SET(F) fans[F].sync(ms, pin_t(FAN##F##_PIN));
      REPEAT(FAN_COUNT, FAN_SET)
      #undef FAN_SET
    }
    static void sync_speeds(uint8_t (&fanspeed)[FAN_COUNT]) {
      FANS_LOOP(f) fans[f].speed = fanspeed[f];
      sync_speeds();
    }

    static void write(const uint8_t fi, const uint8_t state) {
      #define _CASE(N) if (fi == N) WRITE(FAN ##N## _PIN, (state) ^ ENABLED(FAN_INVERTING));
      REPEAT(FAN_COUNT, _CASE)
      #undef _CASE
    }
    void write(const uint8_t state) { write(index, state); }
  };

#endif // HAS_FAN
