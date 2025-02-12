/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../inc/MarlinConfig.h"

#if HAS_FANCHECK

#include "../MarlinCore.h"
#include "../lcd/marlinui.h"

#if ENABLED(AUTO_REPORT_FANS)
  #include "../libs/autoreport.h"
#endif

#if ENABLED(PARK_HEAD_ON_PAUSE)
  #include "../gcode/queue.h"
#endif

/**
 * fancheck.h
 */
#define TACHO_COUNT TERN(HAS_E7_FAN_TACHO, 8, TERN(HAS_E6_FAN_TACHO, 7, TERN(HAS_E5_FAN_TACHO, 6, TERN(HAS_E4_FAN_TACHO, 5, TERN(HAS_E3_FAN_TACHO, 4, TERN(HAS_E2_FAN_TACHO, 3, TERN(HAS_E1_FAN_TACHO, 2, 1)))))))

class FanCheck {
  private:

    enum class TachoError : uint8_t { NONE, DETECTED, REPORTED, FIXED };

    #if HAS_PWMFANCHECK
      static bool measuring;  // For future use (3 wires PWM controlled fans)
    #else
      static constexpr bool measuring = true;
    #endif
    static Flags<TACHO_COUNT> tacho_state;
    static uint16_t edge_counter[TACHO_COUNT];
    static uint8_t rps[TACHO_COUNT];
    static TachoError error;

    static void report_speed_error(uint8_t fan);

  public:

    static bool enabled;

    static void init();
    static void update_tachometers();
    static void compute_speed(uint16_t elapsedTime);
    static void print_fan_states();
    #if HAS_PWMFANCHECK
      static void toggle_measuring() { FLIP(measuring); }
      static bool is_measuring() { return measuring; }
    #endif

    static void check_deferred_error() {
      if (error == TachoError::DETECTED) {
        error = TachoError::REPORTED;
        TERN(PARK_HEAD_ON_PAUSE, queue.inject(F("M125")), kill(GET_TEXT_F(MSG_FAN_SPEED_FAULT)));
      }
    }

    #if ENABLED(AUTO_REPORT_FANS)
      struct AutoReportFan { static void report(); };
      static AutoReporter<AutoReportFan> auto_reporter;
    #endif
};

extern FanCheck fan_check;

#endif // HAS_FANCHECK
