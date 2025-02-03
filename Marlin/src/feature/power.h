/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * power.h - power control
 */

#if PIN_EXISTS(PS_ON_EDM) || (PIN_EXISTS(PS_ON1_EDM) && ENABLED(PSU_OFF_REDUNDANT))
  #define PSU_TRACK_STATE_MS 1
#endif

#if ANY(AUTO_POWER_CONTROL, POWER_OFF_TIMER, PSU_TRACK_STATE_MS)
  #include "../core/millis_t.h"
#endif

class Power {
  public:
    static bool psu_on;

    static void init();
    static void power_on();
    static void power_off();

    #if PSU_TRACK_STATE_MS
      static millis_t last_state_change_ms;
    #endif

    #if ANY(POWER_OFF_TIMER, POWER_OFF_WAIT_FOR_COOLDOWN)
      #if ENABLED(POWER_OFF_TIMER)
        static millis_t power_off_time;
        static void setPowerOffTimer(const millis_t delay_ms);
      #endif
      #if ENABLED(POWER_OFF_WAIT_FOR_COOLDOWN)
        static bool power_off_on_cooldown;
        static void setPowerOffOnCooldown(const bool ena);
      #endif
      static void cancelAutoPowerOff();
      static void checkAutoPowerOff();
    #endif

    #if ENABLED(AUTO_POWER_CONTROL) && POWER_OFF_DELAY > 0
      static void power_off_soon();
    #else
      static void power_off_soon() { power_off(); }
    #endif

    #if ENABLED(AUTO_POWER_CONTROL)
      static void check(const bool pause);

      private:
        static millis_t lastPowerOn;
        static bool is_power_needed();
        static bool is_cooling_needed();
    #elif ENABLED(POWER_OFF_WAIT_FOR_COOLDOWN)
      private:
        static bool is_cooling_needed();
    #endif
};

extern Power powerManager;
