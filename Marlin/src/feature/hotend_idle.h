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
 * feature/hotend_idle.h
 */

#include "../inc/MarlinConfig.h"

typedef struct {
  int16_t timeout, trigger, nozzle_target;
  #if HAS_HEATED_BED
    int16_t bed_target;
  #endif
  void set_defaults() {
    timeout       = HOTEND_IDLE_TIMEOUT_SEC;
    trigger       = HOTEND_IDLE_MIN_TRIGGER;
    nozzle_target = HOTEND_IDLE_NOZZLE_TARGET;
    #if HAS_HEATED_BED
      bed_target  = HOTEND_IDLE_BED_TARGET;
    #endif
  }
} hotend_idle_settings_t;

class HotendIdleProtection {
public:
  static void check();
  static hotend_idle_settings_t cfg;
private:
  static millis_t next_protect_ms;
  static void check_hotends(const millis_t &ms);
  static void check_e_motion(const millis_t &ms);
  static void timed_out();
};

extern HotendIdleProtection hotend_idle;
