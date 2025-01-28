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

/**
 * fans.cpp - Fan class
 */

#include "../inc/MarlinConfig.h"

#if HAS_FAN

#include "fans.h"

#define _FANDEF(N) Fan(N),
Fan fans[FAN_COUNT] = { REPEAT(FAN_COUNT, _FANDEF) };

#if ANY(PROBING_FANS_OFF, ADVANCED_PAUSE_FANS_PAUSE)
  bool Fan::paused; // = false
#endif

void Fan::init_pins() {
  #define _INIT(N) fans[N].init();
  REPEAT(FAN_COUNT, _INIT)
}

#if ENABLED(REPORT_FAN_CHANGE)
  /**
   * Report print fan speed for a target extruder
   */
  void Fan::report_speed(const uint8_t fan_index) {
    PORT_REDIRECT(SerialMask::All);
    SERIAL_ECHOLNPGM("M106 P", fan_index, " S", speed);
  }
#endif

#if ENABLED(EXTRA_FAN_SPEED)
  /**
   * Handle the M106 P<fan> T<speed> command:
   *  T1       = Restore fan speed saved on the last T2
   *  T2       = Save the fan speed, then set to the last T<3-255> value
   *  T<3-255> = Set the "extra fan speed"
   */
  void Fan::set_temp_speed(const uint16_t command_or_speed) {
    switch (command_or_speed) {
      case 1: break;
      case 2: extra.saved = speed; break;
      default: extra.speed = _MIN(command_or_speed, 255U); break;
    }
  }
#endif

#if ANY(PROBING_FANS_OFF, ADVANCED_PAUSE_FANS_PAUSE)
  void Fan::pause(const bool p) {
    if (p) {
      pause_speed = speed;
      speed = 0;
    }
    else
      speed = pause_speed;
  }

  void Fan::all_pause(const bool p) {
    if (p != paused) {
      paused = p;
      FANS_LOOP(i) fans[i].pause(p);
    }
  }
#endif

#endif // HAS_FAN
