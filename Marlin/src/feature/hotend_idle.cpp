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

/**
 * Hotend Idle Timeout
 * Prevent filament in the nozzle from charring and causing a critical jam.
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(HOTEND_IDLE_TIMEOUT)

#include "hotend_idle.h"
#include "../gcode/gcode.h"

#include "../module/temperature.h"
#include "../module/motion.h"
#include "../module/planner.h"
#include "../lcd/marlinui.h"

HotendIdleProtection hotend_idle;

millis_t HotendIdleProtection::next_protect_ms = 0;
hotend_idle_settings_t HotendIdleProtection::cfg; // Initialized by settings.load

void HotendIdleProtection::check_hotends(const millis_t &ms) {
  const bool busy = (TERN0(HAS_RESUME_CONTINUE, wait_for_user) || planner.has_blocks_queued());
  bool do_prot = false;
  if (!busy && cfg.timeout != 0) {
    HOTEND_LOOP() {
      if (thermalManager.degHotend(e) >= cfg.trigger) {
        do_prot = true; break;
      }
    }
  }
  if (!do_prot)
    next_protect_ms = 0;                          // No hotends are hot so cancel timeout
  else if (!next_protect_ms)                      // Timeout is possible?
    next_protect_ms = ms + 1000UL * cfg.timeout;  // Start timeout if not already set
}

void HotendIdleProtection::check_e_motion(const millis_t &ms) {
  static float old_e_position = 0;
  if (old_e_position != current_position.e) {
    old_e_position = current_position.e;            // Track filament motion
    if (next_protect_ms)                            // If some heater is on then...
      next_protect_ms = ms + 1000UL * cfg.timeout;  // ...delay the timeout till later
  }
}

void HotendIdleProtection::check() {
  const millis_t ms = millis();                   // Shared millis

  check_hotends(ms);                              // Any hotends need protection?
  check_e_motion(ms);                             // Motion will protect them

  // Hot and not moving for too long...
  if (next_protect_ms && ELAPSED(ms, next_protect_ms))
    timed_out();
}

// Lower (but don't raise) hotend / bed temperatures
void HotendIdleProtection::timed_out() {
  next_protect_ms = 0;
  SERIAL_ECHOLNPGM("Hotend Idle Timeout");
  LCD_MESSAGE(MSG_HOTEND_IDLE_TIMEOUT);
  HOTEND_LOOP() {
    if (cfg.nozzle_target < thermalManager.degTargetHotend(e))
      thermalManager.setTargetHotend(cfg.nozzle_target, e);
  }
  #if HAS_HEATED_BED
    if (cfg.bed_target < thermalManager.degTargetBed())
      thermalManager.setTargetBed(cfg.bed_target);
  #endif
}

#endif // HOTEND_IDLE_TIMEOUT
