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
 * gcode/temp/M141_M191.cpp
 *
 * Chamber target temperature control
 */

#include "../../inc/MarlinConfig.h"

#if HAS_HEATED_CHAMBER

#include "../gcode.h"
#include "../../module/temperature.h"
#include "../../lcd/marlinui.h"

/**
 * M141: Set chamber temperature
 */
void GcodeSuite::M141() {
  if (DEBUGGING(DRYRUN)) return;
  // Accept 'I' if temperature presets are defined
  #if HAS_PREHEAT
    if (parser.seenval('I')) {
      const uint8_t index = parser.value_byte();
      thermalManager.setTargetChamber(ui.material_preset[_MIN(index, PREHEAT_COUNT - 1)].chamber_temp);
      return;
    }
  #endif
  if (parser.seenval('S')) {
    thermalManager.setTargetChamber(parser.value_celsius());
    #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
      /**
       * Stop the timer at the end of print. Hotend, bed target, and chamber
       * temperatures need to be set below mintemp. Order of M140, M104, and M141
       * at the end of the print does not matter.
       */
      thermalManager.auto_job_check_timer(false, true);
    #endif
  }
}

/**
 * M191: Sxxx Wait for chamber current temp to reach target temp. Waits only when heating
 *       Rxxx Wait for chamber current temp to reach target temp. Waits when heating and cooling
 */
void GcodeSuite::M191() {
  if (DEBUGGING(DRYRUN)) return;

  const bool no_wait_for_cooling = parser.seenval('S');
  if (no_wait_for_cooling || parser.seenval('R')) {
    thermalManager.setTargetChamber(parser.value_celsius());
    TERN_(PRINTJOB_TIMER_AUTOSTART, thermalManager.auto_job_check_timer(true, false));
  }
  else return;

  const bool is_heating = thermalManager.isHeatingChamber();
  if (is_heating || !no_wait_for_cooling) {
    ui.set_status(is_heating ? GET_TEXT_F(MSG_CHAMBER_HEATING) : GET_TEXT_F(MSG_CHAMBER_COOLING));
    thermalManager.wait_for_chamber(false);
  }
}

#endif // HAS_HEATED_CHAMBER
