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
 * gcode/temp/M140_M190.cpp
 *
 * Bed target temperature control
 */

#include "../../inc/MarlinConfig.h"

#if HAS_HEATED_BED

#include "../gcode.h"
#include "../../module/temperature.h"
#include "../../lcd/marlinui.h"

/**
 * M140 - Set Bed Temperature target and return immediately
 * M190 - Set Bed Temperature target and wait
 *
 *  I<index>  : Preset index (if material presets are defined)
 *  S<target> : The target temperature in current units
 *
 * Parameters
 *  I<index>  : Preset index (if material presets are defined)
 *  S<target> : The target temperature in current units. Wait for heating only.
 *
 * M190 Parameters
 *  R<target> : The target temperature in current units. Wait for heating and cooling.
 *
 * Examples
 *  M140 S60 : Set target to 60° and return right away.
 *  M190 R40 : Set target to 40°. Wait until the bed gets close to 40°.
 *
 * With PRINTJOB_TIMER_AUTOSTART turning on heaters will start the print job timer
 *  (used by printingIsActive, etc.) and turning off heaters will stop the timer.
 *
 * With BED_ANNEALING_GCODE:
 *
 * M190 Parameters
 *     T<seconds>: Cooldown time, for more gradual cooling. Use with R parameter.
 *                 M190 R T - Cool the bed down over a given period of time.
 *
 * Examples
 *  M190 R70 T600: Cool down to 70°C over a period of ten minutes.
 *
 */
void GcodeSuite::M140_M190(const bool isM190) {

  if (DEBUGGING(DRYRUN)) return;

  bool got_temp = false;
  celsius_t temp = 0;

  // Accept 'I' if temperature presets are defined
  #if HAS_PREHEAT
    got_temp = parser.seenval('I');
    if (got_temp) {
      const uint8_t index = parser.value_byte();
      temp = ui.material_preset[_MIN(index, PREHEAT_COUNT - 1)].bed_temp;
    }
  #endif

  // Get the temperature from 'S' or 'R'
  bool no_wait_for_cooling = false;
  if (!got_temp) {
    no_wait_for_cooling = parser.seenval('S');
    got_temp = no_wait_for_cooling || (isM190 && parser.seenval('R'));
    if (got_temp) temp = parser.value_celsius();
  }

  if (!got_temp) return;

  #if ENABLED(BED_ANNEALING_GCODE)
    const bool anneal = isM190 && !no_wait_for_cooling && parser.seenval('T');
    const millis_t anneal_ms = anneal ? parser.value_millis_from_seconds() : 0UL;
  #else
    constexpr bool anneal = false;
  #endif

  if (!anneal) {
    thermalManager.setTargetBed(temp);
    thermalManager.isHeatingBed() ? LCD_MESSAGE(MSG_BED_HEATING) : LCD_MESSAGE(MSG_BED_COOLING);
  }

  // With PRINTJOB_TIMER_AUTOSTART, M190 can start the timer, and M140 can stop it
  TERN_(PRINTJOB_TIMER_AUTOSTART, thermalManager.auto_job_check_timer(isM190, !isM190));

  if (isM190) {
    #if ENABLED(BED_ANNEALING_GCODE)
      if (anneal) {
        LCD_MESSAGE(MSG_BED_ANNEALING);
        const millis_t wait_ms = anneal_ms / (thermalManager.degBed() - temp) ;
        // Loop from current temp down to the target
        for (celsius_t cool_temp = thermalManager.degBed() - 1; cool_temp >= temp; --cool_temp) {
          thermalManager.setTargetBed(cool_temp); // Cool by one degree
          dwell(wait_ms); //Wait for going to the next degree
        }
        return;
      }
    #endif

    thermalManager.wait_for_bed(no_wait_for_cooling);
  }
  else {
    ui.set_status_reset_fn([]{
      const celsius_t c = thermalManager.degTargetBed();
      return c < 30 || thermalManager.degBedNear(c);
    });
  }
}

#endif // HAS_HEATED_BED
