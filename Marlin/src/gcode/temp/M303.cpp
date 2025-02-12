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

#include "../../inc/MarlinConfig.h"

#if HAS_PID_HEATING

#include "../gcode.h"
#include "../queue.h" // for flush_tx
#include "../../lcd/marlinui.h"
#include "../../module/temperature.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extui/ui_api.h"
#endif

/**
 * M303: PID relay autotune
 *
 *  S<temperature>  Set the target temperature. (Default: 150C / 70C)
 *  E<extruder>     Extruder number to tune, or -1 for the bed. (Default: E0)
 *  C<cycles>       Number of times to repeat the procedure. (Minimum: 3, Default: 5)
 *  U<bool>         Flag to apply the result to the current PID values
 *
 * With PID_DEBUG, PID_BED_DEBUG, or PID_CHAMBER_DEBUG:
 *  D               Toggle PID debugging and EXIT without further action.
 */

void GcodeSuite::M303() {

  #if HAS_PID_DEBUG
    if (parser.seen_test('D')) {
      FLIP(thermalManager.pid_debug_flag);
      SERIAL_ECHO_START();
      SERIAL_ECHOPGM("PID Debug ");
      serialprintln_onoff(thermalManager.pid_debug_flag);
      return;
    }
  #endif

  const heater_id_t hid = (heater_id_t)parser.intval('E');
  celsius_t default_temp;
  switch (hid) {
    OPTCODE(PIDTEMP,        case 0 ... HOTENDS - 1: default_temp = PREHEAT_1_TEMP_HOTEND;  break)
    OPTCODE(PIDTEMPBED,     case H_BED:             default_temp = PREHEAT_1_TEMP_BED;     break)
    OPTCODE(PIDTEMPCHAMBER, case H_CHAMBER:         default_temp = PREHEAT_1_TEMP_CHAMBER; break)
    default:
      SERIAL_ECHOPGM(STR_PID_AUTOTUNE);
      SERIAL_ECHOLNPGM(STR_PID_BAD_HEATER_ID);
      TERN_(EXTENSIBLE_UI, ExtUI::onPIDTuning(ExtUI::pidresult_t::PID_BAD_HEATER_ID));
      return;
  }

  const bool seenC = parser.seenval('C');
  const int c = seenC ? parser.value_int() : 5;
  const bool seenS = parser.seenval('S');
  const celsius_t temp = seenS ? parser.value_celsius() : default_temp;
  const bool u = parser.boolval('U');

  TERN_(EXTENSIBLE_UI, ExtUI::onStartM303(c, hid, temp));

  IF_DISABLED(BUSY_WHILE_HEATING, KEEPALIVE_STATE(NOT_BUSY));

  LCD_MESSAGE(MSG_PID_AUTOTUNE);
  thermalManager.PID_autotune(temp, hid, c, u);
  ui.reset_status();

  queue.flush_rx();
}

#endif // HAS_PID_HEATING
