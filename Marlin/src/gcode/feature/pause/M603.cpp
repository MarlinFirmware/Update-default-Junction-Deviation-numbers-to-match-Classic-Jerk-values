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

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(CONFIGURE_FILAMENT_CHANGE)

#include "../../gcode.h"
#include "../../../feature/pause.h"
#include "../../../module/motion.h"
#include "../../../module/printcounter.h"

/**
 * M603: Configure filament change
 *
 *  T[toolhead] - Select extruder to configure, active extruder if not specified
 *  U[distance] - Retract distance for removal, for the specified extruder
 *  L[distance] - Extrude distance for insertion, for the specified extruder
 */
void GcodeSuite::M603() {

  if (!parser.seen("TUL")) return M603_report();

  const int_fast8_t target_extruder = get_target_extruder_from_command();
  if (target_extruder < 0) return;

  // Unload length
  if (parser.seenval('U')) {
    fc_settings[target_extruder].unload_length = ABS(parser.value_axis_units(E_AXIS));
    #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
      NOMORE(fc_settings[target_extruder].unload_length, EXTRUDE_MAXLENGTH);
    #endif
  }

  // Load length
  if (parser.seenval('L')) {
    fc_settings[target_extruder].load_length = ABS(parser.value_axis_units(E_AXIS));
    #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
      NOMORE(fc_settings[target_extruder].load_length, EXTRUDE_MAXLENGTH);
    #endif
  }
}

void GcodeSuite::M603_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading(forReplay, F(STR_FILAMENT_LOAD_UNLOAD));

  #if EXTRUDERS == 1
    report_echo_start(forReplay);
    SERIAL_ECHOPGM("  M603 L", LINEAR_UNIT(fc_settings[0].load_length), " U", LINEAR_UNIT(fc_settings[0].unload_length), " ;");
    say_units();
  #else
    EXTRUDER_LOOP() {
      report_echo_start(forReplay);
      SERIAL_ECHOPGM("  M603 T", e, " L", LINEAR_UNIT(fc_settings[e].load_length), " U", LINEAR_UNIT(fc_settings[e].unload_length), " ;");
      say_units();
    }
  #endif
}

#endif // CONFIGURE_FILAMENT_CHANGE
