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

#include "../../inc/MarlinConfigPre.h"

#if HAS_TOOLCHANGE

#include "../gcode.h"
#include "../../module/tool_change.h"

#if ANY(HAS_MULTI_EXTRUDER, DEBUG_LEVELING_FEATURE)
  #include "../../module/motion.h"
#endif

#if HAS_PRUSA_MMU3
  #include "../../feature/mmu3/mmu3.h"
#elif HAS_PRUSA_MMU2
  #include "../../feature/mmu/mmu2.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

/**
 * T0-T<n>: Switch tool, usually switching extruders
 *
 *   F[units/min] Set the movement feedrate
 *   S1           Don't move the tool in XY after change
 *
 * For PRUSA_MMU2(S) and EXTENDABLE_EMU_MMU2(S)
 *   T[n] G-code to extrude at least 38.10 mm at feedrate 19.02 mm/s must follow immediately to load to extruder wheels.
 *   T?   G-code to extrude shouldn't have to follow. Load to extruder wheels is done automatically.
 *   Tx   Same as T?, but nozzle doesn't have to be preheated. Tc requires a preheated nozzle to finish filament load.
 *   Tc   Load to nozzle after filament was prepared by Tc and nozzle is already heated.
 */
void GcodeSuite::T(const int8_t tool_index) {

  #if HAS_MULTI_EXTRUDER
    // For 'T' with no parameter report the current tool.
    if (parser.string_arg && *parser.string_arg == '*') {
      SERIAL_ECHOLNPGM(STR_ACTIVE_EXTRUDER, active_extruder);
      return;
    }
  #endif

  DEBUG_SECTION(log_T, "T", DEBUGGING(LEVELING));
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("...(", tool_index, ")");

  // Count this command as movement / activity
  reset_stepper_timeout();

  #if HAS_PRUSA_MMU3
    if (parser.string_arg) {
      mmu3.tool_change(parser.string_arg[0], uint8_t(tool_index));   // Special commands T?/Tx/Tc
      return;
    }
  #elif HAS_PRUSA_MMU2
    if (parser.string_arg) {
      mmu2.tool_change(parser.string_arg);   // Special commands T?/Tx/Tc
      return;
    }
  #endif

  tool_change(tool_index
    #if HAS_MULTI_EXTRUDER
      , parser.boolval('S')
        || TERN(PARKING_EXTRUDER, false, tool_index == active_extruder) // For PARKING_EXTRUDER motion is decided in tool_change()
    #endif
  );
}

#endif // HAS_TOOLCHANGE
