/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../../../inc/MarlinConfig.h"

#if ENABLED(GCODE_MACROS)

#include "../../gcode.h"
#include "../../queue.h"
#include "../../parser.h"

extern char gcode_macros[GCODE_MACROS_SLOTS][GCODE_MACROS_SLOT_SIZE + 1];

/**
 * M820: List defined M810 - M819 macros
 */
void GcodeSuite::M820() {
  SERIAL_ECHOLNPGM(STR_STORED_MACROS);
  bool some = false;
  for (uint8_t i = 0; i < GCODE_MACROS_SLOTS; ++i) {
    const char *cmd = gcode_macros[i];
    if (*cmd) {
      SERIAL_ECHO(F("M81"), i, C(' '));
      char c;
      while ((c = *cmd++)) SERIAL_CHAR(c == '\n' ? '|' : c);
      SERIAL_EOL();
      some = true;
    }
  }
  if (!some) SERIAL_ECHOLNPGM("None");
}

#endif // GCODE_MACROS
