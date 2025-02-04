/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if ENABLED(MACHINE_COMMAND_MACROS)

#include "../gcode.h"
#include "../../sd/cardreader.h"

/**
 * M98: Select file and run as sub-procedure
 *
 *   P<path> - The plain (DOS 8.3) filepath
 *
 * Example:
 *    M98 P/macros/home.g ; Run home.g (as a procedure)
 *
 */
void GcodeSuite::M98() {
  if (card.isMounted() && parser.seen('P')) {
    char *path = parser.value_string();
    char *lb = strchr(p, ' ');
    if (!lb) lb = strchr(p, ';');
    if (lb) *lb = '\0';
    card.runMacro(path);
  }
}

#endif // MACHINE_COMMAND_MACROS
