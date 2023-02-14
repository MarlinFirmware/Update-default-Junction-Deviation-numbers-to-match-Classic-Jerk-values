/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if ENABLED(CREALITY_RTS)

#include "../gcode.h"

#include "../../lcd/rts/lcd_rts.h"

/**
 * M79: cloud print statistics
 */
void GcodeSuite::M79() {
  if (parser.seenval('S')) RTS_CloudCommand(cloudCommand_t(parser.value_int()));
  if (parser.seenval('T')) RTS_SetStatsFR(parser.value_feedrate());
  if (parser.seenval('C')) RTS_SetStatsTime(parser.value_celsius());
  if (parser.seenval('D')) RTS_SetStatsRemain(parser.value_celsius());
}

#endif // CREALITY_RTS
