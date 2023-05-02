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

#include "../gcode.h"
#include "../../module/motion.h"
#if ENABLED(CREALITY_RTS)
  #include "../../lcd/rts/lcd_rts.h"
#endif

/**
 * M220: Set speed percentage factor, aka "Feed Rate"
 *
 * Parameters
 *   S<percent> : Set the feed rate percentage factor
 *
 * Report the current speed percentage factor if no parameter is specified
 *
 * For MMU2 and MMU2S devices...
 *   B : Flag to back up the current factor
 *   R : Flag to restore the last-saved factor
 */
void GcodeSuite::M220() {
  if (!parser.seen_any()) {
    SERIAL_ECHOLNPGM("FR:", feedrate_percentage, "%");
    return;
  }

  static int16_t backup_feedrate_percentage = 100;
  const int16_t now_feedrate_perc = feedrate_percentage;
  if (parser.seen_test('R')) feedrate_percentage = backup_feedrate_percentage;
  if (parser.seen_test('B')) backup_feedrate_percentage = now_feedrate_perc;
  if (parser.seenval('S')) feedrate_percentage = parser.value_int();

  TERN_(CREALITY_RTS, RTS_UpdateFeedrate(feedrate_percentage));
}
