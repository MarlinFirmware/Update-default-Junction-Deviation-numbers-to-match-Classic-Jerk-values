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

#if HAS_MEDIA

#include "../gcode.h"
#include "../../sd/cardreader.h"
#include "../../lcd/marlinui.h"
#if ENABLED(CREALITY_RTS)
  #include "../../lcd/rts/lcd_rts.h"
#endif

/**
 * M23: Open a file
 *
 * The path is relative to the root directory
 */
void GcodeSuite::M23() {
  // Simplify3D includes the size, so zero out all spaces (#7227)
  for (char *fn = parser.string_arg; *fn; ++fn) if (*fn == ' ') *fn = '\0';
  card.openFileRead(parser.string_arg);

  TERN_(SET_PROGRESS_PERCENT, ui.set_progress(0));
  TERN_(CREALITY_RTS, RTS_OpenFileCloud());
}

#endif // HAS_MEDIA
