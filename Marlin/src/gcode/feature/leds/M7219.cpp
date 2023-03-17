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

#if ENABLED(MAX7219_GCODE)

#include "../../gcode.h"
#include "../../../feature/max7219.h"

/**
 * M7219: Control the Max7219 LED matrix
 *
 *  I         - Initialize (clear) the matrix
 *  F         - Fill the matrix (set all bits)
 *  P         - Dump the led_line[] array values
 *  C<column> - Set a column to the bitmask given by 'V' (Units 0-3 in portrait layout)
 *  R<row>    - Set a row to the bitmask given by 'V' (Units 0-3 in landscape layout)
 *  X<pos>    - X index of an LED to set or toggle
 *  Y<pos>    - Y index of an LED to set or toggle
 *  V<value>  - LED on/off state or row/column bitmask (8, 16, 24, or 32-bits)
 *              ('C' / 'R' can be used to update up to 4 units at once)
 *
 * Directly set a native matrix row to the 8-bit value 'V':
 *  D<line>   - Display line (0..7)
 *  U<unit>   - Unit index (0..MAX7219_NUMBER_UNITS-1)
 */
void GcodeSuite::M7219() {
  if (parser.seen('I')) {
    max7219.register_setup();
    max7219.clear();
  }

  if (parser.seen('F')) max7219.fill();

  const uint32_t v = parser.ulongval('V');

  if (parser.seenval('R')) {
    const uint8_t r = parser.value_byte();
    max7219.set_row(r, v);
  }
  else if (parser.seenval('C')) {
    const uint8_t c = parser.value_byte();
    max7219.set_column(c, v);
  }
  else if (parser.seenval('X') || parser.seenval('Y')) {
    const uint8_t x = parser.byteval('X'), y = parser.byteval('Y');
    if (parser.seenval('V'))
      max7219.led_set(x, y, v > 0);
    else
      max7219.led_toggle(x, y);
  }
  else if (parser.seen('D')) {
    const uint8_t uline = parser.value_byte() & 0x7,
                  line = uline + (parser.byteval('U') << 3);
    if (line < MAX7219_LINES) {
      max7219.led_line[line] = v;
      return max7219.refresh_line(line);
    }
  }

  if (parser.seen('P')) {
    for (uint_fast8_t r = 0; r < MAX7219_LINES; ++r) {
      SERIAL_ECHOPGM("led_line[");
      if (r < 10) SERIAL_CHAR(' ');
      SERIAL_ECHO(r);
      SERIAL_ECHOPGM("]=");
      for (uint_fast8_t b = 8; b--;) SERIAL_CHAR('0' + TEST(max7219.led_line[r], b));
      SERIAL_EOL();
    }
  }
}

#endif // MAX7219_GCODE
