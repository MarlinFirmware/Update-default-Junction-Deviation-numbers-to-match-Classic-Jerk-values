/************************
 * cyrillic_char_set.cpp *
 ************************/

/****************************************************************************
 *   Written By Kirill Shashlov 2020                                        *
 *              Marcio Teixeira 2019 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#include "../ftdi_extended.h"

#if ALL(FTDI_EXTENDED, TOUCH_UI_USE_UTF8, TOUCH_UI_UTF8_CYRILLIC_CHARSET)

  #include "cyrillic_char_set_bitmap_31.h"

  #define NUM_ELEMENTS(a) (sizeof(a)/sizeof(a[0]))

  #define UTF8(A) uint16_t(utf8(U##A))

  using namespace FTDI;

  constexpr static uint8_t cyrillic_font_handle = 6;

  uint32_t FTDI::CyrillicCharSet::bitmap_addr;

  /**
   * Load bitmap data into RAMG. This function is called once at the start
   * of the program.
   *
   * Parameters:
   *
   *   addr  - Address in RAMG where the font data is written
   *
   * Returns: Last wrote address
   */

  uint32_t FTDI::CyrillicCharSet::load_data(uint32_t addr) {
    if (addr % 4 != 0)
      addr += 4 - (addr % 4);

    // Load the alternative font metrics
    CLCD::FontMetrics cyrillic_fm;
    cyrillic_fm.ptr    = addr + 148;
    cyrillic_fm.format = L4;
    cyrillic_fm.stride = 20;
    cyrillic_fm.width  = 40;
    cyrillic_fm.height = 49;
    for (uint_fast8_t i = 0; i < 127; ++i)
      cyrillic_fm.char_widths[i] = 0;

    // For cyrillic characters, copy the character widths from the widths tables
    for (uint_fast8_t i = 0; i < NUM_ELEMENTS(cyrillic_font_widths); ++i) {
      cyrillic_fm.char_widths[i] = cyrillic_font_widths[i];
    }
    CLCD::mem_write_bulk(addr, &cyrillic_fm,  148);

    // Decode the RLE data and load it into RAMG as a bitmap
    uint32_t lastaddr = write_rle_data(addr + 148, cyrillic_font, sizeof(cyrillic_font));

    bitmap_addr = addr;

    return lastaddr;
  }

  /**
   * Populates the bitmap handles for the custom into the display list.
   * This function is called once at the start of each display list.
   *
   * Parameters:
   *
   *   cmd  - Object used for writing to the FTDI chip command queue.
   */

  void FTDI::CyrillicCharSet::load_bitmaps(CommandProcessor& cmd) {
    CLCD::FontMetrics cyrillic_fm;
    cyrillic_fm.ptr    = bitmap_addr + 148;
    cyrillic_fm.format = L4;
    cyrillic_fm.stride = 20;
    cyrillic_fm.width  = 40;
    cyrillic_fm.height = 49;
    set_font_bitmap(cmd, cyrillic_fm, cyrillic_font_handle);
  }

  /**
   * Renders a character at location x and y. The x position is incremented
   * by the width of the character.
   *
   * Parameters:
   *
   *   cmd  - If non-NULL the symbol is drawn to the screen.
   *          If NULL, only increment position for text measurement.
   *
   *   x, y - The location at which to draw the character. On output,
   *          incremented to the location of the next character.
   *
   *   fs   - A scaling object used to scale the font. The display will
   *          already be configured to scale bitmaps, but positions
   *          must be scaled using fs.scale()
   *
   *   c    - The unicode code point to draw. If the renderer does not
   *          support the character, it should return false.
   *
   * Returns: Whether the character was supported.
   */

  bool FTDI::CyrillicCharSet::render_glyph(CommandProcessor* cmd, int &x, int &y, font_size_t fs, utf8_char_t c) {
    // A supported character?
    if ((c < UTF8('А') || c > UTF8('я')) && (c != UTF8('Ё')) && (c != UTF8('ё'))) return false;

    uint8_t idx = (c == UTF8('Ё')) ? 64 :
                  (c == UTF8('ё')) ? 65 :
                  (c < UTF8('р'))  ? c - UTF8('А') :
                                     c - UTF8('р') + 48
                  ;

    uint8_t width = cyrillic_font_widths[idx];

    // Draw the character
    if (cmd) ext_vertex2ii(*cmd, x, y, cyrillic_font_handle, idx);

    // Increment X to the next character position
    x += fs.scale(width);
    return true;
  }

#endif // FTDI_EXTENDED && TOUCH_UI_USE_UTF8 && TOUCH_UI_UTF8_WESTERN_CHARSET
