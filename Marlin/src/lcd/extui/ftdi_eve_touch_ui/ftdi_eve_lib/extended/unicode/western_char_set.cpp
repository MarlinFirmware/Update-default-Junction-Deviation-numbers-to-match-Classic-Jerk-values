/************************
 * western_char_set.cpp *
 ************************/

/****************************************************************************
 *   Written By Marcio Teixeira 2019 - Aleph Objects, Inc.                  *
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

#if ALL(FTDI_EXTENDED, TOUCH_UI_USE_UTF8, TOUCH_UI_UTF8_WESTERN_CHARSET)

  #include "western_char_set_bitmap_31.h"

  #define NUM_ELEMENTS(a) (sizeof(a)/sizeof(a[0]))

  using namespace FTDI;

  constexpr static uint8_t std_font = 31;
  constexpr static uint8_t alt_font = 1;

  uint32_t FTDI::WesternCharSet::bitmap_addr;

  /* Glyphs in the WesternCharSet bitmap */

  enum {
    GRAVE,
    ACUTE,
    CIRCUMFLEX,
    TILDE,
    DIAERESIS,
    DOT_ABOVE,
    CEDILLA,
    NO_DOT_I,
    #if ENABLED(TOUCH_UI_UTF8_GERMANIC)
      SHARP_S,
    #endif
    #if ENABLED(TOUCH_UI_UTF8_SCANDINAVIAN)
      LRG_O_STROKE,
      SML_O_STROKE,
      LRG_AE,
      SML_AE,
      LRG_ETH,
      SML_ETH,
      LRG_THORN,
      SML_THORN,
    #endif
    #if ENABLED(TOUCH_UI_UTF8_PUNCTUATION)
      LEFT_DBL_QUOTE,
      RIGHT_DBL_QUOTE,
      INV_EXCLAMATION,
      INV_QUESTION,
    #endif
    #if ENABLED(TOUCH_UI_UTF8_CURRENCY)
      CENT_SIGN,
      POUND_SIGN,
      CURRENCY_SIGN,
      YEN_SIGN,
    #endif
    #if ENABLED(TOUCH_UI_UTF8_SUPERSCRIPTS)
      SUPERSCRIPT_1,
      SUPERSCRIPT_2,
      SUPERSCRIPT_3,
    #endif
    #if ENABLED(TOUCH_UI_UTF8_ORDINALS)
      MASCULINE_ORDINAL,
      FEMININE_ORDINAL,
    #endif
    #if ENABLED(TOUCH_UI_UTF8_COPYRIGHT)
      COPYRIGHT_SIGN,
      REGISTERED_SIGN,
    #endif
    #if ENABLED(TOUCH_UI_UTF8_MATHEMATICS)
      PLUS_MINUS_SIGN,
      MULTIPLICATION_SIGN,
      DIVISION_SIGN,
    #endif
    #if ENABLED(TOUCH_UI_UTF8_FRACTIONS)
      FRACTION_QUARTER,
      FRACTION_HALF,
      FRACTION_THREE_FOURTHS,
    #endif
    #if ENABLED(TOUCH_UI_UTF8_SYMBOLS)
      MICRON_SIGN,
      PILCROW_SIGN,
      BROKEN_BAR,
      SECTION_SIGN,
      NOT_SIGN
    #endif
  };

  /* Centerline of characters that can take accents */

  constexpr int8_t mid_a = 12,
                   mid_e = 12,
                   mid_i = 5,
                   mid_o = 12,
                   mid_u = 12,
                   mid_y = 11,
                   mid_n = 12,
                   mid_c = 12,
                   mid_A = 13,
                   mid_E = 13,
                   mid_I = 6,
                   mid_O = 14,
                   mid_U = 14,
                   mid_Y = 13,
                   mid_N = 15,
                   mid_C = 13;

  /* Centerline of accent glyphs */

  constexpr int8_t mid_accent = 16;

  /* When reusing the DOT_ABOVE accent glyph for the degree sign, we need to trim the leading space */
  constexpr uint8_t deg_sign_leading = 9;

  /* Look-up table for constructing characters (must be ordered by unicode)
   *
   * Characters are either complete symbols from the Western Char Set bitmap,
   * or they are constructed using a standard letter from the romfont and
   * drawing an accent from the Western Char Set bitmap over it.
   */

  #define UTF8(A) uint16_t(utf8(U##A))

  PROGMEM constexpr struct {
    uint16_t unicode;
    uint8_t  std_char; // Glyph from standard ROMFONT (zero if none)
    uint8_t  alt_char; // Glyph from Western Char Set bitmap
    uint8_t  alt_data; // For accented characters, the centerline; else char width
  } char_recipe[] = {
    {0,          0,  NO_DOT_I,           10   },
    #if ENABLED(TOUCH_UI_UTF8_PUNCTUATION)
      {UTF8('¡'),  0 , INV_EXCLAMATION,    13   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_CURRENCY)
      {UTF8('¢'),  0 , CENT_SIGN,          23   },
      {UTF8('£'),  0 , POUND_SIGN,         24   },
      {UTF8('¤'),  0 , CURRENCY_SIGN,      26   },
      {UTF8('¥'),  0 , YEN_SIGN,           26   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_SYMBOLS)
      {UTF8('¦'),  0 , BROKEN_BAR,         11   },
      {UTF8('§'),  0 , SECTION_SIGN,       21   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_COPYRIGHT)
      {UTF8('©'),  0 , COPYRIGHT_SIGN,     38   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_ORDINALS)
      {UTF8('ª'),  0 , FEMININE_ORDINAL,   19   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_PUNCTUATION)
      {UTF8('«'),  0 , LEFT_DBL_QUOTE,     23   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_SYMBOLS)
      {UTF8('¬'),  0 , NOT_SIGN,           32   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_COPYRIGHT)
      {UTF8('®'),  0 , REGISTERED_SIGN,    38   },
    #endif
      {UTF8('°'),  0 , DOT_ABOVE,          24   },
    #if ENABLED(TOUCH_UI_UTF8_MATHEMATICS)
      {UTF8('±'),  0 , NOT_SIGN,           32   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_SUPERSCRIPTS)
      {UTF8('²'),  0 , SUPERSCRIPT_2,    16   },
      {UTF8('³'),  0 , SUPERSCRIPT_3,  16   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_SYMBOLS)
      {UTF8('µ'),  0 , MICRON_SIGN,        28   },
      {UTF8('¶'),  0 , PILCROW_SIGN,       24   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_SUPERSCRIPTS)
      {UTF8('¹'),  0 , SUPERSCRIPT_1,    16   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_ORDINALS)
      {UTF8('º'),  0 , MASCULINE_ORDINAL,  19   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_PUNCTUATION)
      {UTF8('»'),  0 , RIGHT_DBL_QUOTE,    24   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_FRACTIONS)
      {UTF8('¼'),  0 , FRACTION_QUARTER,   40   },
      {UTF8('½'),  0 , FRACTION_HALF,      40   },
      {UTF8('¾'),  0 , FRACTION_THREE_FOURTHS, 40 },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_PUNCTUATION)
      {UTF8('¿'),  0 , INV_QUESTION,       21   },
    #endif
      {UTF8('À'), 'A', GRAVE,              mid_A},
      {UTF8('Á'), 'A', ACUTE,              mid_A},
      {UTF8('Â'), 'A', CIRCUMFLEX,         mid_A},
      {UTF8('Ã'), 'A', TILDE,              mid_A},
      {UTF8('Ä'), 'A', DIAERESIS,          mid_A},
      {UTF8('Å'), 'A', DOT_ABOVE,          mid_A},
    #if ENABLED(TOUCH_UI_UTF8_SCANDINAVIAN)
      {UTF8('Æ'),  0 , LRG_AE,             40},
    #endif
      {UTF8('Ç'), 'C', CEDILLA,            mid_C},
      {UTF8('È'), 'E', GRAVE,              mid_E},
      {UTF8('É'), 'E', ACUTE,              mid_E},
      {UTF8('Ê'), 'E', CIRCUMFLEX,         mid_E},
      {UTF8('Ë'), 'E', DIAERESIS,          mid_E},
      {UTF8('Ì'), 'I', GRAVE,              mid_I},
      {UTF8('Í'), 'I', ACUTE,              mid_I},
      {UTF8('Î'), 'I', CIRCUMFLEX,         mid_I},
      {UTF8('Ï'), 'I', DIAERESIS,          mid_I},
    #if ENABLED(TOUCH_UI_UTF8_SCANDINAVIAN)
      {UTF8('Ð'),  0,  LRG_ETH,            31   },
    #endif
      {UTF8('Ñ'), 'N', TILDE,              mid_N},
      {UTF8('Ò'), 'O', GRAVE,              mid_O},
      {UTF8('Ó'), 'O', ACUTE,              mid_O},
      {UTF8('Ô'), 'O', CIRCUMFLEX,         mid_O},
      {UTF8('Õ'), 'O', TILDE,              mid_O},
      {UTF8('Ö'), 'O', DIAERESIS,          mid_O},
    #if ENABLED(TOUCH_UI_UTF8_MATHEMATICS)
      {UTF8('×'),  0 , MULTIPLICATION_SIGN, 32 },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_SCANDINAVIAN)
      {UTF8('Ø'),  0 , LRG_O_STROKE,       32   },
    #endif
      {UTF8('Ù'), 'U', GRAVE,              mid_U},
      {UTF8('Ú'), 'U', ACUTE,              mid_U},
      {UTF8('Û'), 'U', CIRCUMFLEX,         mid_U},
      {UTF8('Ü'), 'U', DIAERESIS,          mid_U},
      {UTF8('Ý'), 'Y', ACUTE,              mid_Y},
    #if ENABLED(TOUCH_UI_UTF8_SCANDINAVIAN)
      {UTF8('Þ'),  0 , LRG_THORN,          25   },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_GERMANIC)
      {UTF8('ß'),  0 , SHARP_S,            26   },
    #endif
      {UTF8('à'), 'a', GRAVE,              mid_a},
      {UTF8('á'), 'a', ACUTE,              mid_a},
      {UTF8('â'), 'a', CIRCUMFLEX,         mid_a},
      {UTF8('ã'), 'a', TILDE,              mid_a},
      {UTF8('ä'), 'a', DIAERESIS,          mid_a},
      {UTF8('å'), 'a', DOT_ABOVE,          mid_a},
    #if ENABLED(TOUCH_UI_UTF8_SCANDINAVIAN)
      {UTF8('æ'),  0 , SML_AE,             40   },
    #endif
      {UTF8('ç'), 'c', CEDILLA,            mid_c},
      {UTF8('è'), 'e', GRAVE,              mid_e},
      {UTF8('é'), 'e', ACUTE,              mid_e},
      {UTF8('ê'), 'e', CIRCUMFLEX,         mid_e},
      {UTF8('ë'), 'e', DIAERESIS,          mid_e},
      {UTF8('ì'), 'i', GRAVE,              mid_i},
      {UTF8('í'), 'i', ACUTE,              mid_i},
      {UTF8('î'), 'i', CIRCUMFLEX,         mid_i},
      {UTF8('ï'), 'i', DIAERESIS,          mid_i},
    #if ENABLED(TOUCH_UI_UTF8_SCANDINAVIAN)
      {UTF8('ð'),  0,  SML_ETH,            24   },
    #endif
      {UTF8('ñ'), 'n', TILDE,              mid_n},
      {UTF8('ò'), 'o', GRAVE,              mid_o},
      {UTF8('ó'), 'o', ACUTE,              mid_o},
      {UTF8('ô'), 'o', CIRCUMFLEX,         mid_o},
      {UTF8('õ'), 'o', TILDE,              mid_o},
      {UTF8('ö'), 'o', DIAERESIS,          mid_o},
    #if ENABLED(TOUCH_UI_UTF8_MATHEMATICS)
      {UTF8('÷'),  0 , DIVISION_SIGN,      32 },
    #endif
    #if ENABLED(TOUCH_UI_UTF8_SCANDINAVIAN)
      {UTF8('ø'),  0 , SML_O_STROKE,       25   },
    #endif
      {UTF8('ù'), 'u', GRAVE,              mid_u},
      {UTF8('ú'), 'u', ACUTE,              mid_u},
      {UTF8('û'), 'u', CIRCUMFLEX,         mid_u},
      {UTF8('ü'), 'u', DIAERESIS,          mid_u},
      {UTF8('ý'), 'y', ACUTE,              mid_y},
    #if ENABLED(TOUCH_UI_UTF8_SCANDINAVIAN)
      {UTF8('þ'),  0 , SML_THORN,          25   },
    #endif
    {UTF8('ÿ'), 'y', DIAERESIS,          mid_y},
  };

  static_assert(UTF8('¡') == 0xC2A1, "Incorrect encoding for character");

  /* Compile-time check that the table is in sorted order */

  constexpr bool is_sorted(size_t n) {
    return n < 2 ? true : char_recipe[n-2].unicode < char_recipe[n-1].unicode && is_sorted(n-1);
  }

  static_assert(is_sorted(NUM_ELEMENTS(char_recipe)), "The table must be sorted by unicode value");

  /* Performs a binary search to find a unicode character in the table */

  static int8_t find_char_data(FTDI::utf8_char_t c) {
    int8_t min = 0, max = NUM_ELEMENTS(char_recipe), index;
    for (;;) {
      index = (min + max)/2;
      const uint16_t char_at = pgm_read_word(&char_recipe[index].unicode);
      if (char_at == c) break;
      if (min == max) return -1;
      if (c > char_at)
        min = index + 1;
      else
        max = index;
    }
    return index;
  }

  static void get_char_data(uint8_t index, uint8_t &std_char, uint8_t &alt_char, uint8_t &alt_data) {
    std_char = pgm_read_byte(&char_recipe[index].std_char);
    alt_char = pgm_read_byte(&char_recipe[index].alt_char);
    alt_data = pgm_read_byte(&char_recipe[index].alt_data);
  }

  /**
   * Load bitmap data into RAMG. This function is called once at the start
   * of the program.
   *
   * Parameters:
   *
   *   addr  - Address in RAMG where the font data is written
   */

  uint32_t FTDI::WesternCharSet::load_data(uint32_t addr) {
    if (addr % 4 != 0)
      addr += 4 - (addr % 4);

    // Load the alternative font metrics
    CLCD::FontMetrics alt_fm;
    alt_fm.ptr    = addr + 148;
    alt_fm.format = L4;
    alt_fm.stride = 19;
    alt_fm.width  = 38;
    alt_fm.height = 49;
    for (uint_fast8_t i = 0; i < 127; ++i)
      alt_fm.char_widths[i] = 0;

    // For special characters, copy the character widths from the char tables
    for (uint_fast8_t i = 0; i < NUM_ELEMENTS(char_recipe); ++i) {
      uint8_t std_char, alt_char, alt_data;
      get_char_data(i, std_char, alt_char, alt_data);
      if (std_char == 0)
        alt_fm.char_widths[alt_char] = alt_data;
    }
    CLCD::mem_write_bulk(addr, &alt_fm,  148);

    // Decode the RLE data and load it into RAMG as a bitmap
    uint32_t lastaddr = write_rle_data(addr + 148, font, sizeof(font));

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

  void FTDI::WesternCharSet::load_bitmaps(CommandProcessor& cmd) {
    CLCD::FontMetrics alt_fm;
    alt_fm.ptr    = bitmap_addr + 148;
    alt_fm.format = L4;
    alt_fm.stride = 19;
    alt_fm.width  = 38;
    alt_fm.height = 49;
    set_font_bitmap(cmd, alt_fm, alt_font);
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

  bool FTDI::WesternCharSet::render_glyph(CommandProcessor* cmd, int &x, int &y, font_size_t fs, utf8_char_t c) {

    // A supported character?
    if (c < UTF8('¡') || c > UTF8('ÿ')) return false;

    int8_t index = find_char_data(c);
    if (index == -1) return false;

    // Determine character characteristics
    uint8_t std_char, alt_char, alt_data;
    get_char_data(index, std_char, alt_char, alt_data);

    bool    base_special;
    uint8_t base_width;
    uint8_t base_char;
    uint8_t accent_char;
    int8_t  accent_dx, accent_dy;

    if (std_char == 0) {
      // Special character, non-accented
      base_width   = alt_data;
      base_special = true;
      base_char    = alt_char;
      accent_char  = 0;
      if (c == UTF8('°'))
        x -= fs.scale(deg_sign_leading);
    }
    else {
      // Regular character with accent:
      accent_dx   = alt_data - mid_accent;
      accent_dy   = isupper(std_char) ? -7 : 0;
      accent_char = alt_char;
      base_width  = StandardCharSet::std_char_width(std_char);
      base_special = std_char == 'i';
      base_char   = base_special ? NO_DOT_I : std_char;
    }

    // If cmd != nullptr, draw the glyph to the screen
    if (cmd) {
      ext_vertex2ii(*cmd, x, y, base_special ? alt_font : std_font, base_char);
      if (accent_char)
        ext_vertex2ii(*cmd, x + fs.scale(accent_dx), y + fs.scale(accent_dy), alt_font, accent_char);
    }

    // Increment X to the next character position
    x += fs.scale(base_width);
    return true;
  }

#endif // FTDI_EXTENDED && TOUCH_UI_USE_UTF8 && TOUCH_UI_UTF8_WESTERN_CHARSET
