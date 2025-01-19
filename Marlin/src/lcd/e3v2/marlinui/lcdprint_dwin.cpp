/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * lcd/e3v2/marlinui/lcdprint_dwin.cpp
 *
 * Due to DWIN hardware limitations simplified characters are used
 */

#include "../../../inc/MarlinConfigPre.h"

#if IS_DWIN_MARLINUI

#include "lcdprint_dwin.h"
#include "dwin_lcd.h"
#include "dwin_string.h"

#include "../../marlinui.h"
#include "../../../MarlinCore.h"

#include <string.h>

cursor_t cursor;

extern dwin_font_t dwin_font;

void lcd_moveto_xy(const lcd_uint_t x, const lcd_uint_t y) { cursor.x = x; cursor.y = y; }

void lcd_moveto(const lcd_uint_t col, const lcd_uint_t row) {
  lcd_moveto_xy(col * dwin_font.width, row * (dwin_font.height + EXTRA_ROW_HEIGHT) + EXTRA_ROW_HEIGHT / 2);
}

inline void lcd_advance_cursor(const uint8_t len=1) { cursor.x += len * dwin_font.width; }

void lcd_put_int(const int i) {
  char buf[12]; // 10 digits + sign + null
  itoa(i, buf, 10);
  lcd_put_u8str_max(buf, PIXEL_LEN_NOLIMIT);
}

int lcd_put_dwin_string() {
  dwinDrawString(dwin_font.solid, dwin_font.index, dwin_font.fg, dwin_font.bg, cursor.x, cursor.y, dwin_string.string());
  lcd_advance_cursor(dwin_string.length);
  return dwin_string.length;
}

// return < 0 on error
// return the advanced cols
int lcd_put_lchar_max(const lchar_t &c, const pixel_len_t max_length) {
  dwin_string.set(c);
  dwin_string.truncate(max_length);
  // Draw the char(s) at the cursor and advance the cursor
  dwinDrawString(dwin_font.solid, dwin_font.index, dwin_font.fg, dwin_font.bg, cursor.x, cursor.y, dwin_string.string());
  lcd_advance_cursor(dwin_string.length);
  return dwin_string.length;
}

/**
 * @brief Draw a UTF-8 string
 *
 * @param utf8_str : the UTF-8 string
 * @param cb_read_byte : the callback function to read one byte from the utf8_str (from RAM or ROM)
 * @param max_length : the pixel length of the string allowed (or number of slots in HD44780)
 *
 * @return the number of pixels advanced
 *
 * Draw a UTF-8 string
 */
static int lcd_put_u8str_max_cb(const char * utf8_str, read_byte_cb_t cb_read_byte, const pixel_len_t max_length) {
  const uint8_t *p = (uint8_t *)utf8_str;
  dwin_string.set();
  while (dwin_string.length < max_length) {
    lchar_t wc;
    p = get_utf8_value_cb(p, cb_read_byte, wc);
    if (!wc) break;
    dwin_string.add(wc);
  }
  dwinDrawString(dwin_font.solid, dwin_font.index, dwin_font.fg, dwin_font.bg, cursor.x, cursor.y, dwin_string.string());
  lcd_advance_cursor(dwin_string.length);
  return dwin_string.length;
}

int lcd_put_u8str_max(const char * utf8_str, const pixel_len_t max_length) {
  return lcd_put_u8str_max_cb(utf8_str, read_byte_ram, max_length);
}

int lcd_put_u8str_max_P(PGM_P utf8_pstr, const pixel_len_t max_length) {
  return lcd_put_u8str_max_cb(utf8_pstr, read_byte_rom, max_length);
}

lcd_uint_t lcd_put_u8str_P(PGM_P const ptpl, const int8_t ind, const char * const cstr/*=nullptr*/, FSTR_P const fstr/*=nullptr*/, const lcd_uint_t maxlen/*=LCD_WIDTH*/) {
  dwin_string.set(ptpl, ind, cstr, fstr);
  dwin_string.truncate(maxlen);
  dwinDrawString(dwin_font.solid, dwin_font.index, dwin_font.fg, dwin_font.bg, cursor.x, cursor.y, dwin_string.string());
  lcd_advance_cursor(dwin_string.length);
  return dwin_string.length;
}

#if ENABLED(DEBUG_LCDPRINT)

  int test_dwin_charmap(dwin_charmap_t *data, size_t size, char *name, char flg_show_contents) {
    int ret;
    size_t idx = 0;
    dwin_charmap_t preval = { 0, 0, 0 };
    dwin_charmap_t pinval = { 0, 0, 0 };
    char flg_error = 0;

    int i;

    TRACE("Test %s\n", name);

    for (i = 0; i < size; i ++) {
      memcpy_P(&pinval, &(data[i]), sizeof(pinval));

      if (flg_show_contents) {
        #if 1
          TRACE("[% 4d] % 6" PRIu32 "(0x%04" PRIX32 ") --> 0x%02X,0x%02X%s\n", i, pinval.uchar, pinval.uchar, (unsigned int)(pinval.idx), (unsigned int)(pinval.idx2), (preval.uchar < pinval.uchar?"":" <--- ERROR"));
        #else
          TRACE("[% 4d]", i);
          TRACE("% 6" PRIu32 "(0x%04" PRIX32 "),", pinval.uchar, pinval.uchar);
          TRACE("0x%02X,", (unsigned int)(pinval.idx));
          TRACE("0x%02X,", (unsigned int)(pinval.idx2));
          TRACE("%s", (preval.uchar < pinval.uchar?"":" <--- ERROR"));
        #endif
      }
      if (preval.uchar >= pinval.uchar) {
        flg_error = 1;
        //TRACE("Error: out of order in array %s: idx=%d, val=%d(0x%x)\n", name, i, pinval.uchar, pinval.uchar);
        //return -1;
      }
      memcpy(&preval, &pinval, sizeof(pinval));

      ret = pf_bsearch_r((void *)data, size, pf_bsearch_cb_comp_dwinmap_pgm, (void *)&pinval, &idx);
      if (ret < 0) {
        flg_error = 1;
        TRACE("Error: not found item in array %s: idx=%d, val=%d(0x%x)\n", name, i, pinval.uchar, pinval.uchar);
        //return -1;
      }
      if (idx != i) {
        flg_error = 1;
        TRACE("Error: wrong index found item in array %s: idx=%d, val=%d(0x%x)\n", name, i, pinval.uchar, pinval.uchar);
        //return -1;
      }
    }
    if (flg_error) {
      TRACE("\nError: in array %s\n\n", name);
      return -1;
    }
    TRACE("\nPASS array %s\n\n", name);
    return 0;
  }

  int test_dwin_charmap_all() {
    int flg_error = 0;
    if (test_dwin_charmap(g_dwin_charmap_device, COUNT(g_dwin_charmap_device), "g_dwin_charmap_device", 0) < 0) {
      flg_error = 1;
      test_dwin_charmap(g_dwin_charmap_device, COUNT(g_dwin_charmap_device), "g_dwin_charmap_device", 1);
    }
    if (test_dwin_charmap(g_dwin_charmap_common, COUNT(g_dwin_charmap_common), "g_dwin_charmap_common", 0) < 0) {
      flg_error = 1;
      test_dwin_charmap(g_dwin_charmap_common, COUNT(g_dwin_charmap_common), "g_dwin_charmap_common", 1);
    }
    if (flg_error) {
      TRACE("\nFAILED in dwin tests!\n");
      return -1;
    }
    TRACE("\nPASS in dwin tests.\n");
    return 0;
  }

#endif // DEBUG_LCDPRINT

#endif // IS_DWIN_MARLINUI
