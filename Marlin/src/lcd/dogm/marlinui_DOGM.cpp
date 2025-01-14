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

/**
 * lcd/dogm/marlinui_DOGM.h
 *
 * Implementation of the LCD display routines for a DOGM128 graphic display.
 * by STB for ErikZalm/Marlin. Common LCD 128x64 pixel graphic displays.
 *
 * Demonstrator: https://www.reprap.org/wiki/STB_Electronics
 * License: https://opensource.org/licenses/BSD-3-Clause
 *
 * With the use of:
 *  u8glib by Oliver Kraus
 *  https://github.com/olikraus/U8glib_Arduino
 *  License: https://opensource.org/licenses/BSD-3-Clause
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_U8GLIB

#include "marlinui_DOGM.h"
#include "u8g_fontutf8.h"

#if ENABLED(SHOW_BOOTSCREEN)
  #include "dogm_Bootscreen.h"
#endif

#include "../lcdprint.h"
#include "../../libs/numtostr.h"
#include "../marlinui.h"

#include "../../sd/cardreader.h"
#include "../../module/temperature.h"
#include "../../module/printcounter.h"
#include "../../MarlinCore.h"

#if HAS_MEDIA
  #include "../../libs/duration_t.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "../../feature/bedlevel/bedlevel.h"
#endif

/**
 * Include all needed font files
 * (See https://marlinfw.org/docs/development/fonts.html)
 */
#include "fontdata/fontdata_ISO10646_1.h"
#if ENABLED(USE_SMALL_INFOFONT)
  #include "fontdata/fontdata_6x9_marlin.h"
  #define FONT_STATUSMENU_NAME u8g_font_6x9
#else
  #define FONT_STATUSMENU_NAME MENU_FONT_NAME
#endif

U8G_CLASS u8g;

#include LANGUAGE_DATA_INCL(LCD_LANGUAGE)
#ifdef LCD_LANGUAGE_2
  #include LANGUAGE_DATA_INCL(LCD_LANGUAGE_2)
#endif
#ifdef LCD_LANGUAGE_3
  #include LANGUAGE_DATA_INCL(LCD_LANGUAGE_3)
#endif
#ifdef LCD_LANGUAGE_4
  #include LANGUAGE_DATA_INCL(LCD_LANGUAGE_4)
#endif
#ifdef LCD_LANGUAGE_5
  #include LANGUAGE_DATA_INCL(LCD_LANGUAGE_5)
#endif

#if HAS_LCD_CONTRAST
  void MarlinUI::_set_contrast() { u8g.setContrast(contrast); }
#endif

void MarlinUI::set_font(const MarlinFont font_nr) {
  static char currentfont = 0;
  if (font_nr != currentfont) {
    switch ((currentfont = font_nr)) {
      case FONT_STATUSMENU : u8g.setFont(FONT_STATUSMENU_NAME); break;
      case FONT_EDIT       : u8g.setFont(EDIT_FONT_NAME);       break;
      default:
      case FONT_MENU       : u8g.setFont(MENU_FONT_NAME);       break;
    }
  }
}

bool MarlinUI::detected() { return true; }

#if ENABLED(SHOW_BOOTSCREEN)

  #if ENABLED(SHOW_CUSTOM_BOOTSCREEN)
    // Draws a slice of a particular frame of the custom bootscreen, without the u8g loop
    void MarlinUI::draw_custom_bootscreen(const uint8_t frame/*=0*/) {
      constexpr u8g_uint_t left = u8g_uint_t((LCD_PIXEL_WIDTH  - (CUSTOM_BOOTSCREEN_BMPWIDTH)) / 2),
                            top = u8g_uint_t(CUSTOM_BOOTSCREEN_Y);
      #if ENABLED(CUSTOM_BOOTSCREEN_INVERTED)
        constexpr u8g_uint_t right = left + CUSTOM_BOOTSCREEN_BMPWIDTH,
                            bottom = top + CUSTOM_BOOTSCREEN_BMPHEIGHT;
      #endif

      #if ENABLED(CUSTOM_BOOTSCREEN_ANIMATED)
        #if ENABLED(CUSTOM_BOOTSCREEN_ANIMATED_FRAME_TIME)
          const u8g_pgm_uint8_t * const bmp = (u8g_pgm_uint8_t*)pgm_read_ptr(&custom_bootscreen_animation[frame].bitmap);
        #else
          const u8g_pgm_uint8_t * const bmp = (u8g_pgm_uint8_t*)pgm_read_ptr(&custom_bootscreen_animation[frame]);
        #endif
      #elif ANY(COMPACT_CUSTOM_BOOTSCREEN, COMPACT_CUSTOM_BOOTSCREEN_EXT)
        #define BMPSIZE (CUSTOM_BOOTSCREEN_BMP_BYTEWIDTH * CUSTOM_BOOTSCREEN_BMPHEIGHT)
        uint8_t bmp[BMPSIZE];
        uint8_t *bmp_rle = (uint8_t*)custom_start_bmp_rle;
      #else
        const u8g_pgm_uint8_t * const bmp = custom_start_bmp;
      #endif

      #if ANY(COMPACT_CUSTOM_BOOTSCREEN, COMPACT_CUSTOM_BOOTSCREEN_EXT)

        uint8_t *dst = (uint8_t*)bmp;

        auto rle_nybble = [&](const uint16_t i) -> uint8_t {
          const uint8_t b = pgm_read_byte(&bmp_rle[i / 2]);
          return (i & 1 ? b & 0xF : b >> 4);
        };

        uint8_t workbyte = 0, bitstate = rle_nybble(0) << 7;
        uint16_t inindex = 1, outindex = 0;
        while (outindex < BMPSIZE * 8) {
          int16_t c = rle_nybble(inindex++);
          if (c == 15) {
            const uint8_t d = rle_nybble(inindex++), e = rle_nybble(inindex++);
            #if ENABLED(COMPACT_CUSTOM_BOOTSCREEN_EXT)
              if (d == 15)
                c = 256 + 16 * e + rle_nybble(inindex++) - 1;
              else
            #endif
                c = 16 * d + e + 15; // From 16 to 270
          }
          while (c-- >= 0) {
            const uint8_t bitind = outindex & 7,
                          bitval = bitstate >> bitind;
            workbyte |= bitval;
            if (bitind == 7) { *dst++ = workbyte; workbyte = 0; }
            outindex++;
          }
          bitstate ^= 0x80;
        }

      #endif // COMPACT_CUSTOM_BOOTSCREEN || COMPACT_CUSTOM_BOOTSCREEN_EXT

      u8g.TERN(COMPACT_CUSTOM_BOOTSCREEN, drawBitmap, drawBitmapP)
        (left, top, CUSTOM_BOOTSCREEN_BMP_BYTEWIDTH, CUSTOM_BOOTSCREEN_BMPHEIGHT, bmp);

      UNUSED(frame);
      #if ENABLED(CUSTOM_BOOTSCREEN_INVERTED)
        if (frame == 0) {
          u8g.setColorIndex(1);
          if (top) u8g.drawBox(0, 0, LCD_PIXEL_WIDTH, top);
          if (left) u8g.drawBox(0, top, left, CUSTOM_BOOTSCREEN_BMPHEIGHT);
          if (right < LCD_PIXEL_WIDTH) u8g.drawBox(right, top, LCD_PIXEL_WIDTH - right, CUSTOM_BOOTSCREEN_BMPHEIGHT);
          if (bottom < LCD_PIXEL_HEIGHT) u8g.drawBox(0, bottom, LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT - bottom);
        }
      #endif
    }

    // Shows the custom bootscreen, with the u8g loop, animations and delays
    void MarlinUI::show_custom_bootscreen() {
      #if DISABLED(CUSTOM_BOOTSCREEN_ANIMATED)
        constexpr millis_t frame_time = 0;
        constexpr uint8_t f = 0;
      #else
        #if DISABLED(CUSTOM_BOOTSCREEN_ANIMATED_FRAME_TIME)
          constexpr millis_t frame_time = CUSTOM_BOOTSCREEN_FRAME_TIME;
        #endif
        for (uint8_t f = 0; f < COUNT(custom_bootscreen_animation); ++f)
      #endif
        {
          #if ENABLED(CUSTOM_BOOTSCREEN_ANIMATED_FRAME_TIME)
            const uint8_t fr = _MIN(f, COUNT(custom_bootscreen_animation) - 1);
            const millis_t frame_time = pgm_read_word(&custom_bootscreen_animation[fr].duration);
          #endif
          u8g.firstPage();
          do { draw_custom_bootscreen(f); } while (u8g.nextPage());
          if (frame_time) safe_delay(frame_time);
        }

      #ifndef CUSTOM_BOOTSCREEN_TIMEOUT
        #define CUSTOM_BOOTSCREEN_TIMEOUT 2500
      #endif
      #if CUSTOM_BOOTSCREEN_TIMEOUT
        safe_delay(CUSTOM_BOOTSCREEN_TIMEOUT);
      #endif
    }
  #endif // SHOW_CUSTOM_BOOTSCREEN

  // Two-part needed to display all info
  constexpr bool two_part = ((LCD_PIXEL_HEIGHT) - (START_BMPHEIGHT)) < ((MENU_FONT_ASCENT) * 2);
  constexpr uint8_t bootscreen_pages = 1 + two_part;

  // Draw the static Marlin bootscreen from a u8g loop
  // or the animated boot screen within its own u8g loop
  void MarlinUI::draw_marlin_bootscreen(const bool line2/*=false*/) {

    // Determine text space needed
    constexpr u8g_uint_t text_width_1 = u8g_uint_t((sizeof(SHORT_BUILD_VERSION) - 1) * (MENU_FONT_WIDTH)),
                         text_width_2 = u8g_uint_t((sizeof(MARLIN_WEBSITE_URL) - 1) * (MENU_FONT_WIDTH)),
                         text_max_width = _MAX(text_width_1, text_width_2),
                         text_total_height = (MENU_FONT_HEIGHT) * 2,
                         width = LCD_PIXEL_WIDTH, height = LCD_PIXEL_HEIGHT,
                         rspace = width - (START_BMPWIDTH);

    u8g_int_t offx, offy, txt_base, txt_offx_1, txt_offx_2;

    // Can the text fit to the right of the bitmap?
    if (text_max_width < rspace) {
      constexpr int8_t inter = (width - text_max_width - (START_BMPWIDTH)) / 3; // Evenly distribute horizontal space
      offx = inter;                             // First the boot logo...
      offy = (height - (START_BMPHEIGHT)) / 2;  // ...V-aligned in the full height
      txt_offx_1 = txt_offx_2 = inter + (START_BMPWIDTH) + inter; // Text right of the bitmap
      txt_base = (height + MENU_FONT_ASCENT + text_total_height - (MENU_FONT_HEIGHT)) / 2; // Text vertical center
    }
    else {
      constexpr int8_t inter = (height - text_total_height - (START_BMPHEIGHT)) / 3; // Evenly distribute vertical space
      offx = rspace / 2;                        // Center the boot logo in the whole space
      offy = inter;                             // V-align boot logo proportionally
      txt_offx_1 = (width - text_width_1) / 2;  // Text 1 centered
      txt_offx_2 = (width - text_width_2) / 2;  // Text 2 centered
      txt_base = offy + START_BMPHEIGHT + offy + text_total_height - (MENU_FONT_DESCENT);   // Even spacing looks best
    }
    NOLESS(offx, 0);
    NOLESS(offy, 0);

    auto _draw_bootscreen_bmp = [&](const uint8_t *bitmap) {
      u8g.drawBitmapP(offx, offy, START_BMP_BYTEWIDTH, START_BMPHEIGHT, bitmap);
      set_font(FONT_MENU);
      if (!two_part || !line2) lcd_put_u8str(txt_offx_1, txt_base - (MENU_FONT_HEIGHT), F(SHORT_BUILD_VERSION));
      if (!two_part || line2) lcd_put_u8str(txt_offx_2, txt_base, F(MARLIN_WEBSITE_URL));
    };

    auto draw_bootscreen_bmp = [&](const uint8_t *bitmap) {
      u8g.firstPage(); do { _draw_bootscreen_bmp(bitmap); } while (u8g.nextPage());
    };

    #if DISABLED(BOOT_MARLIN_LOGO_ANIMATED)
      draw_bootscreen_bmp(start_bmp);
    #else
      constexpr millis_t frame_time = MARLIN_BOOTSCREEN_FRAME_TIME;
      for (uint8_t f = 0; f < COUNT(marlin_bootscreen_animation); ++f) {
        draw_bootscreen_bmp((uint8_t*)pgm_read_ptr(&marlin_bootscreen_animation[f]));
        if (frame_time) safe_delay(frame_time);
      }
    #endif
  }

  // Show the Marlin bootscreen, with the u8g loop and delays
  void MarlinUI::show_marlin_bootscreen() {
    for (uint8_t q = bootscreen_pages; q--;) {
      draw_marlin_bootscreen(q == 0);
      if (q) safe_delay((BOOTSCREEN_TIMEOUT) / bootscreen_pages);
    }
  }

  void MarlinUI::show_bootscreen() {
    TERN_(SHOW_CUSTOM_BOOTSCREEN, show_custom_bootscreen());
    show_marlin_bootscreen();
  }

  void MarlinUI::bootscreen_completion(const millis_t sofar) {
    if ((BOOTSCREEN_TIMEOUT) / bootscreen_pages > sofar) safe_delay((BOOTSCREEN_TIMEOUT) / bootscreen_pages - sofar);
  }

#endif // SHOW_BOOTSCREEN

#if ENABLED(LIGHTWEIGHT_UI)
  #include "status_screen_lite_ST7920.h"
#endif

// Initialize or re-initialize the LCD
void MarlinUI::init_lcd() {

  static bool did_init_u8g = false;
  if (!did_init_u8g) {
    u8g.init(U8G_PARAM);
    did_init_u8g = true;
  }

  #if PIN_EXISTS(LCD_BACKLIGHT)
    OUT_WRITE(LCD_BACKLIGHT_PIN, DISABLED(DELAYED_BACKLIGHT_INIT)); // Illuminate after reset or right away
  #endif

  #if ANY(MKS_12864OLED, MKS_12864OLED_SSD1306, FYSETC_242_OLED_12864, ZONESTAR_12864OLED, K3D_242_OLED_CONTROLLER)
    SET_OUTPUT(LCD_PINS_DC);
  #endif

  #if PIN_EXISTS(LCD_RESET)
    // Perform a clean hardware reset with needed delays
    OUT_WRITE(LCD_RESET_PIN, LOW);
    hal.delay_ms(5);
    WRITE(LCD_RESET_PIN, HIGH);
    hal.delay_ms(5);
    u8g.begin();
  #endif

  #if PIN_EXISTS(LCD_BACKLIGHT) && ENABLED(DELAYED_BACKLIGHT_INIT)
    WRITE(LCD_BACKLIGHT_PIN, HIGH);
  #endif

  TERN_(HAS_LCD_CONTRAST, refresh_contrast());

  #if LCD_SCREEN_ROTATE == 90
    u8g.setRot90();
  #elif LCD_SCREEN_ROTATE == 180
    u8g.setRot180();
  #elif LCD_SCREEN_ROTATE == 270
    u8g.setRot270();
  #endif

  update_language_font();
}

void MarlinUI::update_language_font() {
  #if HAS_MULTI_LANGUAGE
    switch (language) {
      default: uxg_SetUtf8Fonts(LANG_FONT_INFO(LCD_LANGUAGE), COUNT(LANG_FONT_INFO(LCD_LANGUAGE))); break;
      #ifdef LCD_LANGUAGE_2
        case 1: uxg_SetUtf8Fonts(LANG_FONT_INFO(LCD_LANGUAGE_2), COUNT(LANG_FONT_INFO(LCD_LANGUAGE_2))); break;
      #endif
      #ifdef LCD_LANGUAGE_3
        case 2: uxg_SetUtf8Fonts(LANG_FONT_INFO(LCD_LANGUAGE_3), COUNT(LANG_FONT_INFO(LCD_LANGUAGE_3))); break;
      #endif
      #ifdef LCD_LANGUAGE_4
        case 3: uxg_SetUtf8Fonts(LANG_FONT_INFO(LCD_LANGUAGE_4), COUNT(LANG_FONT_INFO(LCD_LANGUAGE_4))); break;
      #endif
      #ifdef LCD_LANGUAGE_5
        case 4: uxg_SetUtf8Fonts(LANG_FONT_INFO(LCD_LANGUAGE_5), COUNT(LANG_FONT_INFO(LCD_LANGUAGE_5))); break;
      #endif
    }
  #else
    uxg_SetUtf8Fonts(LANG_FONT_INFO(LCD_LANGUAGE), COUNT(LANG_FONT_INFO(LCD_LANGUAGE)));
  #endif
}

// The kill screen is displayed for unrecoverable conditions
void MarlinUI::draw_kill_screen() {
  TERN_(LIGHTWEIGHT_UI, ST7920_Lite_Status_Screen::clear_text_buffer());
  const u8g_uint_t x = 0, h4 = u8g.getHeight() / 4;
  u8g.firstPage();
  do {
    set_font(FONT_MENU);
    lcd_put_u8str(x, h4 * 1, status_message);
    lcd_put_u8str(x, h4 * 2, GET_TEXT_F(MSG_HALTED));
    lcd_put_u8str(x, h4 * 3, GET_TEXT_F(MSG_PLEASE_RESET));
  } while (u8g.nextPage());
}

// Erase the LCD contents by drawing an empty box.
void MarlinUI::clear_lcd() {
  u8g.setColorIndex(0);
  u8g.firstPage();
  do {
    u8g.drawBox(0, 0, u8g.getWidth(), u8g.getHeight());
  } while (u8g.nextPage());
  u8g.setColorIndex(1);
}

// U8G displays are drawn over multiple loops so must do their own clearing.
void MarlinUI::clear_for_drawing() {
  // Automatically cleared by Picture Loop
}

#if HAS_DISPLAY_SLEEP
  static bool asleep = false;
  bool MarlinUI::display_is_asleep() { return asleep; }
  void MarlinUI::sleep_display(const bool sleep/*=true*/) {
    if (asleep != sleep) {
      sleep ? u8g.sleepOn() : u8g.sleepOff();
      asleep = sleep;
    }
  }
#endif

#if HAS_LCD_BRIGHTNESS

  void MarlinUI::_set_brightness() {
    #if PIN_EXISTS(TFT_BACKLIGHT)
      if (PWM_PIN(TFT_BACKLIGHT_PIN))
        analogWrite(pin_t(TFT_BACKLIGHT_PIN), backlight ? brightness : 0);
    #endif
  }

#endif

#if HAS_MARLINUI_MENU

  #include "../menu/menu.h"

  u8g_uint_t row_y1, row_y2;

  #if ENABLED(ADVANCED_PAUSE_FEATURE)

    void MarlinUI::draw_hotend_status(const uint8_t row, const uint8_t extruder) {
      u8g_uint_t y1 = row * (MENU_FONT_HEIGHT) + 1, y2 = y1 + MENU_FONT_HEIGHT - 1;

      if (!PAGE_CONTAINS(y1 + 1, y2 + 2)) return;

      lcd_put_lchar(LCD_PIXEL_WIDTH - 11 * (MENU_FONT_WIDTH), y2, 'E');
      lcd_put_lchar((char)('1' + extruder));
      lcd_put_u8str(F(" "));
      lcd_put_u8str(i16tostr3rj(thermalManager.wholeDegHotend(extruder)));
      lcd_put_u8str(F("/"));

      if (get_blink() || !thermalManager.heater_idle[extruder].timed_out)
        lcd_put_u8str(i16tostr3rj(thermalManager.degTargetHotend(extruder)));
    }

  #endif // ADVANCED_PAUSE_FEATURE

  // Mark a menu item and set font color if selected.
  // Return 'false' if the item is not on screen.
  static bool mark_as_selected(const uint8_t row, const bool sel) {
    row_y1 = row * (MENU_FONT_HEIGHT) + 1;
    row_y2 = row_y1 + MENU_FONT_HEIGHT - 1;

    if (!PAGE_CONTAINS(row_y1 + 1, row_y2 + 2)) return false;

    if (sel) {
      #if ENABLED(MENU_HOLLOW_FRAME)
        u8g.drawHLine(0, row_y1 + 1, LCD_PIXEL_WIDTH);
        u8g.drawHLine(0, row_y2 + 2, LCD_PIXEL_WIDTH);
      #else
        u8g.setColorIndex(1); // solid outline
        u8g.drawBox(0, row_y1 + 2, LCD_PIXEL_WIDTH, MENU_FONT_HEIGHT - 1);
        u8g.setColorIndex(0); // inverted text
      #endif
    }
    #if DISABLED(MENU_HOLLOW_FRAME)
      else u8g.setColorIndex(1); // solid text
    #endif

    if (!PAGE_CONTAINS(row_y1, row_y2)) return false;

    lcd_moveto(0, row_y2);
    return true;
  }

  // Draw a static line of text in the same idiom as a menu item
  void MenuItem_static::draw(const uint8_t row, FSTR_P const ftpl, const uint8_t style/*=SS_DEFAULT*/, const char *vstr/*=nullptr*/) {
    if (!mark_as_selected(row, style & SS_INVERT)) return;

    pixel_len_t n = LCD_PIXEL_WIDTH; // pixel width of string allowed
    const bool center = bool(style & SS_CENTER), full = bool(style & SS_FULL);

    char estr[calculateWidth(ftpl) + 3] = "\0";
    pixel_len_t lwide = ftpl ? (MENU_FONT_WIDTH) * expand_u8str(estr, ftpl, itemIndex, itemStringC, itemStringF, (LCD_PIXEL_WIDTH) / (MENU_FONT_WIDTH)) : 0;

    // Value length, if any
    int8_t vlen = vstr ? utf8_strlen(vstr) : 0;

    bool mv_colon = false;
    if (vlen && !center) {
      // Move the leading colon from the value to the label below
      mv_colon = (*vstr == ':');
      // Shorter value, wider label
      if (mv_colon) { vstr++; vlen--; lwide += MENU_FONT_WIDTH; }
      // Remove leading spaces from the value and shorten
      while (*vstr == ' ') { vstr++; vlen--; }
    }

    // Padding for center or full justification
    int8_t pad = (center || full) ? ((LCD_PIXEL_WIDTH) - lwide - vlen * (MENU_FONT_WIDTH)) / (MENU_FONT_WIDTH) : 0;

    // SS_CENTER: Pad with half of the unused space first
    if (center) for (int8_t lpad = pad / 2; lpad > 0; --lpad) n -= lcd_put_u8str(F(" "));

    // Draw as much of the label as fits (without the relocated colon, drawn below)
    if (lwide) lcd_put_u8str_max(estr, n);

    // Value string?
    if (vlen) {
      // SS_FULL: Pad with enough space to justify the value
      if (full && !center && n > MENU_FONT_WIDTH) {
        // Draw the leading colon moved from the value to the label
        if (mv_colon) n -= lcd_put_u8str(F(":"));
        // Pad in-between
        for (; pad > 0; --pad) n -= lcd_put_u8str(F(" "));
      }
      // Draw the value string
      n -= lcd_put_u8str_max(vstr, n);
    }
    // Always fill out the rest with spaces
    while (n > MENU_FONT_WIDTH) n -= lcd_put_u8str(F(" "));
  }

  // Draw a generic menu item
  void MenuItemBase::_draw(const bool sel, const uint8_t row, FSTR_P const ftpl, const char, const char post_char) {
    if (!mark_as_selected(row, sel)) return;

    uint8_t n = LCD_WIDTH - 1;
    n -= lcd_put_u8str(ftpl, itemIndex, itemStringC, itemStringF, n);
    for (; n; --n) lcd_put_u8str(F(" "));
    lcd_put_lchar(LCD_PIXEL_WIDTH - (MENU_FONT_WIDTH), row_y2, post_char);
    lcd_put_u8str(F(" "));
  }

  // Draw a menu item with an editable value
  void MenuEditItemBase::draw(const bool sel, const uint8_t row, FSTR_P const ftpl, const char * const inStr, const bool pgm) {
    if (!mark_as_selected(row, sel)) return;

    const uint8_t vallen = (pgm ? utf8_strlen_P(inStr) : utf8_strlen(inStr)),
                  pixelwidth = (pgm ? uxg_GetUtf8StrPixelWidthP(u8g.getU8g(), inStr) : uxg_GetUtf8StrPixelWidth(u8g.getU8g(), inStr));
    const u8g_uint_t prop = USE_WIDE_GLYPH ? 2 : 1;

    uint8_t n = LCD_WIDTH - 2 - vallen * prop;
    n -= lcd_put_u8str(ftpl, itemIndex, itemStringC, itemStringF, n);
    if (vallen) {
      lcd_put_u8str(F(":"));
      for (; n; --n) lcd_put_u8str(F(" "));
      lcd_moveto(LCD_PIXEL_WIDTH - _MAX((MENU_FONT_WIDTH) * vallen, pixelwidth + 2), row_y2);
      if (pgm) lcd_put_u8str_P(inStr); else lcd_put_u8str(inStr);
    }
  }

  void MenuEditItemBase::draw_edit_screen(FSTR_P const ftpl, const char * const value/*=nullptr*/) {
    ui.encoder_direction_normal();

    const u8g_uint_t prop = USE_WIDE_GLYPH ? 2 : 1;
    const u8g_uint_t labellen = utf8_strlen(ftpl), vallen = utf8_strlen(value);
    bool extra_row = labellen * prop > LCD_WIDTH - 2 - vallen * prop;

    #if ENABLED(USE_BIG_EDIT_FONT)
      // Use the menu font if the label won't fit on a single line
      constexpr u8g_uint_t lcd_edit_width = (LCD_PIXEL_WIDTH) / (EDIT_FONT_WIDTH);
      u8g_uint_t lcd_chr_fit, one_chr_width;
      if (labellen * prop <= lcd_edit_width - 1) {
        if (labellen * prop + vallen * prop + 1 > lcd_edit_width) extra_row = true;
        lcd_chr_fit = lcd_edit_width + 1;
        one_chr_width = EDIT_FONT_WIDTH;
        ui.set_font(FONT_EDIT);
      }
      else {
        lcd_chr_fit = LCD_WIDTH;
        one_chr_width = MENU_FONT_WIDTH;
        ui.set_font(FONT_MENU);
      }
    #else
      constexpr u8g_uint_t lcd_chr_fit = LCD_WIDTH,
                           one_chr_width = MENU_FONT_WIDTH;
    #endif

    // Center the label and value lines on the middle line
    u8g_uint_t baseline = extra_row ? (LCD_PIXEL_HEIGHT) / 2 - 1
                                    : (LCD_PIXEL_HEIGHT + EDIT_FONT_ASCENT) / 2;

    // Assume the label is alpha-numeric (with a descender)
    bool onpage = PAGE_CONTAINS(baseline - (EDIT_FONT_ASCENT - 1), baseline + EDIT_FONT_DESCENT);
    if (onpage) lcd_put_u8str(0, baseline, ftpl, itemIndex, itemStringC, itemStringF);

    // If a value is included, print a colon, then print the value right-justified
    if (value) {
      lcd_put_u8str(F(":"));
      if (extra_row) {
        // Assume that value is numeric (with no descender)
        baseline += EDIT_FONT_ASCENT + 2;
        onpage = PAGE_CONTAINS(baseline - (EDIT_FONT_ASCENT - 1), baseline);
      }
      if (onpage) {
        lcd_put_lchar(((lcd_chr_fit - 1) - (vallen * prop + 1)) * one_chr_width, baseline, ' '); // Right-justified, padded, add a leading space
        lcd_put_u8str(value);
      }
    }
    TERN_(USE_BIG_EDIT_FONT, ui.set_font(FONT_MENU));
  }

  inline void draw_boxed_string(const u8g_uint_t x, const u8g_uint_t y, FSTR_P const fstr, const bool inv) {
    const u8g_uint_t len = utf8_strlen(fstr),
                      by = (y + 1) * (MENU_FONT_HEIGHT);
    const u8g_uint_t prop = USE_WIDE_GLYPH ? 2 : 1;
    const pixel_len_t bw = len * prop * (MENU_FONT_WIDTH), bx = x * prop * (MENU_FONT_WIDTH);
    if (inv) {
      u8g.setColorIndex(1);
      u8g.drawBox(bx / prop - 1, by - (MENU_FONT_ASCENT), bw + 2, MENU_FONT_HEIGHT);
      u8g.setColorIndex(0);
    }
    lcd_put_u8str(bx / prop, by, fstr);
    if (inv) u8g.setColorIndex(1);
  }

  void MenuItem_confirm::draw_select_screen(FSTR_P const yes, FSTR_P const no, const bool yesno, FSTR_P const fpre, const char * const string/*=nullptr*/, FSTR_P const fsuf/*=nullptr*/) {
    ui.draw_select_screen_prompt(fpre, string, fsuf);
    if (no)  draw_boxed_string(1, LCD_HEIGHT - 1, no, !yesno);
    if (yes) draw_boxed_string(LCD_WIDTH - (utf8_strlen(yes) * (USE_WIDE_GLYPH ? 2 : 1) + 1), LCD_HEIGHT - 1, yes, yesno);
  }

  #if HAS_MEDIA

    void MenuItem_sdbase::draw(const bool sel, const uint8_t row, FSTR_P const, CardReader &theCard, const bool isDir) {
      if (!mark_as_selected(row, sel)) return;

      const uint8_t maxlen = LCD_WIDTH - isDir;
      if (isDir) lcd_put_lchar(LCD_STR_FOLDER[0]);
      const pixel_len_t pixw = maxlen * (MENU_FONT_WIDTH);
      pixel_len_t n = pixw - lcd_put_u8str_max(ui.scrolled_filename(theCard, maxlen, sel), pixw);
      for (; n > MENU_FONT_WIDTH; n -= MENU_FONT_WIDTH) lcd_put_u8str(F(" "));
    }

  #endif // HAS_MEDIA

  #if ENABLED(AUTO_BED_LEVELING_UBL)

    /**
     * UBL LCD "radar" map data
     */
    #define MAP_UPPER_LEFT_CORNER_X 35  // These probably should be moved to the .h file  But for now,
    #define MAP_UPPER_LEFT_CORNER_Y  8  // it is easier to play with things having them here
    #define MAP_MAX_PIXELS_X        53
    #define MAP_MAX_PIXELS_Y        49

    void MarlinUI::ubl_plot(const uint8_t x_plot, const uint8_t y_plot) {
      // Scale the box pixels appropriately
      u8g_uint_t x_map_pixels = ((MAP_MAX_PIXELS_X - 4) / (GRID_MAX_POINTS_X)) * (GRID_MAX_POINTS_X),
                 y_map_pixels = ((MAP_MAX_PIXELS_Y - 4) / (GRID_MAX_POINTS_Y)) * (GRID_MAX_POINTS_Y),

              pixels_per_x_mesh_pnt = x_map_pixels / (GRID_MAX_POINTS_X),
              pixels_per_y_mesh_pnt = y_map_pixels / (GRID_MAX_POINTS_Y),

              x_offset = MAP_UPPER_LEFT_CORNER_X + 1 + (MAP_MAX_PIXELS_X - x_map_pixels - 2) / 2,
              y_offset = MAP_UPPER_LEFT_CORNER_Y + 1 + (MAP_MAX_PIXELS_Y - y_map_pixels - 2) / 2;

      // Clear the Mesh Map

      if (PAGE_CONTAINS(y_offset - 2, y_offset + y_map_pixels + 4)) {
        u8g.setColorIndex(1);  // First draw the bigger box in White so we have a border around the mesh map box
        u8g.drawBox(x_offset - 2, y_offset - 2, x_map_pixels + 4, y_map_pixels + 4);
        if (PAGE_CONTAINS(y_offset, y_offset + y_map_pixels)) {
          u8g.setColorIndex(0);  // Now actually clear the mesh map box
          u8g.drawBox(x_offset, y_offset, x_map_pixels, y_map_pixels);
        }
      }

      // Display Mesh Point Locations

      u8g.setColorIndex(1);
      const u8g_uint_t sx = x_offset + pixels_per_x_mesh_pnt / 2;
            u8g_uint_t  y = y_offset + pixels_per_y_mesh_pnt / 2;
      for (uint8_t j = 0; j < (GRID_MAX_POINTS_Y); j++, y += pixels_per_y_mesh_pnt)
        if (PAGE_CONTAINS(y, y))
          for (uint8_t i = 0, x = sx; i < (GRID_MAX_POINTS_X); i++, x += pixels_per_x_mesh_pnt)
            u8g.drawBox(x, y, 1, 1);

      // Fill in the Specified Mesh Point

      const uint8_t y_plot_inv = (GRID_MAX_POINTS_Y) - 1 - y_plot;  // The origin is typically in the lower right corner.  We need to
                                                                    // invert the Y to get it to plot in the right location.

      const u8g_uint_t by = y_offset + y_plot_inv * pixels_per_y_mesh_pnt;
      if (PAGE_CONTAINS(by, by + pixels_per_y_mesh_pnt))
        u8g.drawBox(
          x_offset + x_plot * pixels_per_x_mesh_pnt, by,
          pixels_per_x_mesh_pnt, pixels_per_y_mesh_pnt
        );

      // Put Relevant Text on Display

      // Show X and Y positions at top of screen
      u8g.setColorIndex(1);
      if (PAGE_UNDER(7)) {
        const xy_pos_t pos = { bedlevel.get_mesh_x(x_plot), bedlevel.get_mesh_y(y_plot) },
                       lpos = pos.asLogical();
        lcd_put_u8str_P(5, 7, X_LBL);
        lcd_put_u8str(ftostr52(lpos.x));
        lcd_put_u8str_P(74, 7, Y_LBL);
        lcd_put_u8str(ftostr52(lpos.y));
      }

      // Print plot position
      if (PAGE_CONTAINS(LCD_PIXEL_HEIGHT - (INFO_FONT_HEIGHT - 1), LCD_PIXEL_HEIGHT)) {
        lcd_put_lchar(5, LCD_PIXEL_HEIGHT, '(');
        u8g.print(x_plot);
        lcd_put_u8str(F(","));
        u8g.print(y_plot);
        lcd_put_u8str(F(")"));

        // Show the location value
        lcd_put_u8str_P(74, LCD_PIXEL_HEIGHT, Z_LBL);
        if (!isnan(bedlevel.z_values[x_plot][y_plot]))
          lcd_put_u8str(ftostr43sign(bedlevel.z_values[x_plot][y_plot]));
        else
          lcd_put_u8str(F(" -----"));
      }

    }

  #endif // AUTO_BED_LEVELING_UBL

  #if ANY(BABYSTEP_GFX_OVERLAY, MESH_EDIT_GFX_OVERLAY)

    //
    // Draw knob rotation => Z motion key for:
    //  - menu.cpp:lcd_babystep_zoffset
    //  - menu_ubl.cpp:_lcd_mesh_fine_tune
    //

    const unsigned char cw_bmp[] PROGMEM = {
      B00000000,B11111110,B00000000,
      B00000011,B11111111,B10000000,
      B00000111,B11000111,B11000000,
      B00000111,B00000001,B11100000,
      B00000000,B00000000,B11100000,
      B00000000,B00000000,B11110000,
      B00000000,B00000000,B01110000,
      B00000100,B00000000,B01110000,
      B00001110,B00000000,B01110000,
      B00011111,B00000000,B01110000,
      B00111111,B10000000,B11110000,
      B00001110,B00000000,B11100000,
      B00001111,B00000001,B11100000,
      B00000111,B11000111,B11000000,
      B00000011,B11111111,B10000000,
      B00000000,B11111110,B00000000
    };

    const unsigned char ccw_bmp[] PROGMEM = {
      B00000000,B11111110,B00000000,
      B00000011,B11111111,B10000000,
      B00000111,B11000111,B11000000,
      B00001111,B00000001,B11100000,
      B00001110,B00000000,B11100000,
      B00111111,B10000000,B11110000,
      B00011111,B00000000,B01110000,
      B00001110,B00000000,B01110000,
      B00000100,B00000000,B01110000,
      B00000000,B00000000,B01110000,
      B00000000,B00000000,B11110000,
      B00000000,B00000000,B11100000,
      B00000111,B00000001,B11100000,
      B00000111,B11000111,B11000000,
      B00000011,B11111111,B10000000,
      B00000000,B11111110,B00000000
    };

    const unsigned char up_arrow_bmp[] PROGMEM = {
      B00000100,B00000000,
      B00001110,B00000000,
      B00011111,B00000000,
      B00111111,B10000000,
      B01111111,B11000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000
    };

    const unsigned char down_arrow_bmp[] PROGMEM = {
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B00001110,B00000000,
      B01111111,B11000000,
      B00111111,B10000000,
      B00011111,B00000000,
      B00001110,B00000000,
      B00000100,B00000000
    };

    const unsigned char offset_bedline_bmp[] PROGMEM = {
      B11111111,B11111111,B11111111
    };

    const unsigned char nozzle_bmp[] PROGMEM = {
      B01111111,B10000000,
      B11111111,B11000000,
      B11111111,B11000000,
      B11111111,B11000000,
      B01111111,B10000000,
      B01111111,B10000000,
      B11111111,B11000000,
      B11111111,B11000000,
      B11111111,B11000000,
      B00111111,B00000000,
      B00011110,B00000000,
      B00001100,B00000000
    };

    void MarlinUI::zoffset_overlay(const int8_t dir) {
      const unsigned char *rot_up = TERN(OVERLAY_GFX_REVERSE, ccw_bmp,  cw_bmp),
                        *rot_down = TERN(OVERLAY_GFX_REVERSE,  cw_bmp, ccw_bmp);

      const int left = TERN(USE_BIG_EDIT_FONT,  0,  5),
               right = TERN(USE_BIG_EDIT_FONT, 45, 90),
              nozzle = TERN(USE_BIG_EDIT_FONT, 95, 60);

      // Draw nozzle lowered or raised according to direction moved
      if (PAGE_CONTAINS( 3, 16)) u8g.drawBitmapP(nozzle + 6,  4 - dir, 2, 12, nozzle_bmp);
      if (PAGE_CONTAINS(20, 20)) u8g.drawBitmapP(nozzle + 0, 20      , 3,  1, offset_bedline_bmp);

      // Draw cw/ccw indicator and up/down arrows.
      if (PAGE_CONTAINS(47, 62)) {
        u8g.drawBitmapP(right +  0, 48 - dir, 2, 13, up_arrow_bmp);
        u8g.drawBitmapP(left  +  0, 49 - dir, 2, 13, down_arrow_bmp);
        u8g.drawBitmapP(left  + 13, 47      , 3, 16, rot_down);
        u8g.drawBitmapP(right + 13, 47      , 3, 16, rot_up);
      }
    }

  #endif // BABYSTEP_GFX_OVERLAY || MESH_EDIT_GFX_OVERLAY

#endif // HAS_MARLINUI_MENU

#endif // HAS_MARLINUI_U8GLIB
