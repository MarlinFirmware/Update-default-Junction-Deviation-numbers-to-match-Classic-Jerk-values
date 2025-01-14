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

#include "../../../inc/MarlinConfigPre.h"

#if IS_DWIN_MARLINUI

#include "marlinui_dwin.h"
#include "dwin_lcd.h"
#include "dwin_string.h"

#include "lcdprint_dwin.h"
#include "../../utf8.h"
#include "../../../libs/numtostr.h"
#include "../../marlinui.h"

#include "../../../sd/cardreader.h"
#include "../../../module/motion.h"
#include "../../../module/temperature.h"
#include "../../../module/printcounter.h"

#if HAS_MEDIA
  #include "../../../libs/duration_t.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "../../../feature/bedlevel/bedlevel.h"
#endif

// DWIN printing specifies the font on each string operation
// but we'll make the font modal for Marlin
dwin_font_t dwin_font = { font8x16, 8, 16, COLOR_WHITE, COLOR_BG_BLACK, true };
void MarlinUI::set_font(const uint8_t font_nr) {
  if (font_nr != dwin_font.index) {
    dwin_font.index = font_nr;
    uint8_t w, h;
    switch (font_nr) {
      default:
      case font6x12:  w =  6; h = 12; break;
      case font8x16:  w =  8; h = 16; break;
      case font10x20: w = 10; h = 20; break;
      case font12x24: w = 12; h = 24; break;
      case font14x28: w = 14; h = 28; break;
      case font16x32: w = 16; h = 32; break;
      case font20x40: w = 20; h = 40; break;
      case font24x48: w = 24; h = 48; break;
      case font28x56: w = 28; h = 56; break;
      case font32x64: w = 32; h = 64; break;
    }
    dwin_font.width = w;
    dwin_font.height = h;
    // TODO: Array with dimensions, auto fit menu items,
    // update char width / height of the screen based on
    // new (fixed-width) font size.
  }
}

// This display is always detected
bool MarlinUI::detected() { return true; }

// Initialize or re-initialize the LCD
void MarlinUI::init_lcd() { dwinStartup(); }

// This LCD should clear where it will draw anew
void MarlinUI::clear_lcd() {
  dwinIconAnimationControl(0x0000); // disable all icon animations
  dwinJPGShowAndCache(3);
  dwinFrameClear(COLOR_BG_BLACK);
  dwinUpdateLCD();

  did_first_redraw = false;
}

void MarlinUI::clear_for_drawing() { clear_lcd(); }

#if ENABLED(SHOW_BOOTSCREEN)

  void MarlinUI::show_bootscreen() {
    dwin_string.set(F(SHORT_BUILD_VERSION));

    #if ENABLED(SHOW_CUSTOM_BOOTSCREEN) && !defined(CUSTOM_BOOTSCREEN_TIMEOUT)
      #define CUSTOM_BOOTSCREEN_TIMEOUT 3000
    #endif

    #if ENABLED(DWIN_MARLINUI_PORTRAIT)
      #define LOGO_CENTER ((LCD_PIXEL_WIDTH) / 2)
      #define INFO_CENTER LOGO_CENTER
      #define VERSION_Y   330
    #else
      #define LOGO_CENTER (280 / 2)
      #define INFO_CENTER ((LCD_PIXEL_WIDTH) - 200 / 2)
      #define VERSION_Y   84
    #endif

    dwinDrawString(false, font10x20, COLOR_YELLOW, COLOR_BG_BLACK, INFO_CENTER - (dwin_string.length * 10) / 2, VERSION_Y, S(dwin_string.string()));
    TERN_(SHOW_CUSTOM_BOOTSCREEN, safe_delay(CUSTOM_BOOTSCREEN_TIMEOUT));
    clear_for_drawing();

    dwinIconShow(BOOT_ICON, ICON_MarlinBoot, LOGO_CENTER - 266 / 2,  15);
    #if ENABLED(DWIN_MARLINUI_PORTRAIT)
      dwinIconShow(BOOT_ICON, ICON_OpenSource, LOGO_CENTER - 174 / 2, 280);
      dwinIconShow(BOOT_ICON, ICON_GitHubURL,  LOGO_CENTER - 180 / 2, 420);
      dwinIconShow(BOOT_ICON, ICON_MarlinURL,  LOGO_CENTER - 100 / 2, 440);
      dwinIconShow(BOOT_ICON, ICON_Copyright,  LOGO_CENTER - 126 / 2, 460);
    #else
      dwinIconShow(BOOT_ICON, ICON_MarlinBoot, LOGO_CENTER - 266 / 2,  15);
      dwinIconShow(BOOT_ICON, ICON_OpenSource, INFO_CENTER - 174 / 2,  60);
      dwinIconShow(BOOT_ICON, ICON_GitHubURL,  INFO_CENTER - 180 / 2, 130);
      dwinIconShow(BOOT_ICON, ICON_MarlinURL,  INFO_CENTER - 100 / 2, 152);
      dwinIconShow(BOOT_ICON, ICON_Copyright,  INFO_CENTER - 126 / 2, 200);
    #endif
    dwinDrawString(false, font10x20, COLOR_YELLOW, COLOR_BG_BLACK, INFO_CENTER - (dwin_string.length * 10) / 2, VERSION_Y, S(dwin_string.string()));
    dwinUpdateLCD();
  }

  void MarlinUI::bootscreen_completion(const millis_t sofar) {
    if ((BOOTSCREEN_TIMEOUT) > sofar) safe_delay((BOOTSCREEN_TIMEOUT) - sofar);
    clear_for_drawing();
  }

#endif

// The kill screen is displayed for unrecoverable conditions
void MarlinUI::draw_kill_screen() {
  set_font(DWIN_FONT_ALERT);
  dwinFrameClear(COLOR_BG_BLACK);
  dwin_font.fg = COLOR_ERROR_RED;
  dwin_font.solid = false;
  dwinDrawRectangle(1, COLOR_BG_WINDOW, 20, 20, LCD_PIXEL_WIDTH - 20, LCD_PIXEL_HEIGHT - 20);
  // make the frame a few pixels thick
  dwinDrawRectangle(0, COLOR_YELLOW, 20, 20, LCD_PIXEL_WIDTH - 20, LCD_PIXEL_HEIGHT - 20);
  dwinDrawRectangle(0, COLOR_YELLOW, 21, 21, LCD_PIXEL_WIDTH - 21, LCD_PIXEL_HEIGHT - 21);
  dwinDrawRectangle(0, COLOR_YELLOW, 22, 22, LCD_PIXEL_WIDTH - 22, LCD_PIXEL_HEIGHT - 22);

  uint8_t cx = (LCD_PIXEL_WIDTH / dwin_font.width / 2),
          cy = (LCD_PIXEL_HEIGHT / dwin_font.height / 2);

  #if ENABLED(DWIN_MARLINUI_LANDSCAPE)
    cx += (96 / 2 / dwin_font.width);
    dwinIconShow(ICON, ICON_Halted, 40, (LCD_PIXEL_HEIGHT - 96) / 2);
  #else
    dwinIconShow(ICON, ICON_Halted, (LCD_PIXEL_WIDTH - 96) / 2, 40);
  #endif

  uint8_t slen = status_message.glyphs();
  lcd_moveto(cx - (slen / 2), cy - 1);
  lcd_put_u8str(status_message);

  slen = utf8_strlen(S(GET_TEXT_F(MSG_HALTED)));
  lcd_moveto(cx - (slen / 2), cy);
  lcd_put_u8str(GET_TEXT_F(MSG_HALTED));

  slen = utf8_strlen(S(GET_TEXT_F(MSG_HALTED)));
  lcd_moveto(cx - (slen / 2), cy + 1);
  lcd_put_u8str(GET_TEXT_F(MSG_HALTED));
}

//
// Status Message
//
void MarlinUI::draw_status_message(const bool blink) {
  set_font(DWIN_FONT_STAT);
  dwin_font.solid = true;
  dwin_font.fg = COLOR_WHITE;
  dwin_font.bg = COLOR_BG_BLACK;
  lcd_moveto_xy(0, LCD_PIXEL_HEIGHT - (STAT_FONT_HEIGHT) - 1);

  constexpr uint8_t max_status_chars = (LCD_PIXEL_WIDTH) / (STAT_FONT_WIDTH);

  auto status_changed = []{
    static MString<>::hash_t old_hash = 0x0000;
    const MString<>::hash_t hash = status_message.hash();
    const bool hash_changed = hash != old_hash;
    old_hash = hash;
    return hash_changed || !did_first_redraw;
  };

  #if ENABLED(STATUS_MESSAGE_SCROLLING)
    static bool last_blink = false;

    // Get the UTF8 character count of the string
    uint8_t slen = status_message.glyphs();

    // If the string fits into the LCD, just print it and do not scroll it
    if (slen <= max_status_chars) {

      if (status_changed()) {

        // The string isn't scrolling and may not fill the screen
        lcd_put_u8str(status_message);

        // Fill the rest with spaces
        while (slen < max_status_chars) { lcd_put_u8str(F(" ")); ++slen; }
      }
    }
    else {
      // String is larger than the available line space

      // Get a pointer to the next valid UTF8 character
      // and the string remaining length
      uint8_t rlen;
      const char *stat = status_and_len(rlen);
      lcd_put_u8str_max(stat, max_status_chars);

      // If the remaining string doesn't completely fill the line...
      if (rlen < max_status_chars) {
        lcd_put_u8str(F("."));                   // Always at 1+ spaces left, draw a dot
        uint8_t chars = max_status_chars - rlen;  // Amount of space left in characters
        if (--chars) {                        // Draw a second dot if there's space
          lcd_put_u8str(F("."));
          if (--chars)
            lcd_put_u8str_max(status_message, chars); // Print a second copy of the message
        }
      }

      if (last_blink != blink) {
        last_blink = blink;
        advance_status_scroll();
      }
    }

  #else

    UNUSED(blink);

    if (status_changed()) {
      // Get the UTF8 character count of the string
      uint8_t slen = status_message.glyphs();

      // Just print the string to the LCD
      lcd_put_u8str_max(status_message, max_status_chars);

      // Fill the rest with spaces if there are missing spaces
      while (slen < max_status_chars) { lcd_put_u8str(F(" ")); ++slen; }
    }

  #endif
}

#if HAS_LCD_BRIGHTNESS
  void MarlinUI::_set_brightness() { dwinLCDBrightness(backlight ? brightness : 0); }
#endif

#if HAS_MARLINUI_MENU

  #include "../../menu/menu.h"

  #if ENABLED(ADVANCED_PAUSE_FEATURE)

    void MarlinUI::draw_hotend_status(const uint8_t row, const uint8_t extruder) {

      dwin_font.solid = false;
      dwin_font.fg = COLOR_WHITE;
      dwin_string.set('E');
      dwin_string.add('1' + extruder);
      dwin_string.add(' ');
      dwin_string.add(i16tostr3rj(thermalManager.degHotend(extruder)));
      dwin_string.add('/');
      if (get_blink() || !thermalManager.heater_idle[thermalManager.idle_index_for_id(extruder)].timed_out)
        dwin_string.add(i16tostr3rj(thermalManager.degTargetHotend(extruder)));
      else
        dwin_string.add(F("    "));

      lcd_moveto(LCD_WIDTH - dwin_string.length, row);
      lcd_put_dwin_string();
    }

  #endif

  // Set the colors for a menu item based on whether it is selected
  static bool mark_as_selected(const uint8_t row, const bool sel, const bool is_static=false) {
    const dwin_coord_t y = row * (MENU_LINE_HEIGHT) + 1;
    if (y >= LCD_PIXEL_HEIGHT) return false;

    if (is_static && sel)
      dwinDrawBox(1, Color_Bg_Heading, 0, y, LCD_PIXEL_WIDTH, MENU_LINE_HEIGHT - 1);
    else {
      #if ENABLED(MENU_HOLLOW_FRAME)
                 dwinDrawBox(1, COLOR_BG_BLACK, 0, y, LCD_PIXEL_WIDTH, MENU_LINE_HEIGHT - 1);
        if (sel) dwinDrawBox(0, COLOR_SELECT,   0, y, LCD_PIXEL_WIDTH, MENU_LINE_HEIGHT - 1);
      #else
        dwinDrawBox(1, sel ? COLOR_SELECT : COLOR_BG_BLACK, 0, y, LCD_PIXEL_WIDTH, MENU_LINE_HEIGHT - 1);
      #endif
    }

    return true;
  }

  // Draw a static line of text in the same idiom as a menu item

  void MenuItem_static::draw(const uint8_t row, FSTR_P const ftpl, const uint8_t style/*=SS_DEFAULT*/, const char *vstr/*=nullptr*/) {
    // Call mark_as_selected to draw a bigger selection box
    // and draw the text without a background
    if (!mark_as_selected(row, (bool)(style & SS_INVERT), true)) return;

    ui.set_font(DWIN_FONT_MENU);
    dwin_font.solid = false;
    dwin_font.fg = COLOR_WHITE;

    dwin_string.set();

    const bool center = bool(style & SS_CENTER), full = bool(style & SS_FULL);
    int8_t plen = ftpl ? utf8_strlen(ftpl) : 0;
    const int8_t olen = plen;

    // Value length, if any
    int8_t vlen = vstr ? utf8_strlen(vstr) : 0;

    bool mv_colon = false;
    if (vlen && !center) {
      // Move the leading colon from the value to the label below
      mv_colon = (*vstr == ':');
      // Shorter value, wider label
      if (mv_colon) { vstr++; vlen--; plen++; }
      // Remove leading spaces from the value and shorten
      while (*vstr == ' ') { vstr++; vlen--; }
    }

    int8_t pad = (center || full) ? (LCD_WIDTH) - 1 - plen - vlen : 0;

    // SS_CENTER: Pad with half of the unused space first
    if (center) for (int8_t lpad = pad / 2; lpad > 0; --lpad, --pad) dwin_string.add(' ');

    if (plen) {
      // Append the templated label string
      dwin_string.add(ftpl, itemIndex, itemStringC, itemStringF);
      // Remove padding if the string was expanded
      pad -= dwin_string.length - olen;
    }

    // SS_FULL: Pad with enough space to justify the value
    if (vlen) {
      if (full && !center) {
        // Append the leading colon moved from the value to the label
        if (mv_colon) dwin_string.add(':');
        // Pad in-between
        for (; pad > 0; --pad) dwin_string.add(' ');
      }
      // Append the value
      dwin_string.add(vstr);
    }

    // SS_CENTER: Pad the rest of the string
    if (center) while (pad--) dwin_string.add(' ');

    lcd_moveto(1, row);
    lcd_put_dwin_string();
  }

  // Draw a generic menu item
  void MenuItemBase::_draw(const bool sel, const uint8_t row, FSTR_P const ftpl, const char, const char post_char) {
    if (!mark_as_selected(row, sel)) return;

    ui.set_font(DWIN_FONT_MENU);
    dwin_font.solid = false;
    dwin_font.fg = COLOR_WHITE;

    dwin_string.set(ftpl, itemIndex, itemStringC, itemStringF);

    pixel_len_t n = LCD_WIDTH - 1 - dwin_string.length;
    while (--n > 1) dwin_string.add(' ');

    dwin_string.add(post_char);

    lcd_moveto(1, row);
    lcd_put_dwin_string();
  }

  //
  // Draw a menu item with an editable value
  //
  void MenuEditItemBase::draw(const bool sel, const uint8_t row, FSTR_P const ftpl, const char * const inStr, const bool pgm) {
    if (!mark_as_selected(row, sel)) return;

    ui.set_font(DWIN_FONT_MENU);
    dwin_font.solid = false;
    dwin_font.fg = COLOR_WHITE;

    const uint8_t vallen = (pgm ? utf8_strlen_P(inStr) : utf8_strlen(S(inStr)));

    dwin_string.set(ftpl, itemIndex, itemStringC, itemStringF);
    if (vallen) dwin_string.add(':');

    lcd_moveto(1, row);
    lcd_put_dwin_string();

    if (vallen) {
      dwin_font.fg = COLOR_YELLOW;
      dwin_string.set(inStr);
      lcd_moveto(LCD_WIDTH - vallen - 1, row);
      lcd_put_dwin_string();
    }
  }

  //
  // Draw an edit screen with label and current value
  //
  void MenuEditItemBase::draw_edit_screen(FSTR_P const ftpl, const char* const value/*=nullptr*/) {
    ui.encoder_direction_normal();

    const dwin_coord_t labellen = utf8_strlen(ftpl), vallen = utf8_strlen(value);

    dwin_string.set(FTOP(ftpl), itemIndex);
    if (vallen) dwin_string.add(':');  // If a value is included, add a colon

    // Assume the label is alpha-numeric (with a descender)
    const uint16_t row = (LCD_HEIGHT / 2) - 1;

    dwin_font.fg = COLOR_WHITE;
    dwin_font.solid = true;
    lcd_moveto((LCD_WIDTH - labellen + !!vallen) / 2, row);
    lcd_put_dwin_string();

    // If a value is included, print the value in larger text below the label
    if (vallen) {
      dwin_string.set(value);

      const dwin_coord_t by = (row * MENU_LINE_HEIGHT) + MENU_FONT_HEIGHT + EXTRA_ROW_HEIGHT / 2;
      dwinDrawString(true, font16x32, COLOR_YELLOW, COLOR_BG_BLACK, (LCD_PIXEL_WIDTH - vallen * 16) / 2, by, S(dwin_string.string()));

      if (ui.can_show_slider() && maxEditValue > 0) {

        const dwin_coord_t slider_length = LCD_PIXEL_WIDTH - TERN(DWIN_MARLINUI_LANDSCAPE, 120, 20),
                           slider_height = 16,
                           slider_x = (LCD_PIXEL_WIDTH - slider_length) / 2,
                           slider_y = by + 32 + 4,
                           amount = ui.encoderPosition * slider_length / maxEditValue;

        dwinDrawRectangle(1, COLOR_BG_WINDOW, slider_x - 1, slider_y - 1, slider_x - 1 + slider_length + 2 - 1, slider_y - 1 + slider_height + 2 - 1);
        if (amount > 0)
          dwinDrawBox(1, COLOR_BARFILL, slider_x, slider_y, amount, slider_height);
        if (amount < slider_length)
          dwinDrawBox(1, COLOR_BG_BLACK, slider_x + amount, slider_y, slider_length - amount, slider_height);
      }
    }
  }

  inline void draw_boxed_string(const bool yesopt, FSTR_P const fstr, const bool inv) {
    const uint8_t len = utf8_strlen_P(FTOP(fstr)),
                  mar = TERN(DWIN_MARLINUI_PORTRAIT, 1, 4),
                  col = yesopt ? LCD_WIDTH - mar - len : mar,
                  row = (LCD_HEIGHT >= 8 ? LCD_HEIGHT / 2 + 3 : LCD_HEIGHT - 1);
    lcd_moveto(col, row);
    dwinDrawBox(1, inv ? COLOR_SELECT : COLOR_BG_BLACK, cursor.x - dwin_font.width, cursor.y + 1, dwin_font.width * (len + 2), dwin_font.height + 2);
    lcd_put_u8str(col, row, fstr);
  }

  void MenuItem_confirm::draw_select_screen(
    FSTR_P const yes, FSTR_P const no, const bool yesno,
    FSTR_P const fpre, const char * const string/*=nullptr*/, FSTR_P const fsuf/*=nullptr*/
  ) {
    ui.set_font(DWIN_FONT_MENU);
    dwin_font.solid = false;
    dwin_font.fg = COLOR_WHITE;
    ui.draw_select_screen_prompt(fpre, string, fsuf);
    if (no)  draw_boxed_string(false, no, !yesno);
    if (yes) draw_boxed_string(true, yes,  yesno);
  }

  #if HAS_MEDIA

    void MenuItem_sdbase::draw(const bool sel, const uint8_t row, FSTR_P const, CardReader &theCard, const bool isDir) {
      if (!mark_as_selected(row, sel)) return;

      dwin_string.set();

      uint8_t maxlen = LCD_WIDTH - 1;
      if (isDir) {
        dwin_string.add(LCD_STR_FOLDER " ");
        maxlen -= 2;
      }

      dwin_string.add(ui.scrolled_filename(theCard, maxlen, sel), maxlen);
      uint8_t n = maxlen - dwin_string.length;
      while (n > 0) { dwin_string.add(' '); --n; }
      lcd_moveto(1, row);
      lcd_put_dwin_string();
    }

  #endif // HAS_MEDIA

  #if ENABLED(AUTO_BED_LEVELING_UBL)

    /**
     * UBL LCD "radar" map data
     */
    #define MAP_UPPER_LEFT_CORNER_X   5  // These probably should be moved to the .h file  But for now,
    #define MAP_UPPER_LEFT_CORNER_Y   5  // it is easier to play with things having them here
    #define MAP_MAX_PIXELS_X        262  // 272 - 10
    #define MAP_MAX_PIXELS_Y        262

    void MarlinUI::ubl_plot(const uint8_t x_plot, const uint8_t y_plot) {
      // Scale the box pixels appropriately
      dwin_coord_t x_map_pixels = ((MAP_MAX_PIXELS_X - 4) / (GRID_MAX_POINTS_X)) * (GRID_MAX_POINTS_X),
                   y_map_pixels = ((MAP_MAX_PIXELS_Y - 4) / (GRID_MAX_POINTS_Y)) * (GRID_MAX_POINTS_Y),

              pixels_per_x_mesh_pnt = x_map_pixels / (GRID_MAX_POINTS_X),
              pixels_per_y_mesh_pnt = y_map_pixels / (GRID_MAX_POINTS_Y),

              x_offset = MAP_UPPER_LEFT_CORNER_X + 1 + (MAP_MAX_PIXELS_X - x_map_pixels - 2) / 2,
              y_offset = MAP_UPPER_LEFT_CORNER_Y + 1 + (MAP_MAX_PIXELS_Y - y_map_pixels - 2) / 2;

      // Clear the Mesh Map

      // First draw the bigger box in White so we have a border around the mesh map box
      dwinDrawRectangle(1, COLOR_WHITE, x_offset - 2, y_offset - 2, x_offset + 2 + x_map_pixels, y_offset + 2 + y_map_pixels);
      // Now actually clear the mesh map box
      dwinDrawRectangle(1, COLOR_BG_BLACK, x_offset, y_offset, x_offset + x_map_pixels, y_offset + y_map_pixels);

      // Fill in the Specified Mesh Point

      const uint8_t y_plot_inv = (GRID_MAX_POINTS_Y - 1) - y_plot;  // The origin is typically in the lower right corner.  We need to
                                                                    // invert the Y to get it to plot in the right location.

      const dwin_coord_t by = y_offset + y_plot_inv * pixels_per_y_mesh_pnt;
      dwinDrawRectangle(1, COLOR_SELECT,
        x_offset + (x_plot * pixels_per_x_mesh_pnt), by,
        x_offset + (x_plot * pixels_per_x_mesh_pnt) + pixels_per_x_mesh_pnt, by + pixels_per_y_mesh_pnt
      );

      // Display Mesh Point Locations
      const dwin_coord_t sx = x_offset + pixels_per_x_mesh_pnt / 2;
            dwin_coord_t  y = y_offset + pixels_per_y_mesh_pnt / 2;
      for (uint8_t j = 0; j < (GRID_MAX_POINTS_Y); j++, y += pixels_per_y_mesh_pnt)
        for (uint8_t i = 0, x = sx; i < (GRID_MAX_POINTS_X); i++, x += pixels_per_x_mesh_pnt)
          dwinDrawPoint(COLOR_WHITE, 1, 1, x, y);

      // Put Relevant Text on Display

      // Show X and Y positions at top of screen
      dwin_font.fg = COLOR_WHITE;
      dwin_font.solid = true;
      const xy_pos_t pos = { bedlevel.get_mesh_x(x_plot), bedlevel.get_mesh_y(y_plot) },
                     lpos = pos.asLogical();

      lcd_moveto(
        TERN(DWIN_MARLINUI_LANDSCAPE, ((x_offset + x_map_pixels) / MENU_FONT_WIDTH) + 2, 1),
        TERN(DWIN_MARLINUI_LANDSCAPE, 1, ((y_offset + y_map_pixels) / MENU_LINE_HEIGHT) + 1)
      );
      lcd_put_u8str_P(X_LBL);
      lcd_put_u8str(ftostr52(lpos.x));
      lcd_moveto(
        TERN(DWIN_MARLINUI_LANDSCAPE, ((x_offset + x_map_pixels) / MENU_FONT_WIDTH) + 2, 1),
        TERN(DWIN_MARLINUI_LANDSCAPE, 3, ((y_offset + y_map_pixels) / MENU_LINE_HEIGHT) + 2)
      );
      lcd_put_u8str_P(Y_LBL);
      lcd_put_u8str(ftostr52(lpos.y));

      // Print plot position
      dwin_string.set('(');
      dwin_string.add(i8tostr3rj(x_plot));
      dwin_string.add(',');
      dwin_string.add(i8tostr3rj(y_plot));
      dwin_string.add(')');
      lcd_moveto(
        TERN(DWIN_MARLINUI_LANDSCAPE, ((x_offset + x_map_pixels) / MENU_FONT_WIDTH) + 2, LCD_WIDTH - dwin_string.length),
        TERN(DWIN_MARLINUI_LANDSCAPE, LCD_HEIGHT - 2, ((y_offset + y_map_pixels) / MENU_LINE_HEIGHT) + 1)
      );
      lcd_put_dwin_string();

      // Show the location value
      dwin_string.set(Z_LBL);
      if (!isnan(bedlevel.z_values[x_plot][y_plot]))
        dwin_string.add(ftostr43sign(bedlevel.z_values[x_plot][y_plot]));
      else
        dwin_string.add(F(" -----"));
      lcd_moveto(
        TERN(DWIN_MARLINUI_LANDSCAPE, ((x_offset + x_map_pixels) / MENU_FONT_WIDTH) + 2, LCD_WIDTH - dwin_string.length),
        TERN(DWIN_MARLINUI_LANDSCAPE, LCD_HEIGHT - 1, ((y_offset + y_map_pixels) / MENU_LINE_HEIGHT) + 2)
      );
      lcd_put_dwin_string();
    }

  #endif // AUTO_BED_LEVELING_UBL

  #if ANY(BABYSTEP_GFX_OVERLAY, MESH_EDIT_GFX_OVERLAY)

    void MarlinUI::zoffset_overlay(const int8_t dir) {
      const int rot_up = TERN(OVERLAY_GFX_REVERSE, ICON_RotateCCW, ICON_RotateCW),
              rot_down = TERN(OVERLAY_GFX_REVERSE, ICON_RotateCW, ICON_RotateCCW);

      const int nozzle = (LCD_PIXEL_WIDTH / 2) - 20;

      // Draw a representation of the nozzle
      dwinDrawBox(1, COLOR_BG_BLACK, nozzle + 3, 8, 48, 52); // 'clear' the area where the nozzle is drawn in case it was moved up/down
      dwinIconShow(ICON, ICON_HotendOff, nozzle + 3, 10 - dir);
      dwinIconShow(ICON, ICON_BedLine, nozzle, 10 + 36);

      // Draw cw/ccw indicator and up/down arrows
      const int arrow_y = LCD_PIXEL_HEIGHT / 2 - 24;
      dwinIconShow(ICON, ICON_DownArrow, 0, arrow_y - dir);
      dwinIconShow(ICON, rot_down, 48, arrow_y);

      dwinIconShow(ICON, ICON_UpArrow, LCD_PIXEL_WIDTH - 10 - (48*2), arrow_y - dir);
      dwinIconShow(ICON, rot_up, LCD_PIXEL_WIDTH - 10 - 48, arrow_y);
    }

  #endif // BABYSTEP_GFX_OVERLAY || MESH_EDIT_GFX_OVERLAY

#endif // HAS_MARLINUI_MENU

#endif // IS_DWIN_MARLINUI
