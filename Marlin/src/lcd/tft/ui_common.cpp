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

#include "../../inc/MarlinConfigPre.h"

#if HAS_GRAPHICAL_TFT

#include "ui_common.h"
#include "../lcdprint.h"
#include "../../libs/numtostr.h"
#include "../menu/menu.h"
#include "../../gcode/queue.h"
#include "../../module/temperature.h"
#include "../../module/planner.h"
#include "../../module/probe.h"

#if ENABLED(BABYSTEP_ZPROBE_OFFSET)
  #include "../../feature/babystep.h"
#endif

motionAxisState_t motionAxisState;
static xy_uint_t cursor;
#if ENABLED(TOUCH_SCREEN)
  bool draw_menu_navigation = false;
#endif

void menu_pause_option();

void quick_feedback() {
  #if HAS_CHIRP
    ui.chirp(); // Buzz and wait. Is the delay needed for buttons to settle?
    #if ALL(HAS_MARLINUI_MENU, HAS_BEEPER)
      for (int8_t i = 5; i--;) { buzzer.tick(); delay(2); }
    #elif HAS_MARLINUI_MENU
      delay(10);
    #endif
  #endif
}

void disable_steppers() {
  quick_feedback();
  queue.inject(F("M84"));
}

void moveAxis(const AxisEnum axis, const int8_t direction) {
  quick_feedback();

  #if HAS_EXTRUDERS
    if (axis == E_AXIS && thermalManager.tooColdToExtrude(motionAxisState.e_selection)) {
      drawMessage(F("Too cold"));
      return;
    }
  #endif

  float diff = motionAxisState.currentStepSize * direction;

  #if HAS_BED_PROBE

    if (axis == Z_AXIS && motionAxisState.z_selection == Z_SELECTION_Z_PROBE) {

      #if ENABLED(BABYSTEP_ZPROBE_OFFSET)

        diff = 0;

        const int16_t babystep_increment = direction * BABYSTEP_SIZE_Z;
        const bool do_probe = DISABLED(BABYSTEP_HOTEND_Z_OFFSET) || active_extruder == 0;
        const float bsDiff = planner.mm_per_step[Z_AXIS] * babystep_increment,
                    new_probe_offset = probe.offset.z + bsDiff,
                    new_offs = TERN(BABYSTEP_HOTEND_Z_OFFSET
                      , do_probe ? new_probe_offset : hotend_offset[active_extruder].z - bsDiff
                      , new_probe_offset
                    );
        if (WITHIN(new_offs, PROBE_OFFSET_ZMIN, PROBE_OFFSET_ZMAX)) {
          babystep.add_steps(Z_AXIS, babystep_increment);
          if (do_probe)
            probe.offset.z = new_offs;
          else
            TERN(BABYSTEP_HOTEND_Z_OFFSET, hotend_offset[active_extruder].z = new_offs, NOOP);
          drawMessage_P(NUL_STR); // Clear the error
        }
        else
          drawMessage(GET_TEXT_F(MSG_LCD_SOFT_ENDSTOPS));

      #else // !BABYSTEP_ZPROBE_OFFSET

        // Only change probe.offset.z
        probe.offset.z += diff;
        if (direction < 0 && current_position.z < PROBE_OFFSET_ZMIN) {
          current_position.z = PROBE_OFFSET_ZMIN;
          drawMessage(GET_TEXT_F(MSG_LCD_SOFT_ENDSTOPS));
        }
        else if (direction > 0 && current_position.z > PROBE_OFFSET_ZMAX) {
          current_position.z = PROBE_OFFSET_ZMAX;
          drawMessage(GET_TEXT_F(MSG_LCD_SOFT_ENDSTOPS));
        }
        else
          drawMessage_P(NUL_STR); // Clear the error

      #endif // !BABYSTEP_ZPROBE_OFFSET
    }

  #endif // HAS_BED_PROBE

  if (diff && !ui.manual_move.processing) {
    // Get motion limit from software endstops, if any
    float min, max;
    soft_endstop.get_manual_axis_limits(axis, min, max);

    // Delta limits XY based on the current offset from center
    // This assumes the center is 0,0
    #if ENABLED(DELTA)
      if (axis != Z_AXIS && TERN1(HAS_EXTRUDERS, axis != E_AXIS)) {
        max = SQRT(FLOAT_SQ(PRINTABLE_RADIUS) - sq(current_position[Y_AXIS - axis])); // (Y_AXIS - axis) == the other axis
        min = -max;
      }
    #endif

    // Get the new position
    const bool limited = ui.manual_move.apply_diff(axis, diff, min, max);
    #if IS_KINEMATIC
      UNUSED(limited);
    #else
      FSTR_P const msg = limited ? GET_TEXT_F(MSG_LCD_SOFT_ENDSTOPS) : FPSTR(NUL_STR);
      drawMessage(msg);
    #endif

    ui.manual_move.soon(axis OPTARG(MULTI_E_MANUAL, motionAxisState.e_selection));
  }

  drawAxisValue(axis);
}

#if ENABLED(TOUCH_SCREEN)

  void do_home() {
    quick_feedback();
    drawMessage(GET_TEXT_F(MSG_LEVEL_BED_HOMING));
    queue.inject_P(G28_STR);
    // Disable touch until home is done
    touch.disable();
    TERN_(HAS_EXTRUDERS, drawAxisValue(E_AXIS));
    TERN_(HAS_X_AXIS,    drawAxisValue(X_AXIS));
    TERN_(HAS_Y_AXIS,    drawAxisValue(Y_AXIS));
    TERN_(HAS_Z_AXIS,    drawAxisValue(Z_AXIS));
  }

  void step_size() {
    motionAxisState.currentStepSize = motionAxisState.currentStepSize / 10.0;
    if (motionAxisState.currentStepSize < 0.0015) motionAxisState.currentStepSize = 10.0;
    quick_feedback();
    drawCurStepValue();
  }

  #if HAS_EXTRUDERS
    void e_select() {
      if (++motionAxisState.e_selection >= EXTRUDERS)
        motionAxisState.e_selection = 0;
      quick_feedback();
      drawCurESelection();
      drawAxisValue(E_AXIS);
    }
  #endif

  #if HAS_BED_PROBE
    void z_select() {
      motionAxisState.z_selection *= -1;
      quick_feedback();
      drawCurZSelection();
      drawAxisValue(Z_AXIS);
    }
  #endif

#endif

#if ALL(TOUCH_SCREEN, HAS_DISPLAY_SLEEP)

  bool lcd_sleep_task() {
    static bool sleepCleared;
    if (touch.isSleeping()) {
      tft.queue.reset();
      if (!sleepCleared) {
        sleepCleared = true;
        ui.clear_for_drawing();
        tft.queue.async();
      }
      touch.idle();
      return true;
    }
    else
      sleepCleared = false;
    return false;
  }

#endif

void text_line(const uint16_t y, uint16_t color) {
  tft.canvas(0, y, TFT_WIDTH, MENU_ITEM_HEIGHT);
  tft.set_background(color);
}

void menu_line(const uint8_t row, uint16_t color) {
  cursor.set(0, row);
  text_line(MENU_TOP_LINE_Y + cursor.y * MENU_LINE_HEIGHT, color);
}

uint16_t menu_item(const uint8_t row, const bool sel) {
  const uint16_t lineColor = sel ? COLOR_SELECTION_BG : COLOR_BACKGROUND;

  #if ENABLED(TOUCH_SCREEN)
    if (row == 0) {
      touch.clear();
      draw_menu_navigation = TERN(ADVANCED_PAUSE_FEATURE, ui.currentScreen != menu_pause_option, true);
    }
  #endif

  menu_line(row, lineColor);
  #if ENABLED(TOUCH_SCREEN)
    const TouchControlType tct = TERN(SINGLE_TOUCH_NAVIGATION, true, sel) ? MENU_CLICK : MENU_ITEM;
    touch.add_control(tct, 0, MENU_TOP_LINE_Y + row * MENU_LINE_HEIGHT, TFT_WIDTH, MENU_ITEM_HEIGHT, encoderTopLine + row);
  #endif

  return lineColor;
}

void add_control(uint16_t x, uint16_t y, TouchControlType control_type, intptr_t data, MarlinImage image, bool is_enabled, uint16_t color_enabled, uint16_t color_disabled) {
  const uint16_t width = images[image].width, height = images[image].height;
  tft.canvas(x, y, width, height);
  tft.add_image(0, 0, image, is_enabled ? color_enabled : color_disabled);
  #if ENABLED(TOUCH_SCREEN)
    if (is_enabled) touch.add_control(control_type, x, y, width, height, data);
  #endif
}

void drawBtn(const int x, const int y, const char *label, intptr_t data, const MarlinImage btnimg, const MarlinImage img, uint16_t bgColor, const bool enabled) {
  const uint16_t width = images[btnimg].width,
                height = images[btnimg].height;

  if (!enabled) bgColor = COLOR_CONTROL_DISABLED;

  tft.canvas(x, y, width, height);
  tft.set_background(COLOR_BACKGROUND);
  tft.add_image(0, 0, btnimg, bgColor, COLOR_BACKGROUND, COLOR_DARKGREY);

  // TODO: Make an add_text() taking a font arg
  if (label) {
    tft_string.set(label);
    tft_string.trim();
    tft.add_text(tft_string.center(width), height / 2 - tft_string.font_height() / 2, bgColor, tft_string);
  }
  else
    tft.add_image(0, 0, img, bgColor, COLOR_BACKGROUND, COLOR_DARKGREY);

  TERN_(TOUCH_SCREEN, if (enabled) touch.add_control(BUTTON, x, y, width, height, data));
}

//
// lcdprint.h functions
//

#define TFT_COL_WIDTH ((TFT_WIDTH) / (LCD_WIDTH))

void lcd_gotopixel(const uint16_t x, const uint16_t y) {
  if (x >= TFT_WIDTH) return;
  cursor.set(x / (TFT_COL_WIDTH), y / MENU_LINE_HEIGHT);
  tft.canvas(x, MENU_TOP_LINE_Y + y, (TFT_WIDTH) - x, MENU_ITEM_HEIGHT);
  tft.set_background(COLOR_BACKGROUND);
}

void lcd_moveto(const lcd_uint_t col, const lcd_uint_t row) {
  lcd_gotopixel(int(col) * (TFT_COL_WIDTH), int(row) * MENU_LINE_HEIGHT);
}

int lcd_put_lchar_max(const lchar_t &c, const pixel_len_t max_length) {
  if (max_length < 1) return 0;
  tft_string.set(c);
  tft.add_text(MENU_TEXT_X, MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);
  lcd_gotopixel((cursor.x + 1) * (TFT_COL_WIDTH) + tft_string.width(), cursor.y * MENU_LINE_HEIGHT);
  return tft_string.width();
}

int lcd_put_u8str_max_P(PGM_P utf8_pstr, const pixel_len_t max_length) {
  if (max_length < 1) return 0;
  tft_string.set(utf8_pstr);
  tft_string.trim();
  tft_string.truncate(max_length);
  tft.add_text(MENU_TEXT_X, MENU_TEXT_Y, COLOR_MENU_TEXT, tft_string);
  lcd_gotopixel((cursor.x + 1) * (TFT_COL_WIDTH) + tft_string.width(), cursor.y * MENU_LINE_HEIGHT);
  return tft_string.width();
}

int lcd_put_u8str_max(const char * utf8_str, const pixel_len_t max_length) {
  return lcd_put_u8str_max_P(utf8_str, max_length);
}

void lcd_put_int(const int i) {
  // 3 digits max for this one...
  const char* str = i16tostr3left(int16_t(i));
  lcd_put_u8str_max(str, 3);
}

//
// Menu Item methods
//

// Draw a generic menu item with pre_char (if selected) and post_char
void MenuItemBase::_draw(const bool sel, const uint8_t row, FSTR_P const ftpl, const char pre_char, const char post_char) {
  const uint16_t lineColor = menu_item(row, sel);

  const char *string = FTOP(ftpl);

  #ifndef MENU_ITEM_BACK_LINE_SHIFT
    #define MENU_ITEM_BACK_LINE_SHIFT 0
  #endif

  int8_t line_shift = 0;
  uint16_t color = COLOR_MENU_TEXT;

  // Determine an image to show in the left column
  MarlinImage image = noImage;
  if (pre_char == LCD_STR_UPLEVEL[0]) {
    image = imgLeft;
    color = COLOR_MENU_BACK_TEXT;
  }
  else {
    switch (*string) {
      case LCD_STR_REFRESH[0]: image = imgRefresh; break;
      case LCD_STR_FOLDER[0]: image = imgDirectory; break;
    }
    if (image != noImage) {
      line_shift = MENU_ITEM_ICON_SPACE;
      string++;
      if (*string == ' ') string++;
    }
  }

  // For narrower screens text-only items align fully left
  // For wider screens text aligns on the post-icon column
  uint8_t text_x = (TFT_WIDTH >= 480) ? MENU_ITEM_ICON_SPACE : MENU_TEXT_X;
  if (image != noImage) {
    text_x = MENU_ITEM_ICON_SPACE;
    tft.add_image(MENU_ITEM_ICON_X + line_shift, MENU_ITEM_ICON_Y, image, color, lineColor);
  }

  // Add the text content from ftpl
  tft_string.set(string, itemIndex, itemStringC, itemStringF);
  tft.add_text(text_x + line_shift, MENU_TEXT_Y, color, tft_string);

  if (post_char == LCD_STR_ARROW_RIGHT[0])
    tft.add_image(MENU_ITEM_SUBMENU_ICON_X, MENU_ITEM_ICON_Y, imgRight, color, lineColor);
}

// Draw a menu item with a (potentially) editable value
void MenuEditItemBase::draw(const bool sel, const uint8_t row, FSTR_P const ftpl, const char * const inStr, const bool pgm) {
  (void)menu_item(row, sel);

  constexpr uint8_t text_x = (TFT_WIDTH >= 480) ? MENU_ITEM_ICON_SPACE : MENU_TEXT_X;
  tft_string.set(ftpl, itemIndex, itemStringC, itemStringF);
  tft.add_text(text_x, MENU_TEXT_Y, COLOR_MENU_EDIT_TEXT, tft_string);
  if (inStr) {
    tft_string.set(inStr);
    tft.add_text(MENU_EDIT_ITEM_RIGHT_X - tft_string.width(), MENU_TEXT_Y, COLOR_MENU_VALUE, tft_string);
  }
}

// Draw a static item with no left-right margin required. Centered by default.
void MenuItem_static::draw(const uint8_t row, FSTR_P const ftpl, const uint8_t style/*=SS_DEFAULT*/, const char *vstr/*=nullptr*/) {
  (void)menu_item(row);

  if (ftpl)
    tft_string.set(ftpl, itemIndex, itemStringC, itemStringF);
  else
    tft_string.set();

  const bool center = bool(style & SS_CENTER), full = bool(style & SS_FULL);
  if (!full || !vstr) {
    if (vstr) tft_string.add(vstr);
    tft.add_text(center ? tft_string.center(TFT_WIDTH) : 0, MENU_TEXT_Y, COLOR_MENU_STATIC_TEXT, tft_string);
    return;
  }

  // Move the leading colon from the value to the label
  if (*vstr == ':') { tft_string.add(':'); vstr++; }

  // Left-justified label
  tft.add_text(0, MENU_TEXT_Y, COLOR_MENU_STATIC_TEXT, tft_string);

  // Right-justified value, after spaces
  while (*vstr == ' ') vstr++;
  tft_string.set(vstr);
  tft.add_text(TFT_WIDTH - 1 - tft_string.width(), MENU_TEXT_Y, COLOR_MENU_STATIC_TEXT, tft_string);
}

#if HAS_MEDIA

  void MenuItem_sdbase::draw(const bool sel, const uint8_t row, FSTR_P const, CardReader &theCard, const bool isDir) {
    const uint16_t lineColor = menu_item(row, sel);
    constexpr uint8_t line_shift = (TFT_WIDTH >= 480) ? MENU_ITEM_ICON_SPACE : 0;
    if (isDir) tft.add_image(MENU_ITEM_ICON_X + line_shift, MENU_ITEM_ICON_Y, imgDirectory, COLOR_MENU_TEXT, lineColor);
    const char * const filename = ui.scrolled_filename(theCard, LCD_WIDTH - 1, sel);
    //SERIAL_ECHOLNPGM("Drawing filename: ", filename);
    tft.add_text(MENU_ITEM_ICON_SPACE + line_shift, MENU_TEXT_Y, COLOR_MENU_TEXT, filename);
  }

#endif

//
// MarlinUI methods
//

bool MarlinUI::detected() { return true; }

void MarlinUI::init_lcd() {
  tft.init();
  tft.set_font(MENU_FONT_NAME);
  #ifdef SYMBOLS_FONT_NAME
    tft.add_glyphs(SYMBOLS_FONT_NAME);
  #endif
  #ifdef EXTRA_FONT_NAME
    tft.add_glyphs(EXTRA_FONT_NAME);
  #endif
  TERN_(TOUCH_SCREEN, touch.init());
  clear_for_drawing();
}

void MarlinUI::clear_lcd() {
  #if ENABLED(TOUCH_SCREEN)
    touch.reset();
    draw_menu_navigation = false;
  #endif

  tft.queue.reset();
  tft.fill(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BACKGROUND);
  cursor.set(0, 0);
}

void MarlinUI::clear_for_drawing() { clear_lcd(); }

#if HAS_LCD_BRIGHTNESS

  void MarlinUI::_set_brightness() {
    #if PIN_EXISTS(TFT_BACKLIGHT)
      if (PWM_PIN(TFT_BACKLIGHT_PIN))
        analogWrite(pin_t(TFT_BACKLIGHT_PIN), backlight ? brightness : 0);
    #endif
  }

#endif

#if ENABLED(TOUCH_SCREEN_CALIBRATION)

  void MarlinUI::touch_calibration_screen() {
    uint16_t x, y;

    calibrationState stage = touch_calibration.get_calibration_state();

    if (stage == CALIBRATION_NONE) {
      defer_status_screen(true);
      clear_for_drawing();
      stage = touch_calibration.calibration_start();
    }
    else {
      x = touch_calibration.calibration_points[_MIN(stage - 1, CALIBRATION_BOTTOM_LEFT)].x;
      y = touch_calibration.calibration_points[_MIN(stage - 1, CALIBRATION_BOTTOM_LEFT)].y;
      tft.canvas(x - 15, y - 15, 31, 31);
      tft.set_background(COLOR_BACKGROUND);
    }

    touch.clear();

    if (stage < CALIBRATION_SUCCESS) {
      switch (stage) {
        case CALIBRATION_TOP_LEFT:     tft_string.set(GET_TEXT(MSG_TOP_LEFT));     break;
        case CALIBRATION_TOP_RIGHT:    tft_string.set(GET_TEXT(MSG_TOP_RIGHT));    break;
        case CALIBRATION_BOTTOM_RIGHT: tft_string.set(GET_TEXT(MSG_BOTTOM_RIGHT)); break;
        case CALIBRATION_BOTTOM_LEFT:  tft_string.set(GET_TEXT(MSG_BOTTOM_LEFT));  break;
        default: break;
      }

      x = touch_calibration.calibration_points[stage].x;
      y = touch_calibration.calibration_points[stage].y;

      tft.canvas(x - 15, y - 15, 31, 31);
      tft.set_background(COLOR_BACKGROUND);
      tft.add_bar(0, 15, 31, 1, COLOR_TOUCH_CALIBRATION);
      tft.add_bar(15, 0, 1, 31, COLOR_TOUCH_CALIBRATION);

      touch.add_control(CALIBRATE, 0, 0, TFT_WIDTH, TFT_HEIGHT, uint32_t(x) << 16 | uint32_t(y));
    }
    else {
      tft_string.set(stage == CALIBRATION_SUCCESS ? GET_TEXT(MSG_CALIBRATION_COMPLETED) : GET_TEXT(MSG_CALIBRATION_FAILED));
      defer_status_screen(false);
      touch_calibration.calibration_end();
      touch.add_control(BACK, 0, 0, TFT_WIDTH, TFT_HEIGHT);
    }

    tft.canvas(0, (TFT_HEIGHT - tft_string.font_height()) >> 1, TFT_WIDTH, tft_string.font_height());
    tft.set_background(COLOR_BACKGROUND);
    tft.add_text(tft_string.center(TFT_WIDTH), 0, COLOR_MENU_TEXT, tft_string);
  }

#endif // TOUCH_SCREEN_CALIBRATION

#endif // HAS_GRAPHICAL_TFT
