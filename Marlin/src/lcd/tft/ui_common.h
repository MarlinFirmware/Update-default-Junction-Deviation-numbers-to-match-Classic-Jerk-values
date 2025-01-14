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
#pragma once

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(NO_LCD_MENUS)
  #error "Seriously? High resolution TFT screen without menu?"
#endif

#include "tft.h"
#include "tft_image.h"

#if ENABLED(TOUCH_SCREEN)
  #include "touch.h"
  extern bool draw_menu_navigation;
#else
  // add_control() function is used to display encoder-controlled elements
  enum TouchControlType : uint16_t {
    NONE = 0x0000,
  };
#endif

#define UI_INCL_(W, H) STRINGIFY_(ui_##W##x##H.h)
#define UI_INCL(W, H) UI_INCL_(W, H)

#include UI_INCL(TFT_WIDTH, TFT_HEIGHT)
#include "ui_theme.h"
#include "tft_font.h"
#include "tft_color.h"

#ifndef BOOTSCREEN_LOGO_X
  #define BOOTSCREEN_LOGO_X (TFT_WIDTH - BOOTSCREEN_LOGO_W) / 2
#endif
#ifndef BOOTSCREEN_LOGO_Y
  #define BOOTSCREEN_LOGO_Y (TFT_HEIGHT - BOOTSCREEN_LOGO_H) / 2
#endif

#ifndef MENU_EDIT_ITEM_RIGHT_X
  #define MENU_EDIT_ITEM_RIGHT_X ((TFT_WIDTH) - (MENU_TEXT_X))
#endif
#ifndef MENU_ITEM_SUBMENU_ICON_X
  #define MENU_ITEM_SUBMENU_ICON_X (TFT_WIDTH - 32)
#endif

// Common Implementation
#define Z_SELECTION_Z 1
#define Z_SELECTION_Z_PROBE -1

typedef struct {
  #if HAS_X_AXIS
    xy_int_t xValuePos;
  #endif
  #if HAS_Y_AXIS
    xy_int_t yValuePos;
  #endif
  #if HAS_Z_AXIS
    xy_int_t zValuePos, zTypePos;
    int z_selection = Z_SELECTION_Z;
  #endif
  #if HAS_EXTRUDERS
    xy_int_t eValuePos, eNamePos;
    uint8_t e_selection = 0;
  #endif
  xy_int_t stepValuePos;
  float currentStepSize = 10.0;
  bool blocked = false;
  char message[32];
} motionAxisState_t;

extern motionAxisState_t motionAxisState;

void moveAxis(const AxisEnum axis, const int8_t direction);

#if HAS_EXTRUDERS
  inline void e_plus()  { moveAxis(E_AXIS, +1); }
  inline void e_minus() { moveAxis(E_AXIS, -1); }
#endif
#if HAS_X_AXIS
  inline void x_minus() { moveAxis(X_AXIS, -1); }
  inline void x_plus()  { moveAxis(X_AXIS, +1); }
#endif
#if HAS_Y_AXIS
  inline void y_plus()  { moveAxis(Y_AXIS, +1); }
  inline void y_minus() { moveAxis(Y_AXIS, -1); }
#endif
#if HAS_Z_AXIS
  inline void z_plus()  { moveAxis(Z_AXIS, +1); }
  inline void z_minus() { moveAxis(Z_AXIS, -1); }
#endif
void quick_feedback();
void disable_steppers();
#if ENABLED(TOUCH_SCREEN)
  void do_home();
  void step_size();
  #if HAS_BED_PROBE
    void z_select();
  #endif
  #if HAS_EXTRUDERS
    void e_select();
  #endif
  #if HAS_DISPLAY_SLEEP
    bool lcd_sleep_task();
  #endif
#endif

void draw_heater_status(uint16_t x, uint16_t y, const int8_t heater);
void draw_fan_status(uint16_t x, uint16_t y, const bool blink);

void text_line(const uint16_t y, uint16_t color=COLOR_BACKGROUND);
void menu_line(const uint8_t row, uint16_t color=COLOR_BACKGROUND);
uint16_t menu_item(const uint8_t row, bool sel = false);

typedef void (*screenFunc_t)();
void add_control(
  uint16_t x, uint16_t y, TouchControlType control_type, intptr_t data, MarlinImage image, bool is_enabled=true,
  uint16_t color_enabled=COLOR_CONTROL_ENABLED, uint16_t color_disabled=COLOR_CONTROL_DISABLED
);
inline void add_control(
  uint16_t x, uint16_t y, TouchControlType control_type, MarlinImage image,
  bool is_enabled=true, uint16_t color_enabled=COLOR_CONTROL_ENABLED, uint16_t color_disabled=COLOR_CONTROL_DISABLED
) {
  add_control(x, y, control_type, 0, image, is_enabled, color_enabled, color_disabled);
}
#if ENABLED(TOUCH_SCREEN)
  inline void add_control(
    uint16_t x, uint16_t y, TouchControlType control_type, screenFunc_t action, MarlinImage image, bool is_enabled=true,
    uint16_t color_enabled=COLOR_CONTROL_ENABLED, uint16_t color_disabled=COLOR_CONTROL_DISABLED
  ) {
    add_control(x, y, control_type, (intptr_t)action, image, is_enabled, color_enabled, color_disabled);
  }
  inline void add_control(
    uint16_t x, uint16_t y, screenFunc_t screen, MarlinImage image, bool is_enabled=true,
    uint16_t color_enabled=COLOR_CONTROL_ENABLED, uint16_t color_disabled=COLOR_CONTROL_DISABLED
  ) {
    add_control(x, y, MENU_SCREEN, (intptr_t)screen, image, is_enabled, color_enabled, color_disabled);
  }
#endif

void drawBtn(const int x, const int y, const char *label, intptr_t data, const MarlinImage btnimg, const MarlinImage img, uint16_t bgColor, const bool enabled=true);
void drawBtn(const int x, const int y, const char *label, intptr_t data, const MarlinImage img, uint16_t bgColor, const bool enabled=true);
inline void drawBtn(const int x, const int y, const char *label, void (*handler)(), const MarlinImage img, uint16_t bgColor, const bool enabled=true) {
  drawBtn(x, y, label, intptr_t(handler), img, bgColor, enabled);
}

// Custom Implementation
void drawMessage_P(PGM_P const msg);
inline void drawMessage(FSTR_P const fmsg) { drawMessage_P(FTOP(fmsg)); }

void drawAxisValue(const AxisEnum axis);
void drawCurZSelection();
void drawCurESelection();
void drawCurStepValue();

#define ABSOLUTE_ZERO     -273.15

#if DISABLED(CUSTOM_STATUS_SCREEN_ITEMS_ORDER)
  enum {
    OPTITEM(HAS_EXTRUDERS, ITEM_E0)
    OPTITEM(HAS_MULTI_HOTEND, ITEM_E1)
    #if HOTENDS > 2
      ITEM_E2,
    #endif
    OPTITEM(HAS_HEATED_BED, ITEM_BED)
    OPTITEM(HAS_TEMP_CHAMBER, ITEM_CHAMBER)
    OPTITEM(HAS_TEMP_COOLER, ITEM_COOLER)
    OPTITEM(HAS_FAN, ITEM_FAN)
    ITEMS_COUNT
  };
#endif
