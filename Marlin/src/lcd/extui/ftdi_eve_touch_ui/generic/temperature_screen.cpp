/**************************
 * temperature_screen.cpp *
 **************************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *   Written By Brian Kahl      2023 - FAME3D.                              *
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

#include "../config.h"
#include "../screens.h"

#ifdef FTDI_TEMPERATURE_SCREEN

using namespace FTDI;
using namespace Theme;
using namespace ExtUI;

void TemperatureScreen::onRedraw(draw_mode_t what) {
  CommandProcessor cmd;
  widgets_t w(what);
  #if TOUCH_UI_LCD_TEMP_SCALING == 10
    w.precision(1, DEFAULT_MIDRANGE)
  #else
    w.precision(0, getTargetTemp_celsius(E0) == 0 ? DEFAULT_HIGHEST : DEFAULT_MIDRANGE)
  #endif
   .color(temp_button).units(GET_TEXT_F(MSG_UNITS_C));
  w.heading(GET_TEXT_F(MSG_TEMPERATURE));
  //w.button(30, GET_TEXT_F(MSG_COOLDOWN));
  #ifndef NO_TOOLHEAD_HEATER_GCODE
    #if ENABLED(TOUCH_UI_COCOA_PRESS)
      w.adjuster(   2, GET_TEXT_F(MSG_NOZZLE), getTargetTemp_celsius(E0));
      w.adjuster(   4, GET_TEXT_F(MSG_BODY), getTargetTemp_celsius(E1));
      #if ENABLED(COCOA_PRESS_EXTRA_HEATER)
        if (has_extra_heater())
          w.adjuster(6, GET_TEXT_F(MSG_EXTERNAL), getTargetTemp_celsius(E2));
      #endif
    #elif HOTENDS == 1
      w.adjuster(   2, GET_TEXT_F(MSG_NOZZLE),   getTargetTemp_celsius(E0));
    #else
      w.adjuster(   2, F(STR_E0), getTargetTemp_celsius(E0));
      w.adjuster(   4, F(STR_E1), getTargetTemp_celsius(E1));
      #if HOTENDS > 2
        w.adjuster( 6, F(STR_E2), getTargetTemp_celsius(E2));
      #endif
      #if HOTENDS > 3
        w.adjuster( 8, F(STR_E3), getTargetTemp_celsius(E3));
      #endif
    #endif
  #endif
  #if HAS_HEATED_BED
    w.adjuster(    20, GET_TEXT_F(MSG_BED),     getTargetTemp_celsius(BED));
  #endif
  #if HAS_HEATED_CHAMBER
    w.adjuster(    22, GET_TEXT_F(MSG_CHAMBER), getTargetTemp_celsius(CHAMBER));
  #endif
  #if HAS_FAN0
    w.units(GET_TEXT_F(MSG_UNITS_PERCENT));
    w.adjuster(    10, GET_TEXT_F(MSG_FAN_SPEED), getTargetFan_percent(FAN0));
  #endif
  w.increments();

  #define PREHEAT_1_POS         BTN_POS(1,(5+EXTRUDERS)), BTN_SIZE(1,1)
  #define PREHEAT_2_POS         BTN_POS(2,(5+EXTRUDERS)), BTN_SIZE(1,1)
  #define PREHEAT_3_POS         BTN_POS(1,(6+EXTRUDERS)), BTN_SIZE(1,1)
  #define COOLDOWN_POS          BTN_POS(2,(6+EXTRUDERS)), BTN_SIZE(1,1)
  #define TOOLHEAD_SWAP_POS     BTN_POS(1,(7+EXTRUDERS)), BTN_SIZE(2,1)

  if (what & FOREGROUND) {
    #define GRID_COLS 2
    #define GRID_ROWS (8+EXTRUDERS)
    if (!ExtUI::isOngoingPrintJob()) {
      cmd.font(Theme::font_medium)
          .colors(normal_btn);
          #ifdef PREHEAT_1_COMMAND
            cmd.tag(31).button(PREHEAT_1_POS, GET_TEXT_F(MSG_PREHEAT_1));
          #endif
          #ifdef PREHEAT_2_COMMAND
            cmd.tag(32).button(PREHEAT_2_POS, GET_TEXT_F(MSG_PREHEAT_2));
          #endif
          #ifdef PREHEAT_3_COMMAND
            cmd.tag(33).button(PREHEAT_3_POS, GET_TEXT_F(MSG_PREHEAT_3));
          #endif
          cmd.colors(cold_pull_btn)
            .tag(30).button(COOLDOWN_POS, GET_TEXT_F(MSG_COOLDOWN));
          #ifdef PARKING_COMMAND_GCODE
            cmd.colors(normal_btn).tag(35).button(TOOLHEAD_SWAP_POS, GET_TEXT_F(MSG_TOOL_HEAD_SWAP));
          #endif
    }
  }
}

bool TemperatureScreen::onTouchHeld(uint8_t tag) {
  const float increment = getIncrement();
  switch (tag) {
    case 20: UI_DECREMENT(TargetTemp_celsius, BED); break;
    case 21: UI_INCREMENT(TargetTemp_celsius, BED); break;
    case 22: UI_DECREMENT(TargetTemp_celsius, CHAMBER); break;
    case 23: UI_INCREMENT(TargetTemp_celsius, CHAMBER); break;
    #ifndef NO_TOOLHEAD_HEATER_GCODE
      case  2: UI_DECREMENT(TargetTemp_celsius, E0); break;
      case  3: UI_INCREMENT(TargetTemp_celsius, E0); break;
    #endif
    #if HAS_MULTI_HOTEND
      case  4: UI_DECREMENT(TargetTemp_celsius, E1); break;
      case  5: UI_INCREMENT(TargetTemp_celsius, E1); break;
    #endif
    #if HOTENDS > 2
      case  6: UI_DECREMENT(TargetTemp_celsius, E2); break;
      case  7: UI_INCREMENT(TargetTemp_celsius, E2); break;
    #endif
    #if HOTENDS > 3
      case  8: UI_DECREMENT(TargetTemp_celsius, E3); break;
      case  9: UI_INCREMENT(TargetTemp_celsius, E3); break;
    #endif
    #if HAS_FAN
      case 10: UI_DECREMENT(TargetFan_percent, FAN0); break;
      case 11: UI_INCREMENT(TargetFan_percent, FAN0); break;
    #endif
    case 30:
      coolDown();
      TERN_(HAS_HEATED_CHAMBER, setTargetTemp_celsius(0, CHAMBER));
      GOTO_SCREEN(StatusScreen);
      break;
    #ifdef PREHEAT_1_COMMAND
      case 31: injectCommands_P(PSTR(PREHEAT_1_COMMAND)); GOTO_SCREEN(StatusScreen); break;
    #endif
    #ifdef PREHEAT_2_COMMAND
      case 32: injectCommands_P(PSTR(PREHEAT_2_COMMAND)); GOTO_SCREEN(StatusScreen); break;
    #endif
    #ifdef PREHEAT_3COMMAND
      case 33: injectCommands_P(PSTR(PREHEAT_3_COMMAND)); GOTO_SCREEN(StatusScreen); break;
    #endif
    #ifdef PARKING_COMMAND_GCODE
      case 35: injectCommands(F(PARKING_COMMAND_GCODE)); break;
    #endif
    default:
      return false;
  }
  return true;
}

#endif // FTDI_TEMPERATURE_SCREEN
