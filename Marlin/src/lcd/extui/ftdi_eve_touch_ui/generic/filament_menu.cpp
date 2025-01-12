/*********************
 * filament_menu.cpp *
 *********************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
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

#ifdef FTDI_FILAMENT_MENU

using namespace FTDI;
using namespace ExtUI;
using namespace Theme;

#if ENABLED(TOUCH_UI_PORTRAIT)
  #define GRID_COLS 2
  #define GRID_ROWS 9
  #define TITLE_POS          BTN_POS(1,1), BTN_SIZE(2,1)
  #define RUNOUT_SENSOR_POS  BTN_POS(1,2), BTN_SIZE(2,1)
  #define LIN_ADVANCE_POS    BTN_POS(1,3), BTN_SIZE(2,1)
  #define BACK_POS           BTN_POS(1,9), BTN_SIZE(2,1)
#else
  #define GRID_COLS 2
  #define GRID_ROWS 6
  #define TITLE_POS          BTN_POS(1,1), BTN_SIZE(2,1)
  #define RUNOUT_SENSOR_POS  BTN_POS(1,2), BTN_SIZE(2,1)
  #define LIN_ADVANCE_POS    BTN_POS(1,3), BTN_SIZE(2,1)
  #define BACK_POS           BTN_POS(1,6), BTN_SIZE(2,1)
#endif

void FilamentMenu::onRedraw(draw_mode_t what) {
  if (what & BACKGROUND) {
    CommandProcessor cmd;
    cmd.cmd(CLEAR_COLOR_RGB(Theme::bg_color))
       .cmd(CLEAR(true,true,true))
       .tag(0);
  }

  if (what & FOREGROUND) {
    CommandProcessor cmd;
    cmd.font(font_large)
       .text(TITLE_POS, GET_TEXT_F(MSG_FILAMENT))
       .font(font_medium).colors(normal_btn)
       .enabled(ENABLED(HAS_FILAMENT_SENSOR))
       .tag(2).button(RUNOUT_SENSOR_POS, GET_TEXT_F(MSG_RUNOUT_SENSOR))
       .enabled(ENABLED(LIN_ADVANCE))
       .tag(3).button(LIN_ADVANCE_POS, GET_TEXT_F(MSG_LINEAR_ADVANCE))
       .colors(action_btn)
       .tag(1).button(BACK_POS, GET_TEXT_F(MSG_BUTTON_DONE));
  }
}

bool FilamentMenu::onTouchEnd(uint8_t tag) {
  switch (tag) {
    case 1: GOTO_PREVIOUS(); break;
    #if HAS_FILAMENT_SENSOR
      case 2: GOTO_SCREEN(FilamentRunoutScreen); break;
    #endif
    #if ENABLED(LIN_ADVANCE)
      case 3: GOTO_SCREEN(LinearAdvanceScreen);  break;
    #endif
    default: return false;
  }
  return true;
}

#endif // FTDI_FILAMENT_MENU
