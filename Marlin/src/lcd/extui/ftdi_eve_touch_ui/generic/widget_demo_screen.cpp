/**************************
 * widget_demo_screen.cpp *
 **************************/

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

#ifdef FTDI_WIDGET_DEMO_SCREEN

using namespace FTDI;
using namespace ExtUI;
using namespace Theme;

uint16_t slider_val;
bool     show_grid;

void WidgetsScreen::onEntry() {
  BaseScreen::onEntry();
  CLCD::turn_on_backlight();
  SoundPlayer::set_volume(255);
}

void WidgetsScreen::onRedraw(draw_mode_t) {
  using namespace ExtUI;
  CommandProcessor cmd;
  cmd.cmd(CLEAR_COLOR_RGB(bg_color))
     .cmd(CLEAR(true,true,true))
     .cmd(COLOR_RGB(bg_text_enabled))
     .tag(0);

  const uint16_t hrs = (slider_val*12/0xFFFFU);
  const uint16_t m   = (slider_val*12*60/0xFFFFU)%60;
  const uint16_t s   = (slider_val*12*60*60/0xFFFFU)%60;

  #if ENABLED(TOUCH_UI_PORTRAIT)
    #define GRID_COLS 3
    #define GRID_ROWS 8
    cmd.font(font_large)
       .cmd(COLOR_RGB(bg_text_enabled))
              .text      (BTN_POS(1,1),  BTN_SIZE(3,1), F("Sample Widgets"))
       .tag(0).text      (BTN_POS(2,6),  BTN_SIZE(1,1), F("Show grid:"))
       .colors(ui_toggle)
       .tag(2).dial      (BTN_POS(1,2),  BTN_SIZE(1,2), slider_val)
       .tag(0).clock     (BTN_POS(1,4),  BTN_SIZE(1,2), hrs, m, s, 0)
              .gauge     (BTN_POS(1,6),  BTN_SIZE(1,2), 5, 4, slider_val,  0xFFFFU)
       .font(font_medium)
       .colors(ui_slider)
       .tag(4).slider    (BTN_POS(2,3),  BTN_SIZE(2,1), slider_val,        0xFFFFU)
       .tag(5).progress  (BTN_POS(2,4),  BTN_SIZE(2,1), slider_val,        0xFFFFU)
       .tag(6).scrollbar (BTN_POS(2,5),  BTN_SIZE(2,1), slider_val, 1000,  0xFFFFU)
       .font(font_small)
       .colors(ui_toggle)
       .tag(7).toggle    (BTN_POS(3,6),  BTN_SIZE(1,1), F("no\xFFyes"), show_grid)
       .colors(normal_btn)
       .font(font_medium)
       .tag(1)
              .button    (BTN_POS(2, 8), BTN_SIZE(1,1), F("1"))
              .button    (BTN_POS(3, 8), BTN_SIZE(1,1), F("2"))
       .colors(action_btn)
              .button    (BTN_POS(1, 8), BTN_SIZE(1,1), F("Back"));
  #else
    #define GRID_COLS 4
    #define GRID_ROWS 8

    cmd.font(font_large)
              .text      (BTN_POS(1,1),  BTN_SIZE(4,1), F("Sample Widgets"))
       .tag(0).text      (BTN_POS(3,6),  BTN_SIZE(1,1), F("Show grid:"))
       .colors(ui_toggle)
       .tag(2).dial      (BTN_POS(1,2),  BTN_SIZE(1,3), slider_val)
       .tag(3).dial      (BTN_POS(1,5),  BTN_SIZE(1,3), slider_val)
       .tag(0).clock     (BTN_POS(2,2),  BTN_SIZE(1,3), hrs, m, s, 0)
              .gauge     (BTN_POS(2,5),  BTN_SIZE(1,3), 5, 4, slider_val,  0xFFFFU)
       .font(font_medium)
       .colors(ui_slider)
       .tag(4).slider    (BTN_POS(3,3),  BTN_SIZE(2,1), slider_val,        0xFFFFU)
       .tag(5).progress  (BTN_POS(3,4),  BTN_SIZE(2,1), slider_val,        0xFFFFU)
       .tag(6).scrollbar (BTN_POS(3,5),  BTN_SIZE(2,1), slider_val, 1000,  0xFFFFU)
       .font(font_small)
       .colors(ui_toggle)
       .tag(7).toggle    (BTN_POS(4,6),  BTN_SIZE(1,1), F("no\xFFyes"), show_grid)
       .colors(normal_btn)
       .font(font_medium)
       .tag(1).button    (BTN_POS(3, 8), BTN_SIZE(1,1), F("1"))
              .button    (BTN_POS(4, 8), BTN_SIZE(1,1), F("2"))
       .colors(action_btn)
              .button    (BTN_POS(1, 8), BTN_SIZE(2,1), F("Back"));
  #endif

  cmd.cmd(COLOR_RGB(bg_text_enabled));
  if (show_grid) DRAW_LAYOUT_GRID
}

bool WidgetsScreen::onTouchStart(uint8_t tag) {
  CommandProcessor cmd;
  switch (tag) {
    case 1: GOTO_PREVIOUS();                                               break;
  #if ENABLED(TOUCH_UI_PORTRAIT)
    case 2: cmd.track_circular (BTN_POS(1,2), BTN_SIZE(1,2), 2).execute(); break;
    case 4: cmd.track_linear   (BTN_POS(2,3), BTN_SIZE(2,1), 4).execute(); break;
    case 5: cmd.track_linear   (BTN_POS(2,4), BTN_SIZE(2,1), 5).execute(); break;
    case 6: cmd.track_linear   (BTN_POS(2,5), BTN_SIZE(2,1), 6).execute(); break;
  #else
    case 2: cmd.track_circular (BTN_POS(1,2), BTN_SIZE(1,3), 2).execute(); break;
    case 3: cmd.track_circular (BTN_POS(1,5), BTN_SIZE(1,3), 3).execute(); break;
    case 4: cmd.track_linear   (BTN_POS(3,3), BTN_SIZE(2,1), 4).execute(); break;
    case 5: cmd.track_linear   (BTN_POS(3,4), BTN_SIZE(2,1), 5).execute(); break;
    case 6: cmd.track_linear   (BTN_POS(3,5), BTN_SIZE(2,1), 6).execute(); break;
  #endif
    case 7: FLIP(show_grid); break;
    default:
      return false;
  }

  return true;
}

void WidgetsScreen::onIdle() {
  if (refresh_timer.elapsed(TOUCH_UPDATE_INTERVAL)) {
    refresh_timer.start();

    uint16_t value;
    CommandProcessor cmd;
    switch (cmd.track_tag(value)) {
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
        slider_val = value; break;
      default:
        return;
    }
    onRefresh();
  }
  BaseScreen::onIdle();
}

#endif // FTDI_WIDGET_DEMO_SCREEN
