/*************************
 * statistics_screen.cpp *
 *************************/

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

#ifdef FTDI_STATISTICS_SCREEN

using namespace FTDI;
using namespace ExtUI;
using namespace Theme;

#define GRID_COLS 4
#define GRID_ROWS 7

void StatisticsScreen::onRedraw(draw_mode_t what) {
  CommandProcessor cmd;

  if (what & BACKGROUND) {
    char buffer[21];

    cmd.cmd(CLEAR_COLOR_RGB(Theme::bg_color))
       .cmd(CLEAR(true,true,true))
       .cmd(COLOR_RGB(bg_text_enabled))
       .tag(0)

       .font(Theme::font_medium)
       .text(BTN_POS(1,1), BTN_SIZE(4,1), GET_TEXT_F(MSG_INFO_PRINTER_STATS_MENU))
       .font(Theme::font_small)
       .tag(0)
       .text(BTN_POS(1,2), BTN_SIZE(2,1), GET_TEXT_F(MSG_INFO_PRINT_COUNT),      OPT_RIGHTX | OPT_CENTERY)
       .text(BTN_POS(1,3), BTN_SIZE(2,1), GET_TEXT_F(MSG_INFO_COMPLETED_PRINTS), OPT_RIGHTX | OPT_CENTERY)
       .text(BTN_POS(1,4), BTN_SIZE(2,1), GET_TEXT_F(MSG_INFO_PRINT_TIME),       OPT_RIGHTX | OPT_CENTERY)
       .text(BTN_POS(1,5), BTN_SIZE(2,1), GET_TEXT_F(MSG_INFO_PRINT_LONGEST),    OPT_RIGHTX | OPT_CENTERY)
       .text(BTN_POS(1,6), BTN_SIZE(2,1), GET_TEXT_F(MSG_INFO_PRINT_FILAMENT),   OPT_RIGHTX | OPT_CENTERY);
    // Don't chain the following, it causes strange issues with evaluation ordering!
    cmd.text(BTN_POS(3,2), BTN_SIZE(2,1), getTotalPrints_str(buffer));
    cmd.text(BTN_POS(3,3), BTN_SIZE(2,1), getFinishedPrints_str(buffer));
    cmd.text(BTN_POS(3,4), BTN_SIZE(2,1), getTotalPrintTime_str(buffer));
    cmd.text(BTN_POS(3,5), BTN_SIZE(2,1), getLongestPrint_str(buffer));
    cmd.text(BTN_POS(3,6), BTN_SIZE(2,1), getFilamentUsed_str(buffer));
  }

  if (what & FOREGROUND) {
    cmd.font(Theme::font_medium)
       .colors(action_btn)
       .tag(1).button(BTN_POS(1,7), BTN_SIZE(4,1), GET_TEXT_F(MSG_BUTTON_DONE));
  }
}

bool StatisticsScreen::onTouchEnd(uint8_t tag) {
  switch (tag) {
    case 1:        GOTO_PREVIOUS();              return true;
    default:                                     return false;
  }
}

#endif // FTDI_STATISTICS_SCREEN
