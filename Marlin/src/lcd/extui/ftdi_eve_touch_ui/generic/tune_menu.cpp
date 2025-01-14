/*******************
 * tune_menu.cpp *
 *******************/

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

#ifdef FTDI_TUNE_MENU

#include "../../../../feature/host_actions.h"

using namespace FTDI;
using namespace Theme;

#define GRID_COLS 2
#define GRID_ROWS TERN(TOUCH_UI_PORTRAIT, 7, 5)

void TuneMenu::onRedraw(draw_mode_t what) {
  if (what & BACKGROUND) {
    CommandProcessor cmd;
    cmd.cmd(CLEAR_COLOR_RGB(bg_color))
       .cmd(CLEAR(true,true,true));
  }

  #if ENABLED(TOUCH_UI_PORTRAIT)
    #define STOP_POS              BTN_POS(1,1), BTN_SIZE(2,1)
    #define PAUSE_POS             BTN_POS(1,2), BTN_SIZE(2,1)
    #define FLOW_POS              BTN_POS(1,3), BTN_SIZE(2,1)
    #define TEMPERATURE_POS       BTN_POS(1,4), BTN_SIZE(2,1)
    #define ABOUT_PRINTER_POS     BTN_POS(1,5), BTN_SIZE(2,1)
    #define ADVANCED_SETTINGS_POS BTN_POS(1,6), BTN_SIZE(2,1)
    #define BACK_POS              BTN_POS(1,7), BTN_SIZE(2,1)
  #else
    #define TEMPERATURE_POS BTN_POS(1,1), BTN_SIZE(1,1)
    #define PAUSE_POS       BTN_POS(1,3), BTN_SIZE(1,1)
    #define STOP_POS        BTN_POS(2,3), BTN_SIZE(1,1)
    #define FLOW_POS        BTN_POS(2,4), BTN_SIZE(1,1)
    #define ADVANCED_SETTINGS_POS BTN_POS(1,5), BTN_SIZE(1,1)
    #define ABOUT_PRINTER_POS     BTN_POS(1,5), BTN_SIZE(2,1)
    #define BACK_POS        BTN_POS(2,5), BTN_SIZE(1,1)
  #endif

  if (what & FOREGROUND) {
    const bool sdOrHostPaused   = ExtUI::isPrintingPaused();

    CommandProcessor cmd;
    cmd.colors(cancel_btn)
       .font(font_medium)
       .tag(2).button(STOP_POS, GET_TEXT_F(MSG_CANCEL_PRINT))
       .colors(normal_btn)
       .tag(sdOrHostPaused ? 4 : 3)
       .button(PAUSE_POS, sdOrHostPaused ? GET_TEXT_F(MSG_RESUME_PRINT) : GET_TEXT_F(MSG_PAUSE_PRINT))
       .tag(5).button(FLOW_POS, GET_TEXT_F(MSG_FLOW))
       .tag(6).button(TEMPERATURE_POS, GET_TEXT_F(MSG_TEMPERATURE))
       .tag(7).button(ABOUT_PRINTER_POS, GET_TEXT_F(MSG_INFO_MENU))
       .tag(8).button(ADVANCED_SETTINGS_POS, GET_TEXT_F(MSG_ADVANCED_SETTINGS))
       .tag(1).colors(action_btn)
             .button(BACK_POS, GET_TEXT_F(MSG_BUTTON_DONE));
  }
}

bool TuneMenu::onTouchEnd(uint8_t tag) {
  using namespace Theme;
  using namespace ExtUI;
  switch (tag) {
    case 1: SaveSettingsDialogBox::promptToSaveSettings(); break;
    case 2:
      GOTO_SCREEN(ConfirmAbortPrintDialogBox);
      current_screen.forget();
      PUSH_SCREEN(StatusScreen);
      break;
    case 3: injectCommands(F("M117 Print Paused")); pausePrint();  break;
    case 4: injectCommands(F("M117 Print Resumed")); resumePrint(); break;
    case 5: GOTO_SCREEN(FlowPercentScreen);    break;
    case 6: GOTO_SCREEN(TemperatureScreen);    break;
    case 7: GOTO_SCREEN(AboutScreen);          break;
    case 8: GOTO_SCREEN(AdvancedSettingsMenu); break;
    default:
      return false;
  }
  return true;
}

void TuneMenu::pausePrint() {
  // sound.play(twinkle, PLAY_ASYNCHRONOUS);

  // if (ExtUI::isPrintingFromMedia())
  //   ExtUI::pausePrint();
  // #ifdef ACTION_ON_PAUSE
  //   else hostui.pause();
  // #endif

    // This
  sound.play(twinkle, PLAY_ASYNCHRONOUS);

  if (ExtUI::isPrintingFromMedia())
  {
    SERIAL_ECHOLNPGM("Pause: isPrintingFromMedia");
    ExtUI::pausePrint();
  }
  #ifdef ACTION_ON_PAUSE
    else
    {
      SERIAL_ECHOLNPGM("Pause: hostui.pause");
      hostui.pause();
    }
  #endif
  GOTO_SCREEN(StatusScreen);
}

void TuneMenu::resumePrint() {
  // This
  sound.play(twinkle, PLAY_ASYNCHRONOUS);


  // Something is wrong with this
  if (ExtUI::awaitingUserConfirm() && ExtUI::isPrintingPaused())
  {
     SERIAL_ECHOLNPGM("Resume: Awaiting User Confirm");
     #if ENABLED(ADVANCED_PAUSE_FEATURE)
     ExtUI::setPauseMenuResponse(PAUSE_RESPONSE_RESUME_PRINT);
     #endif
     ExtUI::setUserConfirmed();
  }
  if (ExtUI::isPrintingFromMedia())
  {
    ExtUI::setUserConfirmed();
    SERIAL_ECHOLNPGM("Resume: isPrintingFromMedia");
    ExtUI::resumePrint();
  }
  #ifdef ACTION_ON_RESUME
    else
    {
      SERIAL_ECHOLNPGM("Resume: hostui.resume");
      hostui.resume();
    }
  #endif

  // This works
  GOTO_SCREEN(StatusScreen);
}

#endif // FTDI_TUNE_MENU
