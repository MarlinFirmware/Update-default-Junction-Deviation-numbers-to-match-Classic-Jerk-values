/**************************************
 * confirm_user_request_alert_box.cpp *
 **************************************/

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
#include "../screen_data.h"

#ifdef FTDI_CONFIRM_USER_REQUEST_ALERT_BOX

using namespace FTDI;

void ConfirmUserRequestAlertBox::onRedraw(draw_mode_t mode) {
  AlertDialogBox::onRedraw(mode); // Required for the GOTO_SCREEN function to work
}

bool ConfirmUserRequestAlertBox::onTouchEnd(uint8_t tag) {
  switch (tag) {
    case 1:
      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        if (ExtUI::pauseModeStatus == PAUSE_MESSAGE_PURGE || ExtUI::pauseModeStatus == PAUSE_MESSAGE_OPTION)
          ExtUI::setPauseMenuResponse(PAUSE_RESPONSE_RESUME_PRINT);
      #endif
      ExtUI::setUserConfirmed();
      #ifdef FTDI_TUNE_MENU
        if (ExtUI::awaitingUserConfirm()) {
          // The TuneMenu will call ExtUI::setUserConfirmed()
          if (ExtUI::isOngoingPrintJob())
            GOTO_SCREEN(TuneMenu);
          else
            GOTO_SCREEN(StatusScreen);
          current_screen.forget();
          return true;
        }
      #endif
      GOTO_PREVIOUS();
      return true;
    case 2: GOTO_PREVIOUS(); return true;
    default:                 return false;
  }
}

void ConfirmUserRequestAlertBox::show(const char *msg) {
  drawMessage(msg);
  storeBackground();
  screen_data.AlertDialogBox.isError = false;
  if (!AT_SCREEN(ConfirmUserRequestAlertBox))
    GOTO_SCREEN(ConfirmUserRequestAlertBox);
}

void ConfirmUserRequestAlertBox::hide() {
  if (AT_SCREEN(ConfirmUserRequestAlertBox))
    GOTO_PREVIOUS();
}

#endif // FTDI_CONFIRM_USER_REQUEST_ALERT_BOX
