/**************************************
 * filament_prompt_box.cpp *
 **************************************/

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
#include "../screen_data.h"

#ifdef FTDI_FILAMENT_PROMPT_BOX

using namespace FTDI;

// Need to be renamed to Filament Purge prompt?
void FilamentPromptBox::onRedraw(draw_mode_t mode) {
  AlertDialogBox::onRedraw(mode); // Required for the GOTO_SCREEN function to work
  //GET_TEXT_F(MSG_FILAMENT_CHANGE_PURGE_CONTINUE),GET_TEXT_F(MSG_FILAMENT_CHANGE_OPTION_PURGE), GET_TEXT_F(MSG_FILAMENT_CHANGE_OPTION_RESUME))
  drawMessage(GET_TEXT_F(MSG_FILAMENT_CHANGE_PURGE_CONTINUE));
  drawFilamentButtons();
}

bool FilamentPromptBox::onTouchEnd(uint8_t tag) {
  switch (tag) {
    case 1:
      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        pause_menu_response = PAUSE_RESPONSE_EXTRUDE_MORE;
      #endif
      return true;
    case 2:
      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
      #endif
      GOTO_SCREEN(StatusScreen);
      return true;
    default:
      return false;
  }
}

void FilamentPromptBox::show() {
  drawMessage(GET_TEXT_F(MSG_FILAMENT_CHANGE_PURGE_CONTINUE));
  drawFilamentButtons();
  storeBackground();
  screen_data.AlertDialogBox.isError = false;
  if (!AT_SCREEN(FilamentPromptBox))
    GOTO_SCREEN(FilamentPromptBox);
}

void FilamentPromptBox::hide() {
  if (AT_SCREEN(FilamentPromptBox))
    GOTO_PREVIOUS();
}

#endif // FTDI_FILAMENT_PROMPT_BOX
