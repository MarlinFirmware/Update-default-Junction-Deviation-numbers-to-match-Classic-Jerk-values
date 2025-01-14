/************************
 * filament_prompt_dialog_box.cpp *
 ************************/

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

#ifdef FTDI_FILAMENT_PROMPT_DIALOG_BOX

constexpr static FilamentPromptDialogBoxData &mydata = screen_data.FilamentPromptDialogBox;

using namespace FTDI;
using namespace Theme;

void FilamentPromptDialogBox::onEntry() {
  BaseScreen::onEntry();
  sound.play(mydata.isError ? sad_trombone : twinkle, PLAY_ASYNCHRONOUS);
}

void FilamentPromptDialogBox::onRedraw(draw_mode_t what) {
  if (what & FOREGROUND) {
    drawMessage(GET_TEXT_F(MSG_FILAMENT_CHANGE_PURGE_CONTINUE));
    drawFilamentButtons();
  }
}

void FilamentPromptDialogBox::show() {
  drawMessage(GET_TEXT_F(MSG_FILAMENT_CHANGE_PURGE_CONTINUE));
  drawFilamentButtons();
  storeBackground();
  mydata.isError = false;
  GOTO_SCREEN(FilamentPromptDialogBox);
}

void FilamentPromptDialogBox::showError() {
  drawMessage(GET_TEXT_F(MSG_FILAMENT_CHANGE_PURGE_CONTINUE));
  storeBackground();
  mydata.isError = true;
  GOTO_SCREEN(FilamentPromptDialogBox);
}

void FilamentPromptDialogBox::hide() {
  if (AT_SCREEN(FilamentPromptDialogBox))
    GOTO_PREVIOUS();
}

#endif // FTDI_ALERT_DIALOG_BOX
