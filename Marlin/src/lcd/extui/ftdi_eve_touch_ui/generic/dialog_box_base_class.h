/***************************
 * dialog_box_base_class.h *
 ***************************/

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

#pragma once

#define FTDI_DIALOG_BOX_BASE_CLASS
#define FTDI_DIALOG_BOX_BASE_CLASS_CLASS DialogBoxBaseClass

class DialogBoxBaseClass : public BaseScreen {
  protected:
    template<typename T> static void drawMessage(T, const int16_t font=0);
    static void drawMessage(FSTR_P const fstr, const int16_t font=0) { drawMessage(FTOP(fstr), font); }

    template<typename T> static void drawButton(T);
    static void drawYesNoButtons(uint8_t default_btn = 0);
    static void drawStartPrintButtons(uint8_t default_btn = 0);
    static void drawOkayButton();
    static void drawDoneButton();
    static void drawFilamentButtons();

    static void onRedraw(draw_mode_t) {}

  public:
    static bool onTouchEnd(uint8_t tag);
    static void onIdle();
};
