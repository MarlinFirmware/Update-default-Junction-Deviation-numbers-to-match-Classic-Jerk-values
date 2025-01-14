/*************************
 * case_light_screen.cpp *
 *************************/

/****************************************************************************
 *   Written By Marcio Teixeira 2019 - Cocoa Press                          *
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

#ifdef FTDI_CASE_LIGHT_SCREEN

using namespace FTDI;
using namespace ExtUI;
using namespace Theme;

void CaseLightScreen::onRedraw(draw_mode_t what) {
  widgets_t w(what);
  w.heading(   GET_TEXT_F(MSG_CASE_LIGHT));
  w.toggle( 2, GET_TEXT_F(MSG_LIGHTS), getCaseLightState());
  #if DISABLED(CASE_LIGHT_NO_BRIGHTNESS)
  w.precision(0).units(GET_TEXT_F(MSG_UNITS_PERCENT))
                .adjuster(10, GET_TEXT_F(MSG_CASE_LIGHT_BRIGHTNESS), getCaseLightBrightness_percent());
  w.precision(0).increments();
  #endif
}

bool CaseLightScreen::onTouchHeld(uint8_t tag) {
  using namespace ExtUI;
  #if DISABLED(CASE_LIGHT_NO_BRIGHTNESS)
    const float increment = getIncrement();
  #endif
  switch (tag) {
    case 2: setCaseLightState(!getCaseLightState()); break;
    #if DISABLED(CASE_LIGHT_NO_BRIGHTNESS)
      case  10: UI_DECREMENT(CaseLightBrightness_percent); break;
      case  11: UI_INCREMENT(CaseLightBrightness_percent); break;
    #endif
    default:
      return false;
  }

  SaveSettingsDialogBox::settingsChanged();
  return true;
}

#endif // FTDI_CASE_LIGHT_SCREEN
