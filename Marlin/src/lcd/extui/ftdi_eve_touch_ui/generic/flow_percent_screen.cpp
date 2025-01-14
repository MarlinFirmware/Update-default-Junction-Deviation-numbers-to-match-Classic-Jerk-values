/***************************
 * flow_percent_screen.cpp *
 ***************************/

/****************************************************************************
 *   Written By Marcio Teixeira 2021 - Cocoa Press                          *
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

#ifdef FTDI_FLOW_PERCENT_SCREEN

using namespace FTDI;
using namespace ExtUI;

void FlowPercentScreen::onRedraw(draw_mode_t what) {
  widgets_t w(what);
  w.precision(0).units(GET_TEXT_F(MSG_UNITS_PERCENT));

  w.heading(GET_TEXT_F(MSG_FLOW_PERCENTAGE));
  w.adjuster(4,  GET_TEXT_F(MSG_FLOW), getFlow_percent(E0));
  w.increments();
}

bool FlowPercentScreen::onTouchHeld(uint8_t tag) {
  const float increment = getIncrement();
  switch (tag) {
    case 4: UI_DECREMENT(Flow_percent, E0); break;
    case 5: UI_INCREMENT(Flow_percent, E0); break;
    default:
      return false;
  }
  return true;
}

#endif // FTDI_FLOW_PERCENT_SCREEN
