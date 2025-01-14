/***********************
 * z_offset_screen.cpp *
 ***********************/

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

#ifdef COCOA_Z_OFFSET_SCREEN

#include "z_offset_wizard.h"

using namespace FTDI;
using namespace ExtUI;
using namespace Theme;

void ZOffsetScreen::onRedraw(draw_mode_t what) {
  widgets_t w(what);
  w.precision(2, BaseNumericAdjustmentScreen::DEFAULT_MIDRANGE).units(GET_TEXT_F(MSG_UNITS_MM));

  w.heading(                  GET_TEXT_F(MSG_BABYSTEP_PROBE_Z));
  w.color(z_axis).adjuster(4, GET_TEXT_F(MSG_ZPROBE_ZOFFSET), getZOffset_mm());
  w.increments();
  w.button(2, GET_TEXT_F(MSG_PROBE_WIZARD), !isPrinting());
}

bool ZOffsetScreen::onTouchHeld(uint8_t tag) {
  const int16_t steps =   TERN0(BABYSTEPPING, mmToWholeSteps(getIncrement(), Z));
  const float increment = TERN(BABYSTEPPING, mmFromWholeSteps(steps, Z), getIncrement());
  switch (tag) {
    case 2: ZOffsetWizard::runWizard(); break;
    case 4: UI_DECREMENT(ZOffset_mm); TERN(BABYSTEPPING, babystepAxis_steps(-steps, Z), UNUSED(steps)); break;
    case 5: UI_INCREMENT(ZOffset_mm); TERN(BABYSTEPPING, babystepAxis_steps( steps, Z), UNUSED(steps)); break;
    default:
      return false;
  }
  SaveSettingsDialogBox::settingsChanged();
  return true;
}

#endif // COCOA_Z_OFFSET_SCREEN
