/********************
 * nudge_nozzle.cpp *
 ********************/

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

#ifdef FTDI_NUDGE_NOZZLE_SCREEN

using namespace FTDI;
using namespace Theme;
using namespace ExtUI;

constexpr static NudgeNozzleScreenData &mydata = screen_data.NudgeNozzleScreen;

#define GRID_COLS 13
#define GRID_ROWS (9+EXTRUDERS)

void NudgeNozzleScreen::onEntry() {
  mydata.show_offsets = false;
  #if HAS_MULTI_EXTRUDER
    mydata.link_nozzles = true;
  #endif
  mydata.rel.reset();

  BaseNumericAdjustmentScreen::onEntry();
}

void NudgeNozzleScreen::onRedraw(draw_mode_t what) {
  widgets_t w(what);
  CommandProcessor cmd;
  w.precision(2, BaseNumericAdjustmentScreen::DEFAULT_MIDRANGE).units(GET_TEXT_F(MSG_UNITS_MM));

  w.heading(GET_TEXT_F(MSG_ZOFFSET));
  #if ENABLED(BABYSTEP_XY)
  w.color(x_axis).adjuster(2, GET_TEXT_F(MSG_AXIS_X), mydata.rel.x / getAxisSteps_per_mm(X));
  w.color(y_axis).adjuster(4, GET_TEXT_F(MSG_AXIS_Y), mydata.rel.y / getAxisSteps_per_mm(Y));
  #endif
  w.color(z_axis).adjuster(6, GET_TEXT_F(MSG_AXIS_Z), mydata.rel.z / getAxisSteps_per_mm(Z));
  w.increments();
  #if HAS_MULTI_EXTRUDER
    w.toggle(8, GET_TEXT_F(MSG_ADJUST_BOTH_NOZZLES), mydata.link_nozzles);
  #endif

  #if HAS_MULTI_EXTRUDER || HAS_BED_PROBE
    w.toggle(9, GET_TEXT_F(MSG_SHOW_OFFSETS), mydata.show_offsets);

    if (mydata.show_offsets) {
      char str[19];

      w.draw_mode(BOTH);
      w.color(other);

      #if HAS_BED_PROBE
        dtostrf(getZOffset_mm(), 4, 2, str);
        strcat(str, " ");
        strcat_P(str, GET_TEXT(MSG_UNITS_MM));
        w.text_field(0, GET_TEXT_F(MSG_ZOFFSET), str);
      #endif

      #if HAS_MULTI_HOTEND
        format_position(str, getNozzleOffset_mm(X, E1), getNozzleOffset_mm(Y, E1), getNozzleOffset_mm(Z, E1));
        w.text_field(0, GET_TEXT_F(MSG_OFFSETS_MENU), str);
      #endif
    }
  #endif
  if (what & FOREGROUND) {
    cmd.colors(normal_btn)
       .font(font_medium)
       .tag(10).colors(action_btn).button(BTN_POS(1,GRID_ROWS), BTN_SIZE(GRID_COLS,1), GET_TEXT_F(MSG_BUTTON_DONE));
  }
}

bool NudgeNozzleScreen::onTouchHeld(uint8_t tag) {
  const float inc = getIncrement();
  #if HAS_MULTI_EXTRUDER
    const bool link = mydata.link_nozzles;
  #else
    constexpr bool link = true;
  #endif
  int16_t steps;
  switch (tag) {
    case 2: steps = mmToWholeSteps(inc, X); smartAdjustAxis_steps(-steps, X, link); mydata.rel.x -= steps; break;
    case 3: steps = mmToWholeSteps(inc, X); smartAdjustAxis_steps( steps, X, link); mydata.rel.x += steps; break;
    case 4: steps = mmToWholeSteps(inc, Y); smartAdjustAxis_steps(-steps, Y, link); mydata.rel.y -= steps; break;
    case 5: steps = mmToWholeSteps(inc, Y); smartAdjustAxis_steps( steps, Y, link); mydata.rel.y += steps; break;
    case 6: steps = mmToWholeSteps(inc, Z); smartAdjustAxis_steps(-steps, Z, link); mydata.rel.z -= steps; break;
    case 7: steps = mmToWholeSteps(inc, Z); smartAdjustAxis_steps( steps, Z, link); mydata.rel.z += steps; break;
    #if HAS_MULTI_EXTRUDER
      case 8: mydata.link_nozzles = !link; break;
    #endif
    case 9: mydata.show_offsets = !mydata.show_offsets; break;
    case 10: GOTO_SCREEN(SaveSettingsDialogBox); break;
    default: return false;
  }
  #if HAS_MULTI_EXTRUDER || HAS_BED_PROBE
    SaveSettingsDialogBox::settingsChanged();
  #endif
  return true;
}

bool NudgeNozzleScreen::onTouchEnd(uint8_t tag) {
  if (tag == 1) {
    SaveSettingsDialogBox::promptToSaveSettings();
    return true;
  }
  else
    return BaseNumericAdjustmentScreen::onTouchEnd(tag);
}

void NudgeNozzleScreen::onIdle() {
  reset_menu_timeout();
}

#endif // FTDI_NUDGE_NOZZLE_SCREEN
