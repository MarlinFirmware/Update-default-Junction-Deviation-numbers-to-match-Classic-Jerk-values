/***********************
 * z_offset_screen.cpp *
 ***********************/

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

#ifdef FTDI_Z_OFFSET_SCREEN

using namespace FTDI;
using namespace ExtUI;
using namespace Theme;

#define SHEET_THICKNESS 0.1
#define GRID_COLS 13
#define GRID_ROWS (9+EXTRUDERS)

constexpr static ZOffsetScreenData &mydata = screen_data.ZOffsetScreen;

void ZOffsetScreen::onEntry() {
  mydata.z = SHEET_THICKNESS;
  mydata.softEndstopState = getSoftEndstopState();
  BaseNumericAdjustmentScreen::onEntry();
  if (wizardRunning())
    setSoftEndstopState(false);
}

void ZOffsetScreen::onExit() {
  setSoftEndstopState(mydata.softEndstopState);
}

void ZOffsetScreen::onRedraw(draw_mode_t what) {
  widgets_t w(what);
  CommandProcessor cmd;

  w.precision(2, BaseNumericAdjustmentScreen::DEFAULT_MIDRANGE).units(GET_TEXT_F(MSG_UNITS_MM));

  w.heading(                  GET_TEXT_F(MSG_ZOFFSET));
  w.color(z_axis).adjuster(4, GET_TEXT_F(MSG_ZOFFSET), getZOffset_mm());
  w.increments();
  //w.button(2, GET_TEXT_F(MSG_PROBE_WIZARD), !isPrinting() && !wizardRunning());
  if (what & FOREGROUND) {
    cmd.colors(normal_btn)
       .font(font_medium)
       .tag(6).colors(action_btn).button(BTN_POS(1,GRID_ROWS), BTN_SIZE(GRID_COLS,1), GET_TEXT_F(MSG_BUTTON_DONE));
  }
}

void ZOffsetScreen::move(float mm, int16_t steps) {
  if (wizardRunning()) {
    mydata.z += mm;
    setAxisPosition_mm(mydata.z, Z);
  }
  else {
    // Otherwise doing a manual adjustment, possibly during a print.
    TERN(BABYSTEPPING, babystepAxis_steps(steps, Z), UNUSED(steps));
  }
}

void ZOffsetScreen::runWizard() {
  // Restore the default Z offset
  constexpr float offset[] = NOZZLE_TO_PROBE_OFFSET;
  setZOffset_mm(offset[Z_AXIS]);
  // Move above probe point
  char cmd[64], str[10];
  strcpy_P(cmd, PSTR("G28O\nG0 F1000 X"));
  dtostrf(TERN(Z_SAFE_HOMING, Z_SAFE_HOMING_X_POINT, X_CENTER), 3, 1, str);
  strcat(cmd, str);
  strcat_P(cmd, PSTR("Y"));
  dtostrf(TERN(Z_SAFE_HOMING, Z_SAFE_HOMING_Y_POINT, Y_CENTER), 3, 1, str);
  strcat(cmd, str);
  strcat_P(cmd, PSTR("Z"));
  dtostrf(SHEET_THICKNESS, 3, 1, str);
  strcat(cmd, str);
  injectCommands(cmd);
  // Show instructions for user.
  AlertDialogBox::show(F("After the printer finishes homing, adjust the Z Offset so that a sheet of paper can pass between the nozzle and bed with slight resistance."));
}

bool ZOffsetScreen::wizardRunning() {
  // We can't store state after the call to the AlertBox, so
  // check whether the current Z position equals mydata.z in order
  // to know whether the user started the wizard.
  return getAxisPosition_mm(Z) == mydata.z;
}

bool ZOffsetScreen::onTouchHeld(uint8_t tag) {
  const int16_t steps =   TERN0(BABYSTEPPING, mmToWholeSteps(getIncrement(), Z));
  const float increment = TERN(BABYSTEPPING, mmFromWholeSteps(steps, Z), getIncrement());
  switch (tag) {
    case 2: runWizard(); break;
    case 4: UI_DECREMENT(ZOffset_mm); move(-increment, -steps); break;
    case 5: UI_INCREMENT(ZOffset_mm); move( increment,  steps); break;
    case 6: GOTO_SCREEN(SaveSettingsDialogBox); break;
    default:
      return false;
  }
  SaveSettingsDialogBox::settingsChanged();
  return true;
}

#endif // FTDI_Z_OFFSET_SCREEN
