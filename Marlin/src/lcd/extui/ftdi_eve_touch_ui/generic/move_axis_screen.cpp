/************************
 * move_axis_screen.cpp *
 ************************/

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

#ifdef FTDI_MOVE_AXIS_SCREEN

using namespace FTDI;
using namespace ExtUI;
using namespace Theme;

constexpr static MoveAxisScreenData &mydata = screen_data.MoveAxisScreen;

void BaseMoveAxisScreen::onEntry() {
  // Since Marlin keeps only one absolute position for all the extruders,
  // we have to keep track of the relative motion of individual extruders
  // ourselves. The relative distances are reset to zero whenever this
  // screen is entered.

  for (uint8_t i = 0; i < ExtUI::extruderCount; ++i) {
    mydata.e_rel[i] = 0;
  }
  BaseNumericAdjustmentScreen::onEntry();
}
#define GRID_COLS 13
#define GRID_ROWS (8+EXTRUDERS)

void MoveAxisScreen::onRedraw(draw_mode_t what) {
  widgets_t w(what);
  CommandProcessor cmd;
  w.precision(1);
  w.units(GET_TEXT_F(MSG_UNITS_MM));
  w.heading(                           GET_TEXT_F(MSG_MOVE_AXIS));
  w.home_buttons(20);
  w.color(Theme::x_axis).adjuster(  2, GET_TEXT_F(MSG_AXIS_X),  getAxisPosition_mm(X), canMove(X));
  w.color(Theme::y_axis).adjuster(  4, GET_TEXT_F(MSG_AXIS_Y),  getAxisPosition_mm(Y), canMove(Y));
  w.color(Theme::z_axis).adjuster(  6, GET_TEXT_F(MSG_AXIS_Z),  getAxisPosition_mm(Z), canMove(Z));

  w.color(Theme::e_axis);
  #if EXTRUDERS == 1
    w.adjuster(  8, GET_TEXT_F(MSG_AXIS_E),  mydata.e_rel[0], canMove(E0));
  #elif HAS_MULTI_EXTRUDER
    w.adjuster(  8, GET_TEXT_F(MSG_AXIS_E1), mydata.e_rel[0], canMove(E0));
    w.adjuster( 10, GET_TEXT_F(MSG_AXIS_E2), mydata.e_rel[1], canMove(E1));
    #if EXTRUDERS > 2
      w.adjuster( 12, GET_TEXT_F(MSG_AXIS_E3), mydata.e_rel[2], canMove(E2));
      #if EXTRUDERS > 3
        w.adjuster( 14, GET_TEXT_F(MSG_AXIS_E4), mydata.e_rel[3], canMove(E3));
      #endif
    #endif
  #endif
  #if Z_HOME_TO_MIN
    w.button(24, GET_TEXT_F(MSG_MOVE_Z_TO_TOP), !axis_should_home(Z_AXIS));
  #endif
  w.increments();
  #ifdef PARKING_COMMAND_GCODE
    if (!ExtUI::isPrinting()) { // making sure the Tool Head Swap Position is not avalible while printing
      cmd.font(font_medium)
        .colors(normal_btn)
        .tag(25).button(BTN_POS(1,(7+EXTRUDERS)), BTN_SIZE(13,1), GET_TEXT_F(MSG_TOOL_HEAD_SWAP));
    }
  #endif
}

bool BaseMoveAxisScreen::onTouchHeld(const uint8_t tag) {
  #define UI_INCREMENT_AXIS(axis) setManualFeedrate(axis, increment); UI_INCREMENT(AxisPosition_mm, axis);
  #define UI_DECREMENT_AXIS(axis) setManualFeedrate(axis, increment); UI_DECREMENT(AxisPosition_mm, axis);
  const float increment = getIncrement();
  switch (tag) {
    #if HAS_X_AXIS
      case  2: UI_DECREMENT_AXIS(X); break;
      case  3: UI_INCREMENT_AXIS(X); break;
    #endif
    #if HAS_EXTRUDERS
      // For extruders, also update relative distances.
      case  8: UI_DECREMENT_AXIS(E0); mydata.e_rel[0] -= increment; break;
      case  9: UI_INCREMENT_AXIS(E0); mydata.e_rel[0] += increment; break;
      #if HAS_MULTI_EXTRUDER
        case 10: UI_DECREMENT_AXIS(E1); mydata.e_rel[1] -= increment; break;
        case 11: UI_INCREMENT_AXIS(E1); mydata.e_rel[1] += increment; break;
        #if EXTRUDERS > 2
          case 12: UI_DECREMENT_AXIS(E2); mydata.e_rel[2] -= increment; break;
          case 13: UI_INCREMENT_AXIS(E2); mydata.e_rel[2] += increment; break;
          #if EXTRUDERS > 3
            case 14: UI_DECREMENT_AXIS(E3); mydata.e_rel[3] -= increment; break;
            case 15: UI_INCREMENT_AXIS(E3); mydata.e_rel[3] += increment; break;
          #endif
        #endif
      #endif
    #endif
    #if HAS_Y_AXIS
      case  4: UI_DECREMENT_AXIS(Y); break;
      case  5: UI_INCREMENT_AXIS(Y); break;
      case 20: SpinnerDialogBox::enqueueAndWait(F("G28X")); break;
      case 21: SpinnerDialogBox::enqueueAndWait(F("G28Y")); break;
      #if HAS_Z_AXIS
        case  6: UI_DECREMENT_AXIS(Z); break;
        case  7: UI_INCREMENT_AXIS(Z); break;
        case 22: SpinnerDialogBox::enqueueAndWait(F("G28Z")); break;
        case 24: raiseZtoTop(); break;
      #endif
    #endif
    case 23: SpinnerDialogBox::enqueueAndWait(F("G28")); break;
    #ifdef PARKING_COMMAND_GCODE
      case 25: injectCommands(F(PARKING_COMMAND_GCODE)); break;
    #endif
    default:
      return false;
  }
  return true;
}

void BaseMoveAxisScreen::raiseZtoTop() {
  constexpr xyz_feedrate_t homing_feedrate = HOMING_FEEDRATE_MM_M;
  setAxisPosition_mm(Z_MAX_POS - 5, Z, homing_feedrate.z);
}

float BaseMoveAxisScreen::getManualFeedrate(const uint8_t axis, const_float_t increment_mm) {
  // Compute feedrate so that the tool lags the adjuster when it is
  // being held down, this allows enough margin for the planner to
  // connect segments and even out the motion.
  constexpr xyze_feedrate_t max_manual_feedrate = MANUAL_FEEDRATE;
  return min(MMM_TO_MMS(max_manual_feedrate[axis]), ABS(increment_mm * (TOUCH_REPEATS_PER_SECOND) * 0.80f));
}

void BaseMoveAxisScreen::setManualFeedrate(const ExtUI::axis_t axis, const_float_t increment_mm) {
  ExtUI::setFeedrate_mm_s(getManualFeedrate(X_AXIS + (axis - ExtUI::X), increment_mm));
}

#if HAS_EXTRUDERS
  void BaseMoveAxisScreen::setManualFeedrate(const ExtUI::extruder_t, const_float_t increment_mm) {
    ExtUI::setFeedrate_mm_s(getManualFeedrate(E_AXIS, increment_mm));
  }
#endif

void MoveAxisScreen::onIdle() {
  if (refresh_timer.elapsed(STATUS_UPDATE_INTERVAL)) {
    onRefresh();
    refresh_timer.start();
  }
  BaseScreen::onIdle();
}

#endif // FTDI_MOVE_AXIS_SCREEN
