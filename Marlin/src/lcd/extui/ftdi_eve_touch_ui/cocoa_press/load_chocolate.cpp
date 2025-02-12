/**********************************
 * cocoa_press/load_chocolate.cpp *
 **********************************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2020 - Cocoa Press                          *
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
 *   location: <https://www.gnu.org/licenses/>.                              *
 ****************************************************************************/

#include "../config.h"
#include "../screens.h"
#include "../screen_data.h"

#ifdef COCOA_LOAD_CHOCOLATE_SCREEN

#include "cocoa_press_ui.h"

#define POLY(A) PolyUI::poly_reader_t(A, sizeof(A)/sizeof(A[0]))

const uint8_t shadow_depth = 5;

using namespace ExtUI;
using namespace FTDI;
using namespace Theme;

constexpr static LoadChocolateScreenData &mydata = screen_data.LoadChocolateScreen;

void LoadChocolateScreen::draw_buttons(draw_mode_t what) {
  int16_t x, y, h, v;

  CommandProcessor cmd;
  PolyUI ui(cmd, what);

  cmd.font(font_medium).colors(normal_btn);

  ui.bounds(POLY(load_screen_unload_btn), x, y, h, v);
  cmd.tag(2).colors(mydata.repeat_tag == 5 ? action_btn : normal_btn).button(x, y, h, v, GET_TEXT_F(MSG_UNLOAD));

  ui.bounds(POLY(load_screen_load_btn), x, y, h, v);
  cmd.tag(3).colors(mydata.repeat_tag == 6 ? action_btn : normal_btn).button(x, y, h, v, GET_TEXT_F(MSG_LOAD));

  ui.bounds(POLY(load_screen_start_stop_btn), x, y, h, v);
  if (mydata.repeat_tag == 0)
    cmd.colors(normal_btn).enabled(false);
  else
    cmd.colors(mydata.repeating ? action_btn : normal_btn).enabled(true);
  cmd.tag(4).button(x, y, h, v, GET_TEXT_F(MSG_START_STOP));

  ui.bounds(POLY(load_screen_back_btn), x, y, h, v);
  cmd.tag(1).colors(action_btn).button(x, y, h, v, GET_TEXT_F(MSG_BUTTON_DONE));
}

void LoadChocolateScreen::draw_text(draw_mode_t what) {
  if (what & BACKGROUND) {
    int16_t x, y, h, v;

    CommandProcessor cmd;
    PolyUI ui(cmd, what);

    cmd.font(font_medium).cmd(COLOR_RGB(bg_text_enabled));

    ui.bounds(POLY(load_screen_continuous), x, y, h, v);
    cmd.tag(2).text(x, y, h, v, GET_TEXT_F(MSG_CONTINUOUS));

    ui.bounds(POLY(load_screen_increment), x, y, h, v);
    cmd.tag(3).text(x, y, h, v, GET_TEXT_F(MSG_INCREMENT));
  }
}

void LoadChocolateScreen::draw_arrows(draw_mode_t what) {
  CommandProcessor cmd;
  PolyUI ui(cmd, what);

  ui.button_fill  (fill_rgb);
  ui.button_stroke(stroke_rgb, 28);
  ui.button_shadow(shadow_rgb, shadow_depth);

  constexpr uint8_t style = PolyUI::REGULAR;

  ui.button(5, POLY(load_screen_extrude), style);
  ui.button(6, POLY(load_screen_retract), style);
}

void LoadChocolateScreen::onEntry() {
  mydata.repeating = false;
  mydata.repeat_tag = 0;
}

void LoadChocolateScreen::onRedraw(draw_mode_t what) {
  if (what & BACKGROUND) {
    CommandProcessor cmd;
    cmd.cmd(CLEAR_COLOR_RGB(bg_color))
       .cmd(CLEAR(true,true,true))
       .tag(0);
  }

  draw_arrows(what);
  draw_buttons(what);
  draw_text(what);
}

bool LoadChocolateScreen::onTouchStart(uint8_t tag) {
  if (tag != 4) mydata.repeating = false;
  return true;
}

bool LoadChocolateScreen::onTouchEnd(uint8_t tag) {
  using namespace ExtUI;
  switch (tag) {
    case 2: mydata.repeat_tag = 5; break;
    case 3: mydata.repeat_tag = 6; break;
    case 4: FLIP(mydata.repeating); break;
    case 1: GOTO_PREVIOUS(); break;
  }
  return true;
}

void LoadChocolateScreen::setManualFeedrateAndIncrement(float feedrate_mm_s, float &increment_mm) {
  // Compute increment so feedrate so that the tool lags the adjuster when it is
  // being held down, this allows enough margin for the planner to
  // connect segments and even out the motion.
  ExtUI::setFeedrate_mm_s(feedrate_mm_s);
  increment_mm = feedrate_mm_s / ((TOUCH_REPEATS_PER_SECOND) * 0.80f);
}

bool LoadChocolateScreen::onTouchHeld(uint8_t tag) {
  if (ExtUI::isMoving()) return false; // Don't allow moves to accumulate
  float increment;
  setManualFeedrateAndIncrement(20, increment);
  #define UI_INCREMENT_AXIS(axis) UI_INCREMENT(AxisPosition_mm, axis);
  #define UI_DECREMENT_AXIS(axis) UI_DECREMENT(AxisPosition_mm, axis);
  switch (tag) {
    case 5: UI_INCREMENT_AXIS(E0); break;
    case 6: UI_DECREMENT_AXIS(E0); break;
    default: return false;
  }
  #undef UI_DECREMENT_AXIS
  #undef UI_INCREMENT_AXIS
  return false;
}

void LoadChocolateScreen::onIdle() {
  reset_menu_timeout();
  if (mydata.repeating) onTouchHeld(mydata.repeat_tag);
  if (refresh_timer.elapsed(STATUS_UPDATE_INTERVAL)) {
    if (!EventLoop::is_touch_held()) onRefresh();
    refresh_timer.start();
  }
  BaseScreen::onIdle();
}

#endif // COCOA_LOAD_CHOCOLATE_SCREEN
