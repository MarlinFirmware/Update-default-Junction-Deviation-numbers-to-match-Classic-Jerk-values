/**************************
 * stress_test_screen.cpp *
 **************************/

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

#ifdef FTDI_STRESS_TEST_SCREEN

#define STRESS_TEST_CHANGE_INTERVAL 6000

#define GRID_COLS 4
#define GRID_ROWS 9

using namespace FTDI;
using namespace Theme;
using namespace ExtUI;

constexpr static StressTestScreenData &mydata = screen_data.StressTestScreen;

void StressTestScreen::drawDots(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  CommandProcessor cmd;
  for (uint_fast8_t i = 0; i < 100; i++) {
    cmd.cmd(BEGIN(POINTS))
       .cmd(POINT_SIZE(20*16))
       .cmd(COLOR_RGB(random(0xFFFFFF)))
       .cmd(VERTEX2F((x + random(w)) * 16,(y + random(h)) * 16));
  }
}

bool StressTestScreen::watchDogTestNow() {
  return mydata.next_watchdog_trigger &&
         ELAPSED(millis(), mydata.next_watchdog_trigger);
}

void StressTestScreen::onRedraw(draw_mode_t) {
  using namespace ExtUI;
  CommandProcessor cmd;
  cmd.cmd(CLEAR_COLOR_RGB(bg_color))
     .cmd(CLEAR(true,true,true))
     .cmd(COLOR_RGB(bg_text_enabled))
     .font(font_medium)
     .text(BTN_POS(1,1), BTN_SIZE(4,1), FPSTR(mydata.message));

  drawDots(BTN_POS(1,3), BTN_SIZE(4,4));

  cmd.font(font_medium).enabled(!watchDogTestNow()).colors(action_btn).tag(1).button(BTN_POS(2,8), BTN_SIZE(2,1), F("Exit"));
}

bool StressTestScreen::onTouchEnd(uint8_t tag) {
  CommandProcessor cmd;
  switch (tag) {
    case 1:
      runTestOnBootup(false);
      GOTO_SCREEN(StatusScreen);
      break;
    default:
      return false;
  }
  return true;
}

void StressTestScreen::runTestOnBootup(bool enable) {
  // Use a magic value in passcode to indicate
  // whether or not we need to re-run the test
  // at startup.
  LockScreen::set_hash(enable ? 0xDEAD : 0);
  injectCommands(F("M500"));
}

void StressTestScreen::startupCheck() {
  if (LockScreen::get_hash() == 0xDEAD)
    GOTO_SCREEN(StressTestScreen);
}

void StressTestScreen::onEntry() {
  mydata.next_watchdog_trigger = millis() + 10000 + random(40000);
  mydata.message = PSTR("Test 1: Stress testing...");

  // Turn off heaters.
  coolDown();

  runTestOnBootup(true);
}

void StressTestScreen::recursiveLockup() {
  mydata.message = PSTR("Test 2: Printer will restart.");
  current_screen.onRefresh();
  recursiveLockup();
}

void StressTestScreen::iterativeLockup() {
  mydata.message = PSTR("Test 3: Printer will restart.");
  for (;;) current_screen.onRefresh();
}

void StressTestScreen::onIdle() {
  current_screen.onRefresh();
  reset_menu_timeout();

  if (!commandsInQueue()) {
    if (!isPositionKnown()) {
      injectCommands_P(G28_STR);
    }
    else {
      injectCommands(F(
        "G0 X100 Y100 Z100 F6000\n"
        "T0\nG4 S1"
        E_TERN_("\nT1\nG4 S1")
        "\nG0 X150 Y150 Z150"
      ));
    }
  }

  if (refresh_timer.elapsed(STRESS_TEST_CHANGE_INTERVAL)) {
    setTargetFan_percent(random(100),FAN0);
  }

  if (watchDogTestNow()) {
    if (random(2) % 2)
      iterativeLockup();
    else
      recursiveLockup();
  }

  BaseScreen::onIdle();
}

#endif // FTDI_STRESS_TEST_SCREEN
