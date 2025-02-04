/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "../DGUSScreenHandlerBase.h"

//#define DEBUG_DGUS_SCREEN_HANDLER
#define DEBUG_OUT ENABLED(DEBUG_DGUS_SCREEN_HANDLER)
#include "../../../../core/debug_out.h"

enum DGUS_ScreenID : uint8_t;

class DGUSScreenHandlerCreality : public DGUSScreenHandler {
public:
  DGUSScreenHandlerCreality() = default;

  static void handleFeedAmountChanged(DGUS_VP_Variable &var, void *val_ptr);

  #if HAS_BED_PROBE
    static void onLevelingStart();
    static void onLevelingUpdate(const int8_t xpos, const int8_t ypos);
  #endif

  static void handleLEDToggle();
  static void handleStepperState(bool is_enabled);
  static void filamentRunout();
  static void onFactoryReset();

  #if HAS_BUZZER
    static void buzzer(const uint16_t frequency, const uint16_t duration=0);
  #else
    static void buzzer(const uint16_t, const uint16_t=0) {}
  #endif

  static void onHomingStart();
  static void onHomingDone();
  static void onPrintDone();

  static void sendAboutFirmwareVersion(DGUS_VP_Variable &var);
  static void sendAboutPrintSize(DGUS_VP_Variable &var);

  // Send an icon to the display, depending on whether it is true or false
  template<unsigned int value_if_true, unsigned int value_if_false>
  static void sendIconValue(DGUS_VP_Variable &var) {
    if (var.memadr) {
      bool value = *(bool *)var.memadr;
      uint16_t valueToSend = value ? value_if_true : value_if_false;
      dgus.writeVariable(var.VP, valueToSend);
    }
  }

  static void updateCurrentScreen(const DGUS_ScreenID current);

  static float feed_amount;
  static bool are_steppers_enabled;

private:
  static uint8_t meshLevelIndex;
};

typedef DGUSScreenHandlerCreality DGUSScreenHandlerClass;

#if ENABLED(POWER_LOSS_RECOVERY)
  #define PLR_SCREEN_RECOVER DGUS_SCREEN_SDPRINTMANIPULATION
  #define PLR_SCREEN_CANCEL DGUS_SCREEN_MAIN
#endif
