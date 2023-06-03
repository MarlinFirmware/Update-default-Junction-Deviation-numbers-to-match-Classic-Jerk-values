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
#include "../../../../inc/MarlinConfigPre.h"

#if DGUS_LCD_UI_CREALITY_TOUCH

#include "DGUSDisplayDef.h"
#include "../DGUSDisplay.h"
#include "../DGUSScreenHandler.h"
#include "PageHandlers.h"

#include "../../../../module/temperature.h"
#include "../../../../module/motion.h"
#include "../../../../module/planner.h"
#include "../../../../feature/pause.h"
#include "../../../../feature/runout.h"
#include "../../../../module/settings.h"

#include "../../../marlinui.h" // for material presets
#include "../../ui_api.h"

// Definitions of page handlers

void MainMenuHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  // Nothing
}

void ControlMenuHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  switch (var.VP) {
    case VP_BUTTON_PREPAREENTERKEY:
      switch (buttonValue) {
        case 5:       // About
          // Automatically handled
          break;

        case 7:       // Reset to factory settings
          settings.reset();
          settings.save();
          break;

        case 9:       // Back button
          // TODO: should navigate automatically
          break;
      }
      break;

    case VP_BUTTON_ADJUSTENTERKEY:
      screen.handleLEDToggle();
      break;
  }
}

void LevelingModeHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  switch (var.VP) {
    case VP_BUTTON_BEDLEVELKEY:
      switch (buttonValue) {
        case 1: {
          const bool xy_known = ExtUI::isAxisPositionKnown(ExtUI::axis_t::X) && ExtUI::isAxisPositionKnown(ExtUI::axis_t::Y);
          ExtUI::injectCommands_P(xy_known ? PSTR("G28 Z") : PSTR("G28"));
        } break;

        case 2 ... 3: {
          // Increase / Decrease Z-offset
          const int16_t steps = ExtUI::mmToWholeSteps(0.01, ExtUI::axis_t::Z);
          ExtUI::smartAdjustAxis_steps(buttonValue == 2 ? steps : -steps, ExtUI::axis_t::Z, true);
          screen.forceCompleteUpdate();
        } break;
      }

      break;

    case VP_BUTTON_MAINENTERKEY:
      // Go to leveling screen
      ExtUI::injectCommands_P(PSTR("G28\nG29"));
      break;
  }
}

void LevelingHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  switch (var.VP) {
    case VP_BUTTON_BEDLEVELKEY:
      screen.gotoScreen(DGUS_SCREEN_ZOFFSET_LEVEL);

      break;
  }
}

void TempMenuHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  switch (var.VP) {
    case VP_BUTTON_ADJUSTENTERKEY:
      switch (buttonValue) {
        case 3:
          DGUSScreenHandler::handleFanControl(var, &buttonValue);
          break;
      }

      break;
  }
}

void PrepareMenuHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  switch (var.VP) {
    case VP_BUTTON_PREPAREENTERKEY:
      // Disable steppers
      screen.handleMotorLockUnlock(var, &buttonValue);
      break;

    case VP_BUTTON_COOLDOWN:
      screen.handleAllHeatersOff(var, &buttonValue);
      break;

    case VP_BUTTON_TEMPCONTROL:
      #if PREHEAT_COUNT
        switch (buttonValue) {
          case 5:
            thermalManager.setTargetHotend(ui.material_preset[0].hotend_temp, 0);
            thermalManager.setTargetBed(ui.material_preset[0].bed_temp);
            break;
          case 6:
            thermalManager.setTargetHotend(ui.material_preset[1].hotend_temp, 0);
            thermalManager.setTargetBed(ui.material_preset[1].bed_temp);
            break;
        }
      #endif
      break;
  }

  screen.forceCompleteUpdate();
}

void TuneMenuHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  switch (var.VP) {
    case VP_BUTTON_ADJUSTENTERKEY:
      switch (buttonValue) {
        case 2:
          screen.gotoScreen(ExtUI::isPrintingFromMediaPaused() ? DGUS_SCREEN_PRINT_PAUSED : DGUS_SCREEN_PRINT_RUNNING);
          break;

        case 3:
          DGUSScreenHandler::handleFanControl(var, &buttonValue);
          break;

        case 4:
          screen.handleLEDToggle();
          break;
      }
  }
}

void PrintRunningMenuHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  // There are actually no buttons to handle here: all buttons navigate to other screens (like confirmation screens)
}

void PrintPausedMenuHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  switch (var.VP) {
    case VP_BUTTON_RESUMEPRINTKEY:
      runout.reset();
      ExtUI::resumePrint();
      screen.gotoScreen(DGUS_SCREEN_PRINT_RUNNING);
      break;
  }
}

void PrintPauseDialogHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  switch (var.VP) {
    case VP_BUTTON_PAUSEPRINTKEY:
      switch (buttonValue) {
        case 2:
          ExtUI::pausePrint();
          screen.gotoScreen(DGUS_SCREEN_PRINT_PAUSED);
          break;

        case 3:
          screen.gotoScreen(DGUS_SCREEN_PRINT_RUNNING);
          break;
      }
      break;
  }
}

void FilamentRunoutHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  switch (var.VP) {
    case VP_BUTTON_RESUMEPRINTKEY:
      ExtUI::resumePrint();
      screen.gotoScreen(DGUS_SCREEN_PRINT_RUNNING);
      break;

    case VP_BUTTON_STOPPRINTKEY:
      ExtUI::stopPrint();
      screen.gotoScreen(DGUS_SCREEN_MAIN);
      break;
  }
}

void StopConfirmScreenHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  switch (var.VP) {
    case VP_BUTTON_STOPPRINTKEY:
      switch (buttonValue) {
        case 2:
          ExtUI::stopPrint();
          screen.gotoScreen(DGUS_SCREEN_MAIN);
          break;

        case 3:
          screen.gotoScreen(ExtUI::isPrintingFromMediaPaused() ? DGUS_SCREEN_PRINT_PAUSED : DGUS_SCREEN_PRINT_RUNNING);
          break;
      }
      break;
  }
}

void PreheatSettingsScreenHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  switch (var.VP) {
    case VP_BUTTON_PREPAREENTERKEY:
      // Save button, save settings and go back
      settings.save();
      screen.gotoScreen(DGUS_SCREEN_TEMP);
      break;

    case VP_BUTTON_COOLDOWN:
      // Back button, discard settings
      settings.load();
      break;
  }
}

void FeedHandler(DGUS_VP_Variable &var, unsigned short buttonValue) {
  if (var.VP != VP_BUTTON_HEATLOADSTARTKEY) return;

  switch (buttonValue) {
    case 1:
      if (ExtUI::getActualTemp_celsius(ExtUI::H0) < PREHEAT_1_TEMP_HOTEND) {
        ExtUI::setTargetTemp_celsius(PREHEAT_1_TEMP_HOTEND, ExtUI::H0);
        thermalManager.wait_for_hotend(0);
      }

      dgus.writeVariable(VP_FEED_PROGRESS, static_cast<int16_t>(10));

      load_filament(
        FILAMENT_CHANGE_SLOW_LOAD_LENGTH,
        FILAMENT_CHANGE_FAST_LOAD_LENGTH,
        screen.feed_amount,
        FILAMENT_CHANGE_ALERT_BEEPS,
        false,
        thermalManager.still_heating(0),
        PAUSE_MODE_LOAD_FILAMENT
        );

      dgus.writeVariable(VP_FEED_PROGRESS, static_cast<int16_t>(0));
      break;

    case 2:
      if (ExtUI::getActualTemp_celsius(ExtUI::H0) < PREHEAT_1_TEMP_HOTEND) {
        ExtUI::setTargetTemp_celsius(PREHEAT_1_TEMP_HOTEND, ExtUI::H0);
        thermalManager.wait_for_hotend(0);
      }

      dgus.writeVariable(VP_FEED_PROGRESS, static_cast<int16_t>(10));

      unload_filament(screen.feed_amount, false, PAUSE_MODE_UNLOAD_FILAMENT);

      dgus.writeVariable(VP_FEED_PROGRESS, static_cast<int16_t>(0));
      break;
  }

  screen.forceCompleteUpdate();
}

// Register the page handlers
#define PAGE_HANDLER(SCRID, HDLRPTR) { .ScreenID = SCRID, .Handler = HDLRPTR },

const struct PageHandler PageHandlers[] PROGMEM = {
  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_MAIN, MainMenuHandler)

  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_CONTROL, ControlMenuHandler)

  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_ZOFFSET_LEVEL, LevelingModeHandler)
  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_LEVELING, LevelingHandler)

  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_TEMP, TempMenuHandler)
  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_TEMP_PLA, PreheatSettingsScreenHandler)
  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_TEMP_ABS, PreheatSettingsScreenHandler)

  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_TUNING, TuneMenuHandler)
  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_FEED, FeedHandler)

  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_FILAMENTRUNOUT1, FilamentRunoutHandler)

  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_DIALOG_STOP, StopConfirmScreenHandler)

  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_PRINT_RUNNING, PrintRunningMenuHandler)
  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_PRINT_PAUSED, PrintPausedMenuHandler)

  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_DIALOG_PAUSE, PrintPauseDialogHandler)

  PAGE_HANDLER(DGUS_ScreenID::DGUS_SCREEN_PREPARE, PrepareMenuHandler)

  // Terminating
  PAGE_HANDLER(static_cast<DGUS_ScreenID>(0), 0)
};

void DGUSCrealityDisplay_HandleReturnKeyEvent(DGUS_VP_Variable &var, void *val_ptr) {
  const struct PageHandler *map = PageHandlers;
  const uint16_t *ret;
  const DGUS_ScreenID current_screenID = DGUSScreenHandler::getCurrentScreen();

  while ((ret = (uint16_t*) pgm_read_ptr(&(map->Handler)))) {
    if (map->ScreenID == current_screenID) {
      unsigned short button_value = *static_cast<unsigned short*>(val_ptr);
      button_value = (button_value & 0xffU) << 8U | (button_value >> 8U);

      SERIAL_ECHOPGM("Invoking handler for screen ", current_screenID);
      SERIAL_ECHOLNPGM("with VP=", var.VP, " value=", button_value);

      map->Handler(var, button_value);
      return;
    }

    map++;
  }
}

#endif // DGUS_LCD_UI_CREALITY_TOUCH
