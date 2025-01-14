/*****************
 * language_en.h *
 *****************/

/****************************************************************************
 *   Written By Marcio Teixeira 2019 - Aleph Objects, Inc.                  *
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

#pragma once

#include "language.h"

#if ENABLED(TOUCH_UI_UTF8_COPYRIGHT)
  #define COPYRIGHT_SIGN    u8"©"
#else
  #define COPYRIGHT_SIGN    u8"(c)"
#endif

#if ENABLED(TOUCH_UI_UTF8_WESTERN_CHARSET)
  #define DEGREE_SIGN       u8"°"
#else
  #define DEGREE_SIGN       u8" "
#endif

namespace Language_en {
  LSTR MSG_BUTTON_OKAY              = u8"Okay";
  LSTR MSG_BUTTON_MENU              = u8"Menu";
  LSTR MSG_BUTTON_MEDIA             = u8"Media";
  LSTR MSG_BUTTON_USB               = u8"USB";
  LSTR MSG_BUTTON_OPEN              = u8"Open";
  LSTR MSG_CLEAN_NOZZLE             = u8"Clean Nozzle";
  LSTR MSG_VMAX_X                   = u8"Max X Speed";
  LSTR MSG_VMAX_Y                   = u8"Max Y Speed";
  LSTR MSG_VMAX_Z                   = u8"Max Z Speed";
  LSTR MSG_ACCEL_PRINTING           = u8"Printing";
  LSTR MSG_ACCEL_TRAVEL             = u8"Travel";
  LSTR MSG_ACCEL_RETRACT            = u8"Retraction";
  LSTR MSG_AMAX_X                   = u8"Max X Accel.";
  LSTR MSG_AMAX_Y                   = u8"Max Y Accel.";
  LSTR MSG_AMAX_Z                   = u8"Max Z Accel.";
  LSTR MSG_AXIS_X                   = u8"X";
  LSTR MSG_AXIS_X2                  = u8"X2";
  LSTR MSG_AXIS_Y                   = u8"Y";
  LSTR MSG_AXIS_Y2                  = u8"Y2";
  LSTR MSG_AXIS_Z                   = u8"Z";
  LSTR MSG_AXIS_Z2                  = u8"Z2";
  LSTR MSG_AXIS_E                   = u8"E";
  LSTR MSG_AXIS_E1                  = u8"E1";
  LSTR MSG_AXIS_E2                  = u8"E2";
  LSTR MSG_AXIS_E3                  = u8"E3";
  LSTR MSG_AXIS_E4                  = u8"E4";
  LSTR MSG_AXIS_ALL                 = u8"All";
  LSTR MSG_HOME                     = u8"Home";
  LSTR MSG_PRINT_STARTING           = u8"Print starting";
  LSTR MSG_PRINT_FINISHED           = u8"Print finished";
  LSTR MSG_PRINT_COMPLETE           = u8"Print Complete!";
  LSTR MSG_PRINT_ERROR              = u8"Print error";
  LSTR MSG_PRINT_AREA_CLEAR         = u8"Please ensure print area is\nclear and ready to print.";
  LSTR MSG_ABOUT_TOUCH_PANEL_1      = u8"Color Touch Panel";
  LSTR MSG_ABOUT_TOUCH_PANEL_2      = WEBSITE_URL;
  LSTR MSG_LICENSE                  = u8"This program is free software: you can redistribute it and/or modify it under the terms of "
                                                        "the GNU General Public License as published by the Free Software Foundation, either version 3 "
                                                        "of the License, or (at your option) any later version. To view a copy of the GNU General "
                                                        "Public License, go to the following location: https://www.gnu.org/licenses.";
  LSTR MSG_RUNOUT_1                 = u8"Runout 1";
  LSTR MSG_RUNOUT_2                 = u8"Runout 2";
  LSTR MSG_DISPLAY_MENU             = u8"Display";
  LSTR MSG_INTERFACE                = u8"Interface";
  LSTR MSG_MEASURE_AUTOMATICALLY    = u8"Measure automatically";
  LSTR MSG_H_OFFSET                 = u8"H Offset";
  LSTR MSG_V_OFFSET                 = u8"V Offset";
  LSTR MSG_TOUCH_SCREEN             = u8"Touch Screen";
  LSTR MSG_CALIBRATE                = u8"Calibrate";
  LSTR MSG_UNITS_MILLIAMP           = u8"mA";
  LSTR MSG_UNITS_MM                 = u8"mm";
  LSTR MSG_UNITS_MM_S               = u8"mm/s";
  LSTR MSG_UNITS_MM_S2              = u8"mm/s" SUPERSCRIPT_TWO;
  LSTR MSG_UNITS_STEP_MM            = u8"st/mm";
  LSTR MSG_UNITS_PERCENT            = u8"%";
  LSTR MSG_UNITS_C                  = DEGREE_SIGN u8"C";
  LSTR MSG_IDLE                     = u8"idle";
  LSTR MSG_SET_MAXIMUM              = u8"Set Maximum";
  LSTR MSG_PRINT_SPEED              = u8"Print Speed";
  LSTR MSG_SPEED                    = u8"Speed";
  LSTR MSG_LINEAR_ADVANCE_K         = u8"K";
  LSTR MSG_LINEAR_ADVANCE_K1        = u8"K E1";
  LSTR MSG_LINEAR_ADVANCE_K2        = u8"K E2";
  LSTR MSG_LINEAR_ADVANCE_K3        = u8"K E3";
  LSTR MSG_LINEAR_ADVANCE_K4        = u8"K E4";
  LSTR MSG_NUDGE_NOZZLE             = u8"Nudge Nozzle";
  LSTR MSG_ADJUST_BOTH_NOZZLES      = u8"Adjust Both Nozzles";
  LSTR MSG_SHOW_OFFSETS             = u8"Show Offsets";
  LSTR MSG_INCREMENT                = u8"Increment";
  LSTR MSG_ERASE_FLASH_WARNING      = u8"Are you sure? SPI flash will be erased.";
  LSTR MSG_ERASING                  = u8"Erasing...";
  LSTR MSG_ERASED                   = u8"SPI flash erased";
  LSTR MSG_CALIBRATION_WARNING      = u8"For best results, unload the filament and clean the hotend prior to starting calibration. Continue?";
  LSTR MSG_START_PRINT_CONFIRMATION = u8"Start printing %s?";
  LSTR MSG_ABORT_WARNING            = u8"Are you sure you want to cancel the print?";
  LSTR MSG_START_NEXT_PRINT         = u8"Start Next Print";
  LSTR MSG_EXTRUDER_SELECTION       = u8"Extruder Selection";
  LSTR MSG_CURRENT_TEMPERATURE      = u8"Current Temp";
  LSTR MSG_REMOVAL_TEMPERATURE      = u8"Removal Temp";
  LSTR MSG_CAUTION                  = u8"Caution:";
  LSTR MSG_HOT                      = u8"Hot!";
  LSTR MSG_UNLOAD_FILAMENT          = u8"Unload/Retract";
  LSTR MSG_LOAD_FILAMENT            = u8"Load/Extrude";
  LSTR MSG_LOAD                     = u8"Load";
  LSTR MSG_UNLOAD                   = u8"Unload";
  LSTR MSG_MOMENTARY                = u8"Momentary";
  LSTR MSG_CONTINUOUS               = u8"Continuous";
  LSTR MSG_PRINT_MENU               = u8"Print Menu";
  LSTR MSG_FINE_MOTION              = u8"Fine motion";
  LSTR MSG_ENABLE_MEDIA             = u8"Enable Media";
  LSTR MSG_INSERT_MEDIA             = u8"Insert Media...";
  LSTR MSG_LCD_BRIGHTNESS           = u8"LCD brightness";
  LSTR MSG_SOUND_VOLUME             = u8"Sound volume";
  LSTR MSG_SCREEN_LOCK              = u8"Screen lock";
  LSTR MSG_BOOT_SCREEN              = u8"Boot screen";
  LSTR MSG_SOUNDS                   = u8"Sounds";
  LSTR MSG_CLICK_SOUNDS             = u8"Click sounds";
  LSTR MSG_EEPROM_RESTORED          = u8"Settings restored from backup";
  LSTR MSG_EEPROM_RESET             = u8"Settings restored to default";
  LSTR MSG_EEPROM_SAVED             = u8"Settings saved!";
  LSTR MSG_EEPROM_SAVE_PROMPT       = u8"Settings applied. Save these settings for next power-on?";
  LSTR MSG_EEPROM_RESET_WARNING     = u8"Are you sure? Customizations will be lost.";

  LSTR MSG_PASSCODE_REJECTED        = u8"Wrong passcode!";
  LSTR MSG_PASSCODE_ACCEPTED        = u8"Passcode accepted!";
  LSTR MSG_PASSCODE_SELECT          = u8"Select Passcode:";
  LSTR MSG_PASSCODE_REQUEST         = u8"Enter Passcode:";

  LSTR MSG_TOUCH_CALIBRATION_START  = u8"Release to begin screen calibration";
  LSTR MSG_TOUCH_CALIBRATION_PROMPT = u8"Touch the dots to calibrate";
  LSTR MSG_BED_MAPPING_DONE         = u8"Bed mapping finished";
  LSTR MSG_BED_MAPPING_INCOMPLETE   = u8"Not all points probed";
  LSTR MSG_LEVELING                 = u8"Leveling";
  LSTR MSG_AXIS_LEVELING            = u8"Axis Leveling";
  LSTR MSG_PROBE_BED                = u8"Probe Mesh";
  LSTR MSG_PRINT_TEST               = u8"Print Test (PLA)";
  LSTR MSG_MOVE_Z_TO_TOP            = u8"Raise Z to Top";
  LSTR MSG_MAX_SPEED_NO_UNITS       = u8"Max Speed";

  //LSTR MSG_FTDI_HEATER_TIMEOUT    = u8"Idle timeout, temperature decreased. Press Okay to reheat and again to resume.";

  #if ENABLED(TOUCH_UI_LULZBOT_BIO)
    LSTR MSG_MOVE_TO_HOME           = u8"Move to Home";
    LSTR MSG_RAISE_PLUNGER          = u8"Raise Plunger";
    LSTR MSG_RELEASE_XY_AXIS        = u8"Release X and Y Axis";
    LSTR MSG_BED_TEMPERATURE        = u8"Bed Temperature";
    LSTR MSG_HOME_XYZ_WARNING       = u8"About to move to home position. Ensure the top and the bed of the printer are clear.\n\nContinue?";
    LSTR MSG_HOME_E_WARNING         = u8"About to re-home plunger and auto-level. Remove syringe prior to proceeding.\n\nContinue?";
  #endif

  #ifdef TOUCH_UI_COCOA_PRESS
    LSTR MSG_BODY                   = u8"Body";
    LSTR MSG_SELECT_CHOCOLATE_TYPE  = u8"Select Chocolate Type";
    LSTR MSG_EXTERNAL               = u8"External";
    LSTR MSG_CHOCOLATE              = u8"Chocolate";
    LSTR MSG_UNLOAD_CARTRIDGE       = u8"Unload Cartridge";
    LSTR MSG_LOAD_UNLOAD            = u8"Load/Unload";
    LSTR MSG_START_STOP             = u8"Start/Stop";
    LSTR MSG_LOAD                   = u8"Load";
    LSTR MSG_UNLOAD                 = u8"Unload";
    LSTR MSG_PREHEAT_CHOCOLATE      = u8"Preheat Chocolate";
    LSTR MSG_PREHEAT_FINISHED       = u8"Preheat finished";
    LSTR MSG_PREHEAT                = u8"Preheat";
    LSTR MSG_ELAPSED_PRINT          = u8"Elapsed Print";
    LSTR MSG_XYZ_MOVE               = u8"XYZ Move";
    LSTR MSG_E_MOVE                 = u8"Extrusion Move";
  #endif
}; // namespace Language_en
