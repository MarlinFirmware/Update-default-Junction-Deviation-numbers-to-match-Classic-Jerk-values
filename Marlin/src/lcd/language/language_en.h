/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * English
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 *
 * Substitutions are applied for the following characters when used in menu items titles:
 *
 *   $ displays an inserted string
 *   { displays  '0'....'10' for indexes 0 - 10
 *   ~ displays  '1'....'11' for indexes 0 - 10
 *   * displays 'E1'...'E11' for indexes 0 - 10 (By default. Uses LCD_FIRST_TOOL)
 *   @ displays an axis name such as XYZUVW, or E for an extruder
 */

#define en 1234
#if LCD_LANGUAGE == en
  #define NOT_EXTENDED_ISO10646_1_5X7
#endif
#undef en

#define MEDIA_TYPE_EN "Media"

#ifndef PREHEAT_1_LABEL
  #define PREHEAT_1_LABEL ""
#endif

#ifndef PREHEAT_2_LABEL
  #define PREHEAT_2_LABEL ""
#endif

#ifndef PREHEAT_3_LABEL
  #define PREHEAT_3_LABEL ""
#endif

#ifndef PREHEAT_4_LABEL
  #define PREHEAT_4_LABEL ""
#endif

#ifndef CUSTOM_MENU_MAIN_TITLE
  #define CUSTOM_MENU_MAIN_TITLE ""
#endif

namespace LanguageNarrow_en {
  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("English");

  // These strings should be translated
  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" Ready.");
  LSTR MSG_YES                            = _UxGT("YES");
  LSTR MSG_NO                             = _UxGT("NO");
  LSTR MSG_HIGH                           = _UxGT("HIGH");
  LSTR MSG_LOW                            = _UxGT("LOW");
  LSTR MSG_BACK                           = _UxGT("Back");
  LSTR MSG_ERROR                          = _UxGT("Error");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Aborting...");
  LSTR MSG_MEDIA_INSERTED                 = MEDIA_TYPE_EN _UxGT(" Inserted");
  LSTR MSG_MEDIA_REMOVED                  = MEDIA_TYPE_EN _UxGT(" Removed");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("Waiting for ") MEDIA_TYPE_EN;
  LSTR MSG_MEDIA_INIT_FAIL                = MEDIA_TYPE_EN _UxGT(" Init Fail");
  LSTR MSG_MEDIA_READ_ERROR               = MEDIA_TYPE_EN _UxGT(" read error");
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("USB device removed");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("USB start failed");
  LSTR MSG_MEDIA_SORT                     = _UxGT("Sort ") MEDIA_TYPE_EN;
  LSTR MSG_MEDIA_UPDATE                   = MEDIA_TYPE_EN _UxGT(" Update");
  LSTR MSG_KILL_SUBCALL_OVERFLOW          = _UxGT("Subcall Overflow");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Endstops"); // Max length 8 characters
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Soft Endstops");
  LSTR MSG_MAIN_MENU                      = _UxGT("Main Menu");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Advanced Settings");
  LSTR MSG_CONFIGURATION                  = _UxGT("Configuration");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Run Auto Files");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Disable Steppers");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Debug Menu");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Progress Bar Test");
  LSTR MSG_ENDSTOP_TEST                   = _UxGT("Endstop Test");
  LSTR MSG_Z_PROBE                        = _UxGT("Z Probe");
  LSTR MSG_HOMING                         = _UxGT("Homing");
  LSTR MSG_HOMING_FEEDRATE                = _UxGT("Homing Feedrate");
  LSTR MSG_HOMING_FEEDRATE_N              = _UxGT("@ Homing FR");
  LSTR MSG_AUTO_HOME                      = _UxGT("Auto Home");
  LSTR MSG_HOME_ALL                       = _UxGT("Home All");
  LSTR MSG_AUTO_HOME_N                    = _UxGT("Home @");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Home X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Home Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Home Z");
  LSTR MSG_Z_AFTER_HOME                   = _UxGT("Z After Homing");
  LSTR MSG_FILAMENT_SET                   = _UxGT("Filament Settings");
  LSTR MSG_FILAMENT_MAN                   = _UxGT("Filament Management");
  LSTR MSG_MANUAL_LEVELING                = _UxGT("Manual Leveling");
  LSTR MSG_TRAM_FL                        = _UxGT("Front Left");
  LSTR MSG_TRAM_FR                        = _UxGT("Front Right");
  LSTR MSG_TRAM_C                         = _UxGT("Center");
  LSTR MSG_TRAM_BL                        = _UxGT("Back Left");
  LSTR MSG_TRAM_BR                        = _UxGT("Back Right");
  LSTR MSG_MANUAL_MESH                    = _UxGT("Manual Mesh");
  LSTR MSG_AUTO_MESH                      = _UxGT("Auto Build Mesh");
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Auto Z-Align");
  LSTR MSG_ITERATION                      = _UxGT("G34 Iteration: %i");
  LSTR MSG_DECREASING_ACCURACY            = _UxGT("Accuracy Decreasing!");
  LSTR MSG_ACCURACY_ACHIEVED              = _UxGT("Accuracy Achieved");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Homing XYZ");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Click to Begin");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Next Point");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Leveling Done!");
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Fade Height");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Set Home Offsets");
  LSTR MSG_HOME_OFFSET_X                  = _UxGT("Home Offset X"); // DWIN
  LSTR MSG_HOME_OFFSET_Y                  = _UxGT("Home Offset Y"); // DWIN
  LSTR MSG_HOME_OFFSET_Z                  = _UxGT("Home Offset Z"); // DWIN
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Offsets Applied");
  LSTR MSG_ERR_M428_TOO_FAR               = _UxGT("MIN/MAX Too Far");
  LSTR MSG_TRAMMING_WIZARD                = _UxGT("Tramming Wizard");
  LSTR MSG_SELECT_ORIGIN                  = _UxGT("Select Origin");
  LSTR MSG_LAST_VALUE_SP                  = _UxGT("Last value ");

  LSTR MSG_PREHEAT_1                      = _UxGT("Preheat ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Preheat ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" End");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" End ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" All");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" Bed");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" Conf");

  LSTR MSG_PREHEAT_2                      = _UxGT("Preheat ") PREHEAT_2_LABEL;
  LSTR MSG_PREHEAT_3                      = _UxGT("Preheat ") PREHEAT_3_LABEL;
  LSTR MSG_PREHEAT_4                      = PREHEAT_4_LABEL;

  LSTR MSG_PREHEAT_M                      = _UxGT("Preheat $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Preheat $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Preheat $ End");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Preheat $ End ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Preheat $ All");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Preheat $ Bed");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Preheat $ Conf");

  LSTR MSG_PREHEAT_HOTEND                 = _UxGT("Preheat Hotend");
  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Preheat Custom");
  LSTR MSG_PREHEAT                        = _UxGT("Preheat");
  LSTR MSG_COOLDOWN                       = _UxGT("Cooldown");

  LSTR MSG_CUTTER_FREQUENCY               = _UxGT("Frequency");
  LSTR MSG_LASER_MENU                     = _UxGT("Laser Control");
  LSTR MSG_SPINDLE_MENU                   = _UxGT("Spindle Control");
  LSTR MSG_LASER_POWER                    = _UxGT("Laser Power");
  LSTR MSG_SPINDLE_POWER                  = _UxGT("Spindle Pwr");
  LSTR MSG_LASER_TOGGLE                   = _UxGT("Toggle Laser");
  LSTR MSG_LASER_EVAC_TOGGLE              = _UxGT("Toggle Blower");
  LSTR MSG_LASER_ASSIST_TOGGLE            = _UxGT("Air Assist");
  LSTR MSG_LASER_PULSE_MS                 = _UxGT("Test Pulse ms");
  LSTR MSG_LASER_FIRE_PULSE               = _UxGT("Fire Pulse");
  LSTR MSG_FLOWMETER_FAULT                = _UxGT("Coolant Flow Fault");
  LSTR MSG_SPINDLE_TOGGLE                 = _UxGT("Toggle Spindle");
  LSTR MSG_SPINDLE_EVAC_TOGGLE            = _UxGT("Toggle Vacuum");
  LSTR MSG_SPINDLE_FORWARD                = _UxGT("Spindle Forward");
  LSTR MSG_SPINDLE_REVERSE                = _UxGT("Spindle Reverse");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Switch Power On");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Switch Power Off");
  LSTR MSG_POWER_EDM_FAULT                = _UxGT("Power EDM Fault");
  LSTR MSG_EXTRUDE                        = _UxGT("Extrude");
  LSTR MSG_RETRACT                        = _UxGT("Retract");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Move Axis");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Probe and Level");
  LSTR MSG_BED_LEVELING                   = _UxGT("Bed Leveling");
  LSTR MSG_LEVEL_BED                      = _UxGT("Level Bed");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Bed Tramming");
  LSTR MSG_BED_TRAMMING_MANUAL            = _UxGT("Manual Tramming");
  LSTR MSG_BED_TRAMMING_RAISE             = _UxGT("Adjust bed until the probe triggers.");
  LSTR MSG_BED_TRAMMING_IN_RANGE          = _UxGT("Corners within tolerance. Bed trammed.");
  LSTR MSG_BED_TRAMMING_GOOD_POINTS       = _UxGT("Good Points: ");
  LSTR MSG_BED_TRAMMING_LAST_Z            = _UxGT("Last Z: ");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Next Corner");
  LSTR MSG_MESH_EDITOR                    = _UxGT("Mesh Editor");
  LSTR MSG_MESH_VIEWER                    = _UxGT("Mesh Viewer");
  LSTR MSG_EDIT_MESH                      = _UxGT("Edit Mesh");
  LSTR MSG_MESH_VIEW                      = _UxGT("View Mesh");
  LSTR MSG_EDITING_STOPPED                = _UxGT("Mesh Editing Stopped");
  LSTR MSG_NO_VALID_MESH                  = _UxGT("No valid mesh");
  LSTR MSG_ACTIVATE_MESH                  = _UxGT("Activate Leveling");
  LSTR MSG_PROBING_POINT                  = _UxGT("Probing Point");
  LSTR MSG_MESH_X                         = _UxGT("Index X");
  LSTR MSG_MESH_Y                         = _UxGT("Index Y");
  LSTR MSG_MESH_INSET                     = _UxGT("Mesh Inset");
  LSTR MSG_MESH_MIN_X                     = _UxGT("Mesh X Minimum");
  LSTR MSG_MESH_MAX_X                     = _UxGT("Mesh X Maximum");
  LSTR MSG_MESH_MIN_Y                     = _UxGT("Mesh Y Minimum");
  LSTR MSG_MESH_MAX_Y                     = _UxGT("Mesh Y Maximum");
  LSTR MSG_MESH_AMAX                      = _UxGT("Maximize Area");
  LSTR MSG_MESH_CENTER                    = _UxGT("Center Area");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Z Value");
  LSTR MSG_MESH_CANCEL                    = _UxGT("Mesh cancelled");
  LSTR MSG_MESH_RESET                     = _UxGT("Mesh reset");
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("Custom Commands");
  LSTR MSG_CUSTOM_MENU_MAIN_TITLE         = _UxGT(CUSTOM_MENU_MAIN_TITLE);
  LSTR MSG_TOOL_HEAD_TH                   = _UxGT(CUSTOM_MENU_MAIN_TITLE" (TH)");
  LSTR MSG_PRESENT_BED                    = _UxGT("Present Bed");
  LSTR MSG_M48_TEST                       = _UxGT("M48 Probe Test");
  LSTR MSG_M48_POINT                      = _UxGT("M48 Point");
  LSTR MSG_M48_OUT_OF_BOUNDS              = _UxGT("Probe out of bounds");
  LSTR MSG_M48_DEVIATION                  = _UxGT("Deviation");
  LSTR MSG_IDEX_MENU                      = _UxGT("IDEX Mode");
  LSTR MSG_OFFSETS_MENU                   = _UxGT("Tool Offsets");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Auto-Park");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Duplication");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("Mirrored Copy");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("Full Control");
  LSTR MSG_IDEX_DUPE_GAP                  = _UxGT("Duplicate X-Gap");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("2nd Nozzle Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("2nd Nozzle @");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("Doing G29");
  LSTR MSG_UBL_TOOLS                      = _UxGT("UBL Tools");
  LSTR MSG_LCD_TILTING_MESH               = _UxGT("Tilting Point");
  LSTR MSG_UBL_TILT_MESH                  = _UxGT("Tilt Mesh");
  LSTR MSG_UBL_TILTING_GRID               = _UxGT("Tilting Grid Size");
  LSTR MSG_UBL_MESH_TILTED                = _UxGT("Mesh Tilted");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Manually Build Mesh");
  LSTR MSG_UBL_MESH_WIZARD                = _UxGT("UBL Mesh Wizard");
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("Place Shim & Measure");
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Measure");
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Remove & Measure Bed");
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("Moving to next");
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Bed Temp");
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Bed Temp");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Hotend Temp");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Hotend Temp");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Edit Custom Mesh");
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("Fine Tuning Mesh");
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Done Editing Mesh");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Build Custom Mesh");
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Build Mesh");
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("Build Mesh ($)");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Build Cold Mesh");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Adjust Mesh Height");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Height Amount");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Validate Mesh");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Validate Mesh ($)");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Validate Custom Mesh");
  LSTR MSG_G26_HEATING_BED                = _UxGT("G26 Heating Bed");
  LSTR MSG_G26_HEATING_NOZZLE             = _UxGT("G26 Heating Nozzle");
  LSTR MSG_G26_MANUAL_PRIME               = _UxGT("Manual priming...");
  LSTR MSG_G26_FIXED_LENGTH               = _UxGT("Fixed Length Prime");
  LSTR MSG_G26_PRIME_DONE                 = _UxGT("Done Priming");
  LSTR MSG_G26_CANCELED                   = _UxGT("G26 Canceled");
  LSTR MSG_G26_LEAVING                    = _UxGT("Leaving G26");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Continue Bed Mesh");
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Mesh Leveling");
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("3-Point Leveling");
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Grid Mesh Leveling");
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Level Mesh");
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("Side Points");
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Map Type");
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("Output Mesh Map");
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("Output for Host");
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("Output for CSV");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Off Printer Backup");
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("Output UBL Info");
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Fill-in Amount");
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Manual Fill-in");
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Smart Fill-in");
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Fill-in Mesh");
  LSTR MSG_UBL_MESH_FILLED                = _UxGT("Missing Points Filled");
  LSTR MSG_UBL_MESH_INVALID               = _UxGT("Invalid Mesh");
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Invalidate All");
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Invalidate Closest");
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Fine Tune All");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Fine Tune Closest");
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Mesh Storage");
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Memory Slot");
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Load Bed Mesh");
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Save Bed Mesh");
  LSTR MSG_UBL_INVALID_SLOT               = _UxGT("First Select a Mesh Slot");
  LSTR MSG_MESH_LOADED                    = _UxGT("Mesh %i Loaded");
  LSTR MSG_MESH_SAVED                     = _UxGT("Mesh %i Saved");
  LSTR MSG_MESH_ACTIVE                    = _UxGT("Mesh %i active");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("No Storage");
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("UBL Save Error");
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("UBL Restore Error");
  LSTR MSG_UBL_Z_OFFSET                   = _UxGT("Z-Offset: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Z-Offset Stopped");
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("Step-By-Step UBL");
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1. Build Cold Mesh");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2. Smart Fill-in");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3. Validate Mesh");
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4. Fine Tune All");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5. Validate Mesh");
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6. Fine Tune All");
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7. Save Bed Mesh");

  LSTR MSG_LED_CONTROL                    = _UxGT("LED Control");
  LSTR MSG_LIGHTS                         = _UxGT("Lights");
  LSTR MSG_LIGHT_N                        = _UxGT("Light #{");
  LSTR MSG_LED_PRESETS                    = _UxGT("Light Presets");
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Red");
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Orange");
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Yellow");
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Green");
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Blue");
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Indigo");
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Violet");
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("White");
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Default");
  LSTR MSG_LED_CHANNEL_N                  = _UxGT("Channel {");
  LSTR MSG_NEO2_PRESETS                   = _UxGT("Light #2 Presets");
  LSTR MSG_NEO2_BRIGHTNESS                = _UxGT("Brightness");
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Custom Lights");
  LSTR MSG_INTENSITY_R                    = _UxGT("Red Intensity");
  LSTR MSG_INTENSITY_G                    = _UxGT("Green Intensity");
  LSTR MSG_INTENSITY_B                    = _UxGT("Blue Intensity");
  LSTR MSG_INTENSITY_W                    = _UxGT("White Intensity");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Brightness");
  LSTR MSG_LIGHT_ENCODER                  = _UxGT("Encoder Light");
  LSTR MSG_LIGHT_ENCODER_PRESETS          = _UxGT("Encoder Light Presets");

  LSTR MSG_MOVING                         = _UxGT("Moving...");
  LSTR MSG_FREE_XY                        = _UxGT("Free XY");
  LSTR MSG_MOVE_X                         = _UxGT("Move X"); // Used by draw_edit_screen
  LSTR MSG_MOVE_Y                         = _UxGT("Move Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Move Z");
  LSTR MSG_MOVE_N                         = _UxGT("Move @");
  LSTR MSG_MOVE_E                         = _UxGT("Move Extruder");
  LSTR MSG_MOVE_EN                        = _UxGT("Move *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("Hotend too cold");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Move $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Move $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Move $") LCD_STR_DEGREE;
  LSTR MSG_LIVE_MOVE                      = _UxGT("Live Move");
  LSTR MSG_SPEED                          = _UxGT("Speed");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Bed Z");
  LSTR MSG_NOZZLE                         = _UxGT("Nozzle");
  LSTR MSG_NOZZLE_N                       = _UxGT("Nozzle ~");
  LSTR MSG_NOZZLE_PARKED                  = _UxGT("Nozzle Parked");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("Nozzle Standby");
  LSTR MSG_BED                            = _UxGT("Bed");
  LSTR MSG_CHAMBER                        = _UxGT("Enclosure");
  LSTR MSG_COOLER                         = _UxGT("Laser Coolant");
  LSTR MSG_COOLER_TOGGLE                  = _UxGT("Toggle Cooler");
  LSTR MSG_FLOWMETER_SAFETY               = _UxGT("Flow Safety");
  LSTR MSG_CUTTER                         = _UxGT("Cutter");
  LSTR MSG_LASER                          = _UxGT("Laser");
  LSTR MSG_FAN_SPEED                      = _UxGT("Fan Speed");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Fan Speed ~");
  LSTR MSG_STORED_FAN_N                   = _UxGT("Stored Fan ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("Extra Fan Speed");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Extra Fan Speed ~");
  LSTR MSG_CONTROLLER_FAN                 = _UxGT("Controller Fan");
  LSTR MSG_CONTROLLER_FAN_IDLE_SPEED      = _UxGT("Idle Speed");
  LSTR MSG_CONTROLLER_FAN_AUTO_ON         = _UxGT("Auto Mode");
  LSTR MSG_CONTROLLER_FAN_SPEED           = _UxGT("Active Speed");
  LSTR MSG_CONTROLLER_FAN_DURATION        = _UxGT("Idle Period");
  LSTR MSG_FLOW_PERCENTAGE                = _UxGT("Set Flowrate Percentage");
  LSTR MSG_FLOW                           = _UxGT("Flow");
  LSTR MSG_FLOW_N                         = _UxGT("Flow ~");
  LSTR MSG_CONTROL                        = _UxGT("Control");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Fact");
  LSTR MSG_AUTOTEMP                       = _UxGT("Autotemp");
  LSTR MSG_TIMEOUT                        = _UxGT("Timeout");
  LSTR MSG_LCD_ON                         = _UxGT("On");
  LSTR MSG_LCD_OFF                        = _UxGT("Off");

  LSTR MSG_PID_AUTOTUNE                   = _UxGT("PID Autotune");
  LSTR MSG_PID_AUTOTUNE_E                 = _UxGT("Autotune * PID");
  LSTR MSG_PID_CYCLE                      = _UxGT("PID Cycles");
  LSTR MSG_PID_AUTOTUNE_DONE              = _UxGT("PID tuning done");
  LSTR MSG_PID_AUTOTUNE_FAILED            = _UxGT("Autotune failed!");

  LSTR MSG_PID_FOR_NOZZLE                 = _UxGT("for Nozzle is running.");
  LSTR MSG_PID_FOR_BED                    = _UxGT("for BED is running.");
  LSTR MSG_PID_FOR_CHAMBER                = _UxGT("for CHAMBER is running.");

  LSTR MSG_TEMP_NOZZLE                    = _UxGT("Nozzle Temperature");
  LSTR MSG_TEMP_BED                       = _UxGT("Bed Temperature");
  LSTR MSG_TEMP_CHAMBER                   = _UxGT("Chamber Temperature");

  LSTR MSG_BAD_HEATER_ID                  = _UxGT("Bad extruder.");
  LSTR MSG_TEMP_TOO_HIGH                  = _UxGT("Temperature too high.");
  LSTR MSG_TEMP_TOO_LOW                   = _UxGT("Temperature too low");

  LSTR MSG_PID_BAD_HEATER_ID              = _UxGT("Autotune failed! Bad extruder.");
  LSTR MSG_PID_TEMP_TOO_HIGH              = _UxGT("Autotune failed! Temperature too high.");
  LSTR MSG_PID_TIMEOUT                    = _UxGT("Autotune failed! Timeout.");

  LSTR MSG_MPC_MEASURING_AMBIENT          = _UxGT("Testing heat loss");
  LSTR MSG_MPC_HEATING_PAST_200           = _UxGT("Heating to >200C");
  LSTR MSG_MPC_COOLING_TO_AMBIENT         = _UxGT("Cooling to ambient");
  LSTR MSG_MPC_AUTOTUNE                   = _UxGT("MPC Autotune");
  LSTR MSG_MPC_EDIT                       = _UxGT("Edit * MPC");
  LSTR MSG_MPC_POWER                      = _UxGT("Heater Power");
  LSTR MSG_MPC_POWER_E                    = _UxGT("Power *");
  LSTR MSG_MPC_BLOCK_HEAT_CAPACITY        = _UxGT("Heat Capacity");
  LSTR MSG_MPC_BLOCK_HEAT_CAPACITY_E      = _UxGT("Heat Cap. *");
  LSTR MSG_SENSOR_RESPONSIVENESS          = _UxGT("Sensor Resp.");
  LSTR MSG_SENSOR_RESPONSIVENESS_E        = _UxGT("Sensor Resp. *");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF         = _UxGT("Ambient Coeff.");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF_E       = _UxGT("Ambient Co. *");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF_FAN     = _UxGT("Fan coeff.");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF_FAN_E   = _UxGT("Fan coeff. *");

  LSTR MSG_SELECT_E                       = _UxGT("Select *");
  LSTR MSG_ACC                            = _UxGT("Accel");
  LSTR MSG_JERK                           = _UxGT("Jerk");
  LSTR MSG_VA_JERK                        = _UxGT("Max ") STR_A _UxGT(" Jerk");
  LSTR MSG_VB_JERK                        = _UxGT("Max ") STR_B _UxGT(" Jerk");
  LSTR MSG_VC_JERK                        = _UxGT("Max ") STR_C _UxGT(" Jerk");
  LSTR MSG_VN_JERK                        = _UxGT("Max @ Jerk");
  LSTR MSG_VE_JERK                        = _UxGT("Max E Jerk");
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Junction Dev");
  LSTR MSG_STEP_SMOOTHING                 = _UxGT("Step Smoothing");
  LSTR MSG_MAX_SPEED                      = _UxGT("Max Speed (mm/s)");
  LSTR MSG_VMAX_A                         = _UxGT("Max ") STR_A _UxGT(" Speed");
  LSTR MSG_VMAX_B                         = _UxGT("Max ") STR_B _UxGT(" Speed");
  LSTR MSG_VMAX_C                         = _UxGT("Max ") STR_C _UxGT(" Speed");
  LSTR MSG_VMAX_N                         = _UxGT("Max @ Speed");
  LSTR MSG_VMAX_E                         = _UxGT("Max E Speed");
  LSTR MSG_VMAX_EN                        = _UxGT("Max * Speed");
  LSTR MSG_VMIN                           = _UxGT("Min Velocity");
  LSTR MSG_VTRAV_MIN                      = _UxGT("Min Travel Speed");
  LSTR MSG_ACCELERATION                   = _UxGT("Acceleration");
  LSTR MSG_AMAX_A                         = _UxGT("Max ") STR_A _UxGT(" Accel");
  LSTR MSG_AMAX_B                         = _UxGT("Max ") STR_B _UxGT(" Accel");
  LSTR MSG_AMAX_C                         = _UxGT("Max ") STR_C _UxGT(" Accel");
  LSTR MSG_AMAX_N                         = _UxGT("Max @ Accel");
  LSTR MSG_AMAX_E                         = _UxGT("Max E Accel");
  LSTR MSG_AMAX_EN                        = _UxGT("Max * Accel");
  LSTR MSG_A_RETRACT                      = _UxGT("Retract Accel");
  LSTR MSG_A_TRAVEL                       = _UxGT("Travel Accel");
  LSTR MSG_INPUT_SHAPING                  = _UxGT("Input Shaping");
  LSTR MSG_SHAPING_ENABLE_N               = _UxGT("Enable @ shaping");
  LSTR MSG_SHAPING_DISABLE_N              = _UxGT("Disable @ shaping");
  LSTR MSG_SHAPING_FREQ_N                 = _UxGT("@ frequency");
  LSTR MSG_SHAPING_ZETA_N                 = _UxGT("@ damping");
  LSTR MSG_SHAPING_A_FREQ                 = STR_A _UxGT(" frequency");  // ProUI
  LSTR MSG_SHAPING_B_FREQ                 = STR_B _UxGT(" frequency");  // ProUI
  LSTR MSG_SHAPING_C_FREQ                 = STR_C _UxGT(" frequency");  // ProUI
  LSTR MSG_SHAPING_A_ZETA                 = STR_A _UxGT(" damping");    // ProUI
  LSTR MSG_SHAPING_B_ZETA                 = STR_B _UxGT(" damping");    // ProUI
  LSTR MSG_SHAPING_C_ZETA                 = STR_C _UxGT(" damping");    // ProUI
  LSTR MSG_XY_FREQUENCY_LIMIT             = _UxGT("XY Freq Limit");
  LSTR MSG_XY_FREQUENCY_FEEDRATE          = _UxGT("Min FR Factor");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Steps/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" Steps/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" Steps/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" Steps/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ steps/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E steps/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* Steps/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperature");
  LSTR MSG_MOTION                         = _UxGT("Motion");
  LSTR MSG_FILAMENT                       = _UxGT("Filament");
  LSTR MSG_FILAMENT_EN                    = _UxGT("Filament *");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E in mm") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT               = _UxGT("E Limit in mm") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT_E             = _UxGT("E Limit *");
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Fil. Dia.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Fil. Dia. *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Unload mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Load mm");
  LSTR MSG_SEGMENTS_PER_SECOND            = _UxGT("Segments/Sec");
  LSTR MSG_DRAW_MIN_X                     = _UxGT("Draw Min X");
  LSTR MSG_DRAW_MAX_X                     = _UxGT("Draw Max X");
  LSTR MSG_DRAW_MIN_Y                     = _UxGT("Draw Min Y");
  LSTR MSG_DRAW_MAX_Y                     = _UxGT("Draw Max Y");
  LSTR MSG_MAX_BELT_LEN                   = _UxGT("Max Belt Len");
  LSTR MSG_LINEAR_ADVANCE                 = _UxGT("Linear Advance");
  LSTR MSG_ADVANCE_K                      = _UxGT("Advance K");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("Advance K *");
  LSTR MSG_CONTRAST                       = _UxGT("LCD Contrast");
  LSTR MSG_BRIGHTNESS                     = _UxGT("LCD Brightness");
  LSTR MSG_SCREEN_TIMEOUT                 = _UxGT("LCD Timeout (m)");
  LSTR MSG_HOTEND_TEMP_GRAPH              = _UxGT("Hotend Temp Graph");
  LSTR MSG_BED_TEMP_GRAPH                 = _UxGT("Bed Temp Graph");
  LSTR MSG_BRIGHTNESS_OFF                 = _UxGT("Backlight Off");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Store Settings");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Load Settings");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Restore Defaults");
  LSTR MSG_INIT_EEPROM                    = _UxGT("Initialize EEPROM");
  LSTR MSG_EEPROM_INITIALIZED             = _UxGT("EEPROM Initialized");
  LSTR MSG_ERR_EEPROM_CRC                 = _UxGT("Err: EEPROM CRC");
  LSTR MSG_ERR_EEPROM_SIZE                = _UxGT("Err: EEPROM Size");
  LSTR MSG_ERR_EEPROM_VERSION             = _UxGT("Err: EEPROM Version");
  LSTR MSG_ERR_EEPROM_CORRUPT             = _UxGT("Err: EEPROM Corrupt");
  LSTR MSG_SETTINGS_STORED                = _UxGT("Settings Stored");
  LSTR MSG_HAS_PREVIEW                    = _UxGT("Has preview");
  LSTR MSG_RESET_PRINTER                  = _UxGT("Reset Printer");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Refresh");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Info Screen");
  LSTR MSG_INFO_MACHINENAME               = _UxGT("Machine Name");
  LSTR MSG_INFO_SIZE                      = _UxGT("Size");
  LSTR MSG_INFO_FWVERSION                 = _UxGT("Firmware Version");
  LSTR MSG_INFO_BUILD                     = _UxGT("Build Datetime");
  LSTR MSG_PREPARE                        = _UxGT("Prepare");
  LSTR MSG_TUNE                           = _UxGT("Tune");
  LSTR MSG_POWER_MONITOR                  = _UxGT("Power monitor");
  LSTR MSG_CURRENT                        = _UxGT("Current");
  LSTR MSG_VOLTAGE                        = _UxGT("Voltage");
  LSTR MSG_POWER                          = _UxGT("Power");
  LSTR MSG_START_PRINT                    = _UxGT("Start Print");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("Next");
  LSTR MSG_BUTTON_INIT                    = _UxGT("Init");
  LSTR MSG_BUTTON_STOP                    = _UxGT("Stop");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("Print");
  LSTR MSG_BUTTON_RESET                   = _UxGT("Reset");
  LSTR MSG_BUTTON_IGNORE                  = _UxGT("Ignore");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("Cancel");
  LSTR MSG_BUTTON_CONFIRM                 = _UxGT("Confirm");
  LSTR MSG_BUTTON_CONTINUE                = _UxGT("Continue");
  LSTR MSG_BUTTON_DONE                    = _UxGT("Done");
  LSTR MSG_BUTTON_BACK                    = _UxGT("Back");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("Proceed");
  LSTR MSG_BUTTON_SKIP                    = _UxGT("Skip");
  LSTR MSG_BUTTON_INFO                    = _UxGT("Info");
  LSTR MSG_BUTTON_LEVEL                   = _UxGT("Level");
  LSTR MSG_BUTTON_PAUSE                   = _UxGT("Pause");
  LSTR MSG_BUTTON_RESUME                  = _UxGT("Resume");
  LSTR MSG_BUTTON_ADVANCED                = _UxGT("Advanced");
  LSTR MSG_BUTTON_SAVE                    = _UxGT("Save");
  LSTR MSG_BUTTON_PURGE                   = _UxGT("Purge");
  LSTR MSG_PAUSING                        = _UxGT("Pausing...");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Pause Print");
  LSTR MSG_ADVANCED_PAUSE                 = _UxGT("Advanced Pause");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Resume Print");
  LSTR MSG_STOP_PRINT                     = _UxGT("Stop Print");
  LSTR MSG_CANCEL_PRINT                   = _UxGT("Cancel Print");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Power Outage");
  LSTR MSG_RESUME_BED_TEMP                = _UxGT("Resume Bed Temp");
  LSTR MSG_HOST_START_PRINT               = _UxGT("Host Start");
  LSTR MSG_PRINTING_OBJECT                = _UxGT("Print Obj");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("Cancel Obj");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("Cancel Obj {");
  LSTR MSG_CONTINUE_PRINT_JOB             = _UxGT("Continue Job");
  LSTR MSG_MEDIA_MENU                     = MEDIA_TYPE_EN _UxGT(" Print");
  LSTR MSG_TURN_OFF                       = _UxGT("Turn off now");
  LSTR MSG_END_LOOPS                      = _UxGT("End Loops");
  LSTR MSG_NO_MEDIA                       = _UxGT("No ") MEDIA_TYPE_EN;
  LSTR MSG_DWELL                          = _UxGT("Sleep...");
  LSTR MSG_USERWAIT                       = _UxGT("Click to Resume...");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Print Paused");
  LSTR MSG_PRINTING                       = _UxGT("Printing...");
  LSTR MSG_STOPPING                       = _UxGT("Stopping...");
  LSTR MSG_REMAINING_TIME                 = _UxGT("Remaining");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Print Aborted");
  LSTR MSG_PRINT_DONE                     = _UxGT("Print Done");
  LSTR MSG_PRINTER_KILLED                 = _UxGT("Printer killed!");
  LSTR MSG_NO_MOVE                        = _UxGT("No Move.");
  LSTR MSG_KILLED                         = _UxGT("KILLED. ");
  LSTR MSG_STOPPED                        = _UxGT("STOPPED. ");
  LSTR MSG_FWRETRACT                      = _UxGT("Firmware Retract");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Retract mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Swap Re.mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Retract V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Hop mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("S Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Unretract V");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("S UnRet V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Auto-Retract");
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("Swap Length");
  LSTR MSG_FILAMENT_SWAP_EXTRA            = _UxGT("Swap Extra");
  LSTR MSG_FILAMENT_PURGE_LENGTH          = _UxGT("Purge Length");
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Tool Change");
  LSTR MSG_TOOL_HEAD_SWAP                 = _UxGT("Park For Tool Head Swap");
  LSTR MSG_FILAMENT_SWAP                  = _UxGT("Park For Filament Change");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Z Raise");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Prime Speed");
  LSTR MSG_SINGLENOZZLE_WIPE_RETRACT      = _UxGT("Wipe Retract");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Retract Speed");
  LSTR MSG_FILAMENT_PARK_ENABLED          = _UxGT("Park Head");
  LSTR MSG_PARK_FAILED                    = _UxGT("Head cannot be parked");
  LSTR MSG_SINGLENOZZLE_UNRETRACT_SPEED   = _UxGT("Recover Speed");
  LSTR MSG_SINGLENOZZLE_FAN_SPEED         = _UxGT("Fan Speed");
  LSTR MSG_SINGLENOZZLE_FAN_TIME          = _UxGT("Fan Time");
  LSTR MSG_TOOL_MIGRATION_ON              = _UxGT("Auto ON");
  LSTR MSG_TOOL_MIGRATION_OFF             = _UxGT("Auto OFF");
  LSTR MSG_TOOL_MIGRATION                 = _UxGT("Tool Migration");
  LSTR MSG_TOOL_MIGRATION_AUTO            = _UxGT("Auto-migration");
  LSTR MSG_TOOL_MIGRATION_END             = _UxGT("Last Extruder");
  LSTR MSG_TOOL_MIGRATION_SWAP            = _UxGT("Migrate to *");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Change Filament");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Change * Filament");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Load Filament");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Load * Filament");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Unload Filament");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Unload * Filament");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Unload All");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Attach SD Card");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Attach USB Drive");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Attach ") MEDIA_TYPE_EN;
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Change ") MEDIA_TYPE_EN;
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Release ") MEDIA_TYPE_EN;
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Z Probe Past Bed");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Skew Factor");
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("Self-Test");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Reset");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("Stow");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("Deploy");
  LSTR MSG_BLTOUCH_SW_MODE                = _UxGT("SW-Mode");
  LSTR MSG_BLTOUCH_SPEED_MODE             = _UxGT("High Speed");
  LSTR MSG_BLTOUCH_5V_MODE                = _UxGT("5V-Mode");
  LSTR MSG_BLTOUCH_OD_MODE                = _UxGT("OD-Mode");
  LSTR MSG_BLTOUCH_MODE_STORE             = _UxGT("Mode-Store");
  LSTR MSG_BLTOUCH_MODE_STORE_5V          = _UxGT("Set BLTouch to 5V");
  LSTR MSG_BLTOUCH_MODE_STORE_OD          = _UxGT("Set BLTouch to OD");
  LSTR MSG_BLTOUCH_MODE_ECHO              = _UxGT("Report Drain");
  LSTR MSG_BLTOUCH_MODE_CHANGE            = _UxGT("DANGER: Bad settings can cause damage! Proceed anyway?");
  LSTR MSG_TOUCHMI_PROBE                  = _UxGT("TouchMI");
  LSTR MSG_TOUCHMI_INIT                   = _UxGT("Init TouchMI");
  LSTR MSG_TOUCHMI_ZTEST                  = _UxGT("Z Offset Test");
  LSTR MSG_TOUCHMI_SAVE                   = _UxGT("Save");
  LSTR MSG_MANUAL_DEPLOY_TOUCHMI          = _UxGT("Deploy TouchMI");
  LSTR MSG_MANUAL_DEPLOY                  = _UxGT("Deploy Z-Probe");
  LSTR MSG_MANUAL_PENUP                   = _UxGT("Pen up");
  LSTR MSG_MANUAL_PENDOWN                 = _UxGT("Pen down");
  LSTR MSG_MANUAL_STOW                    = _UxGT("Stow Z-Probe");
  LSTR MSG_HOME_FIRST                     = _UxGT("Home %s First");
  LSTR MSG_ZPROBE_SETTINGS                = _UxGT("Probe Settings");
  LSTR MSG_ZPROBE_OFFSETS                 = _UxGT("Probe Offsets");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Probe X Offset");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Probe Y Offset");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Probe Z Offset");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Probe @ Offset");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Babystep Probe Z");
  LSTR MSG_ZPROBE_MARGIN                  = _UxGT("Probe Margin");
  LSTR MSG_ZOFFSET                        = _UxGT("Z Offset");
  LSTR MSG_Z_FEED_RATE                    = _UxGT("Z Feedrate");
  LSTR MSG_ENABLE_HS_MODE                 = _UxGT("Enable HS mode");
  LSTR MSG_MOVE_NOZZLE_TO_BED             = _UxGT("Move Nozzle to Bed");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Babystep X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Babystep Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Babystep Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Babystep @");
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("Total");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Endstop Abort");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Heating Failed");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Err: REDUNDANT TEMP");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("THERMAL RUNAWAY");
  LSTR MSG_ERR_TEMP_MALFUNCTION           = _UxGT("TEMP MALFUNCTION");
  LSTR MSG_ERR_COOLING_FAILED             = _UxGT("Cooling Failed");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Err: MAXTEMP");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Err: MINTEMP");
  LSTR MSG_HALTED                         = _UxGT("PRINTER HALTED");
  LSTR MSG_PLEASE_WAIT                    = _UxGT("Please wait...");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Please Reset");
  LSTR MSG_PREHEATING                     = _UxGT("Preheating...");
  LSTR MSG_HEATING                        = _UxGT("Heating...");
  LSTR MSG_COOLING                        = _UxGT("Cooling...");
  LSTR MSG_BED_HEATING                    = _UxGT("Bed Heating...");
  LSTR MSG_BED_COOLING                    = _UxGT("Bed Cooling...");
  LSTR MSG_BED_ANNEALING                  = _UxGT("Annealing...");
  LSTR MSG_PROBE_HEATING                  = _UxGT("Probe Heating...");
  LSTR MSG_PROBE_COOLING                  = _UxGT("Probe Cooling...");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("Chamber Heating...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("Chamber Cooling...");
  LSTR MSG_LASER_COOLING                  = _UxGT("Laser Cooling...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Delta Calibration");
  LSTR MSG_DELTA_CALIBRATION_IN_PROGRESS  = _UxGT("Delta Calibration in progress");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Calibrate X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Calibrate Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Calibrate Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Calibrate Center");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Delta Settings");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Auto Calibration");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Diag Rod");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Height");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Radius");
  LSTR MSG_INFO_MENU                      = _UxGT("About Printer");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Printer Info");
  LSTR MSG_3POINT_LEVELING                = _UxGT("3-Point Leveling");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Linear Leveling");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("Bilinear Leveling");
  LSTR MSG_UBL_LEVELING                   = _UxGT("Unified Bed Leveling");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Mesh Leveling");
  LSTR MSG_MESH_DONE                      = _UxGT("Mesh probing done");
  LSTR MSG_INFO_PRINTER_STATS_MENU        = _UxGT("Printer Stats");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Stats");
  LSTR MSG_RESET_STATS                    = _UxGT("Reset Print Stats?");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Board Info");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Thermistors");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Extruders");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Baud");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protocol");
  LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Runaway Watch: OFF");
  LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Runaway Watch: ON");
  LSTR MSG_HOTEND_IDLE_TIMEOUT            = _UxGT("Hotend Idle Timeout");
  LSTR MSG_BED_IDLE_TIMEOUT               = _UxGT("Bed Idle Timeout");
  LSTR MSG_HOTEND_IDLE_DISABLE            = _UxGT("Disable Timeout");
  LSTR MSG_HOTEND_IDLE_NOZZLE_TARGET      = _UxGT("Nozzle Idle Temp");
  LSTR MSG_HOTEND_IDLE_BED_TARGET         = _UxGT("Bed Idle Temp");
  LSTR MSG_FAN_SPEED_FAULT                = _UxGT("Fan speed fault");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Case Light");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Light Brightness");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("INCORRECT PRINTER");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Prints");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Total");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Longest");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Extruded");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Completed");
  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Min Temp");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Max Temp");
  LSTR MSG_INFO_PSU                       = _UxGT("PSU");

  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Drive Strength");
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ Driver %");
  LSTR MSG_ERROR_TMC                      = _UxGT("TMC CONNECTION ERROR");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("DAC EEPROM Write");
  LSTR MSG_FILAMENT_CHANGE_HEADER         = _UxGT("FILAMENT CHANGE");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("PRINT PAUSED");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("LOAD FILAMENT");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("UNLOAD FILAMENT");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("RESUME OPTIONS:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("Purge more");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Continue");
  LSTR MSG_FILAMENT_CHANGE_PURGE_CONTINUE = _UxGT("Purge or Continue?");  // ProUI
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Nozzle: ");
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Runout Sensor");
  LSTR MSG_SENSOR                         = _UxGT("Sensor");
  LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("Runout Dist mm");
  LSTR MSG_EXTRUDER_MIN_TEMP              = _UxGT("Extruder Min Temp.");  // ProUI
  LSTR MSG_FANCHECK                       = _UxGT("Fan Tacho Check");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Homing Failed");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Probing Failed");

  // Ender-3 V2 DWIN - ProUI / JyersUI
  LSTR MSG_COLORS_GET                     = _UxGT("Get Color");               // ProUI
  LSTR MSG_COLORS_SELECT                  = _UxGT("Select Colors");           // ProUI
  LSTR MSG_COLORS_APPLIED                 = _UxGT("Colors applied");          // ProUI
  LSTR MSG_COLORS_RED                     = _UxGT("Red");                     // ProUI / JyersUI
  LSTR MSG_COLORS_GREEN                   = _UxGT("Green");                   // ProUI / JyersUI
  LSTR MSG_COLORS_BLUE                    = _UxGT("Blue");                    // ProUI / JyersUI
  LSTR MSG_COLORS_WHITE                   = _UxGT("White");                   // ProUI
  LSTR MSG_UI_LANGUAGE                    = _UxGT("UI Language");             // ProUI
  LSTR MSG_SOUND_ENABLE                   = _UxGT("Enable sound");            // ProUI
  LSTR MSG_LOCKSCREEN                     = _UxGT("Lock Screen");             // ProUI
  LSTR MSG_LOCKSCREEN_LOCKED              = _UxGT("Printer is Locked,");      // ProUI
  LSTR MSG_LOCKSCREEN_UNLOCK              = _UxGT("Scroll to unlock.");       // ProUI
  LSTR MSG_PLEASE_WAIT_REBOOT             = _UxGT("Please wait for reboot."); // ProUI
  LSTR MSG_MEDIA_NOT_INSERTED             = _UxGT("No Media");                // ProUI
  LSTR MSG_PLEASE_PREHEAT                 = _UxGT("Please Preheat");          // ProUI

  // Prusa MMU 2
  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("CHOOSE FILAMENT");
  LSTR MSG_MMU2_MENU                      = _UxGT("MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("Update MMU Firmware!");
  LSTR MSG_MMU2_NOT_RESPONDING            = _UxGT("MMU Needs Attention.");
  LSTR MSG_MMU2_RESUME                    = _UxGT("Resume");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("MMU Resuming...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("Load");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("Load All");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("Load to Nozzle");
  LSTR MSG_MMU2_CUT_FILAMENT              = _UxGT("Cut");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("Eject");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("Eject ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("Unload");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("Filament %i Load...");
  LSTR MSG_MMU2_CUTTING_FILAMENT          = _UxGT("Filament %i Cut...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Filament %i Eject...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("Filament %i Unload...");
  LSTR MSG_MMU2_ALL                       = _UxGT("All");
  LSTR MSG_MMU2_FILAMENT_N                = _UxGT("Filament ~");
  LSTR MSG_MMU2_RESET                     = _UxGT("Reset MMU");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("Resetting...");
  LSTR MSG_MMU2_EJECT_RECOVER             = _UxGT("Eject Recover");
  LSTR MSG_MMU2_REMOVE_AND_CLICK          = _UxGT("Remove and click...");

  LSTR MSG_MMU_SENSITIVITY                = _UxGT("Sensitivity");
  LSTR MSG_MMU_CUTTER                     = _UxGT("Cutter");
  LSTR MSG_MMU_CUTTER_MODE                = _UxGT("Cutter Mode");
  LSTR MSG_MMU_CUTTER_MODE_DISABLE        = _UxGT("Disable");
  LSTR MSG_MMU_CUTTER_MODE_ENABLE         = _UxGT("Enable");
  LSTR MSG_MMU_CUTTER_MODE_ALWAYS         = _UxGT("Always");
  LSTR MSG_MMU_SPOOL_JOIN                 = _UxGT("Spool Join");

  LSTR MSG_MMU_STATISTICS                 = _UxGT("Statistics");
  LSTR MSG_MMU_RESET_FAIL_STATS           = _UxGT("Reset Fail Stats");
  LSTR MSG_MMU_RESET_STATS                = _UxGT("Reset All Stats");
  LSTR MSG_MMU_CURRENT_PRINT              = _UxGT("Curr. print");
  LSTR MSG_MMU_CURRENT_PRINT_FAILURES     = _UxGT("Curr. print failures");
  LSTR MSG_MMU_LAST_PRINT                 = _UxGT("Last print");
  LSTR MSG_MMU_LAST_PRINT_FAILURES        = _UxGT("Last print failures");
  LSTR MSG_MMU_TOTAL                      = _UxGT("Total");
  LSTR MSG_MMU_TOTAL_FAILURES             = _UxGT("Total failures");
  LSTR MSG_MMU_DEV_INCREMENT_FAILS        = _UxGT("Increment fails");
  LSTR MSG_MMU_DEV_INCREMENT_LOAD_FAILS   = _UxGT("Increment load fails");
  LSTR MSG_MMU_FAILS                      = _UxGT("MMU fails");
  LSTR MSG_MMU_LOAD_FAILS                 = _UxGT("MMU load fails");
  LSTR MSG_MMU_POWER_FAILS                = _UxGT("MMU power fails");
  LSTR MSG_MMU_MATERIAL_CHANGES           = _UxGT("Material changes");

  // Mixing Extruder (e.g., Geeetech A10M / A20M)
  LSTR MSG_MIX                            = _UxGT("Mix");
  LSTR MSG_MIX_COMPONENT_N                = _UxGT("Component {");
  LSTR MSG_MIXER                          = _UxGT("Mixer");
  LSTR MSG_GRADIENT                       = _UxGT("Gradient");
  LSTR MSG_FULL_GRADIENT                  = _UxGT("Full Gradient");
  LSTR MSG_TOGGLE_MIX                     = _UxGT("Toggle Mix");
  LSTR MSG_CYCLE_MIX                      = _UxGT("Cycle Mix");
  LSTR MSG_GRADIENT_MIX                   = _UxGT("Gradient Mix");
  LSTR MSG_REVERSE_GRADIENT               = _UxGT("Reverse Gradient");
  LSTR MSG_ACTIVE_VTOOL                   = _UxGT("Active V-tool");
  LSTR MSG_START_VTOOL                    = _UxGT("Start V-tool");
  LSTR MSG_END_VTOOL                      = _UxGT("  End V-tool");
  LSTR MSG_GRADIENT_ALIAS                 = _UxGT("Alias V-tool");
  LSTR MSG_RESET_VTOOLS                   = _UxGT("Reset V-tools");
  LSTR MSG_COMMIT_VTOOL                   = _UxGT("Commit V-tool Mix");
  LSTR MSG_VTOOLS_RESET                   = _UxGT("V-tools Were Reset");
  LSTR MSG_START_Z                        = _UxGT("Start Z:");
  LSTR MSG_END_Z                          = _UxGT("  End Z:");

  // Games Menu
  LSTR MSG_GAMES                          = _UxGT("Games");
  LSTR MSG_BRICKOUT                       = _UxGT("Brickout");
  LSTR MSG_INVADERS                       = _UxGT("Invaders");
  LSTR MSG_SNAKE                          = _UxGT("Sn4k3");
  LSTR MSG_MAZE                           = _UxGT("Maze");

  // Direct Stepping
  LSTR MSG_BAD_PAGE                       = _UxGT("Bad page index");
  LSTR MSG_BAD_PAGE_SPEED                 = _UxGT("Bad page speed");

  // Password Lock
  LSTR MSG_EDIT_PASSWORD                  = _UxGT("Edit Password");
  LSTR MSG_LOGIN_REQUIRED                 = _UxGT("Login Required");
  LSTR MSG_PASSWORD_SETTINGS              = _UxGT("Password Settings");
  LSTR MSG_ENTER_DIGIT                    = _UxGT("Enter Digit");
  LSTR MSG_CHANGE_PASSWORD                = _UxGT("Set/Edit Password");
  LSTR MSG_REMOVE_PASSWORD                = _UxGT("Remove Password");
  LSTR MSG_PASSWORD_SET                   = _UxGT("Password is ");
  LSTR MSG_START_OVER                     = _UxGT("Start Over");
  LSTR MSG_REMINDER_SAVE_SETTINGS         = _UxGT("Remember to Save!");
  LSTR MSG_PASSWORD_REMOVED               = _UxGT("Password Removed");

  // Filament Change screens show up to 2 lines on a 3-line display
  LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_1_LINE("Click to continue"));
  LSTR MSG_PAUSE_PRINT_PARKING            = _UxGT(MSG_1_LINE("Parking..."));
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Please wait..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Insert and Click"));
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_1_LINE("Click to heat"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Heating..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Ejecting..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Loading..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_1_LINE("Purging..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_1_LINE("Click to finish"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("Resuming..."));
  LSTR MSG_TMC_DRIVERS                    = _UxGT("TMC Drivers");
  LSTR MSG_TMC_CURRENT                    = _UxGT("Driver Current");
  LSTR MSG_TMC_ACURRENT                   = STR_A _UxGT("Driver Current");
  LSTR MSG_TMC_BCURRENT                   = STR_B _UxGT("Driver Current");
  LSTR MSG_TMC_CCURRENT                   = STR_C _UxGT("Driver Current");
  LSTR MSG_TMC_ECURRENT                   = _UxGT("E Driver Current");
  LSTR MSG_TMC_HYBRID_THRS                = _UxGT("Hybrid Threshold");
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("Sensorless Homing");
  LSTR MSG_TMC_STEPPING_MODE              = _UxGT("Stepping Mode");
  LSTR MSG_TMC_STEALTH_ENABLED            = _UxGT("StealthChop Enabled");
  LSTR MSG_SERVICE_RESET                  = _UxGT("Reset");
  LSTR MSG_SERVICE_IN                     = _UxGT(" in:");
  LSTR MSG_BACKLASH                       = _UxGT("Backlash");
  LSTR MSG_BACKLASH_CORRECTION            = _UxGT("Correction");
  LSTR MSG_BACKLASH_SMOOTHING             = _UxGT("Smoothing");

  LSTR MSG_FIXED_TIME_MOTION              = _UxGT("Fixed-Time Motion");
  LSTR MSG_FTM_CMPN_MODE                  = _UxGT("@ Comp. Mode:");
  LSTR MSG_FTM_ZV                         = _UxGT("ZV");
  LSTR MSG_FTM_ZVD                        = _UxGT("ZVD");
  LSTR MSG_FTM_ZVDD                       = _UxGT("ZVDD");
  LSTR MSG_FTM_ZVDDD                      = _UxGT("ZVDDD");
  LSTR MSG_FTM_EI                         = _UxGT("EI");
  LSTR MSG_FTM_2HEI                       = _UxGT("2HEI");
  LSTR MSG_FTM_3HEI                       = _UxGT("3HEI");
  LSTR MSG_FTM_MZV                        = _UxGT("MZV");
  //LSTR MSG_FTM_ULENDO_FBS               = _UxGT("Ulendo FBS");
  //LSTR MSG_FTM_DISCTF                   = _UxGT("DISCTF");
  LSTR MSG_FTM_DYN_MODE                   = _UxGT("DF Mode:");
  LSTR MSG_FTM_Z_BASED                    = _UxGT("Z-based");
  LSTR MSG_FTM_MASS_BASED                 = _UxGT("Mass-based");
  LSTR MSG_FTM_BASE_FREQ_N                = _UxGT("@ Base Freq.");
  LSTR MSG_FTM_DFREQ_K_N                  = _UxGT("@ Dyn. Freq.");
  LSTR MSG_FTM_ZETA_N                     = _UxGT("@ Damping");
  LSTR MSG_FTM_VTOL_N                     = _UxGT("@ Vib. Level");

  LSTR MSG_LEVEL_X_AXIS                   = _UxGT("Level X Axis");
  LSTR MSG_AUTO_CALIBRATE                 = _UxGT("Auto Calibrate");
  LSTR MSG_FTDI_HEATER_TIMEOUT            = _UxGT("Idle timeout, temperature decreased. Press Okay to reheat and again to resume.");
  LSTR MSG_HEATER_TIMEOUT                 = _UxGT("Heater Timeout");
  LSTR MSG_REHEAT                         = _UxGT("Reheat");
  LSTR MSG_REHEATING                      = _UxGT("Reheating...");
  LSTR MSG_REHEATDONE                     = _UxGT("Reheat Done");

  LSTR MSG_PROBE_WIZARD                   = _UxGT("Z Probe Wizard");
  LSTR MSG_PROBE_WIZARD_PROBING           = _UxGT("Probing Z Reference");
  LSTR MSG_PROBE_WIZARD_MOVING            = _UxGT("Moving to Probing Pos");

  LSTR MSG_XATC                           = _UxGT("X-Twist Wizard");
  LSTR MSG_XATC_DONE                      = _UxGT("X-Twist Wizard Done!");
  LSTR MSG_XATC_UPDATE_Z_OFFSET           = _UxGT("Update Z-Offset to ");

  LSTR MSG_SOUND                          = _UxGT("Sound");

  LSTR MSG_TOP_LEFT                       = _UxGT("Top Left");
  LSTR MSG_BOTTOM_LEFT                    = _UxGT("Bottom Left");
  LSTR MSG_TOP_RIGHT                      = _UxGT("Top Right");
  LSTR MSG_BOTTOM_RIGHT                   = _UxGT("Bottom Right");
  LSTR MSG_CALIBRATION_COMPLETED          = _UxGT("Calibration Completed");
  LSTR MSG_CALIBRATION_FAILED             = _UxGT("Calibration Failed");

  LSTR MSG_DRIVER_BACKWARD                = _UxGT(" driver backward");

  LSTR MSG_SD_CARD                        = _UxGT("SD Card");
  LSTR MSG_USB_DISK                       = _UxGT("USB Disk");

  LSTR MSG_HOST_SHUTDOWN                  = _UxGT("Host Shutdown");

  LSTR MSG_SHORT_DAY                      = _UxGT("d"); // One character only
  LSTR MSG_SHORT_HOUR                     = _UxGT("h"); // One character only
  LSTR MSG_SHORT_MINUTE                   = _UxGT("m"); // One character only

  // These strings can be the same in all languages
  LSTR MSG_MARLIN                         = _UxGT("Marlin");
  LSTR MSG_PID_P                          = _UxGT("PID-P");
  LSTR MSG_PID_P_E                        = _UxGT("PID-P *");
  LSTR MSG_PID_I                          = _UxGT("PID-I");
  LSTR MSG_PID_I_E                        = _UxGT("PID-I *");
  LSTR MSG_PID_D                          = _UxGT("PID-D");
  LSTR MSG_PID_D_E                        = _UxGT("PID-D *");
  LSTR MSG_PID_C                          = _UxGT("PID-C");
  LSTR MSG_PID_C_E                        = _UxGT("PID-C *");
  LSTR MSG_PID_F                          = _UxGT("PID-F");
  LSTR MSG_PID_F_E                        = _UxGT("PID-F *");
  LSTR MSG_BACKLASH_N                     = _UxGT("@");

  // DGUS-Specific message strings, not used elsewhere
  LSTR DGUS_MSG_NOT_WHILE_PRINTING        = _UxGT("Not allowed during print");
  LSTR DGUS_MSG_NOT_WHILE_IDLE            = _UxGT("Not allowed while idle");
  LSTR DGUS_MSG_NO_FILE_SELECTED          = _UxGT("No file selected");
  LSTR DGUS_MSG_EXECUTING_COMMAND         = _UxGT("Executing command...");
  LSTR DGUS_MSG_BED_PID_DISABLED          = _UxGT("Bed PID disabled");
  LSTR DGUS_MSG_PID_DISABLED              = _UxGT("PID disabled");
  LSTR DGUS_MSG_PID_AUTOTUNING            = _UxGT("PID autotuning...");
  LSTR DGUS_MSG_INVALID_RECOVERY_DATA     = _UxGT("Invalid recovery data");

  LSTR DGUS_MSG_HOMING_REQUIRED           = _UxGT("Homing required");
  LSTR DGUS_MSG_BUSY                      = _UxGT("Busy");
  LSTR DGUS_MSG_HOMING                    = _UxGT("Homing...");
  LSTR DGUS_MSG_FW_OUTDATED               = _UxGT("DWIN GUI/OS update required");
  LSTR DGUS_MSG_ABL_REQUIRED              = _UxGT("Auto bed leveling required");
  LSTR DGUS_MSG_PROBING_FAILED            = _UxGT("Probing failed");
  LSTR DGUS_MSG_PROBING_SUCCESS           = _UxGT("Probing successful");
  LSTR DGUS_MSG_RESET_EEPROM              = _UxGT("EEPROM reset");
  LSTR DGUS_MSG_WRITE_EEPROM_FAILED       = _UxGT("EEPROM write failed");
  LSTR DGUS_MSG_READ_EEPROM_FAILED        = _UxGT("EEPROM read failed");
  LSTR DGUS_MSG_FILAMENT_RUNOUT           = _UxGT("Filament runout E%d");

  //
  // MMU3 Translatable Strings
  //

  LSTR MSG_TITLE_FINDA_DIDNT_TRIGGER      = _UxGT("FINDA DIDNT TRIGGER");
  LSTR MSG_TITLE_FINDA_FILAMENT_STUCK     = _UxGT("FINDA FILAM. STUCK");
  LSTR MSG_TITLE_FSENSOR_DIDNT_TRIGGER    = _UxGT("FSENSOR DIDNT TRIGG.");
  LSTR MSG_TITLE_FSENSOR_FILAMENT_STUCK   = _UxGT("FSENSOR FIL. STUCK");
  LSTR MSG_TITLE_PULLEY_CANNOT_MOVE       = _UxGT("PULLEY CANNOT MOVE");
  LSTR MSG_TITLE_FSENSOR_TOO_EARLY        = _UxGT("FSENSOR TOO EARLY");
  LSTR MSG_TITLE_INSPECT_FINDA            = _UxGT("INSPECT FINDA");
  LSTR MSG_TITLE_LOAD_TO_EXTRUDER_FAILED  = _UxGT("LOAD TO EXTR. FAILED");
  LSTR MSG_TITLE_SELECTOR_CANNOT_MOVE     = _UxGT("SELECTOR CANNOT MOVE");
  LSTR MSG_TITLE_SELECTOR_CANNOT_HOME     = _UxGT("SELECTOR CANNOT HOME");
  LSTR MSG_TITLE_IDLER_CANNOT_MOVE        = _UxGT("IDLER CANNOT MOVE");
  LSTR MSG_TITLE_IDLER_CANNOT_HOME        = _UxGT("IDLER CANNOT HOME");
  LSTR MSG_TITLE_TMC_WARNING_TMC_TOO_HOT  = _UxGT("WARNING TMC TOO HOT");
  LSTR MSG_TITLE_TMC_OVERHEAT_ERROR       = _UxGT("TMC OVERHEAT ERROR");
  LSTR MSG_TITLE_TMC_DRIVER_ERROR         = _UxGT("TMC DRIVER ERROR");
  LSTR MSG_TITLE_TMC_DRIVER_RESET         = _UxGT("TMC DRIVER RESET");
  LSTR MSG_TITLE_TMC_UNDERVOLTAGE_ERROR   = _UxGT("TMC UNDERVOLTAGE ERR");
  LSTR MSG_TITLE_TMC_DRIVER_SHORTED       = _UxGT("TMC DRIVER SHORTED");
  LSTR MSG_TITLE_SELFTEST_FAILED          = _UxGT("MMU SELFTEST FAILED");
  LSTR MSG_TITLE_MMU_MCU_ERROR            = _UxGT("MMU MCU ERROR");
  LSTR MSG_TITLE_MMU_NOT_RESPONDING       = _UxGT("MMU NOT RESPONDING");
  LSTR MSG_TITLE_COMMUNICATION_ERROR      = _UxGT("COMMUNICATION ERROR");
  LSTR MSG_TITLE_FILAMENT_ALREADY_LOADED  = _UxGT("FIL. ALREADY LOADED");
  LSTR MSG_TITLE_INVALID_TOOL             = _UxGT("INVALID TOOL");
  LSTR MSG_TITLE_QUEUE_FULL               = _UxGT("QUEUE FULL");
  LSTR MSG_TITLE_FW_UPDATE_NEEDED         = _UxGT("MMU FW UPDATE NEEDED");
  LSTR MSG_TITLE_FW_RUNTIME_ERROR         = _UxGT("FW RUNTIME ERROR");
  LSTR MSG_TITLE_UNLOAD_MANUALLY          = _UxGT("UNLOAD MANUALLY");
  LSTR MSG_TITLE_FILAMENT_EJECTED         = _UxGT("FILAMENT EJECTED");
  LSTR MSG_TITLE_FILAMENT_CHANGE          = _UxGT("FILAMENT CHANGE");
  LSTR MSG_TITLE_UNKNOWN_ERROR            = _UxGT("UNKNOWN ERROR");

  LSTR MSG_DESC_FINDA_DIDNT_TRIGGER       = _UxGT("FINDA didn't trigger while loading the filament. Ensure the filament can move and FINDA works.");
  LSTR MSG_DESC_FINDA_FILAMENT_STUCK      = _UxGT("FINDA didn't switch off while unloading filament. Try unloading manually. Ensure filament can move and FINDA works.");
  LSTR MSG_DESC_FSENSOR_DIDNT_TRIGGER     = _UxGT("Filament sensor didn't trigger while loading the filament. Ensure the sensor is calibrated and the filament reached it.");
  LSTR MSG_DESC_FSENSOR_FILAMENT_STUCK    = _UxGT("Filament sensor didn't switch off while unloading filament. Ensure filament can move and the sensor works.");
  LSTR MSG_DESC_PULLEY_CANNOT_MOVE        = _UxGT("Pulley motor stalled. Ensure the pulley can move and check the wiring.");
  LSTR MSG_DESC_FSENSOR_TOO_EARLY         = _UxGT("Filament sensor triggered too early while loading to extruder. Check there isn't anything stuck in PTFE tube. Check that sensor reads properly.");
  LSTR MSG_DESC_INSPECT_FINDA             = _UxGT("Selector can't move due to FINDA detecting a filament. Make sure no filament is in Selector and FINDA works properly.");
  LSTR MSG_DESC_LOAD_TO_EXTRUDER_FAILED   = _UxGT("Loading to extruder failed. Inspect the filament tip shape. Refine the sensor calibration, if needed.");
  LSTR MSG_DESC_SELECTOR_CANNOT_HOME      = _UxGT("The Selector cannot home properly. Check for anything blocking its movement.");
  LSTR MSG_DESC_CANNOT_MOVE               = _UxGT("Can't move Selector or Idler.");
  LSTR MSG_DESC_IDLER_CANNOT_HOME         = _UxGT("The Idler cannot home properly. Check for anything blocking its movement.");
  LSTR MSG_DESC_TMC                       = _UxGT("More details online.");
  LSTR MSG_DESC_MMU_NOT_RESPONDING        = _UxGT("MMU not responding. Check the wiring and connectors.");
  LSTR MSG_DESC_COMMUNICATION_ERROR       = _UxGT("MMU not responding correctly. Check the wiring and connectors.");
  LSTR MSG_DESC_FILAMENT_ALREADY_LOADED   = _UxGT("Cannot perform the action, filament is already loaded. Unload it first.");
  LSTR MSG_DESC_INVALID_TOOL              = _UxGT("Requested filament tool is not available on this hardware. Check the G-code for tool index out of range (T0-T4).");
  LSTR MSG_DESC_QUEUE_FULL                = _UxGT("MMU Firmware internal error, please reset the MMU.");
  LSTR MSG_DESC_FW_RUNTIME_ERROR          = _UxGT("Internal runtime error. Try resetting the MMU or updating the firmware.");
  LSTR MSG_DESC_UNLOAD_MANUALLY           = _UxGT("Filament detected unexpectedly. Ensure no filament is loaded. Check the sensors and wiring.");
  LSTR MSG_DESC_FILAMENT_EJECTED          = _UxGT("Remove the ejected filament from the front of the MMU.");
  LSTR MSG_DESC_FILAMENT_CHANGE           = _UxGT("M600 Filament Change. Load a new filament or eject the old one.");
  LSTR MSG_DESC_UNKNOWN_ERROR             = _UxGT("Unexpected error occurred.");

  LSTR MSG_DESC_FW_UPDATE_NEEDED          = _UxGT("MMU FW version is not supported. Update to version " STRINGIFY(mmuVersionMajor) "." STRINGIFY(mmuVersionMinor) "." STRINGIFY(mmuVersionPatch) ".");

  LSTR MSG_BTN_RETRY                      = _UxGT("Retry");
  LSTR MSG_BTN_RESET_MMU                  = _UxGT("ResetMMU");
  LSTR MSG_BTN_UNLOAD                     = _UxGT("Unload");
  LSTR MSG_BTN_LOAD                       = _UxGT("Load");
  LSTR MSG_BTN_EJECT                      = _UxGT("Eject");
  LSTR MSG_BTN_STOP                       = _UxGT("Stop");
  LSTR MSG_BTN_DISABLE_MMU                = _UxGT("Disable");
  LSTR MSG_BTN_MORE                       = _UxGT("More Info");

  // Prusa MMU
  LSTR MSG_DONE                           = _UxGT("Done");
  LSTR MSG_FINISHING_MOVEMENTS            = _UxGT("Finishing movements");
  LSTR MSG_LOADING_FILAMENT               = _UxGT("Loading filament");
  LSTR MSG_UNLOADING_FILAMENT             = _UxGT("Unloading filament");
  LSTR MSG_TESTING_FILAMENT               = _UxGT("Testing filament");
  LSTR MSG_EJECT_FROM_MMU                 = _UxGT("Eject from MMU");
  LSTR MSG_CUT_FILAMENT                   = _UxGT("Cut filament");
  LSTR MSG_OFF                            = _UxGT("Off");
  LSTR MSG_ON                             = _UxGT("On");
  LSTR MSG_PROGRESS_OK                    = _UxGT("OK");
  LSTR MSG_PROGRESS_ENGAGE_IDLER          = _UxGT("Engaging idler");
  LSTR MSG_PROGRESS_DISENGAGE_IDLER       = _UxGT("Disengaging idler");
  LSTR MSG_PROGRESS_UNLOAD_FINDA          = _UxGT("Unloading to FINDA");
  LSTR MSG_PROGRESS_UNLOAD_PULLEY         = _UxGT("Unloading to pulley");
  LSTR MSG_PROGRESS_FEED_FINDA            = _UxGT("Feeding to FINDA");
  LSTR MSG_PROGRESS_FEED_EXTRUDER         = _UxGT("Feeding to extruder");
  LSTR MSG_PROGRESS_FEED_NOZZLE           = _UxGT("Feeding to nozzle");
  LSTR MSG_PROGRESS_AVOID_GRIND           = _UxGT("Avoiding grind");
  LSTR MSG_PROGRESS_WAIT_USER             = _UxGT("ERR Wait for User");
  LSTR MSG_PROGRESS_ERR_INTERNAL          = _UxGT("ERR Internal");
  LSTR MSG_PROGRESS_ERR_HELP_FIL          = _UxGT("ERR Help filament");
  LSTR MSG_PROGRESS_ERR_TMC               = _UxGT("ERR TMC failed");
  LSTR MSG_PROGRESS_SELECT_SLOT           = _UxGT("Selecting fil. slot");
  LSTR MSG_PROGRESS_PREPARE_BLADE         = _UxGT("Preparing blade");
  LSTR MSG_PROGRESS_PUSH_FILAMENT         = _UxGT("Pushing filament");
  LSTR MSG_PROGRESS_PERFORM_CUT           = _UxGT("Performing cut");
  LSTR MSG_PROGRESSPSTRETURN_SELECTOR     = _UxGT("Returning selector");
  LSTR MSG_PROGRESS_PARK_SELECTOR         = _UxGT("Parking selector");
  LSTR MSG_PROGRESS_EJECT_FILAMENT        = _UxGT("Ejecting filament");
  LSTR MSG_PROGRESSPSTRETRACT_FINDA       = _UxGT("Retract from FINDA");
  LSTR MSG_PROGRESS_HOMING                = _UxGT("Homing");
  LSTR MSG_PROGRESS_MOVING_SELECTOR       = _UxGT("Moving selector");
  LSTR MSG_PROGRESS_FEED_FSENSOR          = _UxGT("Feeding to FSensor");
}

namespace LanguageWide_en {
  using namespace LanguageNarrow_en;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_LIVE_MOVE                    = _UxGT("Live Movement");
    LSTR MSG_HOST_START_PRINT             = _UxGT("Start Host Print");
    LSTR MSG_PRINTING_OBJECT              = _UxGT("Printing Object");
    LSTR MSG_CANCEL_OBJECT                = _UxGT("Cancel Object");
    LSTR MSG_CANCEL_OBJECT_N              = _UxGT("Cancel Object {");
    LSTR MSG_CONTINUE_PRINT_JOB           = _UxGT("Continue Print Job");
    LSTR MSG_MEDIA_MENU                   = _UxGT("Select from ") MEDIA_TYPE_EN;
    LSTR MSG_TURN_OFF                     = _UxGT("Turn off the printer");
    LSTR MSG_END_LOOPS                    = _UxGT("End Repeat Loops");
    LSTR MSG_MEDIA_NOT_INSERTED           = _UxGT("No media inserted.");          // ProUI
    LSTR MSG_PLEASE_PREHEAT               = _UxGT("Please preheat the hot end."); // ProUI
    LSTR MSG_INFO_PRINT_COUNT_RESET       = _UxGT("Reset Print Count");           // ProUI
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Print Count");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Print Time");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Longest Job Time");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Extruded Total");
    LSTR MSG_HOMING_FEEDRATE_N            = _UxGT("@ Homing Feedrate");
    LSTR MSG_HOMING_FEEDRATE_X            = _UxGT("X Homing Feedrate");
    LSTR MSG_HOMING_FEEDRATE_Y            = _UxGT("Y Homing Feedrate");
    LSTR MSG_HOMING_FEEDRATE_Z            = _UxGT("Z Homing Feedrate");
    LSTR MSG_EEPROM_INITIALIZED           = _UxGT("Default Settings Restored");
  #endif
}

namespace LanguageTall_en {
  using namespace LanguageWide_en;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Press Button", "to resume print"));
    LSTR MSG_PAUSE_PRINT_PARKING          = _UxGT(MSG_1_LINE("Parking..."));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Wait for", "filament change", "to start"));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Insert filament", "and press button", "to continue"));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Press button", "to heat nozzle"));
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Nozzle heating", "Please wait..."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_2_LINE("Wait for", "filament unload"));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_2_LINE("Wait for", "filament load"));
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_2_LINE("Wait for", "filament purge"));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_2_LINE("Click to finish", "filament purge"));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_2_LINE("Wait for print", "to resume..."));
  #endif
}

namespace Language_en {
  using namespace LanguageTall_en;
}
