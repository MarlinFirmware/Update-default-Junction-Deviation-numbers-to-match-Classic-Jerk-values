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
  * Romanian
  *
  * LCD Menu Messages
  * See also https://marlinfw.org/docs/development/lcd_language.html
  *
  * Translation by cristyanul
  */
namespace LanguageNarrow_ro {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Romanian");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" Pregatit.");
  LSTR MSG_YES                            = _UxGT("DA");
  LSTR MSG_NO                             = _UxGT("NU");
  LSTR MSG_BACK                           = _UxGT("Inapoi");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Abandon...");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Media Introdus");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Media Inlaturat");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("Astept Media");
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("Eroare Citire Media");
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("Dispozitiv USB Inlaturat");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("Pornire USB Esuata");
  LSTR MSG_KILL_SUBCALL_OVERFLOW          = _UxGT("Eroare:Subcall Overflow");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Endstops"); // Max length 8 characters
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Soft Endstops");
  LSTR MSG_MAIN_MENU                      = _UxGT("Principal");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Setari Avansate");
  LSTR MSG_CONFIGURATION                  = _UxGT("Configurare");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Autostart");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Dezactivare Motoare");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Meniu Debug");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Test Bara Progres");
  LSTR MSG_AUTO_HOME                      = _UxGT("Auto Acasa");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Acasa X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Acasa Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Acasa Z");
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Auto Aliniere-Z");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Acasa XYZ");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Click pentru a incepe");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Urmatorul Punct");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Nivelare Terminata!");
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Fade Inaltime");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Seteaza Offseturile Acasa");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Offseturi Aplicate");

  LSTR MSG_PREHEAT_1                      = _UxGT("Preincalzeste ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Preincalzeste ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Preincalzeste ") PREHEAT_1_LABEL _UxGT(" Capatul");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Preincalzeste ") PREHEAT_1_LABEL _UxGT(" Capatul ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Preincalzeste ") PREHEAT_1_LABEL _UxGT(" Tot");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Preincalzeste ") PREHEAT_1_LABEL _UxGT(" Patul");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Preincalzeste ") PREHEAT_1_LABEL _UxGT(" Conf");

  LSTR MSG_PREHEAT_M                      = _UxGT("Preincalzeste $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Preincalzeste $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Preincalzeste $ Capatul");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Preincalzeste $ Capatul ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Preincalzeste $ Tot");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Preincalzeste $ Patul");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Preincalzeste $ Conf");

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Preincalzeste Personalizat");
  LSTR MSG_COOLDOWN                       = _UxGT("Racire");
  LSTR MSG_CUTTER_FREQUENCY               = _UxGT("Frecventa");
  LSTR MSG_LASER_MENU                     = _UxGT("Control Laser");
  LSTR MSG_LASER_POWER                    = _UxGT("Puterea Laserului");
  LSTR MSG_SPINDLE_MENU                   = _UxGT("Controlul Spindle");
  LSTR MSG_SPINDLE_POWER                  = _UxGT("Puterea Spindle");
  LSTR MSG_SPINDLE_REVERSE                = _UxGT("Spindle Invers");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Porneste");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Opreste");
  LSTR MSG_EXTRUDE                        = _UxGT("Extrudeaza");
  LSTR MSG_RETRACT                        = _UxGT("Retracteaza");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Muta Axa");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Probare si Nivelare");
  LSTR MSG_BED_LEVELING                   = _UxGT("Nivelarea Patului");
  LSTR MSG_LEVEL_BED                      = _UxGT("Niveleaza Patul");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Niveleaza Colturile");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Urmatorul Colt");
  LSTR MSG_MESH_EDITOR                    = _UxGT("Editor Mesh");
  LSTR MSG_EDIT_MESH                      = _UxGT("Editeaza Mesh");
  LSTR MSG_EDITING_STOPPED                = _UxGT("Editarea Meshului Oprita");
  LSTR MSG_PROBING_POINT                  = _UxGT("Punctul de Probare");
  LSTR MSG_MESH_X                         = _UxGT("Index X");
  LSTR MSG_MESH_Y                         = _UxGT("Index Y");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Valoare Z");
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("Comenzi Personalizate");
  LSTR MSG_M48_TEST                       = _UxGT("M48 Probe Test");
  LSTR MSG_M48_POINT                      = _UxGT("M48 Point");
  LSTR MSG_M48_DEVIATION                  = _UxGT("Deviation");
  LSTR MSG_IDEX_MENU                      = _UxGT("IDEX Mode");
  LSTR MSG_OFFSETS_MENU                   = _UxGT("Tool Offsets");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Auto-Park");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Duplication");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("Mirrored Copy");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("Full Control");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("2nd Nozzle Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("2nd Nozzle @");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("Doing G29");
  LSTR MSG_UBL_TOOLS                      = _UxGT("UBL Tools");
  LSTR MSG_LCD_TILTING_MESH               = _UxGT("Tilting Point");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Manually Build Mesh");
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
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Validate Mesh ($)");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Build Cold Mesh");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Adjust Mesh Height");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Height Amount");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Validate Mesh");
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
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Off Imprimanta Backup");
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("Output UBL Info");
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Fill-in Amount");
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Manual Fill-in");
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Smart Fill-in");
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Fill-in Mesh");
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Invalidate All");
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Invalidate Closest");
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Fine Tune All");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Fine Tune Closest");
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Mesh Storage");
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Memory Slot");
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Load Bed Mesh");
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Save Bed Mesh");
  LSTR MSG_MESH_LOADED                    = _UxGT("Mesh %i Loaded");
  LSTR MSG_MESH_SAVED                     = _UxGT("Mesh %i Saved");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("No Storage");
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("Err: UBL Save");
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("Err: UBL Restore");
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
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Custom Lights");
  LSTR MSG_INTENSITY_R                    = _UxGT("Red Intensity");
  LSTR MSG_INTENSITY_G                    = _UxGT("Green Intensity");
  LSTR MSG_INTENSITY_B                    = _UxGT("Blue Intensity");
  LSTR MSG_INTENSITY_W                    = _UxGT("White Intensity");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Brightness");

  LSTR MSG_MOVING                         = _UxGT("Moving...");
  LSTR MSG_FREE_XY                        = _UxGT("Free XY");
  LSTR MSG_MOVE_X                         = _UxGT("Move X");
  LSTR MSG_MOVE_Y                         = _UxGT("Move Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Move Z");
  LSTR MSG_MOVE_N                         = _UxGT("Move @");
  LSTR MSG_MOVE_E                         = _UxGT("Extruder");
  LSTR MSG_MOVE_EN                        = _UxGT("Extruder *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("Capat Prea Rece");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Move $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Move $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Move $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Speed");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Bed Z");
  LSTR MSG_NOZZLE                         = _UxGT("Nozzle");
  LSTR MSG_NOZZLE_N                       = _UxGT("Nozzle ~");
  LSTR MSG_NOZZLE_PARKED                  = _UxGT("Nozzle Parked");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("Nozzle Standby");
  LSTR MSG_BED                            = _UxGT("Bed");
  LSTR MSG_CHAMBER                        = _UxGT("Enclosure");
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
  LSTR MSG_FLOW                           = _UxGT("Flow");
  LSTR MSG_FLOW_N                         = _UxGT("Flow ~");
  LSTR MSG_CONTROL                        = _UxGT("Control");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Fact");
  LSTR MSG_AUTOTEMP                       = _UxGT("Autotemp");
  LSTR MSG_LCD_ON                         = _UxGT("On");
  LSTR MSG_LCD_OFF                        = _UxGT("Off");
  LSTR MSG_PID_AUTOTUNE                   = _UxGT("PID Autotune");
  LSTR MSG_PID_AUTOTUNE_E                 = _UxGT("PID Autotune *");
  LSTR MSG_PID_AUTOTUNE_DONE              = _UxGT("PID tuning done");
  LSTR MSG_PID_BAD_HEATER_ID              = _UxGT("Autotune failed! Bad extruder.");
  LSTR MSG_PID_TEMP_TOO_HIGH              = _UxGT("Autotune failed! Temperature too high.");
  LSTR MSG_PID_TIMEOUT                    = _UxGT("Autotune failed! Timeout.");
  LSTR MSG_SELECT_E                       = _UxGT("Select *");
  LSTR MSG_ACC                            = _UxGT("Accel");
  LSTR MSG_JERK                           = _UxGT("Jerk");
  LSTR MSG_VA_JERK                        = _UxGT("V") STR_A _UxGT("-Jerk");
  LSTR MSG_VB_JERK                        = _UxGT("V") STR_B _UxGT("-Jerk");
  LSTR MSG_VC_JERK                        = _UxGT("V") STR_C _UxGT("-Jerk");
  LSTR MSG_VN_JERK                        = _UxGT("V@-Jerk");
  LSTR MSG_VE_JERK                        = _UxGT("Ve-Jerk");
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Junction Dev");
  LSTR MSG_MAX_SPEED                      = _UxGT("Max Speed");
  LSTR MSG_VMAX_A                         = _UxGT("Max Speed ") STR_A;
  LSTR MSG_VMAX_B                         = _UxGT("Max Speed ") STR_B;
  LSTR MSG_VMAX_C                         = _UxGT("Max Speed ") STR_C;
  LSTR MSG_VMAX_N                         = _UxGT("Max Speed @");
  LSTR MSG_VMAX_E                         = _UxGT("Max Speed E");
  LSTR MSG_VMAX_EN                        = _UxGT("Max Speed *");
  LSTR MSG_VMIN                           = _UxGT("Vmin");
  LSTR MSG_VTRAV_MIN                      = _UxGT("VTrav Min");
  LSTR MSG_ACCELERATION                   = _UxGT("Acceleration");
  LSTR MSG_AMAX_A                         = _UxGT("Amax ") STR_A;
  LSTR MSG_AMAX_B                         = _UxGT("Amax ") STR_B;
  LSTR MSG_AMAX_C                         = _UxGT("Amax ") STR_C;
  LSTR MSG_AMAX_N                         = _UxGT("Amax @");
  LSTR MSG_AMAX_E                         = _UxGT("Amax E");
  LSTR MSG_AMAX_EN                        = _UxGT("Amax *");
  LSTR MSG_A_RETRACT                      = _UxGT("A-Retract");
  LSTR MSG_A_TRAVEL                       = _UxGT("A-Travel");
  LSTR MSG_XY_FREQUENCY_LIMIT             = _UxGT("Frequency max");
  LSTR MSG_XY_FREQUENCY_FEEDRATE          = _UxGT("Feed min");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Steps/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" steps/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" steps/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" steps/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ steps/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E steps/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* steps/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperature");
  LSTR MSG_MOTION                         = _UxGT("Motion");
  LSTR MSG_FILAMENT                       = _UxGT("Filament");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E in mm") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT               = _UxGT("E Limit in mm") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT_E             = _UxGT("E Limit *");
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Fil. Dia.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Fil. Dia. *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Unload mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Load mm");
  LSTR MSG_ADVANCE_K                      = _UxGT("Advance K");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("Advance K *");
  LSTR MSG_CONTRAST                       = _UxGT("LCD Contrast");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Store Settings");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Load Settings");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Restore Defaults");
  LSTR MSG_INIT_EEPROM                    = _UxGT("Initialize EEPROM");
  LSTR MSG_ERR_EEPROM_CRC                 = _UxGT("Err: EEPROM CRC");
  LSTR MSG_ERR_EEPROM_SIZE                = _UxGT("Err: Mărimea EEPROM");
  LSTR MSG_ERR_EEPROM_VERSION             = _UxGT("Err: Versiunea EEPROM");
  LSTR MSG_SETTINGS_STORED                = _UxGT("Settings Stored");
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("Media Update");
  LSTR MSG_RESET_PRINTER                  = _UxGT("Reset Imprimanta");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Refresh");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Info Screen");
  LSTR MSG_PREPARE                        = _UxGT("Prepare");
  LSTR MSG_TUNE                           = _UxGT("Tune");
  LSTR MSG_POWER_MONITOR                  = _UxGT("Power monitor");
  LSTR MSG_CURRENT                        = _UxGT("Intensitate");
  LSTR MSG_VOLTAGE                        = _UxGT("Voltaj");
  LSTR MSG_POWER                          = _UxGT("Putere");
  LSTR MSG_START_PRINT                    = _UxGT("Start Imprimare");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("Urmatorul");
  LSTR MSG_BUTTON_INIT                    = _UxGT("Initiere");
  LSTR MSG_BUTTON_STOP                    = _UxGT("Stop");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("Imprimeaza");
  LSTR MSG_BUTTON_RESET                   = _UxGT("Reseteaza");
  LSTR MSG_BUTTON_IGNORE                  = _UxGT("Ignora");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("Anuleaza");
  LSTR MSG_BUTTON_DONE                    = _UxGT("OK");
  LSTR MSG_BUTTON_BACK                    = _UxGT("Inapoi");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("Proceed");
  LSTR MSG_PAUSING                        = _UxGT("Pausing...");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Pause Print");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Resume Print");
  LSTR MSG_STOP_PRINT                     = _UxGT("Stop Print");
  LSTR MSG_PRINTING_OBJECT                = _UxGT("Printing Object");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("Cancel Object");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("Cancel Object {");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Outage Recovery");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Print from Media");
  LSTR MSG_NO_MEDIA                       = _UxGT("No Media");
  LSTR MSG_DWELL                          = _UxGT("Sleep...");
  LSTR MSG_USERWAIT                       = _UxGT("Click to Resume...");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Print Paused");
  LSTR MSG_PRINTING                       = _UxGT("Printing...");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Print Aborted");
  LSTR MSG_PRINT_DONE                     = _UxGT("Print Done");
  LSTR MSG_NO_MOVE                        = _UxGT("No Move.");
  LSTR MSG_KILLED                         = _UxGT("KILLED. ");
  LSTR MSG_STOPPED                        = _UxGT("STOPPED. ");
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
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Tool Inlocuire");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Z Raise");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Prime Speed");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Retract Speed");
  LSTR MSG_FILAMENT_PARK_ENABLED          = _UxGT("Park Head");
  LSTR MSG_SINGLENOZZLE_UNRETRACT_SPEED   = _UxGT("Recover Speed");
  LSTR MSG_SINGLENOZZLE_FAN_SPEED         = _UxGT("Fan Speed");
  LSTR MSG_SINGLENOZZLE_FAN_TIME          = _UxGT("Fan Time");
  LSTR MSG_TOOL_MIGRATION_ON              = _UxGT("Auto Pornit");
  LSTR MSG_TOOL_MIGRATION_OFF             = _UxGT("Auto Oprit");
  LSTR MSG_TOOL_MIGRATION                 = _UxGT("Tool Migration");
  LSTR MSG_TOOL_MIGRATION_AUTO            = _UxGT("Auto-migrare");
  LSTR MSG_TOOL_MIGRATION_END             = _UxGT("Last Extruder");
  LSTR MSG_TOOL_MIGRATION_SWAP            = _UxGT("Migrate to *");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Inlocuire Filament");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Inlocuire Filament *");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Incarcare Filament");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Incarcare Filament *");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Scoatere Filament");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Scoatere Filament *");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Scoate Tot");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Atasare card SD");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Atasare FD USB");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Atasare card SD");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Inlocuire Media");
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Eliberare Media");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Z Probe Past Bed");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Skew Factor");
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("Self-Test");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Reset");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("Stow");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("Deploy");
  LSTR MSG_BLTOUCH_SW_MODE                = _UxGT("SW-Mode");
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
  LSTR MSG_MANUAL_STOW                    = _UxGT("Stow Z-Probe");
  LSTR MSG_HOME_FIRST                     = _UxGT("Home %s First");
  LSTR MSG_ZPROBE_OFFSETS                 = _UxGT("Probe Offsets");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Probe X Offset");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Probe Y Offset");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Probe Z Offset");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Probe @ Offset");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Pas Z ajustare");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Babystep X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Babystep Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Babystep Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Babystep @");
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("Total");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Endstop Abort");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Heating Failed");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Err: REDUNDANT TEMP");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("THERMAL RUNAWAY");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Err: MAXTEMP");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Err: MINTEMP");
  LSTR MSG_HALTED                         = _UxGT("PRINTER HALTED");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Please Reset");
  LSTR MSG_HEATING                        = _UxGT("Heating...");
  LSTR MSG_COOLING                        = _UxGT("Cooling...");
  LSTR MSG_BED_HEATING                    = _UxGT("Bed Heating...");
  LSTR MSG_BED_COOLING                    = _UxGT("Bed Cooling...");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("Chamber Heating...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("Chamber Cooling...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Delta Calibration");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Calibrate X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Calibrate Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Calibrate Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Calibrate Center");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Delta Settings");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Auto Calibration");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Diag Rod");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Inaltime");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Radius");
  LSTR MSG_INFO_MENU                      = _UxGT("Despre Imprimanta");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Info Imprimanta");
  LSTR MSG_3POINT_LEVELING                = _UxGT("Nivelare in 3 puncte");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Nivelare Lineara");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("Nivelare Bilineara");
  LSTR MSG_UBL_LEVELING                   = _UxGT("Nivelarea Patului Unificata");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Nivelare Mesh");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Status Imprimanta");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Informatii Placa");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termistoare");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Extrudere");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Baud");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protocol");
  LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Runaway Watch: OFF");
  LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Runaway Watch: ON");
  LSTR MSG_HOTEND_IDLE_TIMEOUT            = _UxGT("Hotend Idle Timeout");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Case Light");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Light Brightness");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("INCORRECT PRINTER");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Prints");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Completed");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Total");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Longest");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Extruded");

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Temperatura Minima");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Temperatura Maxima");
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
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Nozzle: ");
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Runout Sensor");
  LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("Runout Dist mm");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Homing Failed");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Probing Failed");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("CHOOSE FILAMENT");
  LSTR MSG_MMU2_MENU                      = _UxGT("MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("Update MMU Firmware!");
  LSTR MSG_MMU2_NOT_RESPONDING            = _UxGT("MMU Needs Attention.");
  LSTR MSG_MMU2_RESUME                    = _UxGT("MMU Resume");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("MMU Resuming...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("MMU Load");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("MMU Load All");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("MMU Load to Nozzle");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("MMU Eject");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("MMU Eject ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("MMU Unload");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("Loading Fil. %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Ejecting Fil. ...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("Unloading Fil....");
  LSTR MSG_MMU2_ALL                       = _UxGT("All");
  LSTR MSG_MMU2_FILAMENT_N                = _UxGT("Filament ~");
  LSTR MSG_MMU2_RESET                     = _UxGT("Reset MMU");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("MMU Resetting...");
  LSTR MSG_MMU2_EJECT_RECOVER             = _UxGT("Remove, click");

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

  LSTR MSG_GAMES                          = _UxGT("Jocuri");
  LSTR MSG_BRICKOUT                       = _UxGT("Brickout");
  LSTR MSG_INVADERS                       = _UxGT("Invaders");
  LSTR MSG_SNAKE                          = _UxGT("Sn4k3");
  LSTR MSG_MAZE                           = _UxGT("Maze");

  LSTR MSG_BAD_PAGE                       = _UxGT("Bad page index");
  LSTR MSG_BAD_PAGE_SPEED                 = _UxGT("Bad page speed");

  //
  // Filament Inlocuire screens show up to 2 lines on a 3-line display
  //
  LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_1_LINE("Click to continue"));
  LSTR MSG_PAUSE_PRINT_PARKING            = _UxGT(MSG_1_LINE("Parcare..."));
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Va rog asteptati..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Insert and Click"));
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_1_LINE("Click pentru incalzire"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Incalzire..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Ejectare..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Incarcare..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_1_LINE("Curatare..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_1_LINE("Click pentru a termina"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("Se Reia..."));

  LSTR MSG_TMC_DRIVERS                    = _UxGT("TMC Drivers");
  LSTR MSG_TMC_CURRENT                    = _UxGT("Driver Current");
  LSTR MSG_TMC_HYBRID_THRS                = _UxGT("Hybrid Threshold");
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("Sensorless Homing");
  LSTR MSG_TMC_STEPPING_MODE              = _UxGT("Stepping Mode");
  LSTR MSG_TMC_STEALTH_ENABLED            = _UxGT("StealthChop Enabled");
  LSTR MSG_SERVICE_RESET                  = _UxGT("Reset");
  LSTR MSG_SERVICE_IN                     = _UxGT(" in:");
  LSTR MSG_BACKLASH                       = _UxGT("Backlash");
  LSTR MSG_BACKLASH_CORRECTION            = _UxGT("Corectare");
  LSTR MSG_BACKLASH_SMOOTHING             = _UxGT("Smoothing");

  LSTR MSG_LEVEL_X_AXIS                   = _UxGT("Nivelare Axa X");
  LSTR MSG_AUTO_CALIBRATE                 = _UxGT("Auto Calibrare");
  LSTR MSG_HEATER_TIMEOUT                 = _UxGT("Timeout Incalzitor");
  LSTR MSG_REHEAT                         = _UxGT("Reincalzire");
  LSTR MSG_REHEATING                      = _UxGT("Reincalzire...");
}

namespace LanguageWide_ro {
  using namespace LanguageNarrow_ro;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Total Printuri");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Completat");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Timp Imprimare Total");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Longest Job Time");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Total Extrudat");
  #endif
}

namespace LanguageTall_ro {
  using namespace LanguageWide_ro;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Apasa Butonul", "pentru a reveni la print"));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Astept ca", "inlocuirea filamentului", "sa inceapa"));
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

namespace Language_ro {
  using namespace LanguageTall_ro;
}
