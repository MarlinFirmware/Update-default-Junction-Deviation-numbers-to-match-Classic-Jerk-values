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
 * GNU General Public License för more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * Swedish
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#define DISPLAY_CHARSET_ISO10646_1

namespace LanguageNarrow_sv {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Swedish");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" Redo.");
  LSTR MSG_YES                            = _UxGT("JA");
  LSTR MSG_NO                             = _UxGT("NEJ");
  LSTR MSG_BACK                           = _UxGT("Bakåt");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Avbryter...");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Media Instatt");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Media Borttaget");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("Väntar på media");
  LSTR MSG_MEDIA_INIT_FAIL                = _UxGT("Media init misslyckades");
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("Media läsningsfel");
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("USB enhet borttagen");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("USB start misslyckad");
  LSTR MSG_KILL_SUBCALL_OVERFLOW          = _UxGT("Underanrop överskriden");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Slutstop"); // Max length 8 characters
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Mjuk slutstopp");
  LSTR MSG_MAIN_MENU                      = _UxGT("Huvud");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Advancerade inställningar");
  LSTR MSG_CONFIGURATION                  = _UxGT("Konfiguration");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Autostarta Filer");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Inaktivera Stegare");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Debug Meny");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Framstegsindikator Test");
  LSTR MSG_AUTO_HOME                      = _UxGT("Auto Hem");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Hem X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Hem Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Hem Z");
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Auto Z-Justering");
  LSTR MSG_ITERATION                      = _UxGT("G34 Iteration: %i");
  LSTR MSG_DECREASING_ACCURACY            = _UxGT("Noggrannhet Minskar!");
  LSTR MSG_ACCURACY_ACHIEVED              = _UxGT("Noggrannhet uppnådd");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Hemning XYZ");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Klicka för att börja");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Nästa Punkt");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Nivellering Färdig!");
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Falna Höjd");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Sätt Hem Offset");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Offset Tillämpad");
  LSTR MSG_TRAMMING_WIZARD                = _UxGT("Justerings Wizard");
  LSTR MSG_SELECT_ORIGIN                  = _UxGT("Välj Origo");
  LSTR MSG_LAST_VALUE_SP                  = _UxGT("Senaste värde ");

  LSTR MSG_PREHEAT_1                      = _UxGT("Förvärmning ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Förvärmning ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Förvärmning ") PREHEAT_1_LABEL _UxGT(" Stoppa");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Förvärmning ") PREHEAT_1_LABEL _UxGT(" Stoppa ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Förvärmning ") PREHEAT_1_LABEL _UxGT(" Alla");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Förvärmning ") PREHEAT_1_LABEL _UxGT(" Bädd");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Förvärmning ") PREHEAT_1_LABEL _UxGT(" Konf");

  LSTR MSG_PREHEAT_M                      = _UxGT("Förvärmning $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Förvärmning $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Förvärmning $ Stoppa");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Förvärmning $ Stoppa ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Förvärmning $ Alla");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Förvärmning $ Bädd");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Förvärmning $ Donf");

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Förvärmning Anpassad");
  LSTR MSG_COOLDOWN                       = _UxGT("Nedkylning");
  LSTR MSG_CUTTER_FREQUENCY               = _UxGT("Frekvens");
  LSTR MSG_LASER_MENU                     = _UxGT("Laser kontroll");
  LSTR MSG_SPINDLE_MENU                   = _UxGT("Spindel Kontroll");
  LSTR MSG_LASER_POWER                    = _UxGT("Laser Styrka");
  LSTR MSG_SPINDLE_POWER                  = _UxGT("Spindel Styrka");
  LSTR MSG_LASER_TOGGLE                   = _UxGT("Växla Laser");
  LSTR MSG_LASER_PULSE_MS                 = _UxGT("Test Puls ms");
  LSTR MSG_LASER_FIRE_PULSE               = _UxGT("Avfyra Puls");
  LSTR MSG_SPINDLE_TOGGLE                 = _UxGT("Växla Spindel");
  LSTR MSG_SPINDLE_FORWARD                = _UxGT("Spindel Framåt");
  LSTR MSG_SPINDLE_REVERSE                = _UxGT("Spindel Bakåt");
  LSTR MSG_LASER_OFF                      = _UxGT("Laser Av");
  LSTR MSG_LASER_ON                       = _UxGT("Laser På");
  LSTR MSG_SPINDLE_OFF                    = _UxGT("Spindel Av");
  LSTR MSG_SPINDLE_ON                     = _UxGT("Spindel På");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Sätt på ström");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Stäng av ström");
  LSTR MSG_EXTRUDE                        = _UxGT("Extrudera");
  LSTR MSG_RETRACT                        = _UxGT("Dra tillbaka");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Flytta Axel");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Sond och Nivellera");
  LSTR MSG_BED_LEVELING                   = _UxGT("Bädd Nivellering");
  LSTR MSG_LEVEL_BED                      = _UxGT("Nivellera Bädd");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Bädd Justering");
  LSTR MSG_BED_TRAMMING_RAISE             = _UxGT("Höj Bädd tills nästa Sond Triggad");
  LSTR MSG_BED_TRAMMING_IN_RANGE          = _UxGT("Alla Hörn inom Tolerans. Nivellering Bädd");
  LSTR MSG_BED_TRAMMING_GOOD_POINTS       = _UxGT("Bra Punkter: ");
  LSTR MSG_BED_TRAMMING_LAST_Z            = _UxGT("Senaste Z: ");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Nästa Hörn");
  LSTR MSG_MESH_EDITOR                    = _UxGT("Nät Redigerare");
  LSTR MSG_EDIT_MESH                      = _UxGT("Redigera Nät");
  LSTR MSG_EDITING_STOPPED                = _UxGT("Nätredigering Stoppad");
  LSTR MSG_PROBING_POINT                  = _UxGT("Sonderingspunkt");
  LSTR MSG_MESH_X                         = _UxGT("Index X");
  LSTR MSG_MESH_Y                         = _UxGT("Index Y");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Z Värde");
  LSTR MSG_USER_MENU                      = _UxGT("Anpassade Kommandon");
  LSTR MSG_M48_TEST                       = _UxGT("M48 Sond Test");
  LSTR MSG_M48_POINT                      = _UxGT("M48 Punkt");
  LSTR MSG_M48_OUT_OF_BOUNDS              = _UxGT("Sond utan för gränser");
  LSTR MSG_M48_DEVIATION                  = _UxGT("Avvikelse");
  LSTR MSG_IDEX_MENU                      = _UxGT("IDEX Läge");
  LSTR MSG_OFFSETS_MENU                   = _UxGT("Verktygsoffset");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Auto-Parkera");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Duplicering");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("Speglad Kopia");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("Full Kontroll");
  LSTR MSG_IDEX_DUPE_GAP                  = _UxGT("Duplicera X-Avstånd");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("2:a Munstycke Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("2:a Munstycke @");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("Utför G29");
  LSTR MSG_UBL_TOOLS                      = _UxGT("UBL Verktyg");
  LSTR MSG_LCD_TILTING_MESH               = _UxGT("Lutningspunkt");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Manuellt skapa nät");
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("Placera Shim & Mät");
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Mät");
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Ta bort & Mät bädd");
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("Flyttar till nästa");
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Bädd Temp");
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Bädd Temp");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Hetände Temp");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Hetände Temp");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Redigera Anpassat Nät");
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("Finjustera Nät");
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Färdig Redigera Nät");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Bygg Anpassat Nät");
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Bygg Nät");
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("Bygg Nät ($)");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Bygg Kallt Nät");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Justera Nät Höjd");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Höjd Antal");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Validera Nät");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Validera Nät ($)");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Validera Anpassat Nät");
  LSTR MSG_G26_HEATING_BED                = _UxGT("G26 Värma Bädd");
  LSTR MSG_G26_HEATING_NOZZLE             = _UxGT("G26 Värma Munstycke");
  LSTR MSG_G26_MANUAL_PRIME               = _UxGT("Manuel grundning...");
  LSTR MSG_G26_FIXED_LENGTH               = _UxGT("Fastlängd Grundning");
  LSTR MSG_G26_PRIME_DONE                 = _UxGT("Färdig Grundning");
  LSTR MSG_G26_CANCELED                   = _UxGT("G26 Avbruten");
  LSTR MSG_G26_LEAVING                    = _UxGT("Nivellera G26");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Fortsätt Bädd Nät");
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Nät Nivellering");
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("3-Punkts Nivellering");
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Rutnät Nivellering");
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Nivellera Nät");
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("Sidopunkter");
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Kart Typ");
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("Utmatning Nät Map");
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("Utmatning för Värd");
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("Utmatning för CSV");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Utanför skrivare Backup");
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("Utmatning UBL Info");
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Ifyllnad Mängd");
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Manuell Ifyllnad");
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Smart Ifyllnad");
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Ifyllnad Nät");
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Ogiltigförklara Alla");
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Ogiltigförklara Närmast");
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Finjustera Alla");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Finjustera Närmast");
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Nät Lagra");
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Minnesöppning");
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Ladda Bädd Nät");
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Spara Bädd Nät");
  LSTR MSG_MESH_LOADED                    = _UxGT("Nät %i Ladda");
  LSTR MSG_MESH_SAVED                     = _UxGT("Nät %i Sparad");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("Ingen Lagring");
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("Fel: UBL Sparad");
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("Fel: UBL Återställd");
  LSTR MSG_UBL_Z_OFFSET                   = _UxGT("Z-Offset: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Z-Offset Stoppad");
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("Steg-för-Steg UBL");
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1. Bygg Kallt Nät");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2. Smart Ifyllnad");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3. Validera Nät");
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4. Finjustera Alla");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5. Validera Nät");
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6. Finjustera Alla");
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7. Spara Bädd Nät");

  LSTR MSG_LED_CONTROL                    = _UxGT("LED Kontroll");
  LSTR MSG_LIGHTS                         = _UxGT("Ljus");
  LSTR MSG_LIGHT_N                        = _UxGT("Ljus #{");
  LSTR MSG_LED_PRESETS                    = _UxGT("Ljus Förinställd");
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Röd");
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Orange");
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Gul");
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Grön");
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Blå");
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Indigo");
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Violet");
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("Vitt");
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Standard");
  LSTR MSG_LED_CHANNEL_N                  = _UxGT("Kanal {");
  LSTR MSG_NEO2_PRESETS                   = _UxGT("Ljus #2 Förinställd");
  LSTR MSG_NEO2_BRIGHTNESS                = _UxGT("Ljusstyrka");
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Anpassat Ljus");
  LSTR MSG_INTENSITY_R                    = _UxGT("Rör Intensitet");
  LSTR MSG_INTENSITY_G                    = _UxGT("Grön Intensitet");
  LSTR MSG_INTENSITY_B                    = _UxGT("Blå Intensitet");
  LSTR MSG_INTENSITY_W                    = _UxGT("Vit Intensitet");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Brightness");

  LSTR MSG_MOVING                         = _UxGT("Flyttar...");
  LSTR MSG_FREE_XY                        = _UxGT("Fri XY");
  LSTR MSG_MOVE_X                         = _UxGT("Flytta X");
  LSTR MSG_MOVE_Y                         = _UxGT("Flytta Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Flytta Z");
  LSTR MSG_MOVE_N                         = _UxGT("Flytta @");
  LSTR MSG_MOVE_E                         = _UxGT("Extruder");
  LSTR MSG_MOVE_EN                        = _UxGT("Extruder *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("Hetände för kall");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Flytta $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Flytta $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Flytta $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Hastighet");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Bädd Z");
  LSTR MSG_NOZZLE                         = _UxGT("Munstycke");
  LSTR MSG_NOZZLE_N                       = _UxGT("Munstycke ~");
  LSTR MSG_NOZZLE_PARKED                  = _UxGT("Munstycke Parkerad");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("Munstycke Standby");
  LSTR MSG_BED                            = _UxGT("Bädd");
  LSTR MSG_CHAMBER                        = _UxGT("Inkapsling");
  LSTR MSG_FAN_SPEED                      = _UxGT("Fläkt Hastighet");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Fläkt Hastighet ~");
  LSTR MSG_STORED_FAN_N                   = _UxGT("Lagrad Fläkt ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("Extra Fläkt Hastighet");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Extra Fläkt Hastighet ~");
  LSTR MSG_CONTROLLER_FAN                 = _UxGT("Kontroller Fläkt");
  LSTR MSG_CONTROLLER_FAN_IDLE_SPEED      = _UxGT("Overksam Hastighet");
  LSTR MSG_CONTROLLER_FAN_AUTO_ON         = _UxGT("Auto läga");
  LSTR MSG_CONTROLLER_FAN_SPEED           = _UxGT("Aktive Hastighet");
  LSTR MSG_CONTROLLER_FAN_DURATION        = _UxGT("Overksam Period");
  LSTR MSG_FLOW                           = _UxGT("Flöde");
  LSTR MSG_FLOW_N                         = _UxGT("Flöde ~");
  LSTR MSG_CONTROL                        = _UxGT("Kontroll");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Fakt");
  LSTR MSG_AUTOTEMP                       = _UxGT("Autotemp");
  LSTR MSG_LCD_ON                         = _UxGT("På");
  LSTR MSG_LCD_OFF                        = _UxGT("Av");
  LSTR MSG_PID_AUTOTUNE                   = _UxGT("PID Autojustera");
  LSTR MSG_PID_AUTOTUNE_E                 = _UxGT("PID Autojustera *");
  LSTR MSG_PID_AUTOTUNE_DONE              = _UxGT("PID tuning done");
  LSTR MSG_PID_BAD_HEATER_ID              = _UxGT("Autojustera misslyckad! Dålig extruder.");
  LSTR MSG_PID_TEMP_TOO_HIGH              = _UxGT("Autojustera misslyckad! Temperatur för hög.");
  LSTR MSG_PID_TIMEOUT                    = _UxGT("Autojustera misslyckad! Tidsgräns.");
  LSTR MSG_SELECT_E                       = _UxGT("Välj *");
  LSTR MSG_ACC                            = _UxGT("Accel");
  LSTR MSG_JERK                           = _UxGT("Ryck");
  LSTR MSG_VA_JERK                        = _UxGT("V") STR_A _UxGT("-Ryck");
  LSTR MSG_VB_JERK                        = _UxGT("V") STR_B _UxGT("-Ryck");
  LSTR MSG_VC_JERK                        = _UxGT("V") STR_C _UxGT("-Ryck");
  LSTR MSG_VN_JERK                        = _UxGT("V@-Ryck");
  LSTR MSG_VE_JERK                        = _UxGT("Ve-Ryck");
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Knutpunkt Avv");
  LSTR MSG_MAX_SPEED                      = _UxGT("Hastighet");
  LSTR MSG_VMAX_A                         = _UxGT("Vmax ") STR_A;
  LSTR MSG_VMAX_B                         = _UxGT("Vmax ") STR_B;
  LSTR MSG_VMAX_C                         = _UxGT("Vmax ") STR_C;
  LSTR MSG_VMAX_N                         = _UxGT("Vmax @");
  LSTR MSG_VMAX_E                         = _UxGT("Vmax E");
  LSTR MSG_VMAX_EN                        = _UxGT("Vmax *");
  LSTR MSG_VMIN                           = _UxGT("Vmin");
  LSTR MSG_VTRAV_MIN                      = _UxGT("VTrav Min");
  LSTR MSG_ACCELERATION                   = _UxGT("Acceleration");
  LSTR MSG_AMAX_A                         = _UxGT("Amax ") STR_A;
  LSTR MSG_AMAX_B                         = _UxGT("Amax ") STR_B;
  LSTR MSG_AMAX_C                         = _UxGT("Amax ") STR_C;
  LSTR MSG_AMAX_N                         = _UxGT("Amax @");
  LSTR MSG_AMAX_E                         = _UxGT("Amax E");
  LSTR MSG_AMAX_EN                        = _UxGT("Amax *");
  LSTR MSG_A_RETRACT                      = _UxGT("A-Dra tillbaka");
  LSTR MSG_A_TRAVEL                       = _UxGT("A-Färdas");
  LSTR MSG_XY_FREQUENCY_LIMIT             = _UxGT("Frekvens max");
  LSTR MSG_XY_FREQUENCY_FEEDRATE          = _UxGT("Flöde min");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Steg/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" Steg/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" Steg/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" Steg/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ Steg/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E Steg/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* Steg/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperatur");
  LSTR MSG_MOTION                         = _UxGT("Rörelse");
  LSTR MSG_FILAMENT                       = _UxGT("Tråd");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E i mm³");
  LSTR MSG_VOLUMETRIC_LIMIT               = _UxGT("E Gräns i mm³");
  LSTR MSG_VOLUMETRIC_LIMIT_E             = _UxGT("E Gräns *");
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Tråd Dia.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Tråd Dia. *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Lossa mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Ladda mm");
  LSTR MSG_ADVANCE_K                      = _UxGT("Advancera K");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("Advancera K *");
  LSTR MSG_CONTRAST                       = _UxGT("LCD Kontrast");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Spara Inställningar");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Ladda Inställningar");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Återställ Standard");
  LSTR MSG_INIT_EEPROM                    = _UxGT("Initiera EEPROM");
  LSTR MSG_ERR_EEPROM_CRC                 = _UxGT("EEPROM CRC Fel");
  LSTR MSG_ERR_EEPROM_SIZE                = _UxGT("EEPROM Storlek Fel");
  LSTR MSG_ERR_EEPROM_VERSION             = _UxGT("EEPROM Version Fel");
  LSTR MSG_SETTINGS_STORED                = _UxGT("Inställningar Lagrad");
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("Media Uppdatera");
  LSTR MSG_RESET_PRINTER                  = _UxGT("Återställ Skrivare");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Uppdatera");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Info Skärm");
  LSTR MSG_PREPARE                        = _UxGT("Förbered");
  LSTR MSG_TUNE                           = _UxGT("Justera");
  LSTR MSG_POWER_MONITOR                  = _UxGT("Ström övervakning");
  LSTR MSG_CURRENT                        = _UxGT("Ström");
  LSTR MSG_VOLTAGE                        = _UxGT("Spänning");
  LSTR MSG_POWER                          = _UxGT("Ström");
  LSTR MSG_START_PRINT                    = _UxGT("Start Utskrift");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("Nästa");
  LSTR MSG_BUTTON_INIT                    = _UxGT("Initiera");
  LSTR MSG_BUTTON_STOP                    = _UxGT("Stoppa");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("Skriv");
  LSTR MSG_BUTTON_RESET                   = _UxGT("Återställa");
  LSTR MSG_BUTTON_IGNORE                  = _UxGT("Ignorera");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("Avbryt");
  LSTR MSG_BUTTON_DONE                    = _UxGT("Färdig");
  LSTR MSG_BUTTON_BACK                    = _UxGT("Bakåt");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("Fortsätt");
  LSTR MSG_BUTTON_SKIP                    = _UxGT("Hoppa över");
  LSTR MSG_PAUSING                        = _UxGT("Paus..");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Pausera Utskrift");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Återuppta Utskrift");
  LSTR MSG_HOST_START_PRINT               = _UxGT("Värd Start");
  LSTR MSG_STOP_PRINT                     = _UxGT("Stoppa Utskrift");
  LSTR MSG_END_LOOPS                      = _UxGT("Slut Upprepningsloop");
  LSTR MSG_PRINTING_OBJECT                = _UxGT("Skriver Objekt");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("Avbryt Objekt");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("Avbryt Objekt {");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Ström Avbrott");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Skriv fråm Media");
  LSTR MSG_NO_MEDIA                       = _UxGT("Inget Media");
  LSTR MSG_DWELL                          = _UxGT("Sov...");
  LSTR MSG_USERWAIT                       = _UxGT("Klick för att återuppta...");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Utskrift Pausad");
  LSTR MSG_PRINTING                       = _UxGT("Skriver...");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Utskrift Avbruten");
  LSTR MSG_PRINT_DONE                     = _UxGT("Utskrift Färdig");
  LSTR MSG_NO_MOVE                        = _UxGT("Ingen Flytt.");
  LSTR MSG_KILLED                         = _UxGT("DÖDAD. ");
  LSTR MSG_STOPPED                        = _UxGT("STOPPAD. ");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Dra tillbaka mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Byt Dra.mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Dra tillbaka V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Hoppa mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Åter dra tillbaka. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Byt åter dra t. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Återdrat. V");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("Byt åter dra. V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Auto-Dra-tillbka");
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("Byt Längd");
  LSTR MSG_FILAMENT_SWAP_EXTRA            = _UxGT("Byt Extra");
  LSTR MSG_FILAMENT_PURGE_LENGTH          = _UxGT("Rensa Längd");
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Byt verktyg");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Z Höj");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Grund Hastighet");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Återgå Hastighet");
  LSTR MSG_FILAMENT_PARK_ENABLED          = _UxGT("Parkera Huvud");
  LSTR MSG_SINGLENOZZLE_UNRETRACT_SPEED   = _UxGT("Återgår Hastighet");
  LSTR MSG_SINGLENOZZLE_FAN_SPEED         = _UxGT("Fläkt Hastighet");
  LSTR MSG_SINGLENOZZLE_FAN_TIME          = _UxGT("Fläkt Tid");
  LSTR MSG_TOOL_MIGRATION_ON              = _UxGT("Auto PÅ");
  LSTR MSG_TOOL_MIGRATION_OFF             = _UxGT("Auto AV");
  LSTR MSG_TOOL_MIGRATION                 = _UxGT("Verktyg Migration");
  LSTR MSG_TOOL_MIGRATION_AUTO            = _UxGT("Auto-migration");
  LSTR MSG_TOOL_MIGRATION_END             = _UxGT("Senast Extruder");
  LSTR MSG_TOOL_MIGRATION_SWAP            = _UxGT("Migrera till *");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Byt Tråd");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Byt Tråd *");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Ladda Tråd");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Ladda *");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Lossa Tråd");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Lossa *");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Lossa All");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Bifoga SD-kort");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Bifoga USB-minne");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Bifoga SD-kort");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Byt Media");
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Släpp Media");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Z Sond Utanför Bädd");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Skev Faktor");
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("Själv-Test");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Återställ");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("Stuva undan");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("Fällut");
  LSTR MSG_BLTOUCH_SW_MODE                = _UxGT("SW-Läge");
  LSTR MSG_BLTOUCH_5V_MODE                = _UxGT("5V-Läge");
  LSTR MSG_BLTOUCH_OD_MODE                = _UxGT("OD-Läge");
  LSTR MSG_BLTOUCH_MODE_STORE             = _UxGT("Läge-Lägring");
  LSTR MSG_BLTOUCH_MODE_STORE_5V          = _UxGT("Sätt BLTouch to 5V");
  LSTR MSG_BLTOUCH_MODE_STORE_OD          = _UxGT("Sätt BLTouch to OD");
  LSTR MSG_BLTOUCH_MODE_ECHO              = _UxGT("Reportera Dränering");
  LSTR MSG_BLTOUCH_MODE_CHANGE            = _UxGT("FARA: Dålig inställningar kan orsaka skada! Fortsätt ändå?");
  LSTR MSG_TOUCHMI_PROBE                  = _UxGT("TouchMI");
  LSTR MSG_TOUCHMI_INIT                   = _UxGT("Initiera TouchMI");
  LSTR MSG_TOUCHMI_ZTEST                  = _UxGT("Z Offset Test");
  LSTR MSG_TOUCHMI_SAVE                   = _UxGT("Spara");
  LSTR MSG_MANUAL_DEPLOY_TOUCHMI          = _UxGT("Fällut TouchMI");
  LSTR MSG_MANUAL_DEPLOY                  = _UxGT("Fällut Z-Sond");
  LSTR MSG_MANUAL_STOW                    = _UxGT("Stuva undan Z-Sond");
  LSTR MSG_HOME_FIRST                     = _UxGT("Hem %s Först");
  LSTR MSG_ZPROBE_OFFSETS                 = _UxGT("Sond Offsets");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Sond X Offset");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Sond Y Offset");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Sond Z Offset");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Sond @ Offset");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Steg Z justering");
  LSTR MSG_MOVE_NOZZLE_TO_BED             = _UxGT("Flytta Munstycke till Bädd");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Småsteg X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Småsteg Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Småsteg Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Småsteg @");
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("Total");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Slutstopp Avbrott");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Värma Misslyckad");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Fel: REDUNDANT TEMP");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("TERMISK ÖVERDRIFT");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Fel: MAXTEMP");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Fel: MINTEMP");
  LSTR MSG_HALTED                         = _UxGT("Utskrift stoppad");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Snälla Återställ");
  LSTR MSG_SHORT_HOUR                     = _UxGT("t"); // One character only
  LSTR MSG_HEATING                        = _UxGT("Värmer...");
  LSTR MSG_COOLING                        = _UxGT("Kyler...");
  LSTR MSG_BED_HEATING                    = _UxGT("Bädd Värmer...");
  LSTR MSG_BED_COOLING                    = _UxGT("Bädd Kyler...");
  LSTR MSG_PROBE_HEATING                  = _UxGT("Sond Värmer...");
  LSTR MSG_PROBE_COOLING                  = _UxGT("Sond Kyler...");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("Kammare Värmer...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("Kammare Kyler...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Delta Kalibrering");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Kalibrera X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Kalibrera Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Kalibrera Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Kalibrera Center");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Delta Inställningar");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Auto Kalibrering");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Diag Rod");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Höjd");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Radius");
  LSTR MSG_INFO_MENU                      = _UxGT("Om Skrivaren");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Skrivare Info");
  LSTR MSG_3POINT_LEVELING                = _UxGT("3-Punkt Nivellering");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Linjär Nivellering");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("Bilinjär Nivellering");
  LSTR MSG_UBL_LEVELING                   = _UxGT("Enhetlig Bädd Nivellering (UBL)");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Nät Nivellering");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Skrivar Stats");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Kort Info");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termistor");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Extruderare");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Baud");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protokoll");
  LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Överdrift Övervakning: AV");
  LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Överdrift Övervakning: PÅ");
  LSTR MSG_HOTEND_IDLE_TIMEOUT            = _UxGT("Hetände Overksam Tidsgräns");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Lådljus");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Ljus ljusstyrka");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("INKORREKT SKRIVARE");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Utskrift");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Färdig");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Total");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Längsta");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Extruderad");

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Min Temp");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Max Temp");
  LSTR MSG_INFO_PSU                       = _UxGT("PSU");
  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Driv Styrka");
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ Driver %");
  LSTR MSG_ERROR_TMC                      = _UxGT("TMC KOPPLNINGSFEL");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("DAC EEPROM Skriv");
  LSTR MSG_FILAMENT_CHANGE_HEADER         = _UxGT("TRÅDBYTE");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("UTSKRIFTSPAUSERAD");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("LADDA TRÅD");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("LOSSA TRÅD");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("ÅTERGÅ VAÖ:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("Rensa mer");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Fortsätt");
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Munstycke: ");
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Utskjut Sensor");
  LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("Utskjut Dist mm");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Hemning Misslyckad");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Sondering Misslyckad");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("VÄLJ TRÅD");
  LSTR MSG_MMU2_MENU                      = _UxGT("MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("Uppdatera MMU Firmware!");
  LSTR MSG_MMU2_NOT_RESPONDING            = _UxGT("MMU Behöver uppmärksamhet.");
  LSTR MSG_MMU2_RESUME                    = _UxGT("MMU Återuppta");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("MMU Återupptas...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("MMU Ladda");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("MMU Ladda Alla");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("MMU Ladda till Munstycke");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("MMU Mata ut");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("MMU Mata ut ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("MMU Lossa");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("Ladda Tråd %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Mata ut Tråd ...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("Lossa Tråd...");
  LSTR MSG_MMU2_ALL                       = _UxGT("Alla");
  LSTR MSG_MMU2_FILAMENT_N                = _UxGT("Tråd ~");
  LSTR MSG_MMU2_RESET                     = _UxGT("Återställ MMU");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("MMU Återställer...");
  LSTR MSG_MMU2_EJECT_RECOVER             = _UxGT("Ta bort, Klicka");

  LSTR MSG_MIX                            = _UxGT("Mixa");
  LSTR MSG_MIX_COMPONENT_N                = _UxGT("Komponent {");
  LSTR MSG_MIXER                          = _UxGT("Mixer");
  LSTR MSG_GRADIENT                       = _UxGT("Gradient");
  LSTR MSG_FULL_GRADIENT                  = _UxGT("Full Gradient");
  LSTR MSG_TOGGLE_MIX                     = _UxGT("Växla Mix");
  LSTR MSG_CYCLE_MIX                      = _UxGT("Totera Mix");
  LSTR MSG_GRADIENT_MIX                   = _UxGT("Gradient Mix");
  LSTR MSG_REVERSE_GRADIENT               = _UxGT("Omvänd Gradient");
  LSTR MSG_ACTIVE_VTOOL                   = _UxGT("Aktive V-verktyg");
  LSTR MSG_START_VTOOL                    = _UxGT("Start V-verktyg");
  LSTR MSG_END_VTOOL                      = _UxGT(" Slut V-verktyg");
  LSTR MSG_GRADIENT_ALIAS                 = _UxGT("Alias V-verktyg");
  LSTR MSG_RESET_VTOOLS                   = _UxGT("Återställ V-verktyg");
  LSTR MSG_COMMIT_VTOOL                   = _UxGT("Kommitta V-verktyg Mix");
  LSTR MSG_VTOOLS_RESET                   = _UxGT("V-verktyg blev Återställda");
  LSTR MSG_START_Z                        = _UxGT("Start Z:");
  LSTR MSG_END_Z                          = _UxGT(" Slut Z:");

  LSTR MSG_GAMES                          = _UxGT("Spel");
  LSTR MSG_BRICKOUT                       = _UxGT("Brickout");
  LSTR MSG_INVADERS                       = _UxGT("Invaders");
  LSTR MSG_SNAKE                          = _UxGT("Sn4k3");
  LSTR MSG_MAZE                           = _UxGT("Labyrint");

  LSTR MSG_BAD_PAGE                       = _UxGT("Dålig sida index");
  LSTR MSG_BAD_PAGE_SPEED                 = _UxGT("Dålig sida hastighet");

  LSTR MSG_EDIT_PASSWORD                  = _UxGT("Redigera Lösenord");
  LSTR MSG_LOGIN_REQUIRED                 = _UxGT("Login Krävs");
  LSTR MSG_PASSWORD_SETTINGS              = _UxGT("Lösenordsinställningar");
  LSTR MSG_ENTER_DIGIT                    = _UxGT("Ange Siffra");
  LSTR MSG_CHANGE_PASSWORD                = _UxGT("Sätt/Redigera Lösenord");
  LSTR MSG_REMOVE_PASSWORD                = _UxGT("Ta bort Lösenord");
  LSTR MSG_PASSWORD_SET                   = _UxGT("Lösenord är ");
  LSTR MSG_START_OVER                     = _UxGT("Börja om");
  LSTR MSG_REMINDER_SAVE_SETTINGS         = _UxGT("Kom ihåg att Spara!");
  LSTR MSG_PASSWORD_REMOVED               = _UxGT("Lösenord Bort taget");

  //
  // Filament Change screens show up to 2 lines on a 3-line display
  //
  LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_1_LINE("Klick för att fortsätta"));
  LSTR MSG_PAUSE_PRINT_PARKING          = _UxGT(MSG_1_LINE("Parkera..."));
  LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_1_LINE("Vänta..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_1_LINE("Sätt in och klicka"));
  LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_1_LINE("Klicka för att värma"));
  LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_1_LINE("Värmer..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_1_LINE("Lossar..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_1_LINE("Laddar..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_1_LINE("Rensar..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_1_LINE("Klicka för att slutföra"));
  LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_1_LINE("Återgår..."));

  LSTR MSG_TMC_DRIVERS                    = _UxGT("TMC Drivers");
  LSTR MSG_TMC_CURRENT                    = _UxGT("Driver Ström");
  LSTR MSG_TMC_HYBRID_THRS                = _UxGT("Hybrid Tröskelvärde");
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("Sensorlös Hemning");
  LSTR MSG_TMC_STEPPING_MODE              = _UxGT("Stegningsläge");
  LSTR MSG_TMC_STEALTH_ENABLED            = _UxGT("Smyghack Aktiverad");
  LSTR MSG_SERVICE_RESET                  = _UxGT("Återställ");
  LSTR MSG_SERVICE_IN                     = _UxGT(" in:");
  LSTR MSG_BACKLASH                       = _UxGT("Backlash");
  LSTR MSG_BACKLASH_CORRECTION            = _UxGT("Korrigering");
  LSTR MSG_BACKLASH_SMOOTHING             = _UxGT("Glättning");

  LSTR MSG_LEVEL_X_AXIS                   = _UxGT("Nivå X Axel");
  LSTR MSG_AUTO_CALIBRATE                 = _UxGT("Auto Kalibrera");
  LSTR MSG_FTDI_HEATER_TIMEOUT            = _UxGT("Overksam tidsgräns, temperatur minskning. Tryck ok för att återvärma och igen för att fortsätta.");
  LSTR MSG_HEATER_TIMEOUT                 = _UxGT("Värmare Tidsgräns");
  LSTR MSG_REHEAT                         = _UxGT("Återvärm");
  LSTR MSG_REHEATING                      = _UxGT("Återvärmning...");

  LSTR MSG_PROBE_WIZARD                   = _UxGT("Z Sond Wizard");
  LSTR MSG_PROBE_WIZARD_PROBING           = _UxGT("Sondering Z Referens");
  LSTR MSG_PROBE_WIZARD_MOVING            = _UxGT("Flyttar till Sonderings Pos");

  LSTR MSG_SOUND                          = _UxGT("Ljud");

  LSTR MSG_TOP_LEFT                       = _UxGT("Uppe Vänster");
  LSTR MSG_BOTTOM_LEFT                    = _UxGT("Nere Vänster");
  LSTR MSG_TOP_RIGHT                      = _UxGT("Uppe Höger");
  LSTR MSG_BOTTOM_RIGHT                   = _UxGT("Nere Höger");
  LSTR MSG_CALIBRATION_COMPLETED          = _UxGT("Kalibrering Färdig");
  LSTR MSG_CALIBRATION_FAILED             = _UxGT("Kalibrering Misslyckad");
}

namespace LanguageWide_sv {
  using namespace LanguageNarrow_sv;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Utskriftsantal");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Färdiga");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Total Utskriftstid");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Längsta Jobbtid");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Extruderade Totalt");
  #endif
}

namespace LanguageTall_sv {
  using namespace LanguageWide_sv;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Tryck på knappen", "för att fortsätta utskrift"));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Vänta på", "trådbyte", "att börja"));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Sätt in tråd", "och tryck på knappen", "för att fortsätta"));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Tryck på knappen", "för att värma munstycke"));
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Munstycke värms", "Var snäll och vänta..."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_2_LINE("Väntar på", "trådlossning"));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_2_LINE("Väntar på", "trådladdning"));
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_2_LINE("Väntar på", "tråd utrensning"));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_2_LINE("Klicka för att slutföra", "tråd utrensning"));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_2_LINE("Väntar på utskrift", "att återstarta..."));
  #endif
}

namespace Language_sv {
  using namespace LanguageTall_sv;
}
