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
 * Dutch
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#define DISPLAY_CHARSET_ISO10646_1
#define NOT_EXTENDED_ISO10646_1_5X7

namespace LanguageNarrow_nl {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 1;
  LSTR LANGUAGE                           = _UxGT("Dutch");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" gereed.");
  LSTR MSG_BACK                           = _UxGT("Terug");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Kaart ingestoken");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Kaart verwijderd");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Endstops"); // Max length 8 characters
  LSTR MSG_MAIN_MENU                      = _UxGT("Hoofdmenu");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Autostart");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Motoren uit");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Debug Menu"); // accepted English terms
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Vooruitgang Test");

  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Klik voor begin");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Volgende Plaats");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Bed level kompl.");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Zet home offsets");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("H offset toegep.");

  LSTR MSG_PREHEAT_1                      = PREHEAT_1_LABEL _UxGT(" voorverwarmen");
  LSTR MSG_PREHEAT_1_H                    = PREHEAT_1_LABEL _UxGT(" voorverw. ~");
  LSTR MSG_PREHEAT_1_END                  = PREHEAT_1_LABEL _UxGT(" voorverw. Einde");
  LSTR MSG_PREHEAT_1_END_E                = PREHEAT_1_LABEL _UxGT(" voorverw. Einde ~");
  LSTR MSG_PREHEAT_1_ALL                  = PREHEAT_1_LABEL _UxGT(" voorverw. aan");
  LSTR MSG_PREHEAT_1_BEDONLY              = PREHEAT_1_LABEL _UxGT(" voorverw. Bed");
  LSTR MSG_PREHEAT_1_SETTINGS             = PREHEAT_1_LABEL _UxGT(" verw. conf");

  LSTR MSG_PREHEAT_M                      = _UxGT("$ voorverwarmen");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("$ voorverw. ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("$ voorverw. Einde");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("$ voorverw. Einde ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("$ voorverw. aan");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("$ voorverw. Bed");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("$ verw. conf");

  LSTR MSG_COOLDOWN                       = _UxGT("Afkoelen");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Stroom aan");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Stroom uit");
  LSTR MSG_EXTRUDE                        = _UxGT("Extrude");
  LSTR MSG_RETRACT                        = _UxGT("Retract");
  LSTR MSG_MOVE_AXIS                      = _UxGT("As verplaatsen");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Z probe & Level");
  LSTR MSG_BED_LEVELING                   = _UxGT("Bed Leveling");
  LSTR MSG_LEVEL_BED                      = _UxGT("Level bed");

  LSTR MSG_MOVING                         = _UxGT("Verplaatsen...");
  LSTR MSG_FREE_XY                        = _UxGT("Vrij XY");
  LSTR MSG_MOVE_X                         = _UxGT("Verplaats X");
  LSTR MSG_MOVE_Y                         = _UxGT("Verplaats Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Verplaats Z");
  LSTR MSG_MOVE_N                         = _UxGT("Verplaats @");
  LSTR MSG_MOVE_E                         = _UxGT("Extruder");
  LSTR MSG_MOVE_EN                        = _UxGT("Extruder *");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Verplaats $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Verplaats $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Verplaats $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Snelheid");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Bed Z");
  LSTR MSG_NOZZLE                         = _UxGT("Nozzle");
  LSTR MSG_NOZZLE_N                       = _UxGT("Nozzle ~");
  LSTR MSG_BED                            = _UxGT("Bed");
  LSTR MSG_FAN_SPEED                      = _UxGT("Fan snelheid");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Fan snelheid ~");
  LSTR MSG_FLOW                           = _UxGT("Flow");
  LSTR MSG_FLOW_N                         = _UxGT("Flow ~");
  LSTR MSG_CONTROL                        = _UxGT("Control");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Fact");
  LSTR MSG_AUTOTEMP                       = _UxGT("Autotemp");
  LSTR MSG_LCD_ON                         = _UxGT("Aan");
  LSTR MSG_LCD_OFF                        = _UxGT("Uit");
  LSTR MSG_SELECT_E                       = _UxGT("Selecteer *");
  LSTR MSG_ACC                            = _UxGT("Versn");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperatuur");
  LSTR MSG_MOTION                         = _UxGT("Beweging");
  LSTR MSG_FILAMENT                       = _UxGT("Filament");
  LSTR MSG_ADVANCE_K                      = _UxGT("Advance K"); // accepted english dutch
  LSTR MSG_ADVANCE_K_E                    = _UxGT("Advance K *"); // accepted english dutch
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E in mm") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Fil. Dia.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Fil. Dia. *");
  LSTR MSG_CONTRAST                       = _UxGT("LCD contrast");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Geheugen opslaan");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Geheugen laden");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Noodstop reset");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Ververs");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Info scherm");
  LSTR MSG_PREPARE                        = _UxGT("Voorbereiden");
  LSTR MSG_TUNE                           = _UxGT("Afstellen");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Print pauzeren");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Print hervatten");
  LSTR MSG_STOP_PRINT                     = _UxGT("Print stoppen");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Print van SD");
  LSTR MSG_NO_MEDIA                       = _UxGT("Geen SD kaart");
  LSTR MSG_DWELL                          = _UxGT("Slapen...");
  LSTR MSG_USERWAIT                       = _UxGT("Wachten...");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Print afgebroken");
  LSTR MSG_NO_MOVE                        = _UxGT("Geen beweging.");
  LSTR MSG_KILLED                         = _UxGT("Afgebroken. ");
  LSTR MSG_STOPPED                        = _UxGT("Gestopt. ");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Retract mm"); // accepted English term in Dutch
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Ruil Retract mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Retract  F");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Hop mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Ruil Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Unretr. FR");
  LSTR MSG_AUTORETRACT                    = _UxGT("Auto-Retract");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Verv. Filament");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Verv. Filament *");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Init. SD kaart");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Init. USB stick");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Init. SD kaart");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Verv. SD Kaart");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Z probe uit. bed");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("BLTouch Zelf-Test");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Reset BLTouch");
  LSTR MSG_HOME_FIRST                     = _UxGT("Home %s Eerst");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("X Offset"); // accepted English term in Dutch
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Y Offset");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Z Offset");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("@ Offset");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Babystep Probe Z");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Babystap X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Babystap Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Babystap Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Babystap @");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Endstop afbr.");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Voorverw. fout");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Redun. temp fout");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("Therm. wegloop");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Err: Max. temp");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Err: Min. temp");
  LSTR MSG_HALTED                         = _UxGT("PRINTER GESTOPT");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Reset A.U.B.");
  LSTR MSG_HEATING                        = _UxGT("Voorwarmen...");
  LSTR MSG_BED_HEATING                    = _UxGT("Bed voorverw...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Delta Calibratie");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Kalibreer X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Kalibreer Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Kalibreer Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Kalibreer Midden");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Delta conf");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Auto Calibratie");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Case licht");

  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("Onjuiste printer");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Aantal");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Voltooid");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Printtijd ");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Langste");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Extrud.");

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Min Temp");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Max Temp");
  LSTR MSG_INFO_PSU                       = _UxGT("PSU"); // accepted English term in Dutch

  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Motorstroom");

  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("DAC Opslaan");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Hervat print");
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT(" Nozzle: "); // accepted English term
  //
  // Filament Change screens show up to 2 lines on a 3-line display
  //
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_2_LINE("Wacht voor", "start..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_2_LINE("Wacht voor", "uitladen..."));
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_2_LINE("Klik knop om...", "verw. nozzle.")); //nozzle accepted English term
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Verwarmen..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_2_LINE("Laad filament", "en druk knop"));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_2_LINE("Wacht voor", "inladen..."));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_2_LINE("Wacht voor", "printing..."));
}

namespace LanguageWide_nl {
  using namespace LanguageNarrow_nl;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Printed Aantal");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Totaal Voltooid");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Totale Printtijd");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Langste Printtijd");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Totaal Extrudeert");
  #endif
}

namespace LanguageTall_nl {
  using namespace LanguageWide_nl;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Wacht voor start", "filament te", "verwisselen"));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_3_LINE("Wacht voor", "filament uit", "te laden"));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Klik knop om...", "verw. nozzle.")); //nozzle accepted English term
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Nozzle verw.", "Wacht a.u.b."));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Laad filament", "en druk knop", "om verder..."));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_3_LINE("Wacht voor", "filament te", "laden"));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_3_LINE("Wacht voor print", "om verder", "te gaan"));
  #endif
}

namespace Language_nl {
  using namespace LanguageTall_nl;
}
