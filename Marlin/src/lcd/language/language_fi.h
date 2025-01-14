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
 * Finnish
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#define DISPLAY_CHARSET_ISO10646_1

namespace LanguageNarrow_fi {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Finnish");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" valmis.");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Kortti asetettu");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Kortti poistettu");
  LSTR MSG_MAIN_MENU                      = _UxGT("Palaa");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Automaatti");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Vapauta moottorit");
  LSTR MSG_AUTO_HOME                      = _UxGT("Aja referenssiin");

  LSTR MSG_PREHEAT_1                      = _UxGT("Esilämmitä ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Esilämmitä ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Esilä. ") PREHEAT_1_LABEL _UxGT("Suutin");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Esilä. ") PREHEAT_1_LABEL _UxGT("Suutin ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Esilä. ") PREHEAT_1_LABEL _UxGT(" Kaikki");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Esilä. ") PREHEAT_1_LABEL _UxGT(" Alusta");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Esilämm. ") PREHEAT_1_LABEL _UxGT(" konf");

  LSTR MSG_PREHEAT_M                      = _UxGT("Esilämmitä $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Esilämmitä $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Esilä. $Suutin");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Esilä. $Suutin ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Esilä. $ Kaikki");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Esilä. $ Alusta");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Esilämm. $ konf");

  LSTR MSG_COOLDOWN                       = _UxGT("Jäähdytä");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Virta päälle");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Virta pois");
  LSTR MSG_EXTRUDE                        = _UxGT("Pursota");
  LSTR MSG_RETRACT                        = _UxGT("Vedä takaisin");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Liikuta akseleita");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Tutki ja tasoita");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Säädä Z askeltamalla");
  LSTR MSG_MOVE_X                         = _UxGT("Liikuta X");
  LSTR MSG_MOVE_Y                         = _UxGT("Liikuta Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Liikuta Z");
  LSTR MSG_MOVE_N                         = _UxGT("Liikuta @");
  LSTR MSG_MOVE_E                         = _UxGT("Extruder");
  LSTR MSG_MOVE_EN                        = _UxGT("Extruder *");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Liikuta $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Liikuta $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Liikuta $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Nopeus");
  LSTR MSG_NOZZLE                         = _UxGT("Suutin");
  LSTR MSG_NOZZLE_N                       = _UxGT("Suutin ~");
  LSTR MSG_BED                            = _UxGT("Alusta");
  LSTR MSG_FAN_SPEED                      = _UxGT("Tuul. nopeus");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Tuul. nopeus ~");
  LSTR MSG_FLOW                           = _UxGT("Virtaus");
  LSTR MSG_FLOW_N                         = _UxGT("Virtaus ~");
  LSTR MSG_CONTROL                        = _UxGT("Kontrolli");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Kerr");
  LSTR MSG_AUTOTEMP                       = _UxGT("Autotemp");
  LSTR MSG_ACC                            = _UxGT("Kiihtyv");
  LSTR MSG_VTRAV_MIN                      = _UxGT("VLiike min");
  LSTR MSG_A_RETRACT                      = _UxGT("A-peruuta");
  LSTR MSG_TEMPERATURE                    = _UxGT("Lämpötila");
  LSTR MSG_MOTION                         = _UxGT("Liike");
  LSTR MSG_FILAMENT                       = _UxGT("Filament");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E in mm") SUPERSCRIPT_THREE;
  LSTR MSG_CONTRAST                       = _UxGT("LCD kontrasti");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Tallenna muistiin");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Lataa muistista");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Palauta oletus");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Päivitä");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Seuraa");
  LSTR MSG_PREPARE                        = _UxGT("Valmistele");
  LSTR MSG_TUNE                           = _UxGT("Säädä");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Keskeytä tulostus");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Jatka tulostusta");
  LSTR MSG_STOP_PRINT                     = _UxGT("Pysäytä tulostus");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Korttivalikko");
  LSTR MSG_NO_MEDIA                       = _UxGT("Ei korttia");
  LSTR MSG_DWELL                          = _UxGT("Nukkumassa...");
  LSTR MSG_USERWAIT                       = _UxGT("Odotet. valintaa");
  LSTR MSG_NO_MOVE                        = _UxGT("Ei liiketta.");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Vedä mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Va. Vedä mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Vedä V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Z mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Va. Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Unretract V");
  LSTR MSG_AUTORETRACT                    = _UxGT("AutoVeto.");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Delta Kalibrointi");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Kalibroi X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Kalibroi Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Kalibroi Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Kalibroi Center");

  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("Väärä tulostin");
}

namespace LanguageWide_fi {
  using namespace LanguageNarrow_fi;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
  #endif
}

namespace LanguageTall_fi {
  using namespace LanguageWide_fi;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
  #endif
}

namespace Language_fi {
  using namespace LanguageTall_fi;
}
