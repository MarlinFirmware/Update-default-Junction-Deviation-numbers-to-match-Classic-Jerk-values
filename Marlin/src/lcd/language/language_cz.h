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
 * Czech
 * UTF-8 for Graphical Display
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 *
 * Translated by Petr Zahradnik, Computer Laboratory
 * Blog and video blog Zahradnik se bavi
 * https://www.zahradniksebavi.cz
 */

#define DISPLAY_CHARSET_ISO10646_CZ

namespace LanguageNarrow_cz {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Czech");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" připraven.");
  LSTR MSG_YES                            = _UxGT("ANO");
  LSTR MSG_NO                             = _UxGT("NE");
  LSTR MSG_BACK                           = _UxGT("Zpět");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Rušení...");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Médium vloženo");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Médium vyjmuto");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("Čekání na médium");
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("Chyba čtení média");
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("USB odstraněno");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("Chyba USB");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Endstopy"); // max 8 znaku
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Soft Endstopy");
  LSTR MSG_MAIN_MENU                      = _UxGT("Hlavní nabídka");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Další nastavení");
  LSTR MSG_CONFIGURATION                  = _UxGT("Konfigurace");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Autostart");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Uvolnit motory");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Nabídka ladění");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Test uk. průběhu");
  LSTR MSG_AUTO_HOME                      = _UxGT("Domovská pozice");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Domů osa X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Domů osa Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Domů osa Z");
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Auto srovnání Z");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Měření podložky");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Kliknutím spusťte");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Další bod");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Měření hotovo!");
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Výška srovnávání");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Nastavit ofsety");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Ofsety nastaveny");

  LSTR MSG_PREHEAT_1                      = _UxGT("Zahřát ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Zahřát ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Zahřát ") PREHEAT_1_LABEL _UxGT(" end");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Zahřát ") PREHEAT_1_LABEL _UxGT(" end ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Zahřát ") PREHEAT_1_LABEL _UxGT(" vše");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Zahřát ") PREHEAT_1_LABEL _UxGT(" podlož");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Zahřát ") PREHEAT_1_LABEL _UxGT(" nast");

  LSTR MSG_PREHEAT_M                      = _UxGT("Zahřát $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Zahřát $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Zahřát $ end");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Zahřát $ end ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Zahřát $ vše");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Zahřát $ podlož");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Zahřát $ nast");

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Zahřát vlastní");
  LSTR MSG_COOLDOWN                       = _UxGT("Zchladit");
  LSTR MSG_LASER_MENU                     = _UxGT("Ovládání laseru");
  LSTR MSG_LASER_POWER                    = _UxGT("Výkon laseru");
  LSTR MSG_SPINDLE_MENU                   = _UxGT("Vřeteno ovládání");
  LSTR MSG_SPINDLE_POWER                  = _UxGT("Vřeteno výkon");
  LSTR MSG_SPINDLE_REVERSE                = _UxGT("Vřeteno opačně");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Zapnout napájení");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Vypnout napájení");
  LSTR MSG_EXTRUDE                        = _UxGT("Vytlačit (extr.)");
  LSTR MSG_RETRACT                        = _UxGT("Zatlačit (retr.)");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Posunout osy");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Měření a vyrovnání");
  LSTR MSG_BED_LEVELING                   = _UxGT("Vyrovnat podložku");
  LSTR MSG_LEVEL_BED                      = _UxGT("Vyrovnat podložku");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Vyrovnat rohy");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Další roh");
  LSTR MSG_MESH_EDITOR                    = _UxGT("Editor sítě");
  LSTR MSG_EDIT_MESH                      = _UxGT("Upravit síť bodů");
  LSTR MSG_EDITING_STOPPED                = _UxGT("Konec úprav sítě");
  LSTR MSG_PROBING_POINT                  = _UxGT("Měření bodu");
  LSTR MSG_MESH_X                         = _UxGT("Index X");
  LSTR MSG_MESH_Y                         = _UxGT("Index Y");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Hodnota Z");

  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("Vlastní příkazy");
  LSTR MSG_M48_TEST                       = _UxGT("M48 test sondy");
  LSTR MSG_M48_POINT                      = _UxGT("M48 bod");
  LSTR MSG_M48_DEVIATION                  = _UxGT("Odchylka");
  LSTR MSG_IDEX_MENU                      = _UxGT("Režim IDEX");
  LSTR MSG_OFFSETS_MENU                   = _UxGT("Ofsety nástrojů");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Auto-Park");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Duplikace");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("Zrcadlení");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("Plná kontrola");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("2. tryska Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("2. tryska @");

  LSTR MSG_UBL_DOING_G29                  = _UxGT("Provádím G29");
  LSTR MSG_UBL_TOOLS                      = _UxGT("UBL nástroje");
  LSTR MSG_LCD_TILTING_MESH               = _UxGT("Vyrovnání bodu");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Manuální síť bodů");
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("Vložte kartu, změřte");
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Změřte");
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Odstraňte a změřte");
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("Přesun na další");
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Teplota podložky");
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Teplota podložky");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Teplota hotendu");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Teplota hotendu");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Upravit vlastní síť");
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("Doladit síť bodů");
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Konec úprav sítě");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Vlastní síť");
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Vytvořit síť");
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("Síť bodů $");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Kontrola sítě $");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Studená síť bodů");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Upravit výšku sítě");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Výška");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Zkontrolovat síť");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Kontrola vlast. sítě");
  LSTR MSG_G26_HEATING_BED                = _UxGT("G26 zahřívání podl.");
  LSTR MSG_G26_HEATING_NOZZLE             = _UxGT("G26 zařívání trysky");
  LSTR MSG_G26_MANUAL_PRIME               = _UxGT("Ruční zavedení...");
  LSTR MSG_G26_FIXED_LENGTH               = _UxGT("Pevné zavední");
  LSTR MSG_G26_PRIME_DONE                 = _UxGT("Done Priming");
  LSTR MSG_G26_CANCELED                   = _UxGT("G26 Canceled");
  LSTR MSG_G26_LEAVING                    = _UxGT("Leaving G26");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Pokračovat v síťi");
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Síťové rovnání");
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("3-bodové rovnání");
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Mřížkové rovnání");
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Srovnat podložku");
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("Postranní body");
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Typ sítě bodu");
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("Exportovat síť");
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("Exportovat do PC");
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("Exportovat do CSV");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Záloha do PC");
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("Info o UBL do PC");
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Hustota mřížky");
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Ruční hustota");
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Chytrá hustota");
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Zaplnit mřížku");
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Zrušit všechno");
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Zrušit poslední");
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Upravit všechny");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Upravit poslední");
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Uložiště sítí");
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Paměťový slot");
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Načíst síť bodů");
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Uložit síť bodů");
  LSTR MSG_MESH_LOADED                    = _UxGT("Síť %i načtena");
  LSTR MSG_MESH_SAVED                     = _UxGT("Síť %i uložena");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("Nedostatek místa");
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("Ch.: Uložit UBL");
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("Ch.: Obnovit UBL");
  LSTR MSG_UBL_Z_OFFSET                   = _UxGT("Z-ofset: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Konec Z-ofsetu");
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("UBL Postupně");
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1. Studená síť bodů");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2. Chytrá hustota");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3. Zkontrolovat síť");
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4. Upravit všechny");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5. Zkontrolovat síť");
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6. Upravit všechny");
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7. Uložit síť bodů");

  LSTR MSG_LED_CONTROL                    = _UxGT("Nastavení LED");
  LSTR MSG_LIGHTS                         = _UxGT("Světla");
  LSTR MSG_LIGHT_N                        = _UxGT("Světla #{");
  LSTR MSG_LED_PRESETS                    = _UxGT("Světla Předvolby");
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Červená");
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Oranžová");
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Žlutá");
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Zelená");
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Modrá");
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Indigo");
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Fialová");
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("Bílá");
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Výchozí");
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Vlastní světla");
  LSTR MSG_INTENSITY_R                    = _UxGT("Červená intenzita");
  LSTR MSG_INTENSITY_G                    = _UxGT("Zelená intezita");
  LSTR MSG_INTENSITY_B                    = _UxGT("Modrá intenzita");
  LSTR MSG_INTENSITY_W                    = _UxGT("Bílá intenzita");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Jas");

  LSTR MSG_MOVING                         = _UxGT("Posouvání...");
  LSTR MSG_FREE_XY                        = _UxGT("Uvolnit XY");
  LSTR MSG_MOVE_X                         = _UxGT("Posunout X");
  LSTR MSG_MOVE_Y                         = _UxGT("Posunout Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Posunout Z");
  LSTR MSG_MOVE_N                         = _UxGT("Posunout @");
  LSTR MSG_MOVE_E                         = _UxGT("Extrudér");
  LSTR MSG_MOVE_EN                        = _UxGT("Extrudér *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("Hotend je studený");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Posunout o $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Posunout o $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Posunout o $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Rychlost");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Výška podl.");
  LSTR MSG_NOZZLE                         = _UxGT("Tryska");
  LSTR MSG_NOZZLE_N                       = _UxGT("Tryska ~");
  LSTR MSG_BED                            = _UxGT("Podložka");
  LSTR MSG_CHAMBER                        = _UxGT("Komora");
  LSTR MSG_FAN_SPEED                      = _UxGT("Rychlost vent.");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Rychlost vent. ~");
  LSTR MSG_STORED_FAN_N                   = _UxGT("Ulož. vent. ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("Rychlost ex. vent.");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Rychlost ex. vent. ~");
  LSTR MSG_FLOW                           = _UxGT("Průtok");
  LSTR MSG_FLOW_N                         = _UxGT("Průtok ~");
  LSTR MSG_CONTROL                        = _UxGT("Ovládaní");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" fakt");
  LSTR MSG_AUTOTEMP                       = _UxGT("Autoteplota");
  LSTR MSG_LCD_ON                         = _UxGT("Zap");
  LSTR MSG_LCD_OFF                        = _UxGT("Vyp");
  LSTR MSG_PID_AUTOTUNE                   = _UxGT("PID automatika");
  LSTR MSG_PID_AUTOTUNE_E                 = _UxGT("PID automatika *");
  LSTR MSG_SELECT_E                       = _UxGT("Vybrat *");
  LSTR MSG_ACC                            = _UxGT("Zrychl");
  LSTR MSG_JERK                           = _UxGT("Jerk");
  LSTR MSG_VA_JERK                        = _UxGT("Max ") STR_A _UxGT(" Jerk");
  LSTR MSG_VB_JERK                        = _UxGT("Max ") STR_B _UxGT(" Jerk");
  LSTR MSG_VC_JERK                        = _UxGT("Max ") STR_C _UxGT(" Jerk");
  LSTR MSG_VN_JERK                        = _UxGT("Max @ Jerk");
  LSTR MSG_VE_JERK                        = _UxGT("Max E Jerk");
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Odchylka spoje");
  LSTR MSG_MAX_SPEED                      = _UxGT("Max Rychlost");
  LSTR MSG_VMAX_A                         = _UxGT("Max ") STR_A _UxGT(" Rychlost");
  LSTR MSG_VMAX_B                         = _UxGT("Max ") STR_B _UxGT(" Rychlost");
  LSTR MSG_VMAX_C                         = _UxGT("Max ") STR_C _UxGT(" Rychlost");
  LSTR MSG_VMAX_N                         = _UxGT("Max @ Rychlost");
  LSTR MSG_VMAX_E                         = _UxGT("Max E Rychlost");
  LSTR MSG_VMAX_EN                        = _UxGT("Max * Rychlost");
  LSTR MSG_VMIN                           = _UxGT("Vmin");
  LSTR MSG_VTRAV_MIN                      = _UxGT("VTrav Min");
  LSTR MSG_ACCELERATION                   = _UxGT("Akcelerace");
  LSTR MSG_AMAX_A                         = _UxGT("Max ") STR_A _UxGT(" Akcel");
  LSTR MSG_AMAX_B                         = _UxGT("Max ") STR_B _UxGT(" Akcel");
  LSTR MSG_AMAX_C                         = _UxGT("Max ") STR_C _UxGT(" Akcel");
  LSTR MSG_AMAX_N                         = _UxGT("Max @ Akcel");
  LSTR MSG_AMAX_E                         = _UxGT("Max E Akcel");
  LSTR MSG_AMAX_EN                        = _UxGT("Max * Akcel");
  LSTR MSG_A_RETRACT                      = _UxGT("A-retrakt");
  LSTR MSG_A_TRAVEL                       = _UxGT("A-přejezd");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Kroků/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" kroků/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" kroků/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" kroků/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ kroků/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E kroků/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* kroků/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Teplota");
  LSTR MSG_MOTION                         = _UxGT("Pohyb");
  LSTR MSG_FILAMENT                       = _UxGT("Filament");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E na mm") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Fil. Prum.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Fil. Prum. *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Vysunout mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Zavést mm");
  LSTR MSG_ADVANCE_K                      = _UxGT("K pro posun");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("K pro posun *");
  LSTR MSG_CONTRAST                       = _UxGT("Kontrast LCD");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Uložit nastavení");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Načíst nastavení");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Obnovit výchozí");
  LSTR MSG_INIT_EEPROM                    = _UxGT("Inic. EEPROM");
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("Aktualizace z SD");
  LSTR MSG_RESET_PRINTER                  = _UxGT("Reset tiskárny");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Obnovit");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Info obrazovka");
  LSTR MSG_PREPARE                        = _UxGT("Připrava tisku");
  LSTR MSG_TUNE                           = _UxGT("Doladění tisku");
  LSTR MSG_START_PRINT                    = _UxGT("Spustit tisk");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("Další");
  LSTR MSG_BUTTON_INIT                    = _UxGT("Inicializace");
  LSTR MSG_BUTTON_STOP                    = _UxGT("Stop");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("Tisk");
  LSTR MSG_BUTTON_RESET                   = _UxGT("Reset");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("Zrušit");
  LSTR MSG_BUTTON_DONE                    = _UxGT("Hotovo");
  LSTR MSG_BUTTON_BACK                    = _UxGT("Zpět");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("Pokračovat");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Pozastavit tisk");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Obnovit tisk");
  LSTR MSG_STOP_PRINT                     = _UxGT("Zastavit tisk");
  LSTR MSG_PRINTING_OBJECT                = _UxGT("Tisk objektu");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("Zrušit objekt");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("Zrušit objekt {");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Obnova výpadku");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Tisknout z SD");
  LSTR MSG_NO_MEDIA                       = _UxGT("Žádná SD karta");
  LSTR MSG_DWELL                          = _UxGT("Uspáno...");
  LSTR MSG_USERWAIT                       = _UxGT("Čekání na uživ...");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Tisk pozastaven");
  LSTR MSG_PRINTING                       = _UxGT("Tisknu...");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Tisk zrušen");
  LSTR MSG_NO_MOVE                        = _UxGT("Žádný pohyb.");
  LSTR MSG_KILLED                         = _UxGT("PŘERUSENO. ");
  LSTR MSG_STOPPED                        = _UxGT("ZASTAVENO. ");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Retrakt mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Výměna Re.mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Retraktovat  V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Zvednuti Z mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("S Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Unretract V");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("S UnRet V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Auto-Retract");
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("Délka retrakce");
  LSTR MSG_FILAMENT_PURGE_LENGTH          = _UxGT("Délka zavedení");
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Výměna nástroje");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Zdvih Z");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Rychlost primár.");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Rychlost retrak.");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("Tryska standby");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Vyměnit filament");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Vyměnit filament *");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Zavést filament");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Zavést filament *");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Vysunout filament");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Vysunout filament *");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Vysunout vše");

  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Načíst SD");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Načíst USB");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Načíst SD");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Vyměnit médium");
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Vysunout médium");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Sonda Z mimo podl");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Faktor zkosení");
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("BLTouch self-test");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("BLTouch reset");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("BLTouch zasunout");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("BLTouch vysunout");
  LSTR MSG_BLTOUCH_SW_MODE                = _UxGT("SW výsun BLTouch");
  LSTR MSG_BLTOUCH_5V_MODE                = _UxGT("BLTouch 5V režim");
  LSTR MSG_BLTOUCH_OD_MODE                = _UxGT("BLTouch OD režim");
  LSTR MSG_BLTOUCH_MODE_STORE             = _UxGT("Uložit režim");
  LSTR MSG_BLTOUCH_MODE_STORE_5V          = _UxGT("Nastavit 5V");
  LSTR MSG_BLTOUCH_MODE_STORE_OD          = _UxGT("Nastacit OD");
  LSTR MSG_BLTOUCH_MODE_ECHO              = _UxGT("Vypsat nastavení");
  LSTR MSG_BLTOUCH_MODE_CHANGE            = _UxGT("VAROVANÍ: Špatné nastavení může způsobit škody! Pokračovat?");
  LSTR MSG_TOUCHMI_PROBE                  = _UxGT("TouchMI");
  LSTR MSG_TOUCHMI_INIT                   = _UxGT("Inic. TouchMI");
  LSTR MSG_TOUCHMI_ZTEST                  = _UxGT("Test Z Ofsetu");
  LSTR MSG_TOUCHMI_SAVE                   = _UxGT("Uložiy");
  LSTR MSG_MANUAL_DEPLOY_TOUCHMI          = _UxGT("vysunout TouchMI");
  LSTR MSG_MANUAL_DEPLOY                  = _UxGT("Vysunout Z-sondu");
  LSTR MSG_MANUAL_STOW                    = _UxGT("Zasunout Z-sondu");
  LSTR MSG_HOME_FIRST                     = _UxGT("Domů %s první");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("X ofset");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Y ofset");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Z ofset");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("@ ofset");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Krok a Z offset");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Babystep X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Babystep Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Babystep Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Babystep @");
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("Celkem");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Endstop abort");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Chyba zahřívání");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("REDUND. TEPLOTA");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("TEPLOTNÍ ÚNIK");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("VYSOKÁ TEPLOTA");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("NÍZKA TEPLOTA");
  LSTR MSG_HALTED                         = _UxGT("TISK. ZASTAVENA");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Proveďte reset");
  LSTR MSG_HEATING                        = _UxGT("Zahřívání...");
  LSTR MSG_COOLING                        = _UxGT("Chlazení...");
  LSTR MSG_BED_HEATING                    = _UxGT("Zahřívání podl.");
  LSTR MSG_BED_COOLING                    = _UxGT("Chlazení podl.");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("Zahřívání komory...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("Chlazení komory...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Delta Kalibrace");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Kalibrovat X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Kalibrovat Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Kalibrovat Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Kalibrovat Střed");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Delta nastavení");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Autokalibrace");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Diag rameno");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Výška");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Poloměr");
  LSTR MSG_INFO_MENU                      = _UxGT("O tiskárně");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Info o tiskárně");
  LSTR MSG_3POINT_LEVELING                = _UxGT("3-bodové rovnání");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Lineárni rovnání");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("Bilineární rovnání");
  LSTR MSG_UBL_LEVELING                   = _UxGT("Unified Bed Leveling");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Mřížkové rovnání");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Statistika");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Info o desce");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termistory");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Extrudéry");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Rychlost");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protokol");
  LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Sledování úniku: VYP");
  LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Sledování úniku: ZAP");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Osvětlení");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Jas světla");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("NESPRÁVNÁ TISKÁRNA");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Tisky");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Hotovo");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Čas");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Nejdelší");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Vytlačeno");

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Teplota min");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Teplota max");
  LSTR MSG_INFO_PSU                       = _UxGT("Nap. zdroj");
  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Buzení motorů");
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ Motor %");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("DAC uložit EEPROM");
  LSTR MSG_ERROR_TMC                      = _UxGT("TMC CHYBA SPOJENÍ");
  LSTR MSG_FILAMENT_CHANGE_HEADER         = _UxGT("VÝMĚNA FILAMENTU");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("TISK POZASTAVEN");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("ZAVEDENÍ FILAMENTU");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("VYSUNUTÍ FILAMENTU");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("MOŽNOSTI OBNOVENÍ:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("Vytlačit víc");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Obnovit tisk");
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Tryska: ");
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Senzor filamentu");
  LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("Délka mm senz.fil.");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Parkování selhalo");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Kalibrace selhala");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("VYBERTE FILAMENT");
  LSTR MSG_MMU2_MENU                      = _UxGT("MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("Aktual. MMU firmware!");
  LSTR MSG_MMU2_NOT_RESPONDING            = _UxGT("MMU potř. pozornost.");
  LSTR MSG_MMU2_RESUME                    = _UxGT("Obnovit tisk");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("Obnovování...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("Zavést filament");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("Zavést všechny");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("Zavést do trysky");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("Vysunout filament");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("Vysun. filament ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("Vytáhnout filament");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("Zavádění fil. %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Vytahování fil. ...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("Vysouvání fil....");
  LSTR MSG_MMU2_ALL                       = _UxGT("Všechny");
  LSTR MSG_MMU2_FILAMENT_N                = _UxGT("Filament ~");
  LSTR MSG_MMU2_RESET                     = _UxGT("Resetovat MMU");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("Resetování MMU...");
  LSTR MSG_MMU2_EJECT_RECOVER             = _UxGT("Vytáhněte, klikněte");

  LSTR MSG_MIX                            = _UxGT("Mix");
  LSTR MSG_MIX_COMPONENT_N                = _UxGT("Komponenta {");
  LSTR MSG_MIXER                          = _UxGT("Mixér");
  LSTR MSG_GRADIENT                       = _UxGT("Přechod");
  LSTR MSG_FULL_GRADIENT                  = _UxGT("Celý přechod");
  LSTR MSG_TOGGLE_MIX                     = _UxGT("Přepnout mix");
  LSTR MSG_CYCLE_MIX                      = _UxGT("Střídat mix");
  LSTR MSG_GRADIENT_MIX                   = _UxGT("Přechod mix");
  LSTR MSG_REVERSE_GRADIENT               = _UxGT("Opačný přechod");
  LSTR MSG_ACTIVE_VTOOL                   = _UxGT("Aktivní V-nástr.");
  LSTR MSG_START_VTOOL                    = _UxGT("Spustit V-nástr.");
  LSTR MSG_END_VTOOL                      = _UxGT("Ukončit V-nástr.");
  LSTR MSG_GRADIENT_ALIAS                 = _UxGT("Alias V-nástr.");
  LSTR MSG_RESET_VTOOLS                   = _UxGT("Reset. V-nástr.");
  LSTR MSG_COMMIT_VTOOL                   = _UxGT("Uložit V-nás. mix");
  LSTR MSG_VTOOLS_RESET                   = _UxGT("V-nástr. reset.");
  LSTR MSG_START_Z                        = _UxGT("Počáteční Z:");
  LSTR MSG_END_Z                          = _UxGT("  Koncové Z:");

  LSTR MSG_GAMES                          = _UxGT("Hry");
  LSTR MSG_BRICKOUT                       = _UxGT("Brickout");
  LSTR MSG_INVADERS                       = _UxGT("Invaders");
  LSTR MSG_SNAKE                          = _UxGT("Sn4k3");
  LSTR MSG_MAZE                           = _UxGT("Bludiště");

  // Up to 2 lines allowed
  LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_2_LINE("Stikněte tlač.", "pro obnovení"));
  LSTR MSG_PAUSE_PRINT_PARKING            = _UxGT(MSG_1_LINE("Parkování..."));
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Čekejte..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Vložte, klikněte"));
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_2_LINE("Klikněte pro", "nahřátí"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Nahřívání..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Vysouvání..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Zavádění..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_1_LINE("Vytlačování..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_2_LINE("Klikněte pro", "ukončení"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("Pokračování..."));

  LSTR MSG_TMC_DRIVERS                    = _UxGT("TMC budiče");
  LSTR MSG_TMC_CURRENT                    = _UxGT("Proud budičů");
  LSTR MSG_TMC_HYBRID_THRS                = _UxGT("Hybridní práh");
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("Domů bez senzorů");
  LSTR MSG_TMC_STEPPING_MODE              = _UxGT("Režim kroků");
  LSTR MSG_TMC_STEALTH_ENABLED            = _UxGT("StealthChop povolen");
  LSTR MSG_SERVICE_RESET                  = _UxGT("Reset");
  LSTR MSG_SERVICE_IN                     = _UxGT(" za:");
  LSTR MSG_BACKLASH                       = _UxGT("Vůle");
  LSTR MSG_BACKLASH_CORRECTION            = _UxGT("Korekce");
  LSTR MSG_BACKLASH_SMOOTHING             = _UxGT("Vyhlazení");
}

namespace LanguageWide_cz {
  using namespace LanguageNarrow_cz;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_PROGRESS_BAR_TEST            = _UxGT("Test ukaz. průběhu");
    LSTR MSG_BED_HEATING                  = _UxGT("Zahřívání podložky");
    LSTR MSG_BED_COOLING                  = _UxGT("Chlazení podložky");
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Počet tisků");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Dokončeno");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Celkový čas");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Nejdelší tisk");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Celkem vytlačeno");
    LSTR MSG_ACTIVE_VTOOL                 = _UxGT("Aktivní V-nástroj");
    LSTR MSG_START_VTOOL                  = _UxGT("Spustit V-nástroj");
    LSTR MSG_END_VTOOL                    = _UxGT("Ukončit V-nástroj");
    LSTR MSG_GRADIENT_ALIAS               = _UxGT("Alias V-nástroje");
    LSTR MSG_RESET_VTOOLS                 = _UxGT("Resetovat V-nástroj");
    LSTR MSG_COMMIT_VTOOL                 = _UxGT("Uložit V-nástroj mix");
    LSTR MSG_VTOOLS_RESET                 = _UxGT("V-nástroj resetovat");
  #endif
}

namespace LanguageTall_cz {
  using namespace LanguageWide_cz;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Stikněte tlačítko", "pro obnovení tisku"));
    LSTR MSG_PAUSE_PRINT_PARKING          = _UxGT(MSG_1_LINE("Parkování..."));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Čekejte prosím", "na zahájení", "výměny filamentu"));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Vložte filament", "a stiskněte", "tlačítko..."));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Klikněte pro", "nahřátí trysky"));
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Čekejte prosím", "na nahřátí tr."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_3_LINE("Čekejte prosím", "na vysunuti", "filamentu"));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_3_LINE("Čekejte prosím", "na zavedení", "filamentu"));
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_2_LINE("Vyčkejte na", "vytlačení"));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_3_LINE("Klikněte pro", "ukončení", "vytlačování"));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_3_LINE("Čekejte prosím", "na pokračování", "tisku"));
  #endif
}

namespace Language_cz {
  using namespace LanguageTall_cz;
}
