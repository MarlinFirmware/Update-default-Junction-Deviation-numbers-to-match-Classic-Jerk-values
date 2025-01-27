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
 * Hungarian / Magyar
 *
 * LCD Menu Messages. See also https://marlinfw.org/docs/development/lcd_language.html
 * Hungarian translation by AntoszHUN. I am constantly improving and updating the translation.
 * Translation last updated: 08/30/2021 - 22:20
 *
 * LCD Menü Üzenetek. Lásd még https://marlinfw.org/docs/development/lcd_language.html
 * A Magyar fordítást készítette: AntoszHUN. A fordítást folyamatosan javítom és frissítem.
 * A Fordítás utolsó frissítése: 2021.08.30. - 22:20
 */

namespace LanguageNarrow_hu {
  using namespace Language_en; // A fordítás az örökölt Amerikai Angol (English) karakterláncokat használja.

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Magyar");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" Kész.");
  LSTR MSG_YES                            = _UxGT("IGEN");
  LSTR MSG_NO                             = _UxGT("NEM");
  LSTR MSG_BACK                           = _UxGT("Vissza");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Megszakítás...");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Tároló behelyezve");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Tároló eltávolítva");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("Várakozás a tárolóra");
  LSTR MSG_MEDIA_INIT_FAIL                = _UxGT("Tároló-kártya hiba");
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("Tároló olvasási hiba");
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("USB eltávolítva");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("USB eszköz hiba");
  LSTR MSG_KILL_SUBCALL_OVERFLOW          = _UxGT("Túlfolyás");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Végállás"); // Maximum 8 karakter
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Szoft. végállás");
  LSTR MSG_MAIN_MENU                      = _UxGT("<Fömenü>");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("További beállítások");
  LSTR MSG_CONFIGURATION                  = _UxGT("Konfiguráció");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Fájl auto. futtatás");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Motorok kikapcsolása");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Hiba Menü");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Haladás sáv teszt");
  LSTR MSG_AUTO_HOME                      = _UxGT("X-Y-Z auto kezdöpont");
  LSTR MSG_AUTO_HOME_N                    = _UxGT("Kezdö @");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("X kezdöpont");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Y kezdöpont");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Z kezdöpont");
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Auto Z-igazítás");
  LSTR MSG_ITERATION                      = _UxGT("G34 Ismétlés: %i");
  LSTR MSG_DECREASING_ACCURACY            = _UxGT("Pontosság csökken!");
  LSTR MSG_ACCURACY_ACHIEVED              = _UxGT("Pontosság elérve");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("X-Y-Z kezdöpont");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Kattints a kezdéshez.");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Következö pont");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Szintezés kész!");
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Szint csökkentés");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Kezdöpont eltolás");
  LSTR MSG_HOME_OFFSET_X                  = _UxGT("X Kezdö eltol."); // DWIN
  LSTR MSG_HOME_OFFSET_Y                  = _UxGT("Y Kezdö eltol."); // DWIN
  LSTR MSG_HOME_OFFSET_Z                  = _UxGT("Z Kezdö eltol."); // DWIN
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Eltolás beállítva.");
  LSTR MSG_TRAMMING_WIZARD                = _UxGT("Elektromos varázsló");
  LSTR MSG_SELECT_ORIGIN                  = _UxGT("Eredeti választása");
  LSTR MSG_LAST_VALUE_SP                  = _UxGT("Utolsó érték ");

  LSTR MSG_PREHEAT_1                      = _UxGT("Fütés ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Fütés ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Fütés ") PREHEAT_1_LABEL _UxGT(" Fej");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Fütés ") PREHEAT_1_LABEL _UxGT(" Fej ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Fütés ") PREHEAT_1_LABEL _UxGT(" Mind");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Fütés ") PREHEAT_1_LABEL _UxGT(" Ágy");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Fütés ") PREHEAT_1_LABEL _UxGT(" Beáll");

  LSTR MSG_PREHEAT_M                      = _UxGT("Fütés $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Fütés $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Fütés $ Fej");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Fütés $ Fej ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Fütés $ Mind");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Fütés $ Ágy");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Fütés $ Beáll");

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Egyedi elömelegítés");
  LSTR MSG_COOLDOWN                       = _UxGT("Visszahütés");

  LSTR MSG_CUTTER_FREQUENCY               = _UxGT("Frekvencia");
  LSTR MSG_LASER_MENU                     = _UxGT("Lézer vezérlés");
  LSTR MSG_SPINDLE_MENU                   = _UxGT("Orsó vezérlés");
  LSTR MSG_LASER_POWER                    = _UxGT("Lézer telj.");
  LSTR MSG_SPINDLE_POWER                  = _UxGT("Orsó telj.");
  LSTR MSG_LASER_TOGGLE                   = _UxGT("Lézer váltás");
  LSTR MSG_LASER_EVAC_TOGGLE              = _UxGT("Hütés váltás");
  LSTR MSG_LASER_ASSIST_TOGGLE            = _UxGT("Levegö segéd");
  LSTR MSG_LASER_PULSE_MS                 = _UxGT("Impulzus teszt ms");
  LSTR MSG_LASER_FIRE_PULSE               = _UxGT("Tüz impulzus");
  LSTR MSG_FLOWMETER_FAULT                = _UxGT("Áramlási hiba");
  LSTR MSG_SPINDLE_TOGGLE                 = _UxGT("Orsóváltás");
  LSTR MSG_SPINDLE_EVAC_TOGGLE            = _UxGT("Vákuum váltás");
  LSTR MSG_SPINDLE_FORWARD                = _UxGT("Orsó elöre");
  LSTR MSG_SPINDLE_REVERSE                = _UxGT("Orsó hátra");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Bekapcsolás");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Kikapcsolás");
  LSTR MSG_EXTRUDE                        = _UxGT("Adagol");
  LSTR MSG_RETRACT                        = _UxGT("Visszahúz");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Tengelyek mozgatása");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Érintés/szintezés");
  LSTR MSG_BED_LEVELING                   = _UxGT("Ágy szintezés");
  LSTR MSG_LEVEL_BED                      = _UxGT("Ágy szintezése");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Sarok szint");
  LSTR MSG_BED_TRAMMING_RAISE             = _UxGT("Ágy emelése a szonda váltásig");
  LSTR MSG_BED_TRAMMING_IN_RANGE          = _UxGT("Minden sarok tolerancián belül. Szint jó.");
  LSTR MSG_BED_TRAMMING_GOOD_POINTS       = _UxGT("Jó pontok: ");
  LSTR MSG_BED_TRAMMING_LAST_Z            = _UxGT("Utolsó Z: ");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Következö sarok");
  LSTR MSG_MESH_EDITOR                    = _UxGT("Háló szerkesztö");
  LSTR MSG_EDIT_MESH                      = _UxGT("Háló szerkesztése");
  LSTR MSG_EDITING_STOPPED                = _UxGT("Háló szerk. állj");
  LSTR MSG_PROBING_POINT                  = _UxGT("Próbapont");
  LSTR MSG_MESH_X                         = _UxGT("Index X");
  LSTR MSG_MESH_Y                         = _UxGT("Index Y");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Z érték");
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("Egyéni parancs");
  LSTR MSG_M48_TEST                       = _UxGT("M48 Szonda teszt");
  LSTR MSG_M48_POINT                      = _UxGT("M48 Pont");
  LSTR MSG_M48_OUT_OF_BOUNDS              = _UxGT("Szonda határon kívül");
  LSTR MSG_M48_DEVIATION                  = _UxGT("Eltérés");
  LSTR MSG_IDEX_MENU                      = _UxGT("IDEX mód");
  LSTR MSG_OFFSETS_MENU                   = _UxGT("Eszköz eltolás");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Automata parkolás");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Duplikálás");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("Tükrözött másolás");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("Teljes felügyelet");
  LSTR MSG_IDEX_DUPE_GAP                  = _UxGT("X-hézag másolása");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("2. fej Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("2. fej @");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("Szintezz! G29");
  LSTR MSG_UBL_TOOLS                      = _UxGT("UBL eszköz");
  LSTR MSG_LCD_TILTING_MESH               = _UxGT("Döntési pont");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Kézi háló építés");
  LSTR MSG_UBL_MESH_WIZARD                = _UxGT("UBL Háló varázsló");
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("Tégy alátétet és mérj");
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Mérés");
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Üres ágyat mérj");
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("Továbblépés");
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Ágy höfok");
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Egyéni ágy höfok");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Fejhöfok");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Egyéni fejhöfok");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Egyéni háló szerkesztés");
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("Finomított háló");
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Háló kész");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Egyéni háló építés");
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Háló építés");
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("Háló építés ($)");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Hideg háló építés");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Háló magasság állítás");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Magasság összege");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Háló elfogadás");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Háló elfogadás ($)");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Valódi háló elfogadása");
  LSTR MSG_G26_HEATING_BED                = _UxGT("G26 Ágy fütés");
  LSTR MSG_G26_HEATING_NOZZLE             = _UxGT("G26 Fej fütés");
  LSTR MSG_G26_MANUAL_PRIME               = _UxGT("Kézi alapozás...");
  LSTR MSG_G26_FIXED_LENGTH               = _UxGT("Fix hosszúságú alap");
  LSTR MSG_G26_PRIME_DONE                 = _UxGT("Alapozás kész");
  LSTR MSG_G26_CANCELED                   = _UxGT("G26 Törölve");
  LSTR MSG_G26_LEAVING                    = _UxGT("Kilépö G26");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Ágy háló folyt.");
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Háló szintezés");
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("3-Pontos szintezés");
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Rács szintezés");
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Háló szint");
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("Oldal pontok");
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Térkép típus");
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("Háló térkép kimenet");
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("Host kimenet");
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("CSV kimenet");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Nyomtató bizt.mentés");
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("UBL infó kimenet");
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Kitöltési költség");
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Kézi kitöltés");
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Okos kitöltés");
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Háló kitöltés");
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Minden érvénytelen");
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Közelebbi érvénytelen");
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Mindet finomhangolja");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Közelebbi finomhangolása");
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Háló tárolás");
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Memória foglalat");
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Ágy háló betöltés");
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Ágy háló mentés");
  LSTR MSG_MESH_LOADED                    = _UxGT("M117 Háló %i betöltve");
  LSTR MSG_MESH_SAVED                     = _UxGT("M117 Háló %i mentve");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("Nincs tároló");
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("Hiba: UBL mentés");
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("Hiba: UBL visszaáll.");
  LSTR MSG_UBL_Z_OFFSET                   = _UxGT("Z-eltolás: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Z-eltolás leállítva");
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("Lépésröl lépésre UBL");
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1. Hideg háló készítés");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2. Inteligens kitöltés");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3. Háló érvényesítés");
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4. Minden finomítása");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5. Háló érvényesítés");
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6. Minden finomítása");
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7. Ágy háló mentése");

  LSTR MSG_LED_CONTROL                    = _UxGT("LED vezérlés");
  LSTR MSG_LIGHTS                         = _UxGT("Világítás");
  LSTR MSG_LIGHT_N                        = _UxGT("Világítás #{");
  LSTR MSG_LED_PRESETS                    = _UxGT("Beállított színek");
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Piros");
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Narancs");
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Sárga");
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Zöld");
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Kék");
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Indigó");
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Viola");
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("Fehér");
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Alapérték");
  LSTR MSG_LED_CHANNEL_N                  = _UxGT("Csatorna {");
  LSTR MSG_NEO2_PRESETS                   = _UxGT("Fény #2 megadott");
  LSTR MSG_NEO2_BRIGHTNESS                = _UxGT("Fényerö");
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Egyéni szín");
  LSTR MSG_INTENSITY_R                    = _UxGT("Piros intenzitás");
  LSTR MSG_INTENSITY_G                    = _UxGT("Zöld intenzitás");
  LSTR MSG_INTENSITY_B                    = _UxGT("Kék intenzitás");
  LSTR MSG_INTENSITY_W                    = _UxGT("Fehér intenzitás");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Fényerö");

  LSTR MSG_MOVING                         = _UxGT("Mozgás...");
  LSTR MSG_FREE_XY                        = _UxGT("XY szabad");
  LSTR MSG_MOVE_X                         = _UxGT("X mozgás");
  LSTR MSG_MOVE_Y                         = _UxGT("Y mozgás");
  LSTR MSG_MOVE_Z                         = _UxGT("Z mozgás");
  LSTR MSG_MOVE_N                         = _UxGT("@ mozgás");
  LSTR MSG_MOVE_E                         = _UxGT("Adagoló");
  LSTR MSG_MOVE_EN                        = _UxGT("Adagoló *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("A fej túl hideg");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Mozgás $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Mozgás $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Mozgás $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Sebesség");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Z ágy");
  LSTR MSG_NOZZLE                         = _UxGT("Fej");
  LSTR MSG_NOZZLE_N                       = _UxGT("Fej ~");
  LSTR MSG_NOZZLE_PARKED                  = _UxGT("Fej parkolva");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("Fej készenlétbe");
  LSTR MSG_BED                            = _UxGT("Ágy");
  LSTR MSG_CHAMBER                        = _UxGT("Burkolat");
  LSTR MSG_COOLER                         = _UxGT("Lézer hütövíz");
  LSTR MSG_COOLER_TOGGLE                  = _UxGT("Hütö kapcsoló");
  LSTR MSG_FLOWMETER_SAFETY               = _UxGT("Áramlásbiztonság");
  LSTR MSG_LASER                          = _UxGT("Lézer");
  LSTR MSG_FAN_SPEED                      = _UxGT("Hütés sebesség");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Hütés sebesség {");
  LSTR MSG_STORED_FAN_N                   = _UxGT("Tárolt hütés {");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("Extra hütés sebesség");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Extra hütés sebesség {");
  LSTR MSG_CONTROLLER_FAN                 = _UxGT("Hütésvezérlés");
  LSTR MSG_CONTROLLER_FAN_IDLE_SPEED      = _UxGT("Alapjárat");
  LSTR MSG_CONTROLLER_FAN_AUTO_ON         = _UxGT("Automatikus mód");
  LSTR MSG_CONTROLLER_FAN_SPEED           = _UxGT("Aktív sebesség");
  LSTR MSG_CONTROLLER_FAN_DURATION        = _UxGT("Készenlét");
  LSTR MSG_FLOW                           = _UxGT("Folyás");
  LSTR MSG_FLOW_N                         = _UxGT("Folyás ~");
  LSTR MSG_CONTROL                        = _UxGT("Konfiguráció");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Minimum");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Maximum");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Tényezö");
  LSTR MSG_AUTOTEMP                       = _UxGT("Automata höfok");
  LSTR MSG_LCD_ON                         = _UxGT("Be");
  LSTR MSG_LCD_OFF                        = _UxGT("Ki");
  LSTR MSG_PID_AUTOTUNE                   = _UxGT("PID hangolás");
  LSTR MSG_PID_AUTOTUNE_E                 = _UxGT("PID hangolás *");
  LSTR MSG_PID_CYCLE                      = _UxGT("PID ciklus");
  LSTR MSG_PID_AUTOTUNE_DONE              = _UxGT("PID hangolás kész");
  LSTR MSG_PID_BAD_HEATER_ID              = _UxGT("Hangolási hiba! Rossz adagoló.");
  LSTR MSG_PID_TEMP_TOO_HIGH              = _UxGT("Hangolási hiba! Magas hömérséklet.");
  LSTR MSG_PID_TIMEOUT                    = _UxGT("Hangolási hiba! Idötúllépés.");
  LSTR MSG_SELECT_E                       = _UxGT("Kiválaszt *");
  LSTR MSG_ACC                            = _UxGT("Gyorsítás");
  LSTR MSG_JERK                           = _UxGT("Rántás");
  LSTR MSG_VA_JERK                        = _UxGT("Seb.") STR_A _UxGT("-Rántás");
  LSTR MSG_VB_JERK                        = _UxGT("Seb.") STR_B _UxGT("-Rántás");
  LSTR MSG_VC_JERK                        = _UxGT("Seb.") STR_C _UxGT("-Rántás");
  LSTR MSG_VN_JERK                        = _UxGT("Seb.@-Rántás");
  LSTR MSG_VE_JERK                        = _UxGT("E ránt. seb.");
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Csomopont eltérés");
  LSTR MSG_MAX_SPEED                      = _UxGT("Max Sebesség (mm/s)");
  LSTR MSG_VMAX_A                         = _UxGT("Max Seb. ") STR_A;
  LSTR MSG_VMAX_B                         = _UxGT("Max Seb. ") STR_B;
  LSTR MSG_VMAX_C                         = _UxGT("Max Seb. ") STR_C;
  LSTR MSG_VMAX_N                         = _UxGT("Max Seb. @");
  LSTR MSG_VMAX_E                         = _UxGT("Max Seb. E");
  LSTR MSG_VMAX_EN                        = _UxGT("Max sebesség *");
  LSTR MSG_VMIN                           = _UxGT("Min sebesség");
  LSTR MSG_VTRAV_MIN                      = _UxGT("Min utazó.seb.");
  LSTR MSG_ACCELERATION                   = _UxGT("Gyorsulás");
  LSTR MSG_AMAX_A                         = _UxGT("Max gyors. ") STR_A;
  LSTR MSG_AMAX_B                         = _UxGT("Max gyors. ") STR_B;
  LSTR MSG_AMAX_C                         = _UxGT("Max gyors. ") STR_C;
  LSTR MSG_AMAX_N                         = _UxGT("Max gyors. @");
  LSTR MSG_AMAX_E                         = _UxGT("Max gyors. E");
  LSTR MSG_AMAX_EN                        = _UxGT("Max gyorsulás *");
  LSTR MSG_A_RETRACT                      = _UxGT("Visszahúzás");
  LSTR MSG_A_TRAVEL                       = _UxGT("Utazás");
  LSTR MSG_XY_FREQUENCY_LIMIT             = _UxGT("Max frekvencia");
  LSTR MSG_XY_FREQUENCY_FEEDRATE          = _UxGT("Min elötolás");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Lépés/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" Lépés/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" Lépés/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" Lépés/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ lépés/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E lépés/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("*Lépés/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Höfok");
  LSTR MSG_MOTION                         = _UxGT("Mozgatások");
  LSTR MSG_FILAMENT                       = _UxGT("Nyomtatószál");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E mm³-ben");
  LSTR MSG_VOLUMETRIC_LIMIT               = _UxGT("E Limit mm³-ben");
  LSTR MSG_VOLUMETRIC_LIMIT_E             = _UxGT("E Limit *");
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Szál. átm.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Szál. átm. *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Kiadás mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Betöltés mm");
  LSTR MSG_ADVANCE_K                      = _UxGT("Haladó K");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("Haladó K *");
  LSTR MSG_CONTRAST                       = _UxGT("LCD kontraszt");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Mentés EEPROM");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Betöltés EEPROM");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Alapértelmezett");
  LSTR MSG_INIT_EEPROM                    = _UxGT("EEPROM inicializálás");
  LSTR MSG_ERR_EEPROM_CRC                 = _UxGT("Hiba: EEPROM CRC");
  LSTR MSG_ERR_EEPROM_SIZE                = _UxGT("Hiba: EEPROM mérete");
  LSTR MSG_ERR_EEPROM_VERSION             = _UxGT("Hiba: EEPROM verzió");
  LSTR MSG_SETTINGS_STORED                = _UxGT("Beállítások mentve");
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("Tároló frissítés");
  LSTR MSG_RESET_PRINTER                  = _UxGT("Nyomtató újraindítása");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Frissítés");
  LSTR MSG_INFO_SCREEN                    = _UxGT("<Infó képernyö>");
  LSTR MSG_PREPARE                        = _UxGT("Vezérlés");
  LSTR MSG_TUNE                           = _UxGT("Hangolás");
  LSTR MSG_POWER_MONITOR                  = _UxGT("Teljesítménymonitor");
  LSTR MSG_CURRENT                        = _UxGT("Jelenlegi");
  LSTR MSG_VOLTAGE                        = _UxGT("Feszültség");
  LSTR MSG_POWER                          = _UxGT("Energia");
  LSTR MSG_START_PRINT                    = _UxGT("Nyomtatás indítása");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("Tovább");
  LSTR MSG_BUTTON_INIT                    = _UxGT("Kezdet");
  LSTR MSG_BUTTON_STOP                    = _UxGT("Állj");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("Nyomtatás");
  LSTR MSG_BUTTON_RESET                   = _UxGT("Újraindítás");
  LSTR MSG_BUTTON_IGNORE                  = _UxGT("Mellöz");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("Mégse");
  LSTR MSG_BUTTON_DONE                    = _UxGT("Kész");
  LSTR MSG_BUTTON_BACK                    = _UxGT("Vissza");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("Folytatás");
  LSTR MSG_BUTTON_SKIP                    = _UxGT("Kihagy");
  LSTR MSG_PAUSING                        = _UxGT("Szüneteltetve...");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Nyomtatás szünetelés");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Nyomtatás folytatása");
  LSTR MSG_HOST_START_PRINT               = _UxGT("Hoszt indítás");
  LSTR MSG_STOP_PRINT                     = _UxGT("Nyomtatás leállítása");
  LSTR MSG_END_LOOPS                      = _UxGT("Hurok ismétlés vége");
  LSTR MSG_PRINTING_OBJECT                = _UxGT("Objektum nyomtatása");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("Objektum törlése");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("Objektum törlése {");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Kiesés helyreáll.");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Nyomtatás tárolóról");
  LSTR MSG_NO_MEDIA                       = _UxGT("Nincs tároló");
  LSTR MSG_DWELL                          = _UxGT("Alvás...");
  LSTR MSG_USERWAIT                       = _UxGT("Katt a folytatáshoz...");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Nyomtatás szünetelve");
  LSTR MSG_PRINTING                       = _UxGT("Nyomtatás...");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Nyomtatás leállítva");
  LSTR MSG_PRINT_DONE                     = _UxGT("Nyomtatás kész");
  LSTR MSG_NO_MOVE                        = _UxGT("Nincs mozgás.");
  LSTR MSG_KILLED                         = _UxGT("HALOTT! ");
  LSTR MSG_STOPPED                        = _UxGT("MEGÁLLT! ");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Visszahúzás mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Visszahúzás cs. mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Viszahúzás");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Ugrás mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Visszah.helyre mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Csere.visszah.helyre mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Visszahúzás V");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("S Vissza.h V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Auto visszah.");
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("Visszahúzás távolság");
  LSTR MSG_FILAMENT_SWAP_EXTRA            = _UxGT("Extra csere");
  LSTR MSG_FILAMENT_PURGE_LENGTH          = _UxGT("Tisztítási távolság");
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Szerszámcsere");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Z emelés");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Fösebesség");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Visszah. sebesség");
  LSTR MSG_FILAMENT_PARK_ENABLED          = _UxGT("Fej parkolás");
  LSTR MSG_SINGLENOZZLE_UNRETRACT_SPEED   = _UxGT("Visszav.visszah. sebesség");
  LSTR MSG_SINGLENOZZLE_FAN_SPEED         = _UxGT("FAN sebesség");
  LSTR MSG_SINGLENOZZLE_FAN_TIME          = _UxGT("FAN idö");
  LSTR MSG_TOOL_MIGRATION_ON              = _UxGT("Auto BE");
  LSTR MSG_TOOL_MIGRATION_OFF             = _UxGT("Auto KI");
  LSTR MSG_TOOL_MIGRATION                 = _UxGT("Szerszámcsere");
  LSTR MSG_TOOL_MIGRATION_AUTO            = _UxGT("Automata csere");
  LSTR MSG_TOOL_MIGRATION_END             = _UxGT("Utolsó adagoló");
  LSTR MSG_TOOL_MIGRATION_SWAP            = _UxGT("Csere *");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Szálcsere");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Szálcsere *");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Szál betöltés");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Szál betöltés *");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Szál eltávolítás");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Szál eltávolítás *");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Mindet eltávolít");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Tároló SD");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Tároló USB");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Tároló SD");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Tároló csere");
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Tároló Kiadása");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Z szonda tálcán kivül");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Ferdeség faktor");
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("Önteszt");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Visszaállítás");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("Elhelyez");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("Telepít");
  LSTR MSG_BLTOUCH_SW_MODE                = _UxGT("SW-Mód");
  LSTR MSG_BLTOUCH_5V_MODE                = _UxGT("5V-Mód");
  LSTR MSG_BLTOUCH_OD_MODE                = _UxGT("OD-Mód");
  LSTR MSG_BLTOUCH_MODE_STORE             = _UxGT("Módok");
  LSTR MSG_BLTOUCH_MODE_STORE_5V          = _UxGT("BLTouch 5V mód");
  LSTR MSG_BLTOUCH_MODE_STORE_OD          = _UxGT("BLTouch OD mód");
  LSTR MSG_BLTOUCH_MODE_ECHO              = _UxGT("Jelentés");
  LSTR MSG_BLTOUCH_MODE_CHANGE            = _UxGT("VESZÉLY: A rossz beállítások kárt okozhatnak! Biztos továbblép?");
  LSTR MSG_TOUCHMI_PROBE                  = _UxGT("TouchMI");
  LSTR MSG_TOUCHMI_INIT                   = _UxGT("Kezd TouchMI");
  LSTR MSG_TOUCHMI_ZTEST                  = _UxGT("Z eltolás teszt");
  LSTR MSG_TOUCHMI_SAVE                   = _UxGT("Mentés");
  LSTR MSG_MANUAL_DEPLOY_TOUCHMI          = _UxGT("TouchMI használ");
  LSTR MSG_MANUAL_DEPLOY                  = _UxGT("Z-Szonda telepítés");
  LSTR MSG_MANUAL_STOW                    = _UxGT("Z-Szonda elhelyezés");
  LSTR MSG_HOME_FIRST                     = _UxGT("Elöször %s kell");
  LSTR MSG_ZPROBE_OFFSETS                 = _UxGT("Szonda eltolások");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("X szonda eltolás");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Y szonda eltolás");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Z szonda eltolás");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("@ szonda eltolás");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Z lépés állítás");
  LSTR MSG_MOVE_NOZZLE_TO_BED             = _UxGT("Fej az ágyhoz");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Mikrolépés X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Mikrolépés Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Mikrolépés Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Mikrolépés @");
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("Teljes");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Végállás megszakítva!");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Fütés hiba!");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Hiba: SZÜKSÉGTELEN HÖFOK");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("FÜTÉS KIMARADÁS");
  LSTR MSG_ERR_COOLING_FAILED             = _UxGT("Hütés sikertelen");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Hiba: MAX höfok");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Hiba: MIN höfok");
  LSTR MSG_HALTED                         = _UxGT("A NYOMTATÓ LEÁLLT");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Indítsd újra!");
  LSTR MSG_HEATING                        = _UxGT("Fütés...");
  LSTR MSG_COOLING                        = _UxGT("Hütés...");
  LSTR MSG_BED_HEATING                    = _UxGT("Ágy fütés...");
  LSTR MSG_BED_COOLING                    = _UxGT("Ágy hütés...");
  LSTR MSG_PROBE_HEATING                  = _UxGT("Szonda fütése...");
  LSTR MSG_PROBE_COOLING                  = _UxGT("Szonda hütése...");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("Kamra fütés...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("Kamra hütés...");
  LSTR MSG_LASER_COOLING                  = _UxGT("Lézer hütés...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Delta kalibráció");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("X kalibrálás");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Y kalibrálás");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Z kalibrálás");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Központ kalibrálás");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Delta beállítások");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Auto kalibráció");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Diag rúd");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Magasság");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Sugár");
  LSTR MSG_INFO_MENU                      = _UxGT("A Nyomtatóról");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Nyomtató infó");
  LSTR MSG_3POINT_LEVELING                = _UxGT("3-Pontos szintezés");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Lineáris szintezés");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("Bilineáris szintezés");
  LSTR MSG_UBL_LEVELING                   = _UxGT("Egységes ágy szintezés");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Háló szintezés");
  LSTR MSG_MESH_DONE                      = _UxGT("Háló szintezés kész");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Statisztikák");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Alaplap infó");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termisztorok");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Adagolók");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Átviteli sebesség");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protokol");
  LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Futáselemzés: KI");
  LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Futáselemzés: BE");
  LSTR MSG_HOTEND_IDLE_TIMEOUT            = _UxGT("Fej üresjárati idök.");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Munkalámpa");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Fényerösség");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("HELYTELEN NYOMTATÓ");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Nyomtatások");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Befejezett");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Összes");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Leghosszabb");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Kiadott");

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Min höfok");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Max höfok");
  LSTR MSG_INFO_PSU                       = _UxGT("PSU");
  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Meghajtási erö");
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ meghajtó %");
  LSTR MSG_ERROR_TMC                      = _UxGT("TMC CSATLAKOZÁSI HIBA");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("DAC EEPROM írása");
  LSTR MSG_FILAMENT_CHANGE_HEADER         = _UxGT("NYOMTATÓSZÁL CSERE");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("NYOMTATÁS SZÜNETEL");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("SZÁL BETÖLTÉS");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("SZÁL ELTÁVOLÍTÁS");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("FOLYTATÁSI OPCIÓ:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("Tisztítsd meg");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Folytatás");
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Fej: ");
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Túlfutás szenzor");
  LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("Túlfutás táv. mm");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Tájolási hiba");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Szondázás hiba");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("SZÁLVÁLASZTÁS");
  LSTR MSG_MMU2_MENU                      = _UxGT("MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("MMU szoftver feltöltése!");
  LSTR MSG_MMU2_NOT_RESPONDING            = _UxGT("MMU figyelmeztetés.");
  LSTR MSG_MMU2_RESUME                    = _UxGT("Nyomtatás folytatása");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("Folytatás...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("Szál betöltése");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("Összes betöltése");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("Fej betöltése");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("Szál kidobás");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("Szál kidobás ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("Kiadja a szálat");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("Szál betölt. %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Szál kidobás. ...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("Szál kiadása....");
  LSTR MSG_MMU2_ALL                       = _UxGT("Mind");
  LSTR MSG_MMU2_FILAMENT_N                = _UxGT("Nyomtatószál ~");
  LSTR MSG_MMU2_RESET                     = _UxGT("MMU újraindítás");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("MMU újraindul...");
  LSTR MSG_MMU2_EJECT_RECOVER             = _UxGT("Kidob, kattint");

  LSTR MSG_MIX                            = _UxGT("Kever");
  LSTR MSG_MIX_COMPONENT_N                = _UxGT("Összetevö {");
  LSTR MSG_MIXER                          = _UxGT("Keverö");
  LSTR MSG_GRADIENT                       = _UxGT("Színátm.");
  LSTR MSG_FULL_GRADIENT                  = _UxGT("Teljes színátm.");
  LSTR MSG_TOGGLE_MIX                     = _UxGT("Váltás keverésre");
  LSTR MSG_CYCLE_MIX                      = _UxGT("Ciklikus keverés");
  LSTR MSG_GRADIENT_MIX                   = _UxGT("Színátm. keverés");
  LSTR MSG_REVERSE_GRADIENT               = _UxGT("Fordított színátm.");
  LSTR MSG_ACTIVE_VTOOL                   = _UxGT("Aktív V-szerszám");
  LSTR MSG_START_VTOOL                    = _UxGT("Kezdés V-szerszám");
  LSTR MSG_END_VTOOL                      = _UxGT("  Vége V-szerszám");
  LSTR MSG_GRADIENT_ALIAS                 = _UxGT("Ál V-szerszám");
  LSTR MSG_RESET_VTOOLS                   = _UxGT("Újra V-szerszám");
  LSTR MSG_COMMIT_VTOOL                   = _UxGT("Gyors V-szerszám Kev.");
  LSTR MSG_VTOOLS_RESET                   = _UxGT("V-szersz. visszaáll.");
  LSTR MSG_START_Z                        = _UxGT("Kezdés Z:");
  LSTR MSG_END_Z                          = _UxGT("  Vége Z:");

  LSTR MSG_GAMES                          = _UxGT("Játékok");
  LSTR MSG_BRICKOUT                       = _UxGT("Brickout");
  LSTR MSG_INVADERS                       = _UxGT("Invaders");
  LSTR MSG_SNAKE                          = _UxGT("Sn4k3");
  LSTR MSG_MAZE                           = _UxGT("Maze");

  LSTR MSG_BAD_PAGE                       = _UxGT("Rossz oldalindex");
  LSTR MSG_BAD_PAGE_SPEED                 = _UxGT("Rossz oldalsebesség");

  LSTR MSG_EDIT_PASSWORD                  = _UxGT("Jelszó szerkesztése");
  LSTR MSG_LOGIN_REQUIRED                 = _UxGT("Belépés szükséges");
  LSTR MSG_PASSWORD_SETTINGS              = _UxGT("Jelszóbeállítások");
  LSTR MSG_ENTER_DIGIT                    = _UxGT("Írja be a számokat");
  LSTR MSG_CHANGE_PASSWORD                = _UxGT("Jelszó Beáll/Szerk");
  LSTR MSG_REMOVE_PASSWORD                = _UxGT("Jelszó törlése");
  LSTR MSG_PASSWORD_SET                   = _UxGT("A jelszó ");
  LSTR MSG_START_OVER                     = _UxGT("Újrakezdés");
  LSTR MSG_REMINDER_SAVE_SETTINGS         = _UxGT("Mentsd el!");
  LSTR MSG_PASSWORD_REMOVED               = _UxGT("Jelszó törölve");

  //
  // Filament Change screens show up to 2 lines on a 3-line display
  //
  LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_1_LINE("Katt a folytatáshoz"));
  LSTR MSG_PAUSE_PRINT_PARKING            = _UxGT(MSG_1_LINE("Parkolás..."));
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Kérlek várj..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Behelyez majd katt"));
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_1_LINE("Katt a fütéshez"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Fütés..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Kiadás..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Betöltés..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_1_LINE("Tisztítás..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_1_LINE("Katt ha kész"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("Folytatás..."));

  LSTR MSG_TMC_DRIVERS                    = _UxGT("TMC meghajtók");
  LSTR MSG_TMC_CURRENT                    = _UxGT("Meghajtó áram");
  LSTR MSG_TMC_HYBRID_THRS                = _UxGT("Hibrid küszöbérték");
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("Motoros kezdöpont");
  LSTR MSG_TMC_STEPPING_MODE              = _UxGT("Léptetö mód");
  LSTR MSG_TMC_STEALTH_ENABLED            = _UxGT("StealthChop mód");
  LSTR MSG_SERVICE_RESET                  = _UxGT("Újraindítás");
  LSTR MSG_SERVICE_IN                     = _UxGT(" be:");
  LSTR MSG_BACKLASH                       = _UxGT("Holtjáték");
  LSTR MSG_BACKLASH_CORRECTION            = _UxGT("Korrekció");
  LSTR MSG_BACKLASH_SMOOTHING             = _UxGT("Simítás");

  LSTR MSG_LEVEL_X_AXIS                   = _UxGT("X tengely szint");
  LSTR MSG_AUTO_CALIBRATE                 = _UxGT("Önkalibrálás");
  LSTR MSG_FTDI_HEATER_TIMEOUT            = _UxGT("Tétlenségi idökorlát, a hömérséklet csökkent. Nyomd meg az OK gombot az ismételt felfütéshez, és újra a folytatáshoz.");
  LSTR MSG_HEATER_TIMEOUT                 = _UxGT("Fütés idökorlátja");
  LSTR MSG_REHEAT                         = _UxGT("Újrafüt");
  LSTR MSG_REHEATING                      = _UxGT("Újrafütés...");

  LSTR MSG_PROBE_WIZARD                   = _UxGT("Z szonda varázsló");
  LSTR MSG_PROBE_WIZARD_PROBING           = _UxGT("Z referencia mérés");
  LSTR MSG_PROBE_WIZARD_MOVING            = _UxGT("Menj a próba pontra");

  LSTR MSG_SOUND                          = _UxGT("Hang");

  LSTR MSG_TOP_LEFT                       = _UxGT("Bal felsö");
  LSTR MSG_BOTTOM_LEFT                    = _UxGT("Bal alsó");
  LSTR MSG_TOP_RIGHT                      = _UxGT("Jobb felsö");
  LSTR MSG_BOTTOM_RIGHT                   = _UxGT("Jobb alsó");
  LSTR MSG_CALIBRATION_COMPLETED          = _UxGT("Kalibrálás befejezve");
  LSTR MSG_CALIBRATION_FAILED             = _UxGT("Kalibrálási hiba");

  LSTR MSG_DRIVER_BACKWARD                = _UxGT(" meghajtók hátra");

  LSTR MSG_SD_CARD                        = _UxGT("SD Kártya");
  LSTR MSG_USB_DISK                       = _UxGT("USB Lemez");

  LSTR MSG_SHORT_DAY                      = _UxGT("n"); // Csak egy karakter
  LSTR MSG_SHORT_HOUR                     = _UxGT("ó"); // Csak egy karakter
  LSTR MSG_SHORT_MINUTE                   = _UxGT("p"); // Csak egy karakter
}

namespace LanguageWide_hu {
  using namespace LanguageNarrow_hu;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Nyomtatás számláló");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Befejezett");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Összes nyomtatási idö");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Leghosszabb munkaidö");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Összes anyag");
  #endif
}

namespace LanguageTall_hu {
  using namespace LanguageWide_hu;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Nyomj gombot", "nyomtatás folytatáshoz"));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Várj míg", "szálcsere", "indítás"));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Szál behelyezés", "majd nyomj gombot", "a folytatáshoz"));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Nyomj gombot", "a fej fütéséhez"));
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Fej fütése", "Kérlek várj..."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_2_LINE("Várj a", "szál kiadására"));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_2_LINE("Várj a", "szál betöltésére"));
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_2_LINE("Várj a", "szál tisztításra"));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_2_LINE("Kattints a készre", "szál tiszta"));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_2_LINE("Várj a nyomtatóra", "majd folytat..."));
  #endif
}

namespace Language_hu {
  using namespace LanguageTall_hu;
}
