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
 * Polish - includes accented characters
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

#define DISPLAY_CHARSET_ISO10646_PL

namespace LanguageNarrow_pl {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Polski");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" gotowy.");
  LSTR MSG_YES                            = _UxGT("TAK");
  LSTR MSG_NO                             = _UxGT("NIE");
  LSTR MSG_BACK                           = _UxGT("Wstecz");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Przerywanie...");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Karta włożona");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Karta usunięta");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("Oczekiwanie na kartę");
  LSTR MSG_MEDIA_INIT_FAIL                = _UxGT("Błąd inicializacji karty");
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("Bład odczytu karty");
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("Urządzenie USB usunięte");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("Błąd uruchomienia USB");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Krańców."); // Max length 8 characters
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Progr. Krańcówki");
  LSTR MSG_MAIN_MENU                      = _UxGT("Menu główne");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Ustawienie zaawansowane");
  LSTR MSG_CONFIGURATION                  = _UxGT("Konfiguracja");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Autostart");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Wyłącz silniki");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Menu Debugowania");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Testowy pasek postępu");
  LSTR MSG_AUTO_HOME                      = _UxGT("Pozycja zerowa");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Zeruj X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Zeruj Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Zeruj Z");
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Autowyrównanie Z");
  LSTR MSG_DECREASING_ACCURACY            = _UxGT("Spadek dokładności!");
  LSTR MSG_ACCURACY_ACHIEVED              = _UxGT("Osiągnięto dokładność");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Pozycja zerowa");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Kliknij by rozp.");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Następny punkt");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Wypoziomowano!");
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Wys. zanikania");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Ust. poz. zer.");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Poz. zerowa ust.");
  LSTR MSG_SELECT_ORIGIN                  = _UxGT("Wybierz punkt zero");
  LSTR MSG_LAST_VALUE_SP                  = _UxGT("Poprzednia wartość ");

  LSTR MSG_PREHEAT_1                      = _UxGT("Rozgrzej ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Rozgrzej ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Rozgrzej ") PREHEAT_1_LABEL _UxGT(" Dysza");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Rozgrzej ") PREHEAT_1_LABEL _UxGT(" Dysza ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Rozgrzej ") PREHEAT_1_LABEL _UxGT(" wsz.");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Rozgrzej ") PREHEAT_1_LABEL _UxGT(" stół");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Rozgrzej ") PREHEAT_1_LABEL _UxGT(" ustaw.");

  LSTR MSG_PREHEAT_M                      = _UxGT("Rozgrzej $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Rozgrzej $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Rozgrzej $ Dysza");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Rozgrzej $ Dysza ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Rozgrzej $ wsz.");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Rozgrzej $ stół");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Rozgrzej $ ustaw.");

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Rozgrzej własne ust.");
  LSTR MSG_COOLDOWN                       = _UxGT("Chłodzenie");

  LSTR MSG_CUTTER_FREQUENCY               = _UxGT("Częstotliwość");
  LSTR MSG_LASER_MENU                     = _UxGT("Sterowanie Lasera");
  LSTR MSG_SPINDLE_MENU                   = _UxGT("Sterowanie wrzeciona");
  LSTR MSG_LASER_POWER                    = _UxGT("Zasilanie Lasera");
  LSTR MSG_SPINDLE_POWER                  = _UxGT("Zasilanie wrzeciona");
  LSTR MSG_SPINDLE_REVERSE                = _UxGT("Rewers wrzeciona");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Włącz zasilacz");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Wyłącz zasilacz");
  LSTR MSG_EXTRUDE                        = _UxGT("Ekstruzja");
  LSTR MSG_RETRACT                        = _UxGT("Wycofanie");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Ruch osi");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Sonda/poziomowanie");
  LSTR MSG_BED_LEVELING                   = _UxGT("Poziomowanie stołu");
  LSTR MSG_LEVEL_BED                      = _UxGT("Wypoziomuj stół");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Narożniki poziomowania");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Nastepny narożnik");
  LSTR MSG_MESH_EDITOR                    = _UxGT("Edytor siatki");
  LSTR MSG_EDIT_MESH                      = _UxGT("Edycja siatki");
  LSTR MSG_EDITING_STOPPED                = _UxGT("Edycja siatki zatrzymana");
  LSTR MSG_PROBING_POINT                  = _UxGT("Punkt pomiarowy");
  LSTR MSG_MESH_X                         = _UxGT("Indeks X");
  LSTR MSG_MESH_Y                         = _UxGT("Indeks Y");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Wartość Z");
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("Własne Polecenia");
  LSTR MSG_M48_TEST                       = _UxGT("M48 Test sondy");
  LSTR MSG_M48_POINT                      = _UxGT("M48 Punky");
  LSTR MSG_M48_DEVIATION                  = _UxGT("Odchylenie");
  LSTR MSG_IDEX_MENU                      = _UxGT("Tryb IDEX");
  LSTR MSG_OFFSETS_MENU                   = _UxGT("Przesunięcie narzędzia");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Auto-Parkowanie");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Duplikowanie");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("Kopia lustrzana");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("Pełne sterowanie");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("2ga dysza Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("2ga dysza @");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("Wykonywanie G29");
  LSTR MSG_UBL_TOOLS                      = _UxGT("Narzędzia UBL");
  LSTR MSG_LCD_TILTING_MESH               = _UxGT("Punkt pochylenia");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Ręczne Budowanie Siatki");
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("Umieść podkładkę i zmierz");
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Zmierz");
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Usuń & Zmierz Stół");
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("Przesuwanie do następnego");
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Temperatura stołu");
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Temperatura stołu");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Temperatura dyszy");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Temperatura dyszy");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Edycja własnej siatki");
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("Dostrajanie siatki");
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Koniec edycji siati");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Buduj własna siatkę");
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Buduj siatkę");
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("Buduj siatkę ($)");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Buduj siatkę na zimno");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Dostrojenie wysokości siatki");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Wartość wysokości");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Sprawdzenie siatki");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Sprawdzenie siatki ($)");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Sprawdzenie własnej siatki");
  LSTR MSG_G26_HEATING_BED                = _UxGT("G26 Nagrzewanie stołu");
  LSTR MSG_G26_HEATING_NOZZLE             = _UxGT("G26 Nagrzewanie dyszy");
  LSTR MSG_G26_MANUAL_PRIME               = _UxGT("Napełnianie ręczne...");
  LSTR MSG_G26_FIXED_LENGTH               = _UxGT("Napełnij kreśloną długością");
  LSTR MSG_G26_PRIME_DONE                 = _UxGT("Napełnianie zakończone");
  LSTR MSG_G26_CANCELED                   = _UxGT("G26 Przewane");
  LSTR MSG_G26_LEAVING                    = _UxGT("Opuszczanie G26");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Kontynuuj tworzenie siatki");
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Poziomowanie siatką");
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("Poziomowaie 3-punktowe");
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Poziomowaie według siatki");
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Poziomuj siatkę");
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("Punkty boczne");
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Rodzaj mapy");
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("Wyslij mapę siatki");
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("Wyslij do Hosta");
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("Wyslij do CSV");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Kopia poza drukarką");
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("Wyslij info UBL");
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Stopień wypełnienia");
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Ręczne wypełnienie");
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Inteligentne wypełnienie");
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Wypełnienie siatki");
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Unieważnij wszystko");
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Unieważnij najbliższy");
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Dostrajaj wszystko");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Dostrajaj najbliższy");
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Przechowywanie siatki");
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Slot Pamięci");
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Załaduj siatkę stołu");
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Zapisz siatkę stołu");
  LSTR MSG_MESH_LOADED                    = _UxGT("Siatka %i załadowana");
  LSTR MSG_MESH_SAVED                     = _UxGT("Siatka %i zapisana");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("Brak magazynu");
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("Błąd: Zapis UBL");
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("Bład: Odczyt UBL");
  LSTR MSG_UBL_Z_OFFSET                   = _UxGT("Przesunięcie Z: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Przesunięcie Z zatrzymane");
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("UBL Krok po kroku");
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1. Tworzenie zimnej siatki");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2. Inteligentne wypełnienie");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3. Sprawdzenie siatki");
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4. Dostrojenie wszystkiego");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5. Sprawdzenie siatki");
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6. Dostrojenie wszystkiego");
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7. Zapis siatki stołu");

  LSTR MSG_LED_CONTROL                    = _UxGT("Sterowanie LED");
  LSTR MSG_LIGHTS                         = _UxGT("Światła");
  LSTR MSG_LIGHT_N                        = _UxGT("Światło #{");
  LSTR MSG_LED_PRESETS                    = _UxGT("Ustawienia świateł");
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Czerwony");
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Pomarańczowy");
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Zółty");
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Zielony");
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Niebieski");
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Indygo");
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Fioletowy");
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("Biały");
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Domyślny");
  LSTR MSG_LED_CHANNEL_N                  = _UxGT("Kanał {");
  LSTR MSG_NEO2_BRIGHTNESS                = _UxGT("Jasność");
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Własne światła");
  LSTR MSG_INTENSITY_R                    = _UxGT("Czerwony");
  LSTR MSG_INTENSITY_G                    = _UxGT("Zielony");
  LSTR MSG_INTENSITY_B                    = _UxGT("Niebieski");
  LSTR MSG_INTENSITY_W                    = _UxGT("Biały");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Jasność");

  LSTR MSG_MOVING                         = _UxGT("Ruch...");
  LSTR MSG_FREE_XY                        = _UxGT("Swobodne XY");
  LSTR MSG_MOVE_X                         = _UxGT("Przesuń w X");
  LSTR MSG_MOVE_Y                         = _UxGT("Przesuń w Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Przesuń w Z");
  LSTR MSG_MOVE_N                         = _UxGT("Przesuń w @");
  LSTR MSG_MOVE_E                         = _UxGT("Ekstruzja (os E)");
  LSTR MSG_MOVE_EN                        = _UxGT("Ekstruzja (os E) *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("Dysza za zimna");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Przesuń co $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Przesuń co $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Przesuń co $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Predkość");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Stół Z");
  LSTR MSG_NOZZLE                         = _UxGT("Dysza");
  LSTR MSG_NOZZLE_N                       = _UxGT("Dysza ~");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("Dysza w oczekiwaniu");
  LSTR MSG_BED                            = _UxGT("Stół");
  LSTR MSG_CHAMBER                        = _UxGT("Obudowa");
  LSTR MSG_FAN_SPEED                      = _UxGT("Obroty wentylatora");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Obroty wentylatora ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("Obroty dodatkowego wentylatora");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Obroty dodatkowego wentylatora ~");
  LSTR MSG_CONTROLLER_FAN                 = _UxGT("Wentylator kontrolera");
  LSTR MSG_FLOW                           = _UxGT("Przepływ");
  LSTR MSG_FLOW_N                         = _UxGT("Przepływ ~");
  LSTR MSG_CONTROL                        = _UxGT("Ustawienia");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Mnożnik");
  LSTR MSG_AUTOTEMP                       = _UxGT("Auto. temperatura");
  LSTR MSG_LCD_ON                         = _UxGT("Wł.");
  LSTR MSG_LCD_OFF                        = _UxGT("Wył.");
  LSTR MSG_PID_AUTOTUNE                   = _UxGT("PID Autostrojenie");
  LSTR MSG_PID_AUTOTUNE_E                 = _UxGT("PID Autostrojenie *");
  LSTR MSG_PID_AUTOTUNE_DONE              = _UxGT("Strojenie PID zakończone");
  LSTR MSG_SELECT_E                       = _UxGT("Wybierz *");
  LSTR MSG_ACC                            = _UxGT("Przyspieszenie");
  LSTR MSG_JERK                           = _UxGT("Zryw");
  LSTR MSG_VA_JERK                        = _UxGT("Zryw V") STR_A;
  LSTR MSG_VB_JERK                        = _UxGT("Zryw V") STR_B;
  LSTR MSG_VC_JERK                        = _UxGT("Zryw V") STR_C;
  LSTR MSG_VN_JERK                        = _UxGT("Zryw V@");
  LSTR MSG_VE_JERK                        = _UxGT("Zryw Ve");
  LSTR MSG_MAX_SPEED                      = _UxGT("Prędkość (V)");

  LSTR MSG_VTRAV_MIN                      = _UxGT("Vskok min");
  LSTR MSG_ACCELERATION                   = _UxGT("Przyspieszenie (A)");

  LSTR MSG_A_RETRACT                      = _UxGT("A-wycofanie");
  LSTR MSG_A_TRAVEL                       = _UxGT("A-przesuń.");
  LSTR MSG_XY_FREQUENCY_LIMIT             = _UxGT("Częstotliwość max");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("kroki/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" kroki/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" kroki/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" kroki/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ kroki/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E kroki/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* kroki/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperatura");
  LSTR MSG_MOTION                         = _UxGT("Ruch");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E w mm") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Śr. fil.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Śr. fil. *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Wysuń mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Wsuń mm");
  LSTR MSG_CONTRAST                       = _UxGT("Kontrast LCD");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Zapisz w pamięci");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Wczytaj z pamięci");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Ustaw. fabryczne");
  LSTR MSG_INIT_EEPROM                    = _UxGT("Initializuj EEPROM");
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("Uaktualnij kartę");
  LSTR MSG_RESET_PRINTER                  = _UxGT("Resetuj drukarkę");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Odswież");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Ekran główny");
  LSTR MSG_PREPARE                        = _UxGT("Przygotuj");
  LSTR MSG_TUNE                           = _UxGT("Strojenie");
  LSTR MSG_CURRENT                        = _UxGT("Natężenie");
  LSTR MSG_VOLTAGE                        = _UxGT("Napięcie");
  LSTR MSG_POWER                          = _UxGT("Moc");
  LSTR MSG_START_PRINT                    = _UxGT("Start wydruku");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("Następny");
  LSTR MSG_BUTTON_INIT                    = _UxGT("Inic.");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("Drukuj");
  LSTR MSG_BUTTON_RESET                   = _UxGT("Resetuj");
  LSTR MSG_BUTTON_IGNORE                  = _UxGT("Ignoruj");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("Przerwij");
  LSTR MSG_BUTTON_DONE                    = _UxGT("Gotowe");
  LSTR MSG_BUTTON_BACK                    = _UxGT("Wstecz");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("Kontynuuj");
  LSTR MSG_BUTTON_SKIP                    = _UxGT("Pomiń");
  LSTR MSG_PAUSING                        = _UxGT("Wstrzymywanie...");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Wstrzymaj druk");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Wznowienie");
  LSTR MSG_STOP_PRINT                     = _UxGT("Stop");
  LSTR MSG_PRINTING_OBJECT                = _UxGT("Drukowanie obiektu");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("Anunuj obiekt");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("Anunuj obiekt {");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Odzyskiwanie po awarii");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Karta SD");
  LSTR MSG_NO_MEDIA                       = _UxGT("Brak karty");
  LSTR MSG_DWELL                          = _UxGT("Uśpij...");
  LSTR MSG_USERWAIT                       = _UxGT("Oczekiwanie...");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Druk wstrzymany");
  LSTR MSG_PRINTING                       = _UxGT("Drukowanie...");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Druk przerwany");
  LSTR MSG_PRINT_DONE                     = _UxGT("Druk zakończony");
  LSTR MSG_NO_MOVE                        = _UxGT("Brak ruchu");
  LSTR MSG_KILLED                         = _UxGT("Ubity. ");
  LSTR MSG_STOPPED                        = _UxGT("Zatrzymany. ");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Wycofaj mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Z Wycof. mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Wycofaj  V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Skok Z mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Cof. wycof. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Z Cof. wyc. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Cof. wycof.  V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Auto. wycofanie");
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("Długość zmiany");
  LSTR MSG_FILAMENT_PURGE_LENGTH          = _UxGT("Długość oczyszczania");
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Zmiana narzędzia");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Podniesienie Z");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Prędkość napełniania");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Prędkość wycofania");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Zmień filament");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Zmień filament *");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Wsuń Filament");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Wsuń Filament *");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Wysuń Filament");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Wysuń Filament *");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Wysuń wszystkie");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Inicjal. karty SD");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Inicjal. dysk flash");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Inicjal. karty SD");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Zmiana karty SD");
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Zwolnienie karty");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Sonda Z za stołem");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Współczynik skrzywienia");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("BLTouch Self-Test");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Reset BLTouch");
  LSTR MSG_BLTOUCH_MODE_CHANGE            = _UxGT("UWAGA: Złe ustawienia mogą uszkodzić drukarkę. Kontynuować?");
  LSTR MSG_HOME_FIRST                     = _UxGT("Najpierw Home %s");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Offset X");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Offset Y");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Offset Z");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Offset @");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Krok Z offsetu");
  LSTR MSG_MOVE_NOZZLE_TO_BED             = _UxGT("Przesuń dyszę do stołu");
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("Łącznie");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Błąd krańcówki");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Rozgrz. nieudane");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Błąd temperatury");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("ZANIK TEMPERATURY");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Błąd: MAXTEMP");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Błąd: MINTEMP");
  LSTR MSG_HALTED                         = _UxGT("Drukarka zatrzym.");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Proszę zresetować");
  LSTR MSG_SHORT_HOUR                     = _UxGT("g"); // One character only
  LSTR MSG_HEATING                        = _UxGT("Rozgrzewanie...");
  LSTR MSG_COOLING                        = _UxGT("Chłodzenie...");
  LSTR MSG_BED_HEATING                    = _UxGT("Rozgrzewanie stołu...");
  LSTR MSG_BED_COOLING                    = _UxGT("Chłodzenie stołu...");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("Rozgrzewanie komory...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("Chłodzenie komory...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Kalibrowanie Delty");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Kalibruj X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Kalibruj Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Kalibruj Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Kalibruj środek");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Ustawienia delty");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Auto kalibrowanie");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Ukośne ramię");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Wysokość");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Promień");
  LSTR MSG_INFO_MENU                      = _UxGT("O drukarce");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Info drukarki");
  LSTR MSG_3POINT_LEVELING                = _UxGT("Poziomowanie 3-punktowe");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Poziomowanie liniowe");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("Poziomowanie biliniowe");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Poziomowanie siatką");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Statystyki");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Info płyty");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termistory");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Ekstrudery");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Predkość USB");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protokół");
  LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Zegar pracy: OFF");
  LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Zegar pracy: ON");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Oświetlenie obudowy");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Jasność oświetlenia");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("Niepoprawna drukarka");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Wydrukowano");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Ukończono");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Razem");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Najdł. druk");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Użyty fil.");

  LSTR MSG_INFO_PSU                       = _UxGT("Zasilacz");
  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Siła silnika");
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ Siła %");
  LSTR MSG_ERROR_TMC                      = _UxGT("TMC BŁĄD POŁĄCZENIA");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("Zapisz DAC EEPROM");
  LSTR MSG_FILAMENT_CHANGE_HEADER         = _UxGT("ZMIEŃ FILAMENT");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("WYDRUK WSTRZYMANY");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("WSUŃ FILAMENT");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("WYSUŃ FILAMENT");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("OPCJE WZNOWIENIA:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("Oczyść więcej");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Kontynuuj");
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Dysza: ");
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Czujnik filamentu");
  LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("Dystans do czujnika mm");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Zerowanie nieudane");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Sondowanie nieudane");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("WYBIERZ FILAMENT");
  LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("Uaktualnij firmware MMU!");
  LSTR MSG_MMU2_NOT_RESPONDING            = _UxGT("MMU wymaga uwagi.");
  LSTR MSG_MMU2_RESUME                    = _UxGT("Wznów wydruk");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("Wznawianie...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("Wsuń filament");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("Wsuń wszystko");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("Wsuń do dyszy");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("Wysuń filament");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("Wysuń filament ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("Wysuń filament");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("Wsuwanie fil. %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Wysuwanie fil. ...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("Wysuwanie fil....");
  LSTR MSG_MMU2_ALL                       = _UxGT("Wszystko");
  LSTR MSG_MMU2_RESET                     = _UxGT("Resetuj MMU");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("Resetowanie MMU...");
  LSTR MSG_MMU2_EJECT_RECOVER             = _UxGT("Usuń, kliknij");

  LSTR MSG_MIX                            = _UxGT("Miks");
  LSTR MSG_MIX_COMPONENT_N                = _UxGT("Komponent {");
  LSTR MSG_MIXER                          = _UxGT("Mikser");
  LSTR MSG_FULL_GRADIENT                  = _UxGT("Pełny gradient");
  LSTR MSG_TOGGLE_MIX                     = _UxGT("Przełacz miks");
  LSTR MSG_REVERSE_GRADIENT               = _UxGT("Odwrotny gradient");

  LSTR MSG_GAMES                          = _UxGT("Gry");

  LSTR MSG_EDIT_PASSWORD                  = _UxGT("Zmień hasło");
  LSTR MSG_LOGIN_REQUIRED                 = _UxGT("Wymagane zalogowanie");
  LSTR MSG_PASSWORD_SETTINGS              = _UxGT("Ustawienia hasła");
  LSTR MSG_ENTER_DIGIT                    = _UxGT("Wprowadź cyfrę");
  LSTR MSG_CHANGE_PASSWORD                = _UxGT("Ustaw/zmień hasło");
  LSTR MSG_REMOVE_PASSWORD                = _UxGT("Usuń hasło");
  LSTR MSG_PASSWORD_SET                   = _UxGT("Hasło to ");
  LSTR MSG_START_OVER                     = _UxGT("Od nowa");
  LSTR MSG_REMINDER_SAVE_SETTINGS         = _UxGT("Pamiętaj by zapisać!");
  LSTR MSG_PASSWORD_REMOVED               = _UxGT("Hasło usunięte");

  //
  // Filament Change screens show up to 2 lines on a 3-line display
  //
  LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_1_LINE("Kliknij by kontynuować"));
  LSTR MSG_PAUSE_PRINT_PARKING            = _UxGT(MSG_1_LINE("Parkowanie..."));
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Proszę czekać..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Włóż i kliknij"));
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_1_LINE("Kliknij by nagrzać"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Nagrzewanie..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Wysuwanie..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Wsuwanie..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_1_LINE("Oczyszczanie..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_1_LINE("Kliknij by zakończyć"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("Wznawianie..."));

  LSTR MSG_TMC_DRIVERS                    = _UxGT("Sterowniki TMC");
  LSTR MSG_TMC_CURRENT                    = _UxGT("Prąd sterownika");
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("Zerowanie bezczujnikowe");
  LSTR MSG_BACKLASH_CORRECTION            = _UxGT("Korekcja");
  LSTR MSG_BACKLASH_SMOOTHING             = _UxGT("Wygładzanie");

  LSTR MSG_LEVEL_X_AXIS                   = _UxGT("Wypoziomuj oś X");
  LSTR MSG_AUTO_CALIBRATE                 = _UxGT("Autokalibracja");

  LSTR MSG_SOUND                          = _UxGT("Dźwięk");

  LSTR MSG_CALIBRATION_COMPLETED          = _UxGT("Kalibracja zakończona");
  LSTR MSG_CALIBRATION_FAILED             = _UxGT("Kalibracja nie powiodła się");

}

namespace LanguageWide_pl {
  using namespace LanguageNarrow_pl;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Wydrukowano");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Ukończono");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Czas druku");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Najdł. druk");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Użyty fil.");
  #endif
}

namespace LanguageTall_pl {
  using namespace LanguageWide_pl;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Nacisnik przycisk", "by wznowić drukowanie"));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Czekam na", "zmianę filamentu", "by wystartować"));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Włóż filament", "i naciśnij przycisk", "by kontynuować"));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Naciśnij przycisk", "by nagrzać dyszę"));
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Nagrzewanie dyszy", "Proszę czekać..."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_2_LINE("Czekam na", "wyjęcie filamentu"));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_2_LINE("Czekam na", "włożenie filamentu"));
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_2_LINE("Czekam na", "oczyszczenie filamentu"));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_2_LINE("Kliknij by zakończyć", "oczyszczanie filamentu"));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_2_LINE("Czekam na", "wznowienie wydruku..."));
  #endif
}

namespace Language_pl {
  using namespace LanguageTall_pl;
}
