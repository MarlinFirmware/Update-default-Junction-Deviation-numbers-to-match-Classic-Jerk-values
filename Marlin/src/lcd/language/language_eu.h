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
 * Basque-Euskera
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#define DISPLAY_CHARSET_ISO10646_1
#define NOT_EXTENDED_ISO10646_1_5X7

namespace LanguageNarrow_eu {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 1;
  LSTR LANGUAGE                           = _UxGT("Basque-Euskera");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" prest.");
  LSTR MSG_BACK                           = _UxGT("Atzera");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Txartela sartuta");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Txartela kenduta");
  LSTR MSG_MAIN_MENU                      = _UxGT("Menu nagusia");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Auto hasiera");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Itzali motoreak");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Arazketa Menua");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Prog. Barra Proba");
  LSTR MSG_AUTO_HOME                      = _UxGT("Hasierara joan");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("X jatorrira");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Y jatorrira");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Z jatorrira");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("XYZ hasieraratzen");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Klik egin hasteko");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Hurrengo Puntua");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Berdintzea eginda");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Etxe. offset eza.");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Offsetak ezarrita");

  LSTR MSG_PREHEAT_1                      = _UxGT("Berotu ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Berotu ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Berotu ") PREHEAT_1_LABEL _UxGT(" Amaia");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Berotu ") PREHEAT_1_LABEL _UxGT(" Amaia ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Berotu ") PREHEAT_1_LABEL _UxGT(" Guztia");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Berotu ") PREHEAT_1_LABEL _UxGT(" Ohea");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Berotu ") PREHEAT_1_LABEL _UxGT(" Ezarp.");

  LSTR MSG_PREHEAT_M                      = _UxGT("Berotu $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Berotu $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Berotu $ Amaia");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Berotu $ Amaia ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Berotu $ Guztia");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Berotu $ Ohea");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Berotu $ Ezarp.");

  LSTR MSG_COOLDOWN                       = _UxGT("Hoztu");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Energia piztu");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Energia itzali");
  LSTR MSG_EXTRUDE                        = _UxGT("Estruitu");
  LSTR MSG_RETRACT                        = _UxGT("Atzera eragin");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Ardatzak mugitu");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Proba eta Berdindu");
  LSTR MSG_BED_LEVELING                   = _UxGT("Ohe berdinketa");
  LSTR MSG_LEVEL_BED                      = _UxGT("Ohea berdindu");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Ertzak berdindu");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Hurrengo ertza");
  LSTR MSG_EDIT_MESH                      = _UxGT("Sarea editatu");

  LSTR MSG_UBL_DOING_G29                  = _UxGT("G29 exekutatzen");
  LSTR MSG_UBL_TOOLS                      = _UxGT("UBL Tresnak");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Sarea eskuz sortu");
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Neurtu");
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Ohearen tenperatura");
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Bed Temp");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Mutur beroaren tenp.");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Hotend Temp");
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Sarea editatzea eginda");
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Sarea sortu");
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("$ sarea sortu");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("$ sarea balioetsi");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Sare hotza sortu");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Sarearen altuera doitu");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Sarea balioetsi");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Ohe sarea balioetsi");
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Sare berdinketa");
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("3 puntuko berdinketa");
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Lauki-sare berdinketa");
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Sarea berdindu");
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Mapa mota");
  LSTR MSG_LED_CONTROL                    = _UxGT("LED ezarpenak");
  LSTR MSG_LIGHTS                         = _UxGT("Argiak");
  LSTR MSG_LIGHT_N                        = _UxGT("Argi #{");
  LSTR MSG_LED_PRESETS                    = _UxGT("Argi aurrehautaketak");
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Gorria");
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Laranja");
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Horia");
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Berdea");
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Urdina");
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Indigo");
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Bioleta");
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("Zuria");
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Lehenetsia");
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Argi pertsonalizatuak");
  LSTR MSG_INTENSITY_R                    = _UxGT("Intentsitate gorria");
  LSTR MSG_INTENSITY_G                    = _UxGT("Intentsitate berdea");
  LSTR MSG_INTENSITY_B                    = _UxGT("Intentsitate urdina");
  LSTR MSG_INTENSITY_W                    = _UxGT("Intentsitate zuria");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Distira");

  LSTR MSG_MOVING                         = _UxGT("Mugitzen...");
  LSTR MSG_FREE_XY                        = _UxGT("Askatu XY");
  LSTR MSG_MOVE_X                         = _UxGT("Mugitu X");
  LSTR MSG_MOVE_Y                         = _UxGT("Mugitu Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Mugitu Z");
  LSTR MSG_MOVE_N                         = _UxGT("Mugitu @");
  LSTR MSG_MOVE_E                         = _UxGT("Estrusorea");
  LSTR MSG_MOVE_EN                        = _UxGT("Estrusorea *");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Mugitu $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Mugitu $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Mugitu $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Abiadura");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Z Ohea");
  LSTR MSG_NOZZLE                         = _UxGT("Pita");
  LSTR MSG_NOZZLE_N                       = _UxGT("Pita ~");
  LSTR MSG_BED                            = _UxGT("Ohea");
  LSTR MSG_FAN_SPEED                      = _UxGT("Haizagailu abiadura");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Haizagailu abiadura ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("Haiz.gehig. abiadura");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Haiz.gehig. abiadura ~");
  LSTR MSG_FLOW                           = _UxGT("Fluxua");
  LSTR MSG_FLOW_N                         = _UxGT("Fluxua ~");
  LSTR MSG_CONTROL                        = _UxGT("Kontrola");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Fakt");
  LSTR MSG_AUTOTEMP                       = _UxGT("Auto tenperatura");
  LSTR MSG_SELECT_E                       = _UxGT("Aukeratu *");
  LSTR MSG_ACC                            = _UxGT("Azelerazioa");
  LSTR MSG_JERK                           = _UxGT("Astindua");
  LSTR MSG_VA_JERK                        = _UxGT("V") STR_A _UxGT("-astindua");
  LSTR MSG_VB_JERK                        = _UxGT("V") STR_B _UxGT("-astindua");
  LSTR MSG_VC_JERK                        = _UxGT("V") STR_C _UxGT("-astindua");
  LSTR MSG_VN_JERK                        = _UxGT("V@-astindua");
  LSTR MSG_VE_JERK                        = _UxGT("Ve-astindua");
  LSTR MSG_VTRAV_MIN                      = _UxGT("VBidaia min");
  LSTR MSG_A_RETRACT                      = _UxGT("A-retrakt");
  LSTR MSG_A_TRAVEL                       = _UxGT("A-bidaia");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Pausoak/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" pausoak/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" pausoak/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" pausoak/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ pausoak/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E pausoak/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* pausoak/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Tenperatura");
  LSTR MSG_MOTION                         = _UxGT("Mugimendua");
  LSTR MSG_FILAMENT                       = _UxGT("Harizpia");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E mm3-tan");
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Hariz. Dia.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Hariz. Dia. *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Deskargatu mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Kargatu mm");
  LSTR MSG_ADVANCE_K                      = _UxGT("K Aurrerapena");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("K Aurrerapena *");
  LSTR MSG_CONTRAST                       = _UxGT("LCD kontrastea");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Gorde memoria");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Kargatu memoria");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Larri. berriz.");
  LSTR MSG_INIT_EEPROM                    = _UxGT("EEPROM-a hasieratu");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Berriz kargatu");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Pantaila info");
  LSTR MSG_PREPARE                        = _UxGT("Prestatu");
  LSTR MSG_TUNE                           = _UxGT("Doitu");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Pausatu inprimak.");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Jarraitu inprima.");
  LSTR MSG_STOP_PRINT                     = _UxGT("Gelditu inprima.");
  LSTR MSG_MEDIA_MENU                     = _UxGT("SD-tik inprimatu");
  LSTR MSG_NO_MEDIA                       = _UxGT("Ez dago SD-rik");
  LSTR MSG_DWELL                          = _UxGT("Lo egin...");
  LSTR MSG_USERWAIT                       = _UxGT("Aginduak zain...");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Inprim. geldi.");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Inprim. deusezta.");
  LSTR MSG_NO_MOVE                        = _UxGT("Mugimendu gabe.");
  LSTR MSG_KILLED                         = _UxGT("AKABATUTA. ");
  LSTR MSG_STOPPED                        = _UxGT("GELDITUTA. ");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Atzera egin mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Swap Atzera mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Atzera egin V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Igo mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Atzera egin mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Swap Atzera mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Atzera egin V");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("S UnRet V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Atzera egin");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Aldatu harizpia");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Aldatu harizpia *");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Harizpia kargatu");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Harizpia kargatu *");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Harizpia deskargatu");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Harizpia deskargatu *");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Erabat deskargatu");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Hasieratu SD-a");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Hasieratu USB-a");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Hasieratu SD-a");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Aldatu txartela");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Z zunda kanpora");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Okertze faktorea");
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("BLTouch AutoProba");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("BLTouch berrabia.");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("BLTouch jaitsi/luzatu");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("BLTouch igo/jaso");
  LSTR MSG_HOME_FIRST                     = _UxGT("Etxera %s lehenengo");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("X Konpentsatu");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Y Konpentsatu");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Z Konpentsatu");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("@ Konpentsatu");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Z pausoz egokitu");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Mikro-urratsa X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Mikro-urratsa Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Mikro-urratsa Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Mikro-urratsa @");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Endstop deusezta.");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Err: Beroketa");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Err: Tenperatura");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("TENP. KONTROL EZA");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Err: Tenp Maximoa");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Err: Tenp Minimoa");
  LSTR MSG_HALTED                         = _UxGT("INPRIMA. GELDIRIK");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Berrabia. Mesedez");
  LSTR MSG_HEATING                        = _UxGT("Berotzen...");
  LSTR MSG_BED_HEATING                    = _UxGT("Ohea Berotzen...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Delta Kalibraketa");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Kalibratu X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Kalibratu Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Kalibratu Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Kalibratu Zentrua");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Delta ezarpenak");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Auto Kalibraketa");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Barra diagonala");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Altuera");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Erradioa");
  LSTR MSG_INFO_MENU                      = _UxGT("Inprimagailu Inf.");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Inprimagailu Inf.");
  LSTR MSG_3POINT_LEVELING                = _UxGT("3 puntuko berdinketa");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Berdinketa lineala");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("Berdinketa bilinearra");
  LSTR MSG_UBL_LEVELING                   = _UxGT("Unified Bed Leveling");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Sare berdinketa");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Inprima. estatis.");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Txartelaren Info.");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termistoreak");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Estrusoreak");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Baudioak");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protokoloa");
  LSTR MSG_CASE_LIGHT                     = _UxGT("Kabina Argia");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Argiaren Distira");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Inprimatze");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Burututa");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Guztira");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Luzeena");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Estrusio");

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Tenp. Minimoa");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Tenp. Maximoa");
  LSTR MSG_INFO_PSU                       = _UxGT("Elikadura-iturria");
  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Driver-aren potentzia");
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ Driver %");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("Idatzi DAC EEPROM");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("HARIZPIA ALDATU");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("HARIZPIA KARGATU");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("HARIZPIA DESKARGATU");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("ALDAKETA AUKERAK:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Inprima. jarraitu");
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Pita: ");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Hasi. huts egin du");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Neurketak huts egin du");

  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("Inprimagailu okerra");

  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Mesedez, itxaron..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Deskargatzen..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Sartu eta click egin"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Berotzen..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Kargatzen..."));
}

namespace LanguageWide_eu {
  using namespace LanguageNarrow_eu;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Inprim. Zenbaketa");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Burututa");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Inprim. denbora");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Imprimatze luzeena");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Estruituta guztira");
  #endif
}

namespace LanguageTall_eu {
  using namespace LanguageWide_eu;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
  #endif
}

namespace Language_eu {
  using namespace LanguageTall_eu;
}
