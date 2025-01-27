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
 * Japanese (Kana)
 * UTF-8 for Graphical Display
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

//#define DISPLAY_CHARSET_ISO10646_KANA

namespace LanguageNarrow_jp_kana {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 3;
  LSTR LANGUAGE                           = _UxGT("Japanese");

  // This is just to show the potential benefit of Unicode.
  // This translation can be improved by using the full charset of unicode codeblock U+30A0 to U+30FF.

  // 片仮名表示定義
  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" ジュンビカンリョウ");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("メディアガソウニュウサレマシタ"); // "Card inserted"
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("メディアガアリマセン"); // "Card removed"
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("メディアノトリダシ");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("エンドストップ"); // "Endstops" // Max length 8 characters
  LSTR MSG_MAIN_MENU                      = _UxGT("メイン"); // "Main"
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("ジドウカイシ"); // "Autostart"
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("モーターデンゲン オフ"); // "Disable steppers"
  LSTR MSG_DEBUG_MENU                     = _UxGT("デバッグメニュー"); // "Debug Menu"
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("プログレスバー テスト"); // "Progress Bar Test"
  LSTR MSG_AUTO_HOME                      = _UxGT("ゲンテンフッキ"); // "Auto home"
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Xジク ゲンテンフッキ"); // "Home X"
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Yジク ゲンテンフッキ"); // "Home Y"
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Zジク ゲンテンフッキ"); // "Home Z"
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("ゲンテンフッキチュウ"); // "Homing XYZ"
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("レベリングカイシ"); // "Click to Begin"
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("ツギノソクテイテンヘ"); // "Next Point"
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("レベリングカンリョウ"); // "Leveling Done!"
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("キジュンオフセットセッテイ"); // "Set home offsets"
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("オフセットガテキヨウサレマシタ"); // "Offsets applied"

  LSTR MSG_PREHEAT_1                      = PREHEAT_1_LABEL _UxGT(" ヨネツ"); // "Preheat " PREHEAT_1_LABEL
  LSTR MSG_PREHEAT_1_H                    = PREHEAT_1_LABEL _UxGT(" ヨネツ ~"); // "Preheat " PREHEAT_1_LABEL
  LSTR MSG_PREHEAT_1_END                  = PREHEAT_1_LABEL _UxGT(" ヨネツノズル"); // " Nozzle"
  LSTR MSG_PREHEAT_1_END_E                = PREHEAT_1_LABEL _UxGT(" ヨネツノズル ~"); // " Nozzle"
  LSTR MSG_PREHEAT_1_ALL                  = PREHEAT_1_LABEL _UxGT(" スベテヨネツ"); // " All"
  LSTR MSG_PREHEAT_1_BEDONLY              = PREHEAT_1_LABEL _UxGT(" ベッドヨネツ"); // " Bed"
  LSTR MSG_PREHEAT_1_SETTINGS             = PREHEAT_1_LABEL _UxGT(" ヨネツセッテイ"); // " conf"

  LSTR MSG_PREHEAT_M                      = _UxGT("$ ヨネツ"); // "Preheat " PREHEAT_1_LABEL
  LSTR MSG_PREHEAT_M_H                    = _UxGT("$ ヨネツ ~"); // "Preheat " PREHEAT_1_LABEL
  LSTR MSG_PREHEAT_M_END                  = _UxGT("$ ヨネツノズル"); // " Nozzle"
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("$ ヨネツノズル ~"); // " Nozzle"
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("$ スベテヨネツ"); // " All"
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("$ ベッドヨネツ"); // " Bed"
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("$ ヨネツセッテイ"); // " conf"

  LSTR MSG_COOLDOWN                       = _UxGT("カネツテイシ"); // "Cooldown"
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("デンゲン オン"); // "Switch power on"
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("デンゲン オフ"); // "Switch power off"
  LSTR MSG_EXTRUDE                        = _UxGT("オシダシ"); // "Extrude"
  LSTR MSG_RETRACT                        = _UxGT("ヒキコミセッテイ"); // "Retract"
  LSTR MSG_MOVE_AXIS                      = _UxGT("ジクイドウ"); // "Move axis"
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("プローブ アンド レベリング"); // "Probe and Level"
  LSTR MSG_BED_LEVELING                   = _UxGT("ベッドレベリング"); // "Bed leveling"
  LSTR MSG_LEVEL_BED                      = _UxGT("ベッドレベリング"); // "Level bed"

  LSTR MSG_MOVING                         = _UxGT("イドウチュウ"); // "Moving..."
  LSTR MSG_FREE_XY                        = _UxGT("XYジク カイホウ"); // "Free XY"
  LSTR MSG_MOVE_X                         = _UxGT("Xジク イドウ"); // "Move X"
  LSTR MSG_MOVE_Y                         = _UxGT("Yジク イドウ"); // "Move Y"
  LSTR MSG_MOVE_Z                         = _UxGT("Zジク イドウ"); // "Move Z"
  LSTR MSG_MOVE_N                         = _UxGT("@ジク イドウ"); // "Move @"
  LSTR MSG_MOVE_E                         = _UxGT("エクストルーダー"); // "Extruder"
  LSTR MSG_MOVE_EN                        = _UxGT("* エクストルーダー"); // "En"
  LSTR MSG_MOVE_N_MM                      = _UxGT("$mm イドウ"); // "Move $mm"
  LSTR MSG_MOVE_N_IN                      = _UxGT("$in イドウ"); // "Move $in"
  LSTR MSG_MOVE_N_DEG                     = _UxGT("$" LCD_STR_DEGREE " イドウ"); // "Move $°"
  LSTR MSG_SPEED                          = _UxGT("ソクド"); // "Speed"
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Zオフセット"); // "Bed Z"
  LSTR MSG_NOZZLE                         = _UxGT("ノズル"); // "Nozzle"
  LSTR MSG_BED                            = _UxGT("ベッド"); // "Bed"
  LSTR MSG_FAN_SPEED                      = _UxGT("ファンソクド"); // "Fan speed"
  LSTR MSG_FLOW                           = _UxGT("トシュツリョウ"); // "Flow"
  LSTR MSG_CONTROL                        = _UxGT("セイギョ"); // "Control"
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" サイテイ"); // " Min"
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" サイコウ"); // " Max"
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" ファクター"); // " Fact"
  LSTR MSG_AUTOTEMP                       = _UxGT("ジドウオンドセイギョ"); // "Autotemp"
  LSTR MSG_LCD_ON                         = _UxGT("オン"); // "On"
  LSTR MSG_LCD_OFF                        = _UxGT("オフ"); // "Off"
  LSTR MSG_SELECT_E                       = _UxGT("センタク *");
  LSTR MSG_ACC                            = _UxGT("カソクド mm/s") SUPERSCRIPT_TWO; // "Accel"
  LSTR MSG_JERK                           = _UxGT("ヤクドウ mm/s"); // "Jerk"
  LSTR MSG_VA_JERK                        = _UxGT("ジク ヤクドウ mm/s") STR_A; // "Va-jerk"
  LSTR MSG_VB_JERK                        = _UxGT("ジク ヤクドウ mm/s") STR_B; // "Vb-jerk"
  LSTR MSG_VC_JERK                        = _UxGT("ジク ヤクドウ mm/s") STR_C; // "Vc-jerk"
  LSTR MSG_VN_JERK                        = _UxGT("ジク ヤクドウ mm/s@"); // "V@-jerk"
  LSTR MSG_A_STEPS                        = STR_A _UxGT("ステップ/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT("ステップ/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT("ステップ/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ステップ/mm");
  LSTR MSG_VE_JERK                        = _UxGT("エクストルーダー ヤクド"); // "Ve-jerk"
  LSTR MSG_VMAX_A                         = _UxGT("サイダイオクリソクド ") STR_A; // "Vmax A"
  LSTR MSG_VMAX_B                         = _UxGT("サイダイオクリソクド ") STR_B; // "Vmax B"
  LSTR MSG_VMAX_C                         = _UxGT("サイダイオクリソクド ") STR_C; // "Vmax C"
  LSTR MSG_VMAX_E                         = _UxGT("サイダイオクリソクド E"); // "Vmax E"
  LSTR MSG_VMAX_EN                        = _UxGT("サイダイオクリソクド *"); // "Vmax E1"
  LSTR MSG_VMIN                           = _UxGT("サイショウオクリソクド"); // "Vmin"
  LSTR MSG_VTRAV_MIN                      = _UxGT("サイショウイドウソクド"); // "VTrav min"
  LSTR MSG_ACCELERATION                   = _UxGT("カソクド mm/s") SUPERSCRIPT_TWO; // "Accel"
  LSTR MSG_A_RETRACT                      = _UxGT("ヒキコミカソクド"); // "A-retract"
  LSTR MSG_A_TRAVEL                       = _UxGT("イドウカソクド"); // "A-travel"
  LSTR MSG_TEMPERATURE                    = _UxGT("オンド"); // "Temperature"
  LSTR MSG_MOTION                         = _UxGT("ウゴキセッテイ"); // "Motion"
  LSTR MSG_FILAMENT                       = _UxGT("フィラメント"); // "Filament"
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E in mm") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("フィラメントチョッケイ"); // "Fil. Dia."
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("フィラメントチョッケイ *");
  LSTR MSG_CONTRAST                       = _UxGT("LCDコントラスト"); // "LCD contrast"
  LSTR MSG_STORE_EEPROM                   = _UxGT("EEPROMヘホゾン"); // "Store memory"
  LSTR MSG_LOAD_EEPROM                    = _UxGT("EEPROMカラヨミコミ"); // "Load memory"
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("セッテイリセット"); // "Restore Defaults"
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("リフレッシュ"); // "Refresh"
  LSTR MSG_INFO_SCREEN                    = _UxGT("ジョウホウガメン"); // "Info screen"
  LSTR MSG_PREPARE                        = _UxGT("ジュンビセッテイ"); // "Prepare"
  LSTR MSG_TUNE                           = _UxGT("チョウセイ"); // "Tune"
  LSTR MSG_PAUSE_PRINT                    = _UxGT("イチジテイシ"); // "Pause print"
  LSTR MSG_RESUME_PRINT                   = _UxGT("プリントサイカイ"); // "Resume print"
  LSTR MSG_STOP_PRINT                     = _UxGT("プリントテイシ"); // "Stop print"
  LSTR MSG_MEDIA_MENU                     = _UxGT("メディアカラプリント"); // "Print from SD"
  LSTR MSG_NO_MEDIA                       = _UxGT("メディアガアリマセン"); // "Card removed"
  LSTR MSG_DWELL                          = _UxGT("キュウシ"); // "Sleep..."
  LSTR MSG_USERWAIT                       = _UxGT("ツヅケルニハクリックシテクダサイ"); // "Wait for user..."
  LSTR MSG_PRINT_ABORTED                  = _UxGT("プリントガチュウシサレマシタ"); // "Print aborted"
  LSTR MSG_NO_MOVE                        = _UxGT("ウゴキマセン"); // "No move."
  LSTR MSG_KILLED                         = _UxGT("ヒジョウテイシ"); // "KILLED. "
  LSTR MSG_STOPPED                        = _UxGT("テイシシマシタ"); // "STOPPED. "
  LSTR MSG_CONTROL_RETRACT                = _UxGT("ヒキコミリョウ mm"); // "Retract mm"
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("ヒキコミリョウS mm"); // "Swap Re.mm"
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("ヒキコミソクド mm/s"); // "Retract V"
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("ノズルタイヒ mm"); // "Hop mm"
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("ホショウリョウ mm"); // "Unretr. mm"
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("ホショウリョウS mm"); // "S Unretr. mm"
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("ホショウソクド mm/s"); // "Unretract V"
  LSTR MSG_AUTORETRACT                    = _UxGT("ジドウヒキコミ"); // "Auto-Retract"
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("フィラメントコウカン"); // "Change filament"
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("SDカードをマウントする"); // "Attach SD Card"
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("USBメモリをマウントする"); // "Attach USB Drive"
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("SDカードをマウントする"); // "Attach SD Card"
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("メディアコウカン"); // "Change SD card"
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Zプローブ ベッドガイ"); // "Z probe out. bed"
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("BLTouch ジコシンダン"); // "BLTouch Self-Test"
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("BLTouch リセット"); // "Reset BLTouch"
  LSTR MSG_HOME_FIRST                     = _UxGT("サキニ %s ヲフッキサセテクダサイ"); // "Home ... first"
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Xオフセット"); // "X Offset"
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Yオフセット"); // "Y Offset"
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Zオフセット"); // "Z Offset"
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("@オフセット"); // "@ Offset"
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("プローブZオフセット"); // "Babystep Probe Z"
  LSTR MSG_BABYSTEP_X                     = _UxGT("Xジク ビドウ"); // "Babystep X"
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Yジク ビドウ"); // "Babystep Y"
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Zジク ビドウ"); // "Babystep Z"
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("イドウゲンカイケンチキノウ"); // "Endstop abort"
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("カネツシッパイ"); // "Heating failed"
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("エラー:ジョウチョウサーミスターキノウ"); // "Err: REDUNDANT TEMP"
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("ネツボウソウ"); // "THERMAL RUNAWAY"
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("エラー:サイコウオンチョウカ"); // "Err: MAXTEMP"
  LSTR MSG_ERR_MINTEMP                    = _UxGT("エラー:サイテイオンミマン"); // "Err: MINTEMP"
  LSTR MSG_HALTED                         = _UxGT("プリンターハテイシシマシタ"); // "PRINTER HALTED"
  LSTR MSG_PLEASE_RESET                   = _UxGT("リセットシテクダサイ"); // "Please reset"
  LSTR MSG_HEATING                        = _UxGT("カネツチュウ"); // "Heating..."
  LSTR MSG_BED_HEATING                    = _UxGT("ベッド カネツチュウ"); // "Bed Heating..."
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("デルタ コウセイ"); // "Delta Calibration"
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Xジク コウセイ"); // "Calibrate X"
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Yジク コウセイ"); // "Calibrate Y"
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Zジク コウセイ"); // "Calibrate Z"
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("チュウシン コウセイ"); // "Calibrate Center"
  LSTR MSG_INFO_MENU                      = _UxGT("コノプリンターニツイテ"); // "About Printer"
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("プリンタージョウホウ"); // "Printer Info"
  LSTR MSG_INFO_STATS_MENU                = _UxGT("プリントジョウキョウ"); // "Printer Stats"
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("セイギョケイジョウホウ"); // "Board Info"
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("サーミスター"); // "Thermistors"
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("エクストルーダースウ"); // "Extruders"
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("ボーレート"); // "Baud"
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("プロトコル"); // "Protocol"
  LSTR MSG_CASE_LIGHT                     = _UxGT("キョウタイナイショウメイ"); // "Case light"
  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("プリントスウ "); // "Print Count"
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("カンリョウスウ"); // "Completed"
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("プリントジカンルイケイ"); // "Total print time"
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("サイチョウプリントジカン"); // "Longest job time"
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("フィラメントシヨウリョウルイケイ"); // "Extruded total"
  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("セッテイサイテイオン"); // "Min Temp"
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("セッテイサイコウオン"); // "Max Temp"
  LSTR MSG_INFO_PSU                       = _UxGT("デンゲンシュベツ"); // "Power Supply"
  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("モータークドウリョク"); // "Drive Strength"
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ DACシュツリョク %"); // "@ Driver %"
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("EEPROMヘホゾン"); // "Store memory"
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("イチジテイシ");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("プリントサイカイ"); // "Resume print"
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_2_LINE("コウカンヲカイシシマス", "シバラクオマチクダサイ")); // "Wait for start of the filament"
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_2_LINE("フィラメントヌキダシチュウ", "シバラクオマチクダサイ")); // "Wait for filament unload"
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_2_LINE("フィラメントヲソウニュウシ,", "クリックスルトゾッコウシマス")); // "Insert filament and press button"
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_2_LINE("フィラメントソウテンチュウ", "シバラクオマチクダサイ")); // "Wait for filament load"
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_2_LINE("プリントヲサイカイシマス", "シバラクオマチクダサイ")); // "Wait for print to resume"

  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("マチガッタプリンター"); // "Wrong printer"

  LSTR MSG_CONFIGURATION                  = _UxGT("セッテイカンリ");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("ショウサイセッテイ");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("コショカイフク");
  LSTR MSG_INIT_EEPROM                    = _UxGT("EEPROMショキカ");

  LSTR MSG_BUTTON_NEXT                    = _UxGT("ツギヘ");
  LSTR MSG_BUTTON_INIT                    = _UxGT("ショキカ");
  LSTR MSG_BUTTON_STOP                    = _UxGT("ストップ");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("プリント");
  LSTR MSG_BUTTON_RESET                   = _UxGT("リセット");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("キャンセル");
  LSTR MSG_BUTTON_DONE                    = _UxGT("カンリョウ");

  LSTR MSG_YES                            = _UxGT("ハイ");
  LSTR MSG_NO                             = _UxGT("イイエ");
  LSTR MSG_BACK                           = _UxGT("モドリ");
  LSTR MSG_MAX_SPEED                      = _UxGT("ソクド");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("ステップ/mm");
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("ユーザーコマンド");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("プリントガイチジテイシサレマシタ");
  LSTR MSG_PRINTING                       = _UxGT("プリントチュウ...");
}

namespace LanguageWide_jp_kana {
  using namespace LanguageNarrow_jp_kana;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
  #endif
}

namespace LanguageTall_jp_kana {
  using namespace LanguageWide_jp_kana;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
  #endif
}

namespace Language_jp_kana {
  using namespace LanguageTall_jp_kana;
}
