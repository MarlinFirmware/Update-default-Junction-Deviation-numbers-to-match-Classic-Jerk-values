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
 * Bulgarian
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#define DISPLAY_CHARSET_ISO10646_5

namespace LanguageNarrow_bg {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Bulgarian");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" Готов.");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Картата е поставена");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Картата е извадена");
  LSTR MSG_MAIN_MENU                      = _UxGT("Меню");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Автостарт");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Изкл. двигатели");
  LSTR MSG_AUTO_HOME                      = _UxGT("Паркиране");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Задай Начало");

  LSTR MSG_PREHEAT_1                      = _UxGT("Подгряване ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Подгряване ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Подгряване ") PREHEAT_1_LABEL _UxGT(" Дюза");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Подгряване ") PREHEAT_1_LABEL _UxGT(" Дюза ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Подгр. ") PREHEAT_1_LABEL _UxGT(" Всички");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Подгр. ") PREHEAT_1_LABEL _UxGT(" Легло");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Настройки ") PREHEAT_1_LABEL;

  LSTR MSG_PREHEAT_M                      = _UxGT("Подгряване $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Подгряване $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Подгряване $ Дюза");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Подгряване $ Дюза ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Подгр. $ Всички");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Подгр. $ Легло");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Настройки $");

  LSTR MSG_COOLDOWN                       = _UxGT("Охлаждане");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Вкл. захранване");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Изкл. захранване");
  LSTR MSG_EXTRUDE                        = _UxGT("Екструзия");
  LSTR MSG_RETRACT                        = _UxGT("Откат");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Движение по ос");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Нивелиране");
  LSTR MSG_BED_LEVELING                   = _UxGT("Нивелиране");
  LSTR MSG_LEVEL_BED                      = _UxGT("Нивелиране");
  LSTR MSG_MOVE_X                         = _UxGT("Движение по X");
  LSTR MSG_MOVE_Y                         = _UxGT("Движение по Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Движение по Z");
  LSTR MSG_MOVE_N                         = _UxGT("Движение по @");
  LSTR MSG_MOVE_E                         = _UxGT("Екструдер");
  LSTR MSG_MOVE_EN                        = _UxGT("Екструдер *");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Премести с $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Премести с $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Премести с $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Скорост");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Bed Z");
  LSTR MSG_NOZZLE                         = " " LCD_STR_THERMOMETER _UxGT(" Дюза");
  LSTR MSG_NOZZLE_N                       = " " LCD_STR_THERMOMETER _UxGT(" Дюза ~");
  LSTR MSG_BED                            = " " LCD_STR_THERMOMETER _UxGT(" Легло");
  LSTR MSG_FAN_SPEED                      = _UxGT("Вентилатор");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Вентилатор ~");
  LSTR MSG_FLOW                           = _UxGT("Поток");
  LSTR MSG_FLOW_N                         = _UxGT("Поток ~");
  LSTR MSG_CONTROL                        = _UxGT("Управление");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Минимум");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Максимум");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Фактор");
  LSTR MSG_AUTOTEMP                       = _UxGT("Авто-темп.");
  LSTR MSG_LCD_ON                         = _UxGT("Вкл.");
  LSTR MSG_LCD_OFF                        = _UxGT("Изкл.");
  LSTR MSG_A_RETRACT                      = _UxGT("A-откат");
  LSTR MSG_A_TRAVEL                       = _UxGT("A-travel");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Стъпки/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" стъпки/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" стъпки/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" стъпки/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ стъпки/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E стъпки/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* стъпки/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Температура");
  LSTR MSG_MOTION                         = _UxGT("Движение");
  LSTR MSG_FILAMENT                       = _UxGT("Нишка");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E in mm") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Диам. нишка");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Диам. нишка *");
  LSTR MSG_CONTRAST                       = _UxGT("LCD контраст");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Запази в EPROM");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Зареди от EPROM");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Фабрични настройки");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Обнови");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Преглед");
  LSTR MSG_PREPARE                        = _UxGT("Действия");
  LSTR MSG_TUNE                           = _UxGT("Настройка");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Пауза");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Възобнови печата");
  LSTR MSG_STOP_PRINT                     = _UxGT("Спри печата");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Меню карта");
  LSTR MSG_NO_MEDIA                       = _UxGT("Няма карта");
  LSTR MSG_DWELL                          = _UxGT("Почивка...");
  LSTR MSG_USERWAIT                       = _UxGT("Изчакване");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Печатът е прекъснат");
  LSTR MSG_NO_MOVE                        = _UxGT("Няма движение");
  LSTR MSG_KILLED                         = _UxGT("УБИТО.");
  LSTR MSG_STOPPED                        = _UxGT("СПРЯНО.");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Откат mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Смяна Откат mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Откат  V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Скок mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Възврат mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Смяна Възврат mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Възврат  V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Автоoткат");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Смяна нишка");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Смяна нишка *");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Иниц. SD-Карта");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Иниц. USB-Карта");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Иниц. SD-Карта");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Смяна SD-Карта");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Z-сондата е извадена");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("X Отстояние");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Y Отстояние");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Z Отстояние");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("@ Отстояние");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Промяна на Z");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Министъпка X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Министъпка Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Министъпка Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Министъпка @");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Стоп Кр.Изключватели");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Делта Калибровка");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Калибровка X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Калибровка Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Калибровка Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Калибровка Център");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("Неправилен принтер");
}

namespace LanguageWide_bg {
  using namespace LanguageNarrow_bg;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
  #endif
}

namespace LanguageTall_bg {
  using namespace LanguageWide_bg;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
  #endif
}

namespace Language_bg {
  using namespace LanguageTall_bg;
}
