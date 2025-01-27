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
 * Ukrainian
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#define DISPLAY_CHARSET_ISO10646_5

namespace LanguageNarrow_uk {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                = 2;
  LSTR LANGUAGE                             = _UxGT("Ukrainian");

  LSTR WELCOME_MSG                          = MACHINE_NAME _UxGT(" Готовий.");
  LSTR MSG_YES                              = _UxGT("ТАК");
  LSTR MSG_NO                               = _UxGT("НІ");
  LSTR MSG_BACK                             = _UxGT("Назад");
  LSTR MSG_MEDIA_ABORTING                   = _UxGT("Переривання...");
  LSTR MSG_MEDIA_INSERTED                   = _UxGT("SD-картка вставлена");
  LSTR MSG_MEDIA_REMOVED                    = _UxGT("SD-картка видалена");
  LSTR MSG_MEDIA_WAITING                    = _UxGT("Вставте SD-картку");
  LSTR MSG_MEDIA_INIT_FAIL                  = _UxGT("Збій ініціаліз. SD");
  LSTR MSG_MEDIA_READ_ERROR                 = _UxGT("Помилка зчитування");
  LSTR MSG_MEDIA_USB_REMOVED                = _UxGT("USB диск видалений");
  LSTR MSG_MEDIA_USB_FAILED                 = _UxGT("Помилка USB диску");
  LSTR MSG_KILL_SUBCALL_OVERFLOW            = _UxGT("Переповн. виклику");
  LSTR MSG_LCD_SOFT_ENDSTOPS                = _UxGT("Прогр.кінцевики");
  LSTR MSG_LCD_ENDSTOPS                     = _UxGT("Кінцевик"); // Max length 8 characters
  LSTR MSG_MAIN_MENU                        = _UxGT("Основне меню");
  LSTR MSG_ADVANCED_SETTINGS                = _UxGT("Інші налаштування");
  LSTR MSG_CONFIGURATION                    = _UxGT("Конфігурація");
  LSTR MSG_RUN_AUTO_FILES                   = _UxGT("Автостарт");
  LSTR MSG_DISABLE_STEPPERS                 = _UxGT("Вимкнути двигуни");
  LSTR MSG_DEBUG_MENU                       = _UxGT("Меню Debug");
  LSTR MSG_PROGRESS_BAR_TEST                = _UxGT("Тест лінії прогр.");
  LSTR MSG_AUTO_HOME                        = _UxGT("Авто паркування");
  LSTR MSG_AUTO_HOME_N                      = _UxGT("Паркування @");
  LSTR MSG_AUTO_HOME_X                      = _UxGT("Паркування X");
  LSTR MSG_AUTO_HOME_Y                      = _UxGT("Паркування Y");
  LSTR MSG_AUTO_HOME_Z                      = _UxGT("Паркування Z");
  LSTR MSG_AUTO_Z_ALIGN                     = _UxGT("Авто Z-вирівнювання");
  LSTR MSG_ITERATION                        = _UxGT("G34 Ітерація: %i");
  LSTR MSG_DECREASING_ACCURACY              = _UxGT("Зменьшення точності!");
  LSTR MSG_ACCURACY_ACHIEVED                = _UxGT("Точність досягнута");
  LSTR MSG_LEVEL_BED_HOMING                 = _UxGT("Паркування XYZ");
  LSTR MSG_LEVEL_BED_WAITING                = _UxGT("Почати");
  LSTR MSG_LEVEL_BED_NEXT_POINT             = _UxGT("Наступна точка");
  LSTR MSG_LEVEL_BED_DONE                   = _UxGT("Завершено!");
  LSTR MSG_Z_FADE_HEIGHT                    = _UxGT("Висота спаду");
  LSTR MSG_SET_HOME_OFFSETS                 = _UxGT("Встан. зміщ. дому");
  LSTR MSG_HOME_OFFSETS_APPLIED             = _UxGT("Зміщення прийняті");
  LSTR MSG_SELECT_ORIGIN                    = _UxGT("Оберіть нуль");
  LSTR MSG_LAST_VALUE_SP                    = _UxGT("Останнє знач. ");

  LSTR MSG_PREHEAT_1                        = _UxGT("Нагрів ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                      = _UxGT("Нагрів ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                    = _UxGT("Нагрів ") PREHEAT_1_LABEL _UxGT(" сопло");
  LSTR MSG_PREHEAT_1_END_E                  = _UxGT("Нагрів ") PREHEAT_1_LABEL _UxGT(" сопло ~");
  LSTR MSG_PREHEAT_1_ALL                    = _UxGT("Нагрів ") PREHEAT_1_LABEL _UxGT(" все");
  LSTR MSG_PREHEAT_1_BEDONLY                = _UxGT("Нагрів ") PREHEAT_1_LABEL _UxGT(" стіл");
  LSTR MSG_PREHEAT_1_SETTINGS               = _UxGT("Нагрів ") PREHEAT_1_LABEL _UxGT(" налашт");

  LSTR MSG_PREHEAT_M                        = _UxGT("Нагрів $");
  LSTR MSG_PREHEAT_M_H                      = _UxGT("Нагрів $ ~");
  LSTR MSG_PREHEAT_M_END                    = _UxGT("Нагрів $ сопло");
  LSTR MSG_PREHEAT_M_END_E                  = _UxGT("Нагрів $ сопло ~");
  LSTR MSG_PREHEAT_M_ALL                    = _UxGT("Нагрів $ все");
  LSTR MSG_PREHEAT_M_BEDONLY                = _UxGT("Нагрів $ стіл");
  LSTR MSG_PREHEAT_M_SETTINGS               = _UxGT("Нагрів $ налашт");

  LSTR MSG_PREHEAT_CUSTOM                   = _UxGT("Нагрів свого");
  LSTR MSG_COOLDOWN                         = _UxGT("Вимкнути нагрів");

  LSTR MSG_CUTTER_FREQUENCY                 = _UxGT("Частота");
  LSTR MSG_LASER_MENU                       = _UxGT("Керування лазером");
  LSTR MSG_SPINDLE_MENU                     = _UxGT("Керування шпінделем");
  LSTR MSG_LASER_POWER                      = _UxGT("Потужн. лазера");
  LSTR MSG_SPINDLE_TOGGLE                   = _UxGT("Перемк. шпінд.");
  LSTR MSG_SPINDLE_EVAC_TOGGLE              = _UxGT("Перемк. вакуум");
  LSTR MSG_LASER_TOGGLE                     = _UxGT("Перемкн. лазер");
  LSTR MSG_SPINDLE_POWER                    = _UxGT("Потужн. шпінд.");
  LSTR MSG_LASER_PULSE_MS                   = _UxGT("Тест. імп., мс");
  LSTR MSG_LASER_EVAC_TOGGLE                = _UxGT("Перемкн. обдув");
  LSTR MSG_LASER_ASSIST_TOGGLE              = _UxGT("Керування обдувом");
  LSTR MSG_FLOWMETER_FAULT                  = _UxGT("Помилка обдуву");
  LSTR MSG_LASER_FIRE_PULSE                 = _UxGT("Імпульс лазеру");
  LSTR MSG_SPINDLE_FORWARD                  = _UxGT("Шпіндель вперед");
  LSTR MSG_SPINDLE_REVERSE                  = _UxGT("Шпіндель назад");

  LSTR MSG_SWITCH_PS_ON                     = _UxGT("Увімкнути живлення");
  LSTR MSG_SWITCH_PS_OFF                    = _UxGT("Вимкнути живлення");
  LSTR MSG_EXTRUDE                          = _UxGT("Екструзія");
  LSTR MSG_RETRACT                          = _UxGT("Втягування");
  LSTR MSG_MOVE_AXIS                        = _UxGT("Рух по осям");
  LSTR MSG_PROBE_AND_LEVEL                  = _UxGT("Проба/вирівнювання");
  LSTR MSG_BED_LEVELING                     = _UxGT("Вирівнювання столу");
  LSTR MSG_LEVEL_BED                        = _UxGT("Вирівняти стіл");
  LSTR MSG_BED_TRAMMING                     = _UxGT("Вирівняти кути");
  LSTR MSG_BED_TRAMMING_RAISE               = _UxGT("Вгору до спрац.зонду");
  LSTR MSG_BED_TRAMMING_IN_RANGE            = _UxGT("Кути в межах. Вирівн");
  LSTR MSG_BED_TRAMMING_GOOD_POINTS         = _UxGT("Хороші точки: ");
  LSTR MSG_BED_TRAMMING_LAST_Z              = _UxGT("Остання Z: ");
  LSTR MSG_NEXT_CORNER                      = _UxGT("Наступний кут");
  LSTR MSG_MESH_EDITOR                      = _UxGT("Зміщення Z");
  LSTR MSG_EDIT_MESH                        = _UxGT("Редагувати сітку");
  LSTR MSG_EDITING_STOPPED                  = _UxGT("Редагув. зупинено");
  LSTR MSG_PROBING_POINT                    = _UxGT("Точка сітки");
  LSTR MSG_MESH_X                           = _UxGT("Індекс X");
  LSTR MSG_MESH_Y                           = _UxGT("Індекс Y");
  LSTR MSG_MESH_EDIT_Z                      = _UxGT("Значення Z");
  LSTR MSG_CUSTOM_COMMANDS                  = _UxGT("Власні команди");

  LSTR MSG_M48_TEST                         = _UxGT("M48 тест зонду");
  LSTR MSG_M48_POINT                        = _UxGT("M48 точка");
  LSTR MSG_M48_OUT_OF_BOUNDS                = _UxGT("Зонд за межами");
  LSTR MSG_M48_DEVIATION                    = _UxGT("Відхилення");

  LSTR MSG_IDEX_MENU                        = _UxGT("Режим IDEX");
  LSTR MSG_OFFSETS_MENU                     = _UxGT("Зміщення сопел");
  LSTR MSG_IDEX_MODE_AUTOPARK               = _UxGT("Авто паркування");
  LSTR MSG_IDEX_MODE_DUPLICATE              = _UxGT("Дуплікація");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY          = _UxGT("Дзеркальна копія");
  LSTR MSG_IDEX_MODE_FULL_CTRL              = _UxGT("Повний контроль");
  LSTR MSG_IDEX_DUPE_GAP                    = _UxGT("Дублюв. X-проміжок");

  LSTR MSG_HOTEND_OFFSET_Z                  = _UxGT("Друге сопло Z");
  LSTR MSG_HOTEND_OFFSET_N                  = _UxGT("Друге сопло @");

  LSTR MSG_UBL_DOING_G29                    = _UxGT("Виконується G29");
  LSTR MSG_UBL_TOOLS                        = _UxGT("Інструменти UBL");
  LSTR MSG_LCD_TILTING_MESH                 = _UxGT("Точка нахилу");
  LSTR MSG_UBL_MANUAL_MESH                  = _UxGT("Ручне введ. сітки");
  LSTR MSG_UBL_BC_INSERT                    = _UxGT("Розм. шайбу і вимір.");
  LSTR MSG_UBL_MESH_WIZARD                  = _UxGT("Майстер сіток UBL");
  LSTR MSG_UBL_BC_INSERT2                   = _UxGT("Вимірювання");
  LSTR MSG_UBL_BC_REMOVE                    = _UxGT("Видали і вимір. стіл");
  LSTR MSG_UBL_MOVING_TO_NEXT               = _UxGT("Рух до наступної");
  LSTR MSG_UBL_SET_TEMP_BED                 = LCD_STR_THERMOMETER _UxGT(" столу, ") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_UBL_BED_TEMP_CUSTOM              = _UxGT("Своя ") LCD_STR_THERMOMETER _UxGT(" столу,") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_UBL_SET_TEMP_HOTEND              = LCD_STR_THERMOMETER _UxGT(" сопла, ") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM           = _UxGT("Своя ") LCD_STR_THERMOMETER _UxGT(" сопла,") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH             = _UxGT("Редагувати свою");
  LSTR MSG_UBL_FINE_TUNE_MESH               = _UxGT("Точне редаг. сітки");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH            = _UxGT("Будувати свою");
  LSTR MSG_UBL_DONE_EDITING_MESH            = _UxGT("Сітка побудована");
  LSTR MSG_UBL_BUILD_MESH_MENU              = _UxGT("Будувати сітку");
  LSTR MSG_UBL_BUILD_MESH_M                 = _UxGT("Будувати сітку ($)");
  LSTR MSG_UBL_VALIDATE_MESH_M              = _UxGT("Підтвердити ($)");
  LSTR MSG_UBL_BUILD_COLD_MESH              = _UxGT("Буд. холодну сітку");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST           = _UxGT("Встан.висоту сітки");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT           = _UxGT("Висота");
  LSTR MSG_UBL_VALIDATE_MESH_MENU           = _UxGT("Підтвердити сітку");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH         = _UxGT("Підтвердити свою");

  LSTR MSG_G26_HEATING_BED                  = _UxGT("G26 нагрів столу");
  LSTR MSG_G26_HEATING_NOZZLE               = _UxGT("G26 нагрів сопла");
  LSTR MSG_G26_MANUAL_PRIME                 = _UxGT("Ручне грунтування");
  LSTR MSG_G26_FIXED_LENGTH                 = _UxGT("Фікс. довж. грунт.");
  LSTR MSG_G26_PRIME_DONE                   = _UxGT("Грунтув. виконане");
  LSTR MSG_G26_CANCELED                     = _UxGT("G26 скасовано");
  LSTR MSG_G26_LEAVING                      = _UxGT("Вийти з G26");
  LSTR MSG_UBL_CONTINUE_MESH                = _UxGT("Продовжити сітку");
  LSTR MSG_UBL_MESH_LEVELING                = _UxGT("Вирівнювання сітки");
  LSTR MSG_UBL_3POINT_MESH_LEVELING         = _UxGT("3-точкове вирівн.");
  LSTR MSG_UBL_GRID_MESH_LEVELING           = _UxGT("Вирівнюв. растру");
  LSTR MSG_UBL_MESH_LEVEL                   = _UxGT("Вирівняти сітку");
  LSTR MSG_UBL_SIDE_POINTS                  = _UxGT("Крайні точки");
  LSTR MSG_UBL_MAP_TYPE                     = _UxGT("Тип мапи сітки");
  LSTR MSG_UBL_OUTPUT_MAP                   = _UxGT("Вивести мапу сітки");
  LSTR MSG_UBL_OUTPUT_MAP_HOST              = _UxGT("Вивести на хост");
  LSTR MSG_UBL_OUTPUT_MAP_CSV               = _UxGT("Вивести в CSV");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP            = _UxGT("Зберегти зовні");
  LSTR MSG_UBL_INFO_UBL                     = _UxGT("Інформація по UBL");
  LSTR MSG_UBL_FILLIN_AMOUNT                = _UxGT("Обсяг заповн.");
  LSTR MSG_UBL_MANUAL_FILLIN                = _UxGT("Ручне заповнення");
  LSTR MSG_UBL_SMART_FILLIN                 = _UxGT("Розумне заповнення");
  LSTR MSG_UBL_FILLIN_MESH                  = _UxGT("Заповнити сітку");
  LSTR MSG_UBL_INVALIDATE_ALL               = _UxGT("Анулювати все");
  LSTR MSG_UBL_INVALIDATE_CLOSEST           = _UxGT("Анулювати найближчу");
  LSTR MSG_UBL_FINE_TUNE_ALL                = _UxGT("Точно налашт. все");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST            = _UxGT("Точно найближчу");
  LSTR MSG_UBL_STORAGE_MESH_MENU            = _UxGT("Збереження сітки");
  LSTR MSG_UBL_STORAGE_SLOT                 = _UxGT("Слот пам'яті");
  LSTR MSG_UBL_LOAD_MESH                    = _UxGT("Завантажити сітку");
  LSTR MSG_UBL_SAVE_MESH                    = _UxGT("Зберегти сітку");
  LSTR MSG_MESH_LOADED                      = _UxGT("Сітка %i завантажена");
  LSTR MSG_MESH_SAVED                       = _UxGT("Сітка %i збережена");
  LSTR MSG_UBL_NO_STORAGE                   = _UxGT("Немає носія");
  LSTR MSG_UBL_SAVE_ERROR                   = _UxGT("Збій: збереж. UBL");
  LSTR MSG_UBL_RESTORE_ERROR                = _UxGT("Збій: відновл. UBL");
  LSTR MSG_UBL_Z_OFFSET                     = _UxGT("Зміщення Z: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED             = _UxGT("Зміщення Z зупинено");
  LSTR MSG_UBL_STEP_BY_STEP_MENU            = _UxGT("UBL покроково");
  LSTR MSG_UBL_1_BUILD_COLD_MESH            = _UxGT("1.Збудувати холодну");
  LSTR MSG_UBL_2_SMART_FILLIN               = _UxGT("2.Розумне заповн-я");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU         = _UxGT("3.Затвердити сітку");
  LSTR MSG_UBL_4_FINE_TUNE_ALL              = _UxGT("4.Точно налашт.все");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU         = _UxGT("5.Затвердити сітку");
  LSTR MSG_UBL_6_FINE_TUNE_ALL              = _UxGT("6.Точно налашт.все");
  LSTR MSG_UBL_7_SAVE_MESH                  = _UxGT("7.Зберегти сітку");

  LSTR MSG_LED_CONTROL                      = _UxGT("Керування світлом");
  LSTR MSG_LIGHTS                           = _UxGT("Підсвітка");
  LSTR MSG_LIGHT_N                          = _UxGT("Світло #{");
  LSTR MSG_LED_PRESETS                      = _UxGT("Передустан. світла");
  LSTR MSG_SET_LEDS_RED                     = _UxGT("Червоний");
  LSTR MSG_SET_LEDS_ORANGE                  = _UxGT("Помаранчевий");
  LSTR MSG_SET_LEDS_YELLOW                  = _UxGT("Жовтий");
  LSTR MSG_SET_LEDS_GREEN                   = _UxGT("Зелений");
  LSTR MSG_SET_LEDS_BLUE                    = _UxGT("Синій");
  LSTR MSG_SET_LEDS_INDIGO                  = _UxGT("Індіго");
  LSTR MSG_SET_LEDS_VIOLET                  = _UxGT("Фіолетовий");
  LSTR MSG_SET_LEDS_WHITE                   = _UxGT("Білий");
  LSTR MSG_SET_LEDS_DEFAULT                 = _UxGT("За умовчанням");
  LSTR MSG_LED_CHANNEL_N                    = _UxGT("Канал {");
  LSTR MSG_NEO2_PRESETS                     = _UxGT("Передуст. світла #2");
  LSTR MSG_NEO2_BRIGHTNESS                  = _UxGT("Яскравість");
  LSTR MSG_CUSTOM_LEDS                      = _UxGT("Своє світло");
  LSTR MSG_INTENSITY_R                      = _UxGT("Рівень червоного");
  LSTR MSG_INTENSITY_G                      = _UxGT("Рівень зеленого");
  LSTR MSG_INTENSITY_B                      = _UxGT("Рівень синього");
  LSTR MSG_INTENSITY_W                      = _UxGT("Рівень білого");
  LSTR MSG_LED_BRIGHTNESS                   = _UxGT("Яскравість");

  LSTR MSG_MOVING                           = _UxGT("Рух...");
  LSTR MSG_FREE_XY                          = _UxGT("Звільнити XY");
  LSTR MSG_MOVE_X                           = _UxGT("Рух по X");
  LSTR MSG_MOVE_Y                           = _UxGT("Рух по Y");
  LSTR MSG_MOVE_Z                           = _UxGT("Рух по Z");
  LSTR MSG_MOVE_N                           = _UxGT("Рух по @");
  LSTR MSG_MOVE_E                           = _UxGT("Екструдер");
  LSTR MSG_MOVE_EN                          = _UxGT("Екструдер *");
  LSTR MSG_HOTEND_TOO_COLD                  = _UxGT("Сопло дуже холодне");
  LSTR MSG_MOVE_N_MM                        = _UxGT("Рух $мм");
  LSTR MSG_MOVE_N_IN                        = _UxGT("Рух $in");
  LSTR MSG_MOVE_N_DEG                       = _UxGT("Рух $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                            = _UxGT("Швидкість");
  LSTR MSG_MESH_Z_OFFSET                    = _UxGT("Z Столу");
  LSTR MSG_NOZZLE                           = _UxGT("Сопло, ") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_NOZZLE_N                         = _UxGT("Сопло ~");
  LSTR MSG_NOZZLE_PARKED                    = _UxGT("Сопло запарковане");
  LSTR MSG_NOZZLE_STANDBY                   = _UxGT("Сопло очікує");
  LSTR MSG_BED                              = _UxGT("Стіл,  ") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_CHAMBER                          = _UxGT("Камера,") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_COOLER                           = _UxGT("Охолодж. лазеру");
  LSTR MSG_COOLER_TOGGLE                    = _UxGT("Перемк.охолод");
  LSTR MSG_FLOWMETER_SAFETY                 = _UxGT("Безпека потоку");
  LSTR MSG_LASER                            = _UxGT("Лазер");
  LSTR MSG_FAN_SPEED                        = _UxGT("Швидк. вент.");
  LSTR MSG_FAN_SPEED_N                      = _UxGT("Швидк. вент. ~");
  LSTR MSG_STORED_FAN_N                     = _UxGT("Збереж. вент. ~");
  LSTR MSG_EXTRA_FAN_SPEED_N                = _UxGT("Додат.вент. ~");
  LSTR MSG_EXTRA_FAN_SPEED                  = _UxGT("Дод. швидк. вент.");
  LSTR MSG_CONTROLLER_FAN                   = _UxGT("Вент. контролера");
  LSTR MSG_CONTROLLER_FAN_IDLE_SPEED        = _UxGT("Холості оберти");
  LSTR MSG_CONTROLLER_FAN_AUTO_ON           = _UxGT("Авто-режим");
  LSTR MSG_CONTROLLER_FAN_SPEED             = _UxGT("Робочі оберти");
  LSTR MSG_CONTROLLER_FAN_DURATION          = _UxGT("Період простою");
  LSTR MSG_FLOW                             = _UxGT("Потік");
  LSTR MSG_FLOW_N                           = _UxGT("Потік ~");
  LSTR MSG_CONTROL                          = _UxGT("Налаштування");
  LSTR MSG_MIN                              = " " LCD_STR_THERMOMETER ", " LCD_STR_DEGREE _UxGT("С  мін");
  LSTR MSG_MAX                              = " " LCD_STR_THERMOMETER ", " LCD_STR_DEGREE _UxGT("С макс");
  LSTR MSG_FACTOR                           = " " LCD_STR_THERMOMETER _UxGT(" Фактор");
  LSTR MSG_AUTOTEMP                         = _UxGT("Автотемпер.");
  LSTR MSG_LCD_ON                           = _UxGT("Увім");
  LSTR MSG_LCD_OFF                          = _UxGT("Вимк");

  LSTR MSG_PID_AUTOTUNE                     = _UxGT("Автопідбір PID");
  LSTR MSG_PID_AUTOTUNE_E                   = _UxGT("Автопідбір PID *");
  LSTR MSG_PID_AUTOTUNE_DONE                = _UxGT("Підбір PID виконано");
  LSTR MSG_PID_BAD_HEATER_ID                = _UxGT("Збій автопідбору! Поганий екструдер.");
  LSTR MSG_PID_TEMP_TOO_HIGH                = _UxGT("Збій автопідбору! Температура завищена.");
  LSTR MSG_PID_TIMEOUT                      = _UxGT("Збій автопідбору! Вичерпан час.");

  LSTR MSG_SELECT_E                         = _UxGT("Вибрати *");
  LSTR MSG_ACC                              = _UxGT("Прискорорення");
  LSTR MSG_JERK                             = _UxGT("Ривок");
  LSTR MSG_VA_JERK                          = _UxGT("V") STR_A _UxGT("-ривок");
  LSTR MSG_VB_JERK                          = _UxGT("V") STR_B _UxGT("-ривок");
  LSTR MSG_VC_JERK                          = _UxGT("V") STR_C _UxGT("-ривок");
  LSTR MSG_VN_JERK                          = _UxGT("V@-ривок");
  LSTR MSG_VE_JERK                          = _UxGT("Ve-ривок");
  LSTR MSG_JUNCTION_DEVIATION               = _UxGT("Відхил.вузла");
  LSTR MSG_MAX_SPEED                        = _UxGT("Швидкість, мм/с");
  LSTR MSG_VMAX_A                           = _UxGT("Швидк.макс ") STR_A;
  LSTR MSG_VMAX_B                           = _UxGT("Швидк.макс ") STR_B;
  LSTR MSG_VMAX_C                           = _UxGT("Швидк.макс ") STR_C;
  LSTR MSG_VMAX_N                           = _UxGT("Швидк.макс @");
  LSTR MSG_VMAX_E                           = _UxGT("Швидк.макс E");
  LSTR MSG_VMAX_EN                          = _UxGT("Швидк.макс *");
  LSTR MSG_VMIN                             = _UxGT("Швидк. мін");
  LSTR MSG_VTRAV_MIN                        = _UxGT("Переміщ. мін");
  LSTR MSG_ACCELERATION                     = _UxGT("Прискорення, мм/с2");
  LSTR MSG_AMAX_A                           = _UxGT("Приск.макс ") STR_A;
  LSTR MSG_AMAX_B                           = _UxGT("Приск.макс ") STR_B;
  LSTR MSG_AMAX_C                           = _UxGT("Приск.макс ") STR_C;
  LSTR MSG_AMAX_N                           = _UxGT("Приск.макс @");
  LSTR MSG_AMAX_E                           = _UxGT("Приск.макс E");
  LSTR MSG_AMAX_EN                          = _UxGT("Приск.макс *");
  LSTR MSG_A_RETRACT                        = _UxGT("Приск.втягув.");
  LSTR MSG_A_TRAVEL                         = _UxGT("Приск.переміщ.");
  LSTR MSG_XY_FREQUENCY_LIMIT               = _UxGT("Частота макс.");
  LSTR MSG_XY_FREQUENCY_FEEDRATE            = _UxGT("Подача мін.");
  LSTR MSG_STEPS_PER_MM                     = _UxGT("Кроків на мм");
  LSTR MSG_A_STEPS                          = STR_A _UxGT(" кроків/мм");
  LSTR MSG_B_STEPS                          = STR_B _UxGT(" кроків/мм");
  LSTR MSG_C_STEPS                          = STR_C _UxGT(" кроків/мм");
  LSTR MSG_N_STEPS                          = _UxGT("@ кроків/мм");
  LSTR MSG_E_STEPS                          = _UxGT("E кроків/мм");
  LSTR MSG_EN_STEPS                         = _UxGT("* кроків/мм");
  LSTR MSG_TEMPERATURE                      = _UxGT("Температура");
  LSTR MSG_MOTION                           = _UxGT("Рух");
  LSTR MSG_FILAMENT                         = _UxGT("Пруток");
  LSTR MSG_VOLUMETRIC_ENABLED               = _UxGT("E, мм") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT                 = _UxGT("E обмеж.,мм") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT_E               = _UxGT("E обмеж. *");
  LSTR MSG_FILAMENT_DIAM                    = _UxGT("Діам. прутка");
  LSTR MSG_FILAMENT_DIAM_E                  = _UxGT("Діам. прутка *");
  LSTR MSG_FILAMENT_UNLOAD                  = _UxGT("Вивантаж., мм");
  LSTR MSG_FILAMENT_LOAD                    = _UxGT("Завантаж., мм");
  LSTR MSG_ADVANCE_K                        = _UxGT("Kоеф. просув.");
  LSTR MSG_ADVANCE_K_E                      = _UxGT("Kоеф. просув. *");
  LSTR MSG_CONTRAST                         = _UxGT("Контраст");
  LSTR MSG_BRIGHTNESS                       = _UxGT("Яскравість");
  LSTR MSG_SCREEN_TIMEOUT                   = _UxGT("LCD Таймаут, x");
  LSTR MSG_BRIGHTNESS_OFF                   = _UxGT("Підсвітка вимк.");
  LSTR MSG_STORE_EEPROM                     = _UxGT("Зберегти в EEPROM");
  LSTR MSG_LOAD_EEPROM                      = _UxGT("Зчитати з EEPROM");
  LSTR MSG_RESTORE_DEFAULTS                 = _UxGT("На базові параметри");
  LSTR MSG_INIT_EEPROM                      = _UxGT("Ініціаліз. EEPROM");
  LSTR MSG_ERR_EEPROM_CRC                   = _UxGT("Збій EEPROM: CRC");
  LSTR MSG_ERR_EEPROM_SIZE                  = _UxGT("Збій EEPROM: розмір");
  LSTR MSG_ERR_EEPROM_VERSION               = _UxGT("Збій EEPROM: версія");
  LSTR MSG_SETTINGS_STORED                  = _UxGT("Параметри збережені");
  LSTR MSG_MEDIA_UPDATE                     = _UxGT("Оновити SD-картку");
  LSTR MSG_RESET_PRINTER                    = _UxGT("Зкинути принтер");
  LSTR MSG_REFRESH                          = LCD_STR_REFRESH _UxGT(" Поновити");
  LSTR MSG_INFO_SCREEN                      = _UxGT("Головний екран");
  LSTR MSG_PREPARE                          = _UxGT("Підготувати");
  LSTR MSG_TUNE                             = _UxGT("Підлаштування");
  LSTR MSG_POWER_MONITOR                    = _UxGT("Монітор живлення");
  LSTR MSG_CURRENT                          = _UxGT("Струм");
  LSTR MSG_VOLTAGE                          = _UxGT("Напруга");
  LSTR MSG_POWER                            = _UxGT("Потужність");
  LSTR MSG_START_PRINT                      = _UxGT("Почати друк");

  LSTR MSG_BUTTON_NEXT                      = _UxGT("Далі"); //short text for buttons
  LSTR MSG_BUTTON_INIT                      = _UxGT("Ініц-я");
  LSTR MSG_BUTTON_STOP                      = _UxGT("Зупинка");
  LSTR MSG_BUTTON_PRINT                     = _UxGT("Друк");
  LSTR MSG_BUTTON_RESET                     = _UxGT("Зкинути");
  LSTR MSG_BUTTON_IGNORE                    = _UxGT("Ігнорув.");
  LSTR MSG_BUTTON_CANCEL                    = _UxGT("Відміна");
  LSTR MSG_BUTTON_CONFIRM                   = _UxGT("Підтвер.");
  LSTR MSG_BUTTON_CONTINUE                  = _UxGT("Продовж.");
  LSTR MSG_BUTTON_DONE                      = _UxGT("Готово");
  LSTR MSG_BUTTON_BACK                      = _UxGT("Назад");
  LSTR MSG_BUTTON_PROCEED                   = _UxGT("Продовж.");
  LSTR MSG_BUTTON_SKIP                      = _UxGT("Пропуск");
  LSTR MSG_BUTTON_INFO                      = _UxGT("Інфо");
  LSTR MSG_BUTTON_LEVEL                     = _UxGT("Рівень");
  LSTR MSG_BUTTON_PAUSE                     = _UxGT("Пауза");
  LSTR MSG_BUTTON_RESUME                    = _UxGT("Звіт");
  LSTR MSG_BUTTON_SAVE                      = _UxGT("Зберегти");

  LSTR MSG_PAUSING                          = _UxGT("Призупинення...");
  LSTR MSG_PAUSE_PRINT                      = _UxGT("Призупинити друк");
  LSTR MSG_RESUME_PRINT                     = _UxGT("Відновити друк");
  LSTR MSG_HOST_START_PRINT                 = _UxGT("Старт з хосту");
  LSTR MSG_STOP_PRINT                       = _UxGT("Скасувати друк");
  LSTR MSG_END_LOOPS                        = _UxGT("End Repeat Loops"); // needs translation
  LSTR MSG_PRINTING_OBJECT                  = _UxGT("Друк об'єкта");
  LSTR MSG_CANCEL_OBJECT                    = _UxGT("Завершити об'єкт");
  LSTR MSG_CANCEL_OBJECT_N                  = _UxGT("Завершити об'єкт {");
  LSTR MSG_OUTAGE_RECOVERY                  = _UxGT("Віднов. після збою");
  LSTR MSG_MEDIA_MENU                       = _UxGT("Друкувати з SD");
  LSTR MSG_NO_MEDIA                         = _UxGT("SD-картки немає");
  LSTR MSG_DWELL                            = _UxGT("Сон...");
  LSTR MSG_USERWAIT                         = _UxGT("Продовжити...");
  LSTR MSG_PRINT_PAUSED                     = _UxGT("Друк призупинено");
  LSTR MSG_PRINTING                         = _UxGT("Друк...");
  LSTR MSG_STOPPING                         = _UxGT("Зупинка...");
  LSTR MSG_REMAINING_TIME                   = _UxGT("Залишилось");
  LSTR MSG_PRINT_ABORTED                    = _UxGT("Друк скасовано");
  LSTR MSG_PRINT_DONE                       = _UxGT("Друк завершено");
  LSTR MSG_NO_MOVE                          = _UxGT("Немає руху.");
  LSTR MSG_KILLED                           = _UxGT("ПЕРЕРВАНО. ");
  LSTR MSG_STOPPED                          = _UxGT("ЗУПИНЕНО. ");
  LSTR MSG_CONTROL_RETRACT                  = _UxGT("Втягув., мм");
  LSTR MSG_CONTROL_RETRACT_SWAP             = _UxGT("Зміна втяг.мм");
  LSTR MSG_CONTROL_RETRACT_RECOVER          = _UxGT("Поверн., мм");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP     = _UxGT("Повер.зміни,мм");
  LSTR MSG_AUTORETRACT                      = _UxGT("Автовтягув.");
  LSTR MSG_CONTROL_RETRACTF                 = _UxGT("Втягування V");
  LSTR MSG_CONTROL_RETRACT_ZHOP             = _UxGT("Підскок, мм");
  LSTR MSG_CONTROL_RETRACT_RECOVERF         = _UxGT("Повернення V");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF    = _UxGT("Поверн.зміни V");
  LSTR MSG_FILAMENT_SWAP_LENGTH             = _UxGT("Змінити довжини");
  LSTR MSG_FILAMENT_SWAP_EXTRA              = _UxGT("Змінити додатково");
  LSTR MSG_FILAMENT_PURGE_LENGTH            = _UxGT("Очистити довжину");
  LSTR MSG_TOOL_CHANGE                      = _UxGT("Зміна сопла");
  LSTR MSG_TOOL_CHANGE_ZLIFT                = _UxGT("Підняти по Z");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED         = _UxGT("Початк.швидк.");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED       = _UxGT("Швидк.втягув.");
  LSTR MSG_FILAMENT_PARK_ENABLED            = _UxGT("Паркувати голову");
  LSTR MSG_SINGLENOZZLE_UNRETRACT_SPEED     = _UxGT("Відновити швидкість");
  LSTR MSG_SINGLENOZZLE_FAN_SPEED           = _UxGT("Оберти вент.");
  LSTR MSG_SINGLENOZZLE_FAN_TIME            = _UxGT("Час вент.");
  LSTR MSG_TOOL_MIGRATION_ON                = _UxGT("Авто Увімк.");
  LSTR MSG_TOOL_MIGRATION_OFF               = _UxGT("Авто Вимкн.");
  LSTR MSG_TOOL_MIGRATION                   = _UxGT("Заміна інструменту");
  LSTR MSG_TOOL_MIGRATION_AUTO              = _UxGT("Авто заміна");
  LSTR MSG_TOOL_MIGRATION_END               = _UxGT("Останній екструдер");
  LSTR MSG_TOOL_MIGRATION_SWAP              = _UxGT("Заміна на *");
  LSTR MSG_FILAMENTCHANGE                   = _UxGT("Заміна прутка");
  LSTR MSG_FILAMENTCHANGE_E                 = _UxGT("Заміна прутка *");
  LSTR MSG_FILAMENTLOAD                     = _UxGT("Подати пруток");
  LSTR MSG_FILAMENTLOAD_E                   = _UxGT("Подати пруток *");
  LSTR MSG_FILAMENTUNLOAD                   = _UxGT("Видалити пруток");
  LSTR MSG_FILAMENTUNLOAD_E                 = _UxGT("Видалити пруток *");
  LSTR MSG_FILAMENTUNLOAD_ALL               = _UxGT("Видалити все");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA                = _UxGT("Вставити SD-картку");
    LSTR MSG_ATTACH_USB_MEDIA               = _UxGT("Вставити USB флешка");
  #else
    LSTR MSG_ATTACH_MEDIA                   = _UxGT("Вставити SD-картку");
  #endif
  LSTR MSG_CHANGE_MEDIA                     = _UxGT("Заміна SD-картки");
  LSTR MSG_RELEASE_MEDIA                    = _UxGT("Видаліть SD-картку");
  LSTR MSG_ZPROBE_OUT                       = _UxGT("Z-Зонд поза столом");
  LSTR MSG_SKEW_FACTOR                      = _UxGT("Фактор нахилу");
  LSTR MSG_BLTOUCH                          = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST                 = _UxGT("Само-тест");
  LSTR MSG_BLTOUCH_RESET                    = _UxGT("Зкинути зонд");
  LSTR MSG_BLTOUCH_STOW                     = _UxGT("Підняти зонд");
  LSTR MSG_BLTOUCH_DEPLOY                   = _UxGT("Опустити зонд");
  LSTR MSG_BLTOUCH_SW_MODE                  = _UxGT("Режим SW");
  LSTR MSG_BLTOUCH_5V_MODE                  = _UxGT("Режим 5V");
  LSTR MSG_BLTOUCH_OD_MODE                  = _UxGT("Режим OD");
  LSTR MSG_BLTOUCH_MODE_STORE               = _UxGT("Режим збереження");
  LSTR MSG_BLTOUCH_MODE_STORE_5V            = _UxGT("Встановити BLT на 5V");
  LSTR MSG_BLTOUCH_MODE_STORE_OD            = _UxGT("Встановити BLT на OD");
  LSTR MSG_BLTOUCH_MODE_ECHO                = _UxGT("Звітувати злив");
  LSTR MSG_BLTOUCH_MODE_CHANGE              = _UxGT("НЕБЕЗПЕКА: Неправильні параметри приводять до пошкоджень! Продовжити?");
  LSTR MSG_TOUCHMI_PROBE                    = _UxGT("Z-Зонд TouchMI");
  LSTR MSG_TOUCHMI_INIT                     = _UxGT("Ініц. TouchMI");
  LSTR MSG_TOUCHMI_ZTEST                    = _UxGT("Тест Z-зміщення");
  LSTR MSG_TOUCHMI_SAVE                     = _UxGT("Зберегти");
  LSTR MSG_MANUAL_DEPLOY_TOUCHMI            = _UxGT("Установити TouchMI");
  LSTR MSG_MANUAL_DEPLOY                    = _UxGT("Установити Z-зонд");
  LSTR MSG_MANUAL_STOW                      = _UxGT("Завантажити Z-зонд");
  LSTR MSG_HOME_FIRST                       = _UxGT("Спочатку дім %s");
  LSTR MSG_ZPROBE_OFFSETS                   = _UxGT("Зміщення зонду");
  LSTR MSG_ZPROBE_XOFFSET                   = _UxGT("Зміщення по X");
  LSTR MSG_ZPROBE_YOFFSET                   = _UxGT("Зміщення по Y");
  LSTR MSG_ZPROBE_ZOFFSET                   = _UxGT("Зміщення по Z");
  LSTR MSG_ZPROBE_OFFSET_N                  = _UxGT("Зміщення по @");
  LSTR MSG_BABYSTEP_PROBE_Z                 = _UxGT("Крок Z зміщення");
  LSTR MSG_MOVE_NOZZLE_TO_BED               = _UxGT("Рухати сопло до столу");
  LSTR MSG_BABYSTEP_X                       = _UxGT("Мікрокрок X");
  LSTR MSG_BABYSTEP_Y                       = _UxGT("Мікрокрок Y");
  LSTR MSG_BABYSTEP_Z                       = _UxGT("Мікрокрок Z");
  LSTR MSG_BABYSTEP_N                       = _UxGT("Мікрокрок @");
  LSTR MSG_BABYSTEP_TOTAL                   = _UxGT("Сумарно");
  LSTR MSG_ENDSTOP_ABORT                    = _UxGT("Кінцевик спрацював");
  LSTR MSG_ERR_HEATING_FAILED               = _UxGT("Збій нагріву");
  LSTR MSG_ERR_REDUNDANT_TEMP               = _UxGT("ЗАВИЩЕНА Т") LCD_STR_DEGREE;
  LSTR MSG_ERR_THERMAL_RUNAWAY              = _UxGT("ВИТІК ТЕПЛА");
  LSTR MSG_ERR_COOLING_FAILED               = _UxGT("ОХОЛОДЖ. НЕ ВДАЛОСЬ");
  LSTR MSG_ERR_MAXTEMP                      = _UxGT("МАКСИМАЛЬНА Т") LCD_STR_DEGREE;
  LSTR MSG_ERR_MINTEMP                      = _UxGT("МІНІМАЛЬНА Т") LCD_STR_DEGREE;
  LSTR MSG_HALTED                           = _UxGT("ПРИНТЕР ЗУПИНЕНО");
  LSTR MSG_PLEASE_RESET                     = _UxGT("Перезавантажте");
  LSTR MSG_HEATING                          = _UxGT("Нагрівання...");
  LSTR MSG_COOLING                          = _UxGT("Охолодження...");
  LSTR MSG_BED_HEATING                      = _UxGT("Нагрів столу...");
  LSTR MSG_PROBE_HEATING                    = _UxGT("Нагрів зонду...");
  LSTR MSG_CHAMBER_HEATING                  = _UxGT("Нагрів камери...");
  LSTR MSG_BED_COOLING                      = _UxGT("Охолодж. столу...");
  LSTR MSG_PROBE_COOLING                    = _UxGT("Охолодж. зонду...");
  LSTR MSG_CHAMBER_COOLING                  = _UxGT("Охолодж. камери...");
  LSTR MSG_LASER_COOLING                    = _UxGT("Охолодж. лазеру...");
  LSTR MSG_DELTA_CALIBRATE                  = _UxGT("Калібрування Delta");
  LSTR MSG_DELTA_CALIBRATE_X                = _UxGT("Калібрувати X");
  LSTR MSG_DELTA_CALIBRATE_Y                = _UxGT("Калібрувати Y");
  LSTR MSG_DELTA_CALIBRATE_Z                = _UxGT("Калібрувати Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER           = _UxGT("Калібр. центр");
  LSTR MSG_DELTA_SETTINGS                   = _UxGT("Параметри Delta");
  LSTR MSG_DELTA_AUTO_CALIBRATE             = _UxGT("Автокалібрування");
  LSTR MSG_DELTA_DIAG_ROD                   = _UxGT("Діагональ стрижня");
  LSTR MSG_DELTA_HEIGHT                     = _UxGT("Висота");
  LSTR MSG_DELTA_RADIUS                     = _UxGT("Радіус");

  LSTR MSG_INFO_MENU                        = _UxGT("Про принтер");
  LSTR MSG_INFO_PRINTER_MENU                = _UxGT("Дані принтера");

  LSTR MSG_3POINT_LEVELING                  = _UxGT("3-точкове вирівн.");
  LSTR MSG_LINEAR_LEVELING                  = _UxGT("Лінійне вирівн.");
  LSTR MSG_BILINEAR_LEVELING                = _UxGT("Білінійне вирівн.");
  LSTR MSG_UBL_LEVELING                     = _UxGT("UBL");
  LSTR MSG_MESH_LEVELING                    = _UxGT("Вирівнювання сітки");
  LSTR MSG_MESH_DONE                        = _UxGT("Зондування виконано");

  LSTR MSG_INFO_STATS_MENU                  = _UxGT("Статистика принтера");
  LSTR MSG_INFO_BOARD_MENU                  = _UxGT("Про плату");
  LSTR MSG_INFO_THERMISTOR_MENU             = _UxGT("Термістори");
  LSTR MSG_INFO_EXTRUDERS                   = _UxGT("Екструдери");
  LSTR MSG_INFO_BAUDRATE                    = _UxGT("Бод");
  LSTR MSG_INFO_PROTOCOL                    = _UxGT("Протокол");
  LSTR MSG_INFO_RUNAWAY_OFF                 = _UxGT("Контр.витіку ") LCD_STR_THERMOMETER _UxGT(" Вимк");
  LSTR MSG_INFO_RUNAWAY_ON                  = _UxGT("Контр.витіку ") LCD_STR_THERMOMETER _UxGT(" Увімк");
  LSTR MSG_HOTEND_IDLE_TIMEOUT              = _UxGT("Час прост. хот-у");

  LSTR MSG_CASE_LIGHT                       = _UxGT("Підсвітка");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS            = _UxGT("Яскравість світла");
  LSTR MSG_KILL_EXPECTED_PRINTER            = _UxGT("НЕ ТОЙ ПРИНТЕР");

  LSTR MSG_COLORS_GET                       = _UxGT("Отримати колір");  // ProUI
  LSTR MSG_COLORS_SELECT                    = _UxGT("Кольори");         // ProUI
  LSTR MSG_COLORS_APPLIED                   = _UxGT("Кольори застос."); // ProUI
  LSTR MSG_COLORS_RED                       = _UxGT("Червоний");        // ProUI / JyersUI
  LSTR MSG_COLORS_GREEN                     = _UxGT("Зелений");         // ProUI / JyersUI
  LSTR MSG_COLORS_BLUE                      = _UxGT("Синій");           // ProUI / JyersUI
  LSTR MSG_COLORS_WHITE                     = _UxGT("Білий");           // ProUI
  LSTR MSG_UI_LANGUAGE                      = _UxGT("Мова");            // ProUI
  LSTR MSG_SOUND_ENABLE                     = _UxGT("Дозволити звук");  // ProUI
  LSTR MSG_LOCKSCREEN                       = _UxGT("Блокувати екран"); // ProUI
  //MSG_LOCKSCREEN_LOCKED
  //MSG_LOCKSCREEN_UNLOCK
  LSTR MSG_PLEASE_WAIT_REBOOT               = _UxGT("Перезавантаж..."); // ProUI
  LSTR MSG_MEDIA_NOT_INSERTED               = _UxGT("Немає носія");     // ProUI
  LSTR MSG_PLEASE_PREHEAT                   = _UxGT("Нагрійте хотенд"); // ProUI

  LSTR MSG_INFO_PRINT_COUNT                 = _UxGT("Друків");
  LSTR MSG_INFO_PRINT_TIME                  = _UxGT("Загалом");
  LSTR MSG_INFO_PRINT_LONGEST               = _UxGT("Найдовше");
  LSTR MSG_INFO_PRINT_FILAMENT              = _UxGT("Екструдовано");
  LSTR MSG_INFO_COMPLETED_PRINTS            = _UxGT("Завершено");
  LSTR MSG_INFO_MIN_TEMP                    = _UxGT("Мін.  ") LCD_STR_THERMOMETER;
  LSTR MSG_INFO_MAX_TEMP                    = _UxGT("Макс. ") LCD_STR_THERMOMETER;
  LSTR MSG_INFO_PSU                         = _UxGT("Блок жив-ня");

  LSTR MSG_DRIVE_STRENGTH                   = _UxGT("Сила мотору");
  LSTR MSG_DAC_PERCENT_N                    = _UxGT("Драйвер @, %");
  LSTR MSG_ERROR_TMC                        = _UxGT("ЗБІЙ ЗВ'ЯЗКУ З TMC");
  LSTR MSG_DAC_EEPROM_WRITE                 = _UxGT("Запис ЦАП у EEPROM");
  LSTR MSG_FILAMENT_CHANGE_HEADER           = _UxGT("ЗАМІНА ПРУТКА");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE     = _UxGT("ЗУПИНКА ДРУКУ");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD      = _UxGT("ЗАВАНТАЖИТИ ПРУТОК");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD    = _UxGT("ВИВАНТАЖИТИ ПРУТОК");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER    = _UxGT("ПАРАМ.ПРОДОВЖЕННЯ:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE     = _UxGT("Видавити ще");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME    = _UxGT("Відновити друк");
  LSTR MSG_FILAMENT_CHANGE_NOZZLE           = _UxGT("  Сопло: ");
  LSTR MSG_RUNOUT_SENSOR                    = _UxGT("Датчик закінч.прутка");
  LSTR MSG_RUNOUT_DISTANCE_MM               = _UxGT("До закінч.,мм");
  LSTR MSG_KILL_HOMING_FAILED               = _UxGT("Помилка паркування");
  LSTR MSG_LCD_PROBING_FAILED               = _UxGT("Помилка зондування");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER      = _UxGT("ОБЕРІТЬ ПРУТОК");
  LSTR MSG_MMU2_MENU                        = _UxGT("MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE               = _UxGT("Онови прошивку MMU!");
  LSTR MSG_MMU2_NOT_RESPONDING              = _UxGT("MMU потребує уваги");
  LSTR MSG_MMU2_RESUME                      = _UxGT("MMU Продовжити");
  LSTR MSG_MMU2_RESUMING                    = _UxGT("MMU Продовження...");
  LSTR MSG_MMU2_LOAD_FILAMENT               = _UxGT("MMU Завантажити");
  LSTR MSG_MMU2_LOAD_ALL                    = _UxGT("MMU Завантажити все");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE              = _UxGT("MMU Завант. в сопло");
  LSTR MSG_MMU2_EJECT_FILAMENT              = _UxGT("MMU Звільнити");
  LSTR MSG_MMU2_EJECT_FILAMENT_N            = _UxGT("MMU Звільнити ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT             = _UxGT("MMU Вивантажити");
  LSTR MSG_MMU2_LOADING_FILAMENT            = _UxGT("Завантаж. %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT           = _UxGT("Викидання прутка...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT          = _UxGT("Вивантаження...");
  LSTR MSG_MMU2_ALL                         = _UxGT("Все");
  LSTR MSG_MMU2_FILAMENT_N                  = _UxGT("Пруток ~");
  LSTR MSG_MMU2_RESET                       = _UxGT("Перезапуск MMU");
  LSTR MSG_MMU2_RESETTING                   = _UxGT("MMU Перезапуск...");
  LSTR MSG_MMU2_EJECT_RECOVER               = _UxGT("Видаліть, натисніть");

  LSTR MSG_MIX                              = _UxGT("Змішув.");
  LSTR MSG_MIX_COMPONENT_N                  = _UxGT("Компонент {");
  LSTR MSG_MIXER                            = _UxGT("Змішувач");
  LSTR MSG_GRADIENT                         = _UxGT("Градієнт");
  LSTR MSG_FULL_GRADIENT                    = _UxGT("Повний градієнт");
  LSTR MSG_TOGGLE_MIX                       = _UxGT("Переключ.змішування");
  LSTR MSG_CYCLE_MIX                        = _UxGT("Циклічне змішування");
  LSTR MSG_GRADIENT_MIX                     = _UxGT("Градієнт змішування");
  LSTR MSG_REVERSE_GRADIENT                 = _UxGT("Змінити градієнт");
  LSTR MSG_ACTIVE_VTOOL                     = _UxGT("Актив. В-інструм.");
  LSTR MSG_GRADIENT_ALIAS                   = _UxGT("Псевдонім В-інструм");
  LSTR MSG_RESET_VTOOLS                     = _UxGT("Зкидання В-інструм.");
  LSTR MSG_START_VTOOL                      = _UxGT("Початок В-інструменту");
  LSTR MSG_END_VTOOL                        = _UxGT("Кінець В-інструменту");
  LSTR MSG_COMMIT_VTOOL                     = _UxGT("Змішати В-інструменти");
  LSTR MSG_VTOOLS_RESET                     = _UxGT("В-інструменти зкинуті");
  LSTR MSG_START_Z                          = _UxGT("Початок Z:");
  LSTR MSG_END_Z                            = _UxGT(" Кінець Z:");

  LSTR MSG_GAMES                            = _UxGT("Ігри");
  LSTR MSG_BRICKOUT                         = _UxGT("Цеглини");
  LSTR MSG_INVADERS                         = _UxGT("Вторгнення");
  LSTR MSG_SNAKE                            = _UxGT("Zм!йк@");
  LSTR MSG_MAZE                             = _UxGT("Лабіринт");

  LSTR MSG_BAD_PAGE                         = _UxGT("Погана сторінка");
  LSTR MSG_BAD_PAGE_SPEED                   = _UxGT("Погана швидк. стор.");

  LSTR MSG_EDIT_PASSWORD                    = _UxGT("Редагувати пароль");
  LSTR MSG_LOGIN_REQUIRED                   = _UxGT("Потрібен логін");
  LSTR MSG_PASSWORD_SETTINGS                = _UxGT("Параметри паролю");
  LSTR MSG_ENTER_DIGIT                      = _UxGT("Введіть цифру");
  LSTR MSG_CHANGE_PASSWORD                  = _UxGT("Змінити пароль");
  LSTR MSG_REMOVE_PASSWORD                  = _UxGT("Видалити пароль");
  LSTR MSG_PASSWORD_SET                     = _UxGT("Пароль: ");
  LSTR MSG_START_OVER                       = _UxGT("Старт через");
  LSTR MSG_REMINDER_SAVE_SETTINGS           = _UxGT("Не забудь зберегти!");
  LSTR MSG_PASSWORD_REMOVED                 = _UxGT("Пароль видалений");

  //
  // Filament Change screens show up to 2 lines on a 3-line display
  //
  LSTR MSG_PAUSE_PRINT_PARKING              = _UxGT(MSG_1_LINE("Паркування..."));
  LSTR MSG_ADVANCED_PAUSE_WAITING           = _UxGT(MSG_1_LINE("Продовжити друк"));
  LSTR MSG_FILAMENT_CHANGE_INIT             = _UxGT(MSG_1_LINE("Зачекайте..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT           = _UxGT(MSG_1_LINE("Вставте і натисніть"));
  LSTR MSG_FILAMENT_CHANGE_HEAT             = _UxGT(MSG_1_LINE("Нагріти сопло"));
  LSTR MSG_FILAMENT_CHANGE_HEATING          = _UxGT(MSG_1_LINE("Нагрів сопла..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD           = _UxGT(MSG_1_LINE("Вивід прутка..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD             = _UxGT(MSG_1_LINE("Ввід прутка..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE            = _UxGT(MSG_1_LINE("Очищення прутка..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE       = _UxGT(MSG_1_LINE("Завершити очищення"));
  LSTR MSG_FILAMENT_CHANGE_RESUME           = _UxGT(MSG_1_LINE("Поновлення друку..."));

  LSTR MSG_TMC_DRIVERS                      = _UxGT("Драйвери TMC");
  LSTR MSG_TMC_CURRENT                      = _UxGT("Струм драйвера");
  LSTR MSG_TMC_HYBRID_THRS                  = _UxGT("Гібридний поріг");
  LSTR MSG_TMC_HOMING_THRS                  = _UxGT("Дім без кінцевиків");
  LSTR MSG_TMC_STEPPING_MODE                = _UxGT("Режим мікрокроку");
  LSTR MSG_TMC_STEALTH_ENABLED              = _UxGT("Тихий режим увімк.");
  LSTR MSG_SERVICE_RESET                    = _UxGT("Зкидання");
  LSTR MSG_SERVICE_IN                       = _UxGT(" в:");
  LSTR MSG_BACKLASH                         = _UxGT("Люфт");
  LSTR MSG_BACKLASH_CORRECTION              = _UxGT("Виправлення");
  LSTR MSG_BACKLASH_SMOOTHING               = _UxGT("Зглажування");

  LSTR MSG_LEVEL_X_AXIS                     = _UxGT("Рівень вісі X");
  LSTR MSG_AUTO_CALIBRATE                   = _UxGT("Авто калібрування");

  LSTR MSG_FTDI_HEATER_TIMEOUT              = _UxGT("Час простою збіг, температура впала. Натисніть ОК, щоби знову нагріти та продовжити");
  LSTR MSG_HEATER_TIMEOUT                   = _UxGT("Час нагрівача збіг");
  LSTR MSG_REHEAT                           = _UxGT("Поновити нагрів");
  LSTR MSG_REHEATING                        = _UxGT("Нагрівання...");

  LSTR MSG_PROBE_WIZARD                     = _UxGT("Майстер Z-зонда");
  LSTR MSG_PROBE_WIZARD_PROBING             = _UxGT("Зондув.контр.точки Z");
  LSTR MSG_PROBE_WIZARD_MOVING              = _UxGT("Рух до точки зондув.");

  LSTR MSG_SOUND                            = _UxGT("Звук");

  LSTR MSG_TOP_LEFT                         = _UxGT("Верхній лівий");
  LSTR MSG_BOTTOM_LEFT                      = _UxGT("Нижній лівий");
  LSTR MSG_TOP_RIGHT                        = _UxGT("Верхній правий");
  LSTR MSG_BOTTOM_RIGHT                     = _UxGT("Нижній правий");
  LSTR MSG_CALIBRATION_COMPLETED            = _UxGT("Калібрування успішне");
  LSTR MSG_CALIBRATION_FAILED               = _UxGT("Збій калібрування");

  LSTR MSG_DRIVER_BACKWARD                  = _UxGT(" драйвер назад");

  LSTR MSG_SD_CARD                          = _UxGT("SD Картка");
  LSTR MSG_USB_DISK                         = _UxGT("USB Диск");

  LSTR MSG_SHORT_DAY                        = _UxGT("д"); // One character only
  LSTR MSG_SHORT_HOUR                       = _UxGT("г"); // One character only
  LSTR MSG_SHORT_MINUTE                     = _UxGT("х"); // One character only
}

namespace LanguageWide_uk {
  using namespace LanguageNarrow_uk;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_MEDIA_INIT_FAIL                = _UxGT("Збій ініціалізації SD");
    LSTR MSG_KILL_SUBCALL_OVERFLOW          = _UxGT("Переповнення виклику");
    LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Програмні кінцевики");
    LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Встанов. зміщення дому");
    LSTR MSG_HOME_OFFSET_X                  = _UxGT("Зміщення дому X"); // DWIN
    LSTR MSG_HOME_OFFSET_Y                  = _UxGT("Зміщення дому Y"); // DWIN
    LSTR MSG_HOME_OFFSET_Z                  = _UxGT("Зміщення дому Z"); // DWIN
    LSTR MSG_LAST_VALUE_SP                  = _UxGT("Останнє значення ");
    LSTR MSG_LASER_POWER                    = _UxGT("Потужність лазера");
    LSTR MSG_SPINDLE_TOGGLE                 = _UxGT("Перемкн. шпіндель");
    LSTR MSG_SPINDLE_EVAC_TOGGLE            = _UxGT("Перемкнути вакуум");
    LSTR MSG_LASER_TOGGLE                   = _UxGT("Перемкнути лазер");
    LSTR MSG_SPINDLE_POWER                  = _UxGT("Потужн. шпінделя");
    LSTR MSG_LASER_PULSE_MS                 = _UxGT("Тестовий імпульс мс");
    LSTR MSG_LASER_EVAC_TOGGLE              = _UxGT("Перемкнути обдув");
    LSTR MSG_BED_TRAMMING_RAISE             = _UxGT("Вгору до спрацюв. зонду");
    LSTR MSG_BED_TRAMMING_IN_RANGE          = _UxGT("Кути в межах. Вирів.столу");
    LSTR MSG_MESH_EDITOR                    = _UxGT("Зміщення по Z");
    LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Ручне введення сітки");
    LSTR MSG_UBL_BC_INSERT                  = _UxGT("Розмістити шайбу і вимір.");
    LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Видалити і виміряти стіл");
    LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Редагувати свою сітку");
    LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("Точне редагування сітки");
    LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Будувати свою сітку");
    LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Вирівнювання растру");
    LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Обсяг заповнюв.");
    LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Точно налаштувати все");
    LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Точно налашт.найближчу");
    LSTR MSG_LED_PRESETS                    = _UxGT("Передустановки світла");
    LSTR MSG_NEO2_PRESETS                   = _UxGT("Передустановка світла #2");
    LSTR MSG_COOLER                         = _UxGT("Охолодження лазеру");
    LSTR MSG_COOLER_TOGGLE                  = _UxGT("Перемк. охолодж.");
    LSTR MSG_STORED_FAN_N                   = _UxGT("Збереж.швидк.вент. ~");
    LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Дод. швидк. вент. ~");
    LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Відхилення вузла");
    LSTR MSG_VTRAV_MIN                      = _UxGT("Переміщення мін");
    LSTR MSG_CONTRAST                       = _UxGT("Контраст екрану");
    LSTR MSG_BRIGHTNESS                     = _UxGT("Яскравість LCD");
    LSTR MSG_INIT_EEPROM                    = _UxGT("Ініціалізація EEPROM");
    LSTR MSG_CONTROL_RETRACT                = _UxGT("Втягування, мм");
    LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Зміна втягув.,мм");
    LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Повернення, мм");
    LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Поверн.зміни, мм");
    LSTR MSG_AUTORETRACT                    = _UxGT("Автовтягування");
    LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Початк.швидкість");
    LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Швидкість втягув.");
    LSTR MSG_SINGLENOZZLE_FAN_SPEED         = _UxGT("Оберти вентилятора");
    LSTR MSG_SINGLENOZZLE_FAN_TIME          = _UxGT("Час вентилятора");
    LSTR MSG_ERR_COOLING_FAILED             = _UxGT("ОХОЛОДЖЕННЯ НЕ ВДАЛОСЬ");
    LSTR MSG_BED_COOLING                    = _UxGT("Охолодження столу...");
    LSTR MSG_PROBE_COOLING                  = _UxGT("Охолодження зонду...");
    LSTR MSG_CHAMBER_COOLING                = _UxGT("Охолодження камери...");
    LSTR MSG_LASER_COOLING                  = _UxGT("Охолодження лазеру...");
    LSTR MSG_3POINT_LEVELING                = _UxGT("3-точкове вирівнювання");
    LSTR MSG_LINEAR_LEVELING                = _UxGT("Лінійне вирівнювання");
    LSTR MSG_BILINEAR_LEVELING              = _UxGT("Білінійне вирівнювання");
    LSTR MSG_MESH_DONE                      = _UxGT("Зондування сітки виконано");
    LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Контроль витіку ") LCD_STR_THERMOMETER _UxGT(" Вимк");
    LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Контроль витіку ") LCD_STR_THERMOMETER _UxGT(" Увімк");
    LSTR MSG_HOTEND_IDLE_TIMEOUT            = _UxGT("Час простою хотенду");
    LSTR MSG_MEDIA_NOT_INSERTED             = _UxGT("Носій не вставлений");
    LSTR MSG_PLEASE_WAIT_REBOOT             = _UxGT("Перезавантаження...");
    LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Кількість друків");
    LSTR MSG_INFO_PRINT_TIME                = _UxGT("Час друку");
    LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Найдовший час");
    LSTR MSG_COLORS_SELECT                  = _UxGT("Обрати кольори");
    LSTR MSG_COLORS_APPLIED                 = _UxGT("Кольори застосовані");
    LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("ПАРАМЕТРИ ПРОДОВЖЕННЯ:");
    LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Датчик закінчення прутка");
    LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("Відстань закінч.,мм");
    LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("MMU Завантажити в сопло");
    LSTR MSG_MIX                            = _UxGT("Змішування");
    LSTR MSG_TOGGLE_MIX                     = _UxGT("Переключити змішування");
    LSTR MSG_ACTIVE_VTOOL                   = _UxGT("Активація В-інструменту");
    LSTR MSG_GRADIENT_ALIAS                 = _UxGT("Псевдонім В-інструменту");
    LSTR MSG_RESET_VTOOLS                   = _UxGT("Зкидання В-інструментів");
    LSTR MSG_BAD_PAGE_SPEED                 = _UxGT("Погана швидкість стор.");
    LSTR MSG_PROBE_WIZARD_PROBING           = _UxGT("Зондув. контрольної точки Z");
    LSTR MSG_PROBE_WIZARD_MOVING            = _UxGT("Рух до точки зондування");
  #endif
}

namespace LanguageTall_uk {
  using namespace LanguageWide_uk;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_3_LINE("Натисніть кнопку", "для продовження", "друку"));
    LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_3_LINE("Зачекайте", "на початок", "заміни прутка"));
    LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_3_LINE("Вставте пруток", "та натисніть", "для продовження"));
    LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_2_LINE("Натисніть кнопку", "для нагріву сопла"));
    LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_2_LINE("Сопло нагрівається", "зачекайте..."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_2_LINE("Зачекайте", "на вивід прутка"));
    LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_2_LINE("Зачекайте", "на ввід прутка"));
    LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_2_LINE("Дочекайтесь", "очищення прутка"));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_3_LINE("Натисніть кнопку", "для завершення", "очищення прутка"));
    LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_3_LINE("Зачекайте", "на відновлення", "друку"));
  #endif
}

namespace Language_uk {
  using namespace LanguageTall_uk;
}
