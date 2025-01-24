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
 * Russian
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */
#define DISPLAY_CHARSET_ISO10646_5

namespace LanguageNarrow_ru {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                = 2;
  LSTR LANGUAGE                             = _UxGT("Russian");

  LSTR WELCOME_MSG                          = MACHINE_NAME _UxGT(" Готов.");
  LSTR MSG_YES                              = _UxGT("Да");
  LSTR MSG_NO                               = _UxGT("Нет");
  LSTR MSG_BACK                             = _UxGT("Назад");
  LSTR MSG_MEDIA_ABORTING                   = _UxGT("Прерывание...");
  LSTR MSG_MEDIA_INSERTED                   = _UxGT("SD карта вставлена");
  LSTR MSG_MEDIA_REMOVED                    = _UxGT("SD карта извлечена");
  LSTR MSG_MEDIA_WAITING                    = _UxGT("Вставьте SD карту");
  LSTR MSG_MEDIA_INIT_FAIL                  = _UxGT("Сбой инициализ. SD");
  LSTR MSG_ADVANCED_SETTINGS                = _UxGT("Расшир. настройки");
  LSTR MSG_KILL_SUBCALL_OVERFLOW            = _UxGT("Переполн. вызова");
  LSTR MSG_MEDIA_READ_ERROR                 = _UxGT("Ошибка чтения");
  LSTR MSG_MEDIA_USB_REMOVED                = _UxGT("USB диск удалён");
  LSTR MSG_MEDIA_USB_FAILED                 = _UxGT("Ошибка USB диска");
  LSTR MSG_LCD_SOFT_ENDSTOPS                = _UxGT("Прогр. концевики");
  LSTR MSG_LCD_ENDSTOPS                     = _UxGT("Концевик"); // Max length 8 characters
  LSTR MSG_MAIN_MENU                        = _UxGT("Главное меню");
  LSTR MSG_CONFIGURATION                    = _UxGT("Конфигурация");
  LSTR MSG_RUN_AUTO_FILES                   = _UxGT("Автостарт");
  LSTR MSG_DISABLE_STEPPERS                 = _UxGT("Выключить двигатели");
  LSTR MSG_DEBUG_MENU                       = _UxGT("Меню отладки");
  LSTR MSG_PROGRESS_BAR_TEST                = _UxGT("Тест индикатора");
  LSTR MSG_AUTO_HOME                        = _UxGT("Парковка XYZ");
  LSTR MSG_AUTO_HOME_N                      = _UxGT("Парковка @");
  LSTR MSG_AUTO_HOME_X                      = _UxGT("Парковка X");
  LSTR MSG_AUTO_HOME_Y                      = _UxGT("Парковка Y");
  LSTR MSG_AUTO_HOME_Z                      = _UxGT("Парковка Z");
  LSTR MSG_AUTO_Z_ALIGN                     = _UxGT("Авто-выравнивание Z");
  LSTR MSG_ITERATION                        = _UxGT("G34 Итерация: %i");
  LSTR MSG_DECREASING_ACCURACY              = _UxGT("Уменьшение точности!");
  LSTR MSG_ACCURACY_ACHIEVED                = _UxGT("Точность достигнута");
  LSTR MSG_LEVEL_BED_HOMING                 = _UxGT("Нулевое положение");
  LSTR MSG_LEVEL_BED_WAITING                = _UxGT("Нажмите чтобы начать");
  LSTR MSG_LEVEL_BED_NEXT_POINT             = _UxGT("Следующая точка");
  LSTR MSG_LEVEL_BED_DONE                   = _UxGT("Выравнивание готово!");
  LSTR MSG_Z_FADE_HEIGHT                    = _UxGT("Лимит выранивания");

  LSTR MSG_SET_HOME_OFFSETS                 = _UxGT("Установ.смещ.дома");

  LSTR MSG_HOME_OFFSETS_APPLIED             = _UxGT("Смещения применены");
  LSTR MSG_SELECT_ORIGIN                    = _UxGT("Выберите ноль");
  LSTR MSG_LAST_VALUE_SP                    = _UxGT("Послед. знач. ");

  LSTR MSG_PREHEAT_1                        = _UxGT("Нагрев ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                      = _UxGT("Нагреть ~ ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                    = _UxGT("Нагреть сопло ") PREHEAT_1_LABEL _UxGT(" сопло");
  LSTR MSG_PREHEAT_1_END_E                  = _UxGT("Нагреть сопло ~") PREHEAT_1_LABEL _UxGT(" сопло ~");;
  LSTR MSG_PREHEAT_1_ALL                    = _UxGT("Нагреть всё ") PREHEAT_1_LABEL _UxGT(" всё");
  LSTR MSG_PREHEAT_1_BEDONLY                = _UxGT("Нагреть стол ") PREHEAT_1_LABEL _UxGT(" стол");
  LSTR MSG_PREHEAT_1_SETTINGS               = _UxGT("Правка предн. ") PREHEAT_1_LABEL _UxGT(" наст.");

  LSTR MSG_PREHEAT_M                        = _UxGT("Нагрев $");
  LSTR MSG_PREHEAT_M_H                      = _UxGT("Нагреть ~ $");
  LSTR MSG_PREHEAT_M_END                    = _UxGT("Нагреть сопло $");
  LSTR MSG_PREHEAT_M_END_E                  = _UxGT("Нагреть сопло ~ $");
  LSTR MSG_PREHEAT_M_ALL                    = _UxGT("Нагреть всё $");
  LSTR MSG_PREHEAT_M_BEDONLY                = _UxGT("Нагреть стол $");
  LSTR MSG_PREHEAT_M_SETTINGS               = _UxGT("Настр.нагрева $");

  LSTR MSG_PREHEAT_CUSTOM                   = _UxGT("Нагрев Свой");
  LSTR MSG_COOLDOWN                         = _UxGT("Охлаждение");
  LSTR MSG_CUTTER_FREQUENCY                 = _UxGT("Частота");
  LSTR MSG_LASER_MENU                       = _UxGT("Управление лазером");

  LSTR MSG_SPINDLE_MENU                     = _UxGT("Управл. шпинд.");
  LSTR MSG_LASER_TOGGLE                     = _UxGT("Переключ.лазер");
  LSTR MSG_SPINDLE_TOGGLE                   = _UxGT("Переключ.шпинд");
  LSTR MSG_SPINDLE_POWER                    = _UxGT("Мощн.шпинделя");
  LSTR MSG_LASER_POWER                      = _UxGT("Мощн. лазера");
  LSTR MSG_LASER_PULSE_MS                   = _UxGT("Тест. имп. мс");
  LSTR MSG_LASER_EVAC_TOGGLE                = _UxGT("Переключ. обдув");
  LSTR MSG_SPINDLE_EVAC_TOGGLE              = _UxGT("Переключ. вакуум");

  LSTR MSG_LASER_ASSIST_TOGGLE              = _UxGT("Управление обдувом");
  LSTR MSG_FLOWMETER_FAULT                  = _UxGT("Ошибка обдува");
  LSTR MSG_LASER_FIRE_PULSE                 = _UxGT("Импульс лазера");
  LSTR MSG_SPINDLE_FORWARD                  = _UxGT("Шпиндель вперёд");
  LSTR MSG_SPINDLE_REVERSE                  = _UxGT("Инверсия шпинделя");

  LSTR MSG_SWITCH_PS_ON                     = _UxGT("Включить питание");
  LSTR MSG_SWITCH_PS_OFF                    = _UxGT("Выключить питание");
  LSTR MSG_EXTRUDE                          = _UxGT("Выдавить");
  LSTR MSG_RETRACT                          = _UxGT("Втянуть");
  LSTR MSG_MOVE_AXIS                        = _UxGT("Движение по осям");
  LSTR MSG_PROBE_AND_LEVEL                  = _UxGT("Проба/выравнивание");
  LSTR MSG_BED_LEVELING                     = _UxGT("Выравнивание стола");
  LSTR MSG_LEVEL_BED                        = _UxGT("Выровнять стол");
  LSTR MSG_BED_TRAMMING                     = _UxGT("Выровнять углы");
  LSTR MSG_NEXT_CORNER                      = _UxGT("Следующий угол");
  LSTR MSG_BED_TRAMMING_RAISE               = _UxGT("Вверх до сраб. зонда");
  LSTR MSG_BED_TRAMMING_IN_RANGE            = _UxGT("Углы в норме. Вырав.");
  LSTR MSG_BED_TRAMMING_GOOD_POINTS         = _UxGT("Хорошие точки: ");
  LSTR MSG_BED_TRAMMING_LAST_Z              = _UxGT("Последняя Z: ");
  LSTR MSG_MESH_EDITOR                      = _UxGT("Смещение Z");
  LSTR MSG_EDITING_STOPPED                  = _UxGT("Правка окончена");
  LSTR MSG_EDIT_MESH                        = _UxGT("Редактировать сетку");
  LSTR MSG_PROBING_POINT                    = _UxGT("Точка сетки");
  LSTR MSG_MESH_X                           = _UxGT("Индекс X");
  LSTR MSG_MESH_Y                           = _UxGT("Индекс Y");
  LSTR MSG_MESH_EDIT_Z                      = _UxGT("Значение Z");
  LSTR MSG_CUSTOM_COMMANDS                  = _UxGT("Свои команды");

  LSTR MSG_M48_TEST                         = _UxGT("M48 тест Z-зонда");
  LSTR MSG_M48_DEVIATION                    = _UxGT("Отклонение");
  LSTR MSG_M48_POINT                        = _UxGT("M48 точка");
  LSTR MSG_M48_OUT_OF_BOUNDS                = _UxGT("Зонд за пределами");

  LSTR MSG_IDEX_MENU                        = _UxGT("Меню IDEX");
  LSTR MSG_OFFSETS_MENU                     = _UxGT("Размещение сопел");
  LSTR MSG_IDEX_MODE_AUTOPARK               = _UxGT("Авто парковка");
  LSTR MSG_IDEX_MODE_DUPLICATE              = _UxGT("Размножение");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY          = _UxGT("Зеркальная копия");
  LSTR MSG_IDEX_MODE_FULL_CTRL              = _UxGT("Полный контроль");
  LSTR MSG_IDEX_DUPE_GAP                    = _UxGT("Дублировать X-зазор");

  LSTR MSG_HOTEND_OFFSET_Z                  = _UxGT("2-е сопло Z");
  LSTR MSG_HOTEND_OFFSET_N                  = _UxGT("2-е сопло @");

  LSTR MSG_UBL_DOING_G29                    = _UxGT("Выполняем G29");
  LSTR MSG_UBL_TOOLS                        = _UxGT("Инструменты UBL");
  LSTR MSG_LCD_TILTING_MESH                 = _UxGT("Точка разворота");
  LSTR MSG_UBL_MANUAL_MESH                  = _UxGT("Ручной ввод сетки");
  LSTR MSG_UBL_BC_INSERT                    = _UxGT("Разм.шайбу, измерить");
  LSTR MSG_UBL_BC_REMOVE                    = _UxGT("Убрать, измер. стол");
  LSTR MSG_UBL_MESH_WIZARD                  = _UxGT("Мастер сеток UBL");
  LSTR MSG_UBL_BC_INSERT2                   = _UxGT("Измерение");
  LSTR MSG_UBL_MOVING_TO_NEXT               = _UxGT("Двигаемся дальше");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH             = _UxGT("Править свою сетку");
  LSTR MSG_UBL_SET_TEMP_BED                 = LCD_STR_THERMOMETER _UxGT(" стола, ") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_UBL_BED_TEMP_CUSTOM              = _UxGT("Своя ") LCD_STR_THERMOMETER _UxGT(" стола,") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_UBL_SET_TEMP_HOTEND              = LCD_STR_THERMOMETER _UxGT(" сопла, ") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM           = _UxGT("Своя ") LCD_STR_THERMOMETER _UxGT(" сопла,") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH            = _UxGT("Построить свою");
  LSTR MSG_UBL_DONE_EDITING_MESH            = _UxGT("Правка завершена");
  LSTR MSG_UBL_FINE_TUNE_MESH               = _UxGT("Точная правка сетки");
  LSTR MSG_UBL_BUILD_MESH_MENU              = _UxGT("Построить сетку");
  LSTR MSG_UBL_BUILD_MESH_M                 = _UxGT("Построить сетку $");
  LSTR MSG_UBL_BUILD_COLD_MESH              = _UxGT("Строить холод.сетку");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST           = _UxGT("Правка высоты сетки");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT           = _UxGT("Высота");
  LSTR MSG_UBL_VALIDATE_MESH_MENU           = _UxGT("Проверить сетку");
  LSTR MSG_UBL_VALIDATE_MESH_M              = _UxGT("Провер. сетку $");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH         = _UxGT("Провер. свою сетку");
  LSTR MSG_G26_HEATING_BED                  = _UxGT("G26 нагрев стола");
  LSTR MSG_G26_HEATING_NOZZLE               = _UxGT("G26 нагрев сопла");
  LSTR MSG_G26_MANUAL_PRIME                 = _UxGT("Ручная прочистка");
  LSTR MSG_G26_FIXED_LENGTH                 = _UxGT("Прочистка фикс.длины");
  LSTR MSG_G26_PRIME_DONE                   = _UxGT("Прочистка окончена");
  LSTR MSG_G26_CANCELED                     = _UxGT("G26 завершена");
  LSTR MSG_G26_LEAVING                      = _UxGT("Выйти из G26");
  LSTR MSG_UBL_CONTINUE_MESH                = _UxGT("Продолжить сетку");
  LSTR MSG_UBL_MESH_LEVELING                = _UxGT("Выравнивание сетки");
  LSTR MSG_UBL_3POINT_MESH_LEVELING         = _UxGT("3-точечное выравн.");
  LSTR MSG_UBL_GRID_MESH_LEVELING           = _UxGT("Выравнивание сеткой");
  LSTR MSG_UBL_MESH_LEVEL                   = _UxGT("Выровнять сетку");
  LSTR MSG_UBL_SIDE_POINTS                  = _UxGT("Крайние точки");
  LSTR MSG_UBL_MAP_TYPE                     = _UxGT("Тип карты");
  LSTR MSG_UBL_OUTPUT_MAP                   = _UxGT("Вывести карту сетки");
  LSTR MSG_UBL_OUTPUT_MAP_HOST              = _UxGT("Вывести на хост");
  LSTR MSG_UBL_OUTPUT_MAP_CSV               = _UxGT("Вывести в CSV");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP            = _UxGT("Сохранить снаружи");
  LSTR MSG_UBL_INFO_UBL                     = _UxGT("Информация UBL");
  LSTR MSG_UBL_FILLIN_AMOUNT                = _UxGT("Кол-во заполн.");
  LSTR MSG_UBL_MANUAL_FILLIN                = _UxGT("Ручное заполнение");
  LSTR MSG_UBL_SMART_FILLIN                 = _UxGT("Умное заполнение");
  LSTR MSG_UBL_FILLIN_MESH                  = _UxGT("Заполнить сетку");
  LSTR MSG_UBL_INVALIDATE_ALL               = _UxGT("Аннулировать всё");
  LSTR MSG_UBL_INVALIDATE_CLOSEST           = _UxGT("Аннулир. ближайшую");
  LSTR MSG_UBL_FINE_TUNE_ALL                = _UxGT("Точно править всё");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST            = _UxGT("Настр. ближ. точку");
  LSTR MSG_UBL_STORAGE_MESH_MENU            = _UxGT("Хранилище сеток");
  LSTR MSG_UBL_STORAGE_SLOT                 = _UxGT("Слот памяти");
  LSTR MSG_UBL_LOAD_MESH                    = _UxGT("Загрузить сетку");
  LSTR MSG_UBL_SAVE_MESH                    = _UxGT("Сохранить сетку");
  LSTR MSG_MESH_LOADED                      = _UxGT("Сетка %i загружена");
  LSTR MSG_MESH_SAVED                       = _UxGT("Сетка %i сохранена");
  LSTR MSG_UBL_NO_STORAGE                   = _UxGT("Нет хранилища");
  LSTR MSG_UBL_SAVE_ERROR                   = _UxGT("Ошибка: Сохран. UBL");
  LSTR MSG_UBL_RESTORE_ERROR                = _UxGT("Ошибка: Восстан.UBL");
  LSTR MSG_UBL_Z_OFFSET                     = _UxGT("Смещение Z: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED             = _UxGT("Смещение Z останов.");
  LSTR MSG_UBL_STEP_BY_STEP_MENU            = _UxGT("Пошаговая настр. UBL");
  LSTR MSG_UBL_1_BUILD_COLD_MESH            = _UxGT("1.Строить холодную");
  LSTR MSG_UBL_2_SMART_FILLIN               = _UxGT("2.Умное заполнение");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU         = _UxGT("3.Проверить сетку");
  LSTR MSG_UBL_4_FINE_TUNE_ALL              = _UxGT("4.Точно настр. всё");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU         = _UxGT("5.Проверить сетку");
  LSTR MSG_UBL_6_FINE_TUNE_ALL              = _UxGT("6.Точно настр. всё");
  LSTR MSG_UBL_7_SAVE_MESH                  = _UxGT("7.Сохранить сетку");

  LSTR MSG_LED_CONTROL                      = _UxGT("Настройка подсветки");
  LSTR MSG_LIGHTS                           = _UxGT("Подсветка");
  LSTR MSG_LIGHT_N                          = _UxGT("Свет #{");
  LSTR MSG_LED_PRESETS                      = _UxGT("Предустановки света");
  LSTR MSG_SET_LEDS_RED                     = _UxGT("Красный");
  LSTR MSG_SET_LEDS_ORANGE                  = _UxGT("Оранжевый");
  LSTR MSG_SET_LEDS_YELLOW                  = _UxGT("Жёлтый");
  LSTR MSG_SET_LEDS_GREEN                   = _UxGT("Зелёный");
  LSTR MSG_SET_LEDS_BLUE                    = _UxGT("Синий");
  LSTR MSG_SET_LEDS_INDIGO                  = _UxGT("Индиго");
  LSTR MSG_SET_LEDS_VIOLET                  = _UxGT("Фиолетовый");
  LSTR MSG_SET_LEDS_WHITE                   = _UxGT("Белый");
  LSTR MSG_SET_LEDS_DEFAULT                 = _UxGT("Свет по умолчанию");
  LSTR MSG_LED_CHANNEL_N                    = _UxGT("Канал {");
  LSTR MSG_NEO2_PRESETS                     = _UxGT("Свет #2 предустан.");
  LSTR MSG_NEO2_BRIGHTNESS                  = _UxGT("Яркость");
  LSTR MSG_CUSTOM_LEDS                      = _UxGT("Свой цвет подсветки");
  LSTR MSG_INTENSITY_R                      = _UxGT("Уровень красного");
  LSTR MSG_INTENSITY_G                      = _UxGT("Уровень зелёного");
  LSTR MSG_INTENSITY_B                      = _UxGT("Уровень синего");
  LSTR MSG_INTENSITY_W                      = _UxGT("Уровень белого");
  LSTR MSG_LED_BRIGHTNESS                   = _UxGT("Яркость");

  LSTR MSG_MOVING                           = _UxGT("Движемся...");
  LSTR MSG_FREE_XY                          = _UxGT("Освободить XY");
  LSTR MSG_MOVE_X                           = _UxGT("Движение по X");
  LSTR MSG_MOVE_Y                           = _UxGT("Движение по Y");
  LSTR MSG_MOVE_Z                           = _UxGT("Движение по Z");
  LSTR MSG_MOVE_N                           = _UxGT("Движение по @");
  LSTR MSG_MOVE_E                           = _UxGT("Экструдер");
  LSTR MSG_MOVE_EN                          = _UxGT("Экструдер *");
  LSTR MSG_HOTEND_TOO_COLD                  = _UxGT("Сопло не нагрето");
  LSTR MSG_MOVE_N_MM                        = _UxGT("Движение $мм");
  LSTR MSG_MOVE_N_IN                        = _UxGT("Движение $in");
  LSTR MSG_MOVE_N_DEG                       = _UxGT("Движение $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                            = _UxGT("Скорость");
  LSTR MSG_MESH_Z_OFFSET                    = _UxGT("Z стола");
  LSTR MSG_NOZZLE                           = _UxGT("Сопло, ") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_NOZZLE_N                         = _UxGT("Сопло ~");
  LSTR MSG_NOZZLE_PARKED                    = _UxGT("Сопло запарковано");
  LSTR MSG_NOZZLE_STANDBY                   = _UxGT("Сопло ожидает");
  LSTR MSG_BED                              = _UxGT("Стол,  ") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_CHAMBER                          = _UxGT("Камера,") LCD_STR_DEGREE _UxGT("C");
  LSTR MSG_COOLER                           = _UxGT("Охлажд. лазера");
  LSTR MSG_COOLER_TOGGLE                    = _UxGT("Переключ. охл.");
  LSTR MSG_FLOWMETER_SAFETY                 = _UxGT("Безопас.потока");
  LSTR MSG_LASER                            = _UxGT("Лазер");
  LSTR MSG_FAN_SPEED                        = _UxGT("Кулер");
  LSTR MSG_FAN_SPEED_N                      = _UxGT("Кулер ~");
  LSTR MSG_STORED_FAN_N                     = _UxGT("Сохранённый кулер ~");
  LSTR MSG_EXTRA_FAN_SPEED                  = _UxGT("Дополн. кулер");
  LSTR MSG_EXTRA_FAN_SPEED_N                = _UxGT("Дополн. кулер ~");
  LSTR MSG_CONTROLLER_FAN                   = _UxGT("Кулер контроллера");
  LSTR MSG_CONTROLLER_FAN_IDLE_SPEED        = _UxGT("Холостые обороты");
  LSTR MSG_CONTROLLER_FAN_AUTO_ON           = _UxGT("Автовключение");
  LSTR MSG_CONTROLLER_FAN_SPEED             = _UxGT("Рабочие обороты");
  LSTR MSG_CONTROLLER_FAN_DURATION          = _UxGT("Период простоя");
  LSTR MSG_FLOW                             = _UxGT("Поток");
  LSTR MSG_FLOW_N                           = _UxGT("Поток ~");
  LSTR MSG_CONTROL                          = _UxGT("Настройки");
  LSTR MSG_MIN                              = " " LCD_STR_THERMOMETER ", " LCD_STR_DEGREE _UxGT("С  мин");
  LSTR MSG_MAX                              = " " LCD_STR_THERMOMETER ", " LCD_STR_DEGREE _UxGT("С макс");
  LSTR MSG_FACTOR                           = " " LCD_STR_THERMOMETER _UxGT(" Фактор");
  LSTR MSG_AUTOTEMP                         = _UxGT("Автотемпература");
  LSTR MSG_TIMEOUT                          = _UxGT("Таймаут.");
  LSTR MSG_LCD_ON                           = _UxGT("Вкл");
  LSTR MSG_LCD_OFF                          = _UxGT("Выкл");

  LSTR MSG_PID_AUTOTUNE                     = _UxGT("Автокалибровка PID");
  LSTR MSG_PID_AUTOTUNE_E                   = _UxGT("Автокалибр. PID *");
  LSTR MSG_PID_AUTOTUNE_DONE                = _UxGT("Калибр.PID выполнена");
  LSTR MSG_PID_BAD_HEATER_ID                = _UxGT("Сбой калибровки! Неверный экструдер.");
  LSTR MSG_PID_TEMP_TOO_HIGH                = _UxGT("Сбой калибровки! Температура завышена.");
  LSTR MSG_PID_TIMEOUT                      = _UxGT("Сбой калибровки! Таймаут.");

  LSTR MSG_SELECT_E                         = _UxGT("Выбор *");
  LSTR MSG_ACC                              = _UxGT("Ускорение");
  LSTR MSG_JERK                             = _UxGT("Рывок");
  LSTR MSG_VA_JERK                          = _UxGT("V") STR_A _UxGT("-рывок");
  LSTR MSG_VB_JERK                          = _UxGT("V") STR_B _UxGT("-рывок");
  LSTR MSG_VC_JERK                          = _UxGT("V") STR_C _UxGT("-рывок");
  LSTR MSG_VN_JERK                          = _UxGT("V@-рывок");
  LSTR MSG_VE_JERK                          = _UxGT("Ve-рывок");
  LSTR MSG_JUNCTION_DEVIATION               = _UxGT("Отклон.угла");
  LSTR MSG_MAX_SPEED                        = _UxGT("Скорость, мм/с");
  LSTR MSG_VMAX_A                           = _UxGT("Скор.макс ") STR_A;
  LSTR MSG_VMAX_B                           = _UxGT("Скор.макс ") STR_B;
  LSTR MSG_VMAX_C                           = _UxGT("Скор.макс ") STR_C;
  LSTR MSG_VMAX_N                           = _UxGT("Скор.макс @");
  LSTR MSG_VMAX_E                           = _UxGT("Скор.макс E");
  LSTR MSG_VMAX_EN                          = _UxGT("Скор.макс *");
  LSTR MSG_VMIN                             = _UxGT("Скор.мин");
  LSTR MSG_VTRAV_MIN                        = _UxGT("Перемещение мин");
  LSTR MSG_ACCELERATION                     = _UxGT("Ускорение, мм/с2");
  LSTR MSG_AMAX_A                           = _UxGT("Ускор.макс ") STR_A;
  LSTR MSG_AMAX_B                           = _UxGT("Ускор.макс ") STR_B;
  LSTR MSG_AMAX_C                           = _UxGT("Ускор.макс ") STR_C;
  LSTR MSG_AMAX_N                           = _UxGT("Ускор.макс @");
  LSTR MSG_AMAX_E                           = _UxGT("Ускор.макс E");
  LSTR MSG_AMAX_EN                          = _UxGT("Ускор.макс *");
  LSTR MSG_A_RETRACT                        = _UxGT("Ускор.втягив.");
  LSTR MSG_A_TRAVEL                         = _UxGT("Ускор.путеш.");
  LSTR MSG_XY_FREQUENCY_LIMIT               = _UxGT("Частота макс.");
  LSTR MSG_XY_FREQUENCY_FEEDRATE            = _UxGT("Подача мин.");
  LSTR MSG_STEPS_PER_MM                     = _UxGT("Шагов/мм");
  LSTR MSG_A_STEPS                          = STR_A _UxGT(" шаг/мм");
  LSTR MSG_B_STEPS                          = STR_B _UxGT(" шаг/мм");
  LSTR MSG_C_STEPS                          = STR_C _UxGT(" шаг/мм");
  LSTR MSG_N_STEPS                          = _UxGT("@ шаг/мм");
  LSTR MSG_E_STEPS                          = _UxGT("E шаг/мм");
  LSTR MSG_EN_STEPS                         = _UxGT("* шаг/мм");
  LSTR MSG_TEMPERATURE                      = _UxGT("Температура");
  LSTR MSG_MOTION                           = _UxGT("Движение");
  LSTR MSG_FILAMENT                         = _UxGT("Филамент");
  LSTR MSG_VOLUMETRIC_ENABLED               = _UxGT("E в мм") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT                 = _UxGT("E огран.,мм") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT_E               = _UxGT("E огран. *");
  LSTR MSG_FILAMENT_DIAM                    = _UxGT("Диам. филам.");
  LSTR MSG_FILAMENT_DIAM_E                  = _UxGT("Диам. филам. *");
  LSTR MSG_FILAMENT_UNLOAD                  = _UxGT("Загрузка, мм");
  LSTR MSG_FILAMENT_LOAD                    = _UxGT("Выгрузка, мм");
  LSTR MSG_ADVANCE_K                        = _UxGT("К-фактор LA");
  LSTR MSG_ADVANCE_K_E                      = _UxGT("К-фактор LA *");
  LSTR MSG_CONTRAST                         = _UxGT("Контраст экрана");
  LSTR MSG_STORE_EEPROM                     = _UxGT("Сохранить настройки");
  LSTR MSG_LOAD_EEPROM                      = _UxGT("Загрузить настройки");
  LSTR MSG_RESTORE_DEFAULTS                 = _UxGT("На базовые парам.");
  LSTR MSG_INIT_EEPROM                      = _UxGT("Инициализ. EEPROM");
  LSTR MSG_ERR_EEPROM_CRC                   = _UxGT("Сбой EEPROM: CRC");
  LSTR MSG_ERR_EEPROM_SIZE                  = _UxGT("Сбой EEPROM: размер");
  LSTR MSG_ERR_EEPROM_VERSION               = _UxGT("Сбой EEPROM: версия");
  LSTR MSG_ERR_EEPROM_CORRUPT               = _UxGT("Сбой EEPROM: данные");
  LSTR MSG_SETTINGS_STORED                  = _UxGT("Параметры сохранены");
  LSTR MSG_MEDIA_UPDATE                     = _UxGT("Обновление прошивки");
  LSTR MSG_RESET_PRINTER                    = _UxGT("Сброс принтера");
  LSTR MSG_REFRESH                          = LCD_STR_REFRESH _UxGT(" Обновить");
  LSTR MSG_INFO_SCREEN                      = _UxGT("Главный экран");
  LSTR MSG_PREPARE                          = _UxGT("Подготовить");
  LSTR MSG_TUNE                             = _UxGT("Настроить");
  LSTR MSG_POWER_MONITOR                    = _UxGT("Монитор питания");
  LSTR MSG_CURRENT                          = _UxGT("Ток");
  LSTR MSG_VOLTAGE                          = _UxGT("Напряжение");
  LSTR MSG_POWER                            = _UxGT("Мощность");
  LSTR MSG_START_PRINT                      = _UxGT("Запустить печать");

  LSTR MSG_BUTTON_NEXT                      = _UxGT("Дальше"); //short text for buttons
  LSTR MSG_BUTTON_INIT                      = _UxGT("Иниц-я");
  LSTR MSG_BUTTON_STOP                      = _UxGT("Стоп");
  LSTR MSG_BUTTON_PRINT                     = _UxGT("Печать");
  LSTR MSG_BUTTON_RESET                     = _UxGT("Сброс");
  LSTR MSG_BUTTON_IGNORE                    = _UxGT("Игнорир.");
  LSTR MSG_BUTTON_CANCEL                    = _UxGT("Отмена");
  LSTR MSG_BUTTON_DONE                      = _UxGT("Готово");
  LSTR MSG_BUTTON_BACK                      = _UxGT("Назад");
  LSTR MSG_BUTTON_PROCEED                   = _UxGT("Продолжить");
  LSTR MSG_BUTTON_SKIP                      = _UxGT("Пропустить");

  LSTR MSG_PAUSE_PRINT                      = _UxGT("Пауза печати");
  LSTR MSG_RESUME_PRINT                     = _UxGT("Продолжить печать");
  LSTR MSG_HOST_START_PRINT                 = _UxGT("Старт с хоста");
  LSTR MSG_STOP_PRINT                       = _UxGT("Остановить печать");
  LSTR MSG_PRINTING_OBJECT                  = _UxGT("Печать объекта");
  LSTR MSG_CANCEL_OBJECT                    = _UxGT("Завершить объект");
  LSTR MSG_CANCEL_OBJECT_N                  = _UxGT("Завершить объект {");
  LSTR MSG_OUTAGE_RECOVERY                  = _UxGT("Восстановение сбоя");
  LSTR MSG_MEDIA_MENU                       = _UxGT("Печать с SD карты");
  LSTR MSG_NO_MEDIA                         = _UxGT("Нет SD карты");
  LSTR MSG_DWELL                            = _UxGT("Сон...");
  LSTR MSG_USERWAIT                         = _UxGT("Продолжить...");
  LSTR MSG_PRINT_PAUSED                     = _UxGT("Печать на паузе");
  LSTR MSG_PRINTING                         = _UxGT("Печать...");
  LSTR MSG_PRINT_ABORTED                    = _UxGT("Печать отменена");
  LSTR MSG_PRINT_DONE                       = _UxGT("Печать завершена");
  LSTR MSG_NO_MOVE                          = _UxGT("Нет движения.");
  LSTR MSG_KILLED                           = _UxGT("УБИТО. ");
  LSTR MSG_STOPPED                          = _UxGT("ОСТАНОВЛЕНО. ");
  LSTR MSG_CONTROL_RETRACT                  = _UxGT("Откат, мм");
  LSTR MSG_CONTROL_RETRACT_SWAP             = _UxGT("Откат смены,мм");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP     = _UxGT("Возвр.смены,мм");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF    = _UxGT("Возвр.смены V");
  LSTR MSG_AUTORETRACT                      = _UxGT("Автооткат");
  LSTR MSG_CONTROL_RETRACT_ZHOP             = _UxGT("Подскок Z, мм");
  LSTR MSG_CONTROL_RETRACTF                 = _UxGT("Втягивание V");
  LSTR MSG_CONTROL_RETRACT_RECOVER          = _UxGT("Возврат, мм");
  LSTR MSG_CONTROL_RETRACT_RECOVERF         = _UxGT("Возврат V");

  LSTR MSG_FILAMENT_SWAP_LENGTH             = _UxGT("Длина замены");
  LSTR MSG_FILAMENT_SWAP_EXTRA              = _UxGT("Доп. длина");
  LSTR MSG_FILAMENT_PURGE_LENGTH            = _UxGT("Длина прочистки");

  LSTR MSG_TOOL_CHANGE                      = _UxGT("Смена сопел");
  LSTR MSG_TOOL_CHANGE_ZLIFT                = _UxGT("Поднятие по Z");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED         = _UxGT("Началь.скор.");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED       = _UxGT("Скор.отката");
  LSTR MSG_FILAMENT_PARK_ENABLED            = _UxGT("Парковать голову");
  LSTR MSG_SINGLENOZZLE_UNRETRACT_SPEED     = _UxGT("Скорость возврата");
  LSTR MSG_SINGLENOZZLE_FAN_SPEED           = _UxGT("Скорость кулера");
  LSTR MSG_SINGLENOZZLE_FAN_TIME            = _UxGT("Время кулера");
  LSTR MSG_TOOL_MIGRATION_ON                = _UxGT("Авто Вкл.");
  LSTR MSG_TOOL_MIGRATION_OFF               = _UxGT("Авто Выкл.");
  LSTR MSG_TOOL_MIGRATION                   = _UxGT("Смена инструмента");
  LSTR MSG_TOOL_MIGRATION_AUTO              = _UxGT("Авто смена");
  LSTR MSG_TOOL_MIGRATION_END               = _UxGT("Последний экструдер");
  LSTR MSG_TOOL_MIGRATION_SWAP              = _UxGT("Замена на *");
  LSTR MSG_FILAMENTCHANGE                   = _UxGT("Смена филамента");
  LSTR MSG_FILAMENTCHANGE_E                 = _UxGT("Смена филамента *");
  LSTR MSG_FILAMENTLOAD                     = _UxGT("Загрузить филамент");
  LSTR MSG_FILAMENTLOAD_E                   = _UxGT("Подать филамент *");
  LSTR MSG_FILAMENTUNLOAD_E                 = _UxGT("Убрать филамент *");
  LSTR MSG_FILAMENTUNLOAD_ALL               = _UxGT("Выгрузить всё");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA                = _UxGT("Установить SD карту");
    LSTR MSG_ATTACH_USB_MEDIA               = _UxGT("Установить флешка");
  #else
    LSTR MSG_ATTACH_MEDIA                   = _UxGT("Установить SD карту");
  #endif
  LSTR MSG_CHANGE_MEDIA                     = _UxGT("Сменить SD карту");
  LSTR MSG_RELEASE_MEDIA                    = _UxGT("Освободить SD карту");
  LSTR MSG_ZPROBE_OUT                       = _UxGT("Z-зонд вне стола");
  LSTR MSG_SKEW_FACTOR                      = _UxGT("Фактор перекоса");
  LSTR MSG_BLTOUCH                          = _UxGT("Z-зонд BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST                 = _UxGT("Тестирование BLTouch");
  LSTR MSG_BLTOUCH_RESET                    = _UxGT("Сброс BLTouch");
  LSTR MSG_BLTOUCH_STOW                     = _UxGT("Втянуть BLTouch");
  LSTR MSG_BLTOUCH_DEPLOY                   = _UxGT("Выдвинуть BLTouch");
  LSTR MSG_BLTOUCH_SW_MODE                  = _UxGT("Режим SW");
  LSTR MSG_BLTOUCH_5V_MODE                  = _UxGT("Режим 5V");
  LSTR MSG_BLTOUCH_OD_MODE                  = _UxGT("Режим OD");
  LSTR MSG_BLTOUCH_MODE_STORE               = _UxGT("Сохранить режим");
  LSTR MSG_BLTOUCH_MODE_STORE_5V            = _UxGT("Установить в 5V");
  LSTR MSG_BLTOUCH_MODE_STORE_OD            = _UxGT("Установить в OD");
  LSTR MSG_BLTOUCH_MODE_ECHO                = _UxGT("Текущий режим");
  LSTR MSG_BLTOUCH_MODE_CHANGE              = _UxGT("ОПАСНОСТЬ: Неправильные параметры приводят к повреждениям! Продолжить?");
  LSTR MSG_TOUCHMI_PROBE                    = _UxGT("Z-Зонд TouchMI");
  LSTR MSG_TOUCHMI_INIT                     = _UxGT("Инициализация");
  LSTR MSG_TOUCHMI_ZTEST                    = _UxGT("Тест Z-смещения");
  LSTR MSG_TOUCHMI_SAVE                     = _UxGT("Сохранить");
  LSTR MSG_MANUAL_DEPLOY_TOUCHMI            = _UxGT("Установить TouchMI");
  LSTR MSG_MANUAL_DEPLOY                    = _UxGT("Выдвинуть зонд");
  LSTR MSG_MANUAL_STOW                      = _UxGT("Втянуть зонд");
  LSTR MSG_HOME_FIRST                       = _UxGT("Сначала паркуй %s");
  LSTR MSG_ZPROBE_OFFSETS                   = _UxGT("Смещение Z-зонда");
  LSTR MSG_ZPROBE_XOFFSET                   = _UxGT("Смещение X");
  LSTR MSG_ZPROBE_YOFFSET                   = _UxGT("Смещение Y");
  LSTR MSG_ZPROBE_ZOFFSET                   = _UxGT("Смещение Z");
  LSTR MSG_ZPROBE_OFFSET_N                  = _UxGT("Смещение @");
  LSTR MSG_BABYSTEP_PROBE_Z                 = _UxGT("Шаг Z смещения");
  LSTR MSG_MOVE_NOZZLE_TO_BED               = _UxGT("Двиг. сопло к столу");
  LSTR MSG_BABYSTEP_X                       = _UxGT("Микрошаг X");
  LSTR MSG_BABYSTEP_Y                       = _UxGT("Микрошаг Y");
  LSTR MSG_BABYSTEP_Z                       = _UxGT("Микрошаг Z");
  LSTR MSG_BABYSTEP_N                       = _UxGT("Микрошаг @");
  LSTR MSG_BABYSTEP_TOTAL                   = _UxGT("Суммарно");
  LSTR MSG_ENDSTOP_ABORT                    = _UxGT("Сработал концевик");
  LSTR MSG_ERR_HEATING_FAILED               = _UxGT("Разогрев не удался");
  LSTR MSG_ERR_THERMAL_RUNAWAY              = _UxGT("УТЕЧКА ТЕМПЕРАТУРЫ");
  LSTR MSG_ERR_COOLING_FAILED               = _UxGT("НЕ УДАЛОСЬ ОХЛАДИТЬ");
  LSTR MSG_ERR_MAXTEMP                      = _UxGT("Ошибка: Т макс.");
  LSTR MSG_ERR_MINTEMP                      = _UxGT("Ошибка: Т мин.");
  LSTR MSG_ERR_REDUNDANT_TEMP               = _UxGT("Ошибка: Т контр.");
  LSTR MSG_HALTED                           = _UxGT("ПРИНТЕР ОСТАНОВЛЕН");
  LSTR MSG_PLEASE_RESET                     = _UxGT("Сделайте сброс");
  LSTR MSG_HEATING                          = _UxGT("Нагрев...");
  LSTR MSG_COOLING                          = _UxGT("Охлаждение...");
  LSTR MSG_BED_HEATING                      = _UxGT("Нагрев стола...");
  LSTR MSG_BED_COOLING                      = _UxGT("Охлаждение стола...");
  LSTR MSG_PROBE_HEATING                    = _UxGT("Нагрев зонда...");
  LSTR MSG_PROBE_COOLING                    = _UxGT("Охлаждение зонда...");
  LSTR MSG_CHAMBER_HEATING                  = _UxGT("Нагрев камеры...");
  LSTR MSG_CHAMBER_COOLING                  = _UxGT("Охладжение камеры...");
  LSTR MSG_LASER_COOLING                    = _UxGT("Охлаждение лазера...");
  LSTR MSG_DELTA_CALIBRATE                  = _UxGT("Калибровка дельты");
  LSTR MSG_DELTA_CALIBRATE_X                = _UxGT("Калибровать X");
  LSTR MSG_DELTA_CALIBRATE_Y                = _UxGT("Калибровать Y");
  LSTR MSG_DELTA_CALIBRATE_Z                = _UxGT("Калибровать Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER           = _UxGT("Калибровать центр");
  LSTR MSG_DELTA_SETTINGS                   = _UxGT("Настройки дельты");
  LSTR MSG_DELTA_AUTO_CALIBRATE             = _UxGT("Автокалибр. дельты");
  LSTR MSG_DELTA_DIAG_ROD                   = _UxGT("Стержень диаг.");
  LSTR MSG_DELTA_HEIGHT                     = _UxGT("Высота");
  LSTR MSG_DELTA_RADIUS                     = _UxGT("Радиус");
  LSTR MSG_INFO_MENU                        = _UxGT("О принтере");
  LSTR MSG_INFO_PRINTER_MENU                = _UxGT("Данные принтера");
  LSTR MSG_3POINT_LEVELING                  = _UxGT("3-точ. выравнив.");
  LSTR MSG_LINEAR_LEVELING                  = _UxGT("Линейное выравн.");
  LSTR MSG_BILINEAR_LEVELING                = _UxGT("Билин. выравнив.");
  LSTR MSG_UBL_LEVELING                     = _UxGT("Выравнивание UBL");
  LSTR MSG_MESH_LEVELING                    = _UxGT("Выравнивание сеткой");
  LSTR MSG_MESH_DONE                        = _UxGT("Зондиров. выполнено");

  LSTR MSG_INFO_STATS_MENU                  = _UxGT("Статистика принтера");
  LSTR MSG_INFO_BOARD_MENU                  = _UxGT("Данные платы");
  LSTR MSG_INFO_THERMISTOR_MENU             = _UxGT("Термисторы");
  LSTR MSG_INFO_EXTRUDERS                   = _UxGT("Экструдеры");
  LSTR MSG_INFO_BAUDRATE                    = _UxGT("Скорость,БОД");
  LSTR MSG_INFO_PROTOCOL                    = _UxGT("Протокол");
  LSTR MSG_INFO_RUNAWAY_OFF                 = _UxGT("Контр.утечки Т:Выкл");
  LSTR MSG_INFO_RUNAWAY_ON                  = _UxGT("Контр.утечки Т:Вкл");
  LSTR MSG_HOTEND_IDLE_TIMEOUT              = _UxGT("Время прост.хот-а");
  LSTR MSG_CASE_LIGHT                       = _UxGT("Подсветка корпуса");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS            = _UxGT("Яркость подсветки");
  LSTR MSG_KILL_EXPECTED_PRINTER            = _UxGT("НЕВЕРНЫЙ ПРИНТЕР");

  LSTR MSG_INFO_PRINT_COUNT                 = _UxGT("Напечатано");
  LSTR MSG_INFO_PRINT_TIME                  = _UxGT("Общее время");
  LSTR MSG_INFO_PRINT_LONGEST               = _UxGT("Наидольшее");
  LSTR MSG_INFO_PRINT_FILAMENT              = _UxGT("Выдавлено");
  LSTR MSG_INFO_COMPLETED_PRINTS            = _UxGT("Завершено");

  LSTR MSG_INFO_MIN_TEMP                    = _UxGT("Мин.  ") LCD_STR_THERMOMETER;
  LSTR MSG_INFO_MAX_TEMP                    = _UxGT("Макс. ") LCD_STR_THERMOMETER;
  LSTR MSG_INFO_PSU                         = _UxGT("БП");
  LSTR MSG_DRIVE_STRENGTH                   = _UxGT("Сила привода");
  LSTR MSG_DAC_PERCENT_N                    = _UxGT("@ Привод, %");
  LSTR MSG_ERROR_TMC                        = _UxGT("СБОЙ СВЯЗИ С TMC");
  LSTR MSG_DAC_EEPROM_WRITE                 = _UxGT("Запись DAC в EEPROM");
  LSTR MSG_FILAMENT_CHANGE_HEADER           = _UxGT("ЗАМЕНА ФИЛАМЕНТА");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE     = _UxGT("ПЕЧАТЬ НА ПАУЗЕ");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD      = _UxGT("ЗАГРУЗКА ФИЛАМЕНТА");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD    = _UxGT("ВЫГРУЗКА ФИЛАМЕНТА");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER    = _UxGT("ОПЦИИ ПРОДОЛЖЕНИЯ:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE     = _UxGT("Выдавить ещё");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME    = _UxGT("Возобновить печать");
  LSTR MSG_FILAMENT_CHANGE_NOZZLE           = _UxGT("  Сопла: ");
  LSTR MSG_RUNOUT_SENSOR                    = _UxGT("Датчик оконч.филам.");
  LSTR MSG_RUNOUT_DISTANCE_MM               = _UxGT("До конца, мм");
  LSTR MSG_KILL_HOMING_FAILED               = _UxGT("Ошибка парковки");
  LSTR MSG_LCD_PROBING_FAILED               = _UxGT("Ошибка зондирования");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER      = _UxGT("ВЫБИРЕТЕ ФИЛАМЕНТ");
  LSTR MSG_MMU2_MENU                        = _UxGT("Настройки MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE               = _UxGT("Обнови прошивку MMU");
  LSTR MSG_MMU2_NOT_RESPONDING              = _UxGT("MMU требует внимания");
  LSTR MSG_MMU2_RESUME                      = _UxGT("Продолжить печать");
  LSTR MSG_MMU2_RESUMING                    = _UxGT("Продолжение...");
  LSTR MSG_MMU2_LOAD_FILAMENT               = _UxGT("Загрузить филамент");
  LSTR MSG_MMU2_LOAD_ALL                    = _UxGT("Загрузить всё");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE              = _UxGT("Загрузить в сопло");
  LSTR MSG_MMU2_EJECT_FILAMENT              = _UxGT("Извлечь филамент");
  LSTR MSG_MMU2_EJECT_FILAMENT_N            = _UxGT("Извлечь филамент ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT             = _UxGT("Выгрузить филамент");
  LSTR MSG_MMU2_LOADING_FILAMENT            = _UxGT("Загрузка %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT           = _UxGT("Извлеч.филамента...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT          = _UxGT("Выгрузка....");
  LSTR MSG_MMU2_ALL                         = _UxGT("Всё");
  LSTR MSG_MMU2_FILAMENT_N                  = _UxGT("Филамент ~");
  LSTR MSG_MMU2_RESET                       = _UxGT("Перезапуск MMU");
  LSTR MSG_MMU2_RESETTING                   = _UxGT("Перезапуск MMU...");
  LSTR MSG_MMU2_EJECT_RECOVER               = _UxGT("Удалите и нажмите");

  LSTR MSG_MIX                              = _UxGT("Смешив.");
  LSTR MSG_MIX_COMPONENT_N                  = _UxGT("Компонент {");
  LSTR MSG_MIXER                            = _UxGT("Смеситель");
  LSTR MSG_GRADIENT                         = _UxGT("Градиент");
  LSTR MSG_FULL_GRADIENT                    = _UxGT("Полный градиент");
  LSTR MSG_CYCLE_MIX                        = _UxGT("Цикличное смешивание");
  LSTR MSG_GRADIENT_MIX                     = _UxGT("Градиент смешивания");
  LSTR MSG_REVERSE_GRADIENT                 = _UxGT("Сменить градиент");
  LSTR MSG_TOGGLE_MIX                       = _UxGT("Перекл. смешивание");
  LSTR MSG_ACTIVE_VTOOL                     = _UxGT("Актив.В-инструм.");
  LSTR MSG_START_VTOOL                      = _UxGT("В-инструм.нач.");
  LSTR MSG_END_VTOOL                        = _UxGT("В-инструм.кон.");
  LSTR MSG_GRADIENT_ALIAS                   = _UxGT("Псевдоним В-инстр.");
  LSTR MSG_RESET_VTOOLS                     = _UxGT("Сброс В-инструм.");
  LSTR MSG_COMMIT_VTOOL                     = _UxGT("Смешать В-инструм.");
  LSTR MSG_VTOOLS_RESET                     = _UxGT("В-инструм. сброшены");

  LSTR MSG_START_Z                          = _UxGT("Начало Z");
  LSTR MSG_END_Z                            = _UxGT(" Конец Z");

  LSTR MSG_GAMES                            = _UxGT("Игры");
  LSTR MSG_BRICKOUT                         = _UxGT("Brickout");
  LSTR MSG_INVADERS                         = _UxGT("Invaders");
  LSTR MSG_SNAKE                            = _UxGT("Sn4k3");
  LSTR MSG_MAZE                             = _UxGT("Maze");

  LSTR MSG_BAD_PAGE                         = _UxGT("Плохая страница");
  LSTR MSG_BAD_PAGE_SPEED                   = _UxGT("Плохая скор.стран.");

  LSTR MSG_EDIT_PASSWORD                    = _UxGT("Редактировать пароль");
  LSTR MSG_LOGIN_REQUIRED                   = _UxGT("Нужен логин");
  LSTR MSG_PASSWORD_SETTINGS                = _UxGT("Настройки пароля");
  LSTR MSG_ENTER_DIGIT                      = _UxGT("Введите цифру");
  LSTR MSG_CHANGE_PASSWORD                  = _UxGT("Смените пароль");
  LSTR MSG_REMOVE_PASSWORD                  = _UxGT("Удалить пароль");
  LSTR MSG_PASSWORD_SET                     = _UxGT("Пароль это ");
  LSTR MSG_START_OVER                       = _UxGT("Старт через");
  LSTR MSG_REMINDER_SAVE_SETTINGS           = _UxGT("Не забудь сохранить");
  LSTR MSG_PASSWORD_REMOVED                 = _UxGT("Пароль удалён");

  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //
  LSTR MSG_PAUSE_PRINT_PARKING              = _UxGT(MSG_1_LINE("Парковка..."));
  LSTR MSG_ADVANCED_PAUSE_WAITING           = _UxGT(MSG_1_LINE("Продолжить печать"));
  LSTR MSG_FILAMENT_CHANGE_INIT             = _UxGT(MSG_1_LINE("Ожидайте..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT           = _UxGT(MSG_1_LINE("Вставь и нажми"));
  LSTR MSG_FILAMENT_CHANGE_HEAT             = _UxGT(MSG_1_LINE("Нагреть сопло"));
  LSTR MSG_FILAMENT_CHANGE_HEATING          = _UxGT(MSG_1_LINE("Нагрев..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD           = _UxGT(MSG_1_LINE("Выгрузка..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD             = _UxGT(MSG_1_LINE("Загрузка..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE            = _UxGT(MSG_1_LINE("Прочистка..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE       = _UxGT(MSG_1_LINE("Завершить прочистку"));
  LSTR MSG_FILAMENT_CHANGE_RESUME           = _UxGT(MSG_1_LINE("Возобновление..."));

  LSTR MSG_TMC_DRIVERS                      = _UxGT("Драйвера TMC");
  LSTR MSG_TMC_CURRENT                      = _UxGT("Ток двигателей");
  LSTR MSG_TMC_HYBRID_THRS                  = _UxGT("Гибридный режим");
  LSTR MSG_TMC_HOMING_THRS                  = _UxGT("Чувствительность");
  LSTR MSG_TMC_STEPPING_MODE                = _UxGT("Режим драйвера");
  LSTR MSG_TMC_STEALTH_ENABLED              = _UxGT("Тихий режим вкл");

  LSTR MSG_SERVICE_RESET                    = _UxGT("Сброс");
  LSTR MSG_SERVICE_IN                       = _UxGT(" в:");
  LSTR MSG_BACKLASH                         = _UxGT("Люфт");
  LSTR MSG_BACKLASH_CORRECTION              = _UxGT("Исправление");
  LSTR MSG_BACKLASH_SMOOTHING               = _UxGT("Сглаживание");

  LSTR MSG_LEVEL_X_AXIS                     = _UxGT("Выровнять ось X");
  LSTR MSG_AUTO_CALIBRATE                   = _UxGT("Автокалибровка");
  LSTR MSG_HEATER_TIMEOUT                   = _UxGT("Таймаут нагрева");
  LSTR MSG_REHEAT                           = _UxGT("Возобновить нагрев");
  LSTR MSG_REHEATING                        = _UxGT("Нагрев...");

  LSTR MSG_PROBE_WIZARD                     = _UxGT("Мастер Z-зонда");
  LSTR MSG_PROBE_WIZARD_PROBING             = _UxGT("Зондир.контр.точки Z");
  LSTR MSG_PROBE_WIZARD_MOVING              = _UxGT("Движ.к точке зондир.");

  LSTR MSG_SOUND                            = _UxGT("Звук");

  LSTR MSG_TOP_LEFT                         = _UxGT("Верхний левый");
  LSTR MSG_BOTTOM_LEFT                      = _UxGT("Нижний левый");
  LSTR MSG_TOP_RIGHT                        = _UxGT("Верхний правый");
  LSTR MSG_BOTTOM_RIGHT                     = _UxGT("Нижний правый");
  LSTR MSG_CALIBRATION_COMPLETED            = _UxGT("Калибровка успешна");
  LSTR MSG_CALIBRATION_FAILED               = _UxGT("Ошибка калибровки");

  LSTR MSG_DRIVER_BACKWARD                  = _UxGT(" драйвер наоборот");

  LSTR MSG_SD_CARD                          = _UxGT("SD Карта");
  LSTR MSG_USB_DISK                         = _UxGT("USB Диск");

  LSTR MSG_SHORT_DAY                        = _UxGT("д"); // One character only
  LSTR MSG_SHORT_HOUR                       = _UxGT("ч"); // One character only
  LSTR MSG_SHORT_MINUTE                     = _UxGT("м"); // One character only

  LSTR MSG_FILAMENT_MAN                     = _UxGT("Управл.филаментом");
  LSTR MSG_MANUAL_LEVELING                  = _UxGT("Ручное выравнивание");
  LSTR MSG_TRAM_FL                          = _UxGT("Передний левый");
  LSTR MSG_TRAM_FR                          = _UxGT("Передний правый");
  LSTR MSG_TRAM_C                           = _UxGT("Центр");
  LSTR MSG_TRAM_BL                          = _UxGT("Задний левый");
  LSTR MSG_TRAM_BR                          = _UxGT("Задний правый");
  LSTR MSG_MANUAL_MESH                      = _UxGT("Сетка вручную");
  LSTR MSG_AUTO_MESH                        = _UxGT("Сетка автоматически");
  LSTR MSG_ERR_M428_TOO_FAR                 = _UxGT("Ошибка: слишком далеко!");
  LSTR MSG_TRAMMING_WIZARD                  = _UxGT("Помощник выравнив.");
  LSTR MSG_PREHEAT_HOTEND                   = _UxGT("Нагреть сопло");
  LSTR MSG_BED_TRAMMING_MANUAL              = _UxGT("Ручное выравнив.");
  LSTR MSG_MESH_VIEWER                      = _UxGT("Просмотр сетки");
  LSTR MSG_MESH_VIEW                        = _UxGT("Смотреть сетку");
  LSTR MSG_NO_VALID_MESH                    = _UxGT("Нет годной сетки");
  LSTR MSG_ACTIVATE_MESH                    = _UxGT("Включить сетку");
  LSTR MSG_MESH_INSET                       = _UxGT("Отступы сетки");
  LSTR MSG_MESH_MIN_X                       = _UxGT("Сетка X минимум");
  LSTR MSG_MESH_MAX_X                       = _UxGT("Сетка X максимум");
  LSTR MSG_MESH_MIN_Y                       = _UxGT("Сетка Y минимум");
  LSTR MSG_MESH_MAX_Y                       = _UxGT("Сетка Y максимум");
  LSTR MSG_MESH_AMAX                        = _UxGT("Максимальная зона");
  LSTR MSG_MESH_CENTER                      = _UxGT("Центрировать зону");
  LSTR MSG_MESH_CANCEL                      = _UxGT("Сетка отменена");
  LSTR MSG_UBL_TILT_MESH                    = _UxGT("Наколнить сетку");
  LSTR MSG_UBL_TILTING_GRID                 = _UxGT("Величина наклона");
  LSTR MSG_UBL_MESH_TILTED                  = _UxGT("Сетка наклонена");
  LSTR MSG_UBL_MESH_FILLED                  = _UxGT("Попущенные точки заполнены");
  LSTR MSG_UBL_MESH_INVALID                 = _UxGT("Негодная сетка");
  LSTR MSG_UBL_INVALID_SLOT                 = _UxGT("Сперва выберите слот сетки");
  LSTR MSG_MESH_ACTIVE                      = _UxGT("Сетка %i активна");
  LSTR MSG_LIVE_MOVE                        = _UxGT("Живое перемещение");
  LSTR MSG_CUTTER                           = _UxGT("Резак");
  LSTR MSG_PID_CYCLE                        = _UxGT("Циклы PID");
  LSTR MSG_PID_AUTOTUNE_FAILED              = _UxGT("Автонастройка PID не удалась!");
  LSTR MSG_BAD_HEATER_ID                    = _UxGT("Неверный экструдер.");
  LSTR MSG_TEMP_TOO_HIGH                    = _UxGT("Слишком высокая температура.");
  LSTR MSG_MPC_MEASURING_AMBIENT            = _UxGT("Тест потери тепла");
  LSTR MSG_MPC_HEATING_PAST_200             = _UxGT("Нагрев выше >200C");
  LSTR MSG_MPC_COOLING_TO_AMBIENT           = _UxGT("Охлаждение до окружающей");
  LSTR MSG_MPC_AUTOTUNE                     = _UxGT("Автонастройка MPC");
  LSTR MSG_MPC_EDIT                         = _UxGT("Изменить * MPC");
  LSTR MSG_MPC_POWER                        = _UxGT("Мощность нагревателя");
  LSTR MSG_MPC_POWER_E                      = _UxGT("Мощность *");
  LSTR MSG_MPC_BLOCK_HEAT_CAPACITY          = _UxGT("Теплоёмкость");
  LSTR MSG_MPC_BLOCK_HEAT_CAPACITY_E        = _UxGT("Теплоёмк. *");
  LSTR MSG_SENSOR_RESPONSIVENESS            = _UxGT("Отклик датчика");
  LSTR MSG_SENSOR_RESPONSIVENESS_E          = _UxGT("Отклик датч. *");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF           = _UxGT("Коэфф.окружения");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF_E         = _UxGT("Коэфф.окруж *");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF_FAN       = _UxGT("Коэфф.кулера");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF_FAN_E     = _UxGT("Коэфф.кулер *");
  LSTR MSG_INPUT_SHAPING                    = _UxGT("Input Shaping");
  LSTR MSG_SHAPING_ENABLE_N                 = _UxGT("Включить шейпинг @");
  LSTR MSG_SHAPING_DISABLE_N                = _UxGT("Выключить шейпинг @");
  LSTR MSG_SHAPING_FREQ_N                   = _UxGT("@ частота");
  LSTR MSG_SHAPING_ZETA_N                   = _UxGT("@ подавление");
  LSTR MSG_FILAMENT_EN                      = _UxGT("Филамент *");
  LSTR MSG_SEGMENTS_PER_SECOND              = _UxGT("Сегментов/сек");
  LSTR MSG_DRAW_MIN_X                       = _UxGT("Рисовать мин X");
  LSTR MSG_DRAW_MAX_X                       = _UxGT("Рисовать макс X");
  LSTR MSG_DRAW_MIN_Y                       = _UxGT("Рисовать мин Y");
  LSTR MSG_DRAW_MAX_Y                       = _UxGT("Рисовать макс Y");
  LSTR MSG_MAX_BELT_LEN                     = _UxGT("Макс.длина ремня");
  LSTR MSG_LINEAR_ADVANCE                   = _UxGT("Linear Advance");
  LSTR MSG_BRIGHTNESS                       = _UxGT("Яркость LCD");
  LSTR MSG_SCREEN_TIMEOUT                   = _UxGT("Таймаут LCD (м)");
  LSTR MSG_BRIGHTNESS_OFF                   = _UxGT("Выкл.подсветку");
  LSTR MSG_INFO_MACHINENAME                 = _UxGT("Название машины");
  LSTR MSG_INFO_SIZE                        = _UxGT("Размер");
  LSTR MSG_INFO_FWVERSION                   = _UxGT("Версия прошивки");
  LSTR MSG_INFO_BUILD                       = _UxGT("Дата сборки");
  LSTR MSG_BUTTON_CONFIRM                   = _UxGT("Подтвердить");
  LSTR MSG_BUTTON_CONTINUE                  = _UxGT("Продолжить");
  LSTR MSG_BUTTON_INFO                      = _UxGT("Инфо");
  LSTR MSG_BUTTON_LEVEL                     = _UxGT("Выровнять");
  LSTR MSG_BUTTON_PAUSE                     = _UxGT("Пауза");
  LSTR MSG_BUTTON_RESUME                    = _UxGT("Продолжить");
  LSTR MSG_BUTTON_ADVANCED                  = _UxGT("Расширанные");
  LSTR MSG_BUTTON_SAVE                      = _UxGT("Сохранить");
  LSTR MSG_BUTTON_PURGE                     = _UxGT("Прочистить");
  LSTR MSG_PAUSING                          = _UxGT("Пауза...");
  LSTR MSG_ADVANCED_PAUSE                   = _UxGT("Расширенная пауза");
  LSTR MSG_CONTINUE_PRINT_JOB               = _UxGT("Продолжить печать");
  LSTR MSG_TURN_OFF                         = _UxGT("Выключить принтер");
  LSTR MSG_END_LOOPS                        = _UxGT("Завершить петлю");
  LSTR MSG_STOPPING                         = _UxGT("Остановка...");
  LSTR MSG_REMAINING_TIME                   = _UxGT("Осталось");
  LSTR MSG_PRINTER_KILLED                   = _UxGT("Принтер убит!");
  LSTR MSG_FWRETRACT                        = _UxGT("Откат принтера");
  LSTR MSG_SINGLENOZZLE_WIPE_RETRACT        = _UxGT("Вытирание при откате");
  LSTR MSG_PARK_FAILED                      = _UxGT("Не удалось запарковать");
  LSTR MSG_FILAMENTUNLOAD                   = _UxGT("Выгрузить филамент");
  LSTR MSG_ATTACH_USB_MEDIA                 = _UxGT("Монтировать USB");
  LSTR MSG_BLTOUCH_SPEED_MODE               = _UxGT("Высокая скорость");
  LSTR MSG_MANUAL_PENUP                     = _UxGT("Поднять перо");
  LSTR MSG_MANUAL_PENDOWN                   = _UxGT("Опустить перо");
  LSTR MSG_ZPROBE_SETTINGS                  = _UxGT("Наторойки зонда");
  LSTR MSG_ZPROBE_MARGIN                    = _UxGT("Отступы зонда");
  LSTR MSG_Z_FEED_RATE                      = _UxGT("Скорость Z");
  LSTR MSG_ENABLE_HS_MODE                   = _UxGT("Включить режим ВС");
  LSTR MSG_ERR_TEMP_MALFUNCTION             = _UxGT("СБОЙ ТЕМПЕРАТУРЫ");
  LSTR MSG_PLEASE_WAIT                      = _UxGT("Ожидайте...");
  LSTR MSG_PREHEATING                       = _UxGT("Нагреваю...");
  LSTR MSG_DELTA_CALIBRATION_IN_PROGRESS    = _UxGT("Делаю дельта-калибровку");
  LSTR MSG_RESET_STATS                      = _UxGT("Сбросить статистику печати?");
  LSTR MSG_FAN_SPEED_FAULT                  = _UxGT("Сбой скорости кулера");

  LSTR MSG_COLORS_GET                       = _UxGT("Получить цвет");               // ProUI
  LSTR MSG_COLORS_SELECT                    = _UxGT("Выбрать цвета");               // ProUI
  LSTR MSG_COLORS_APPLIED                   = _UxGT("Цвета применены");             // ProUI
  LSTR MSG_COLORS_RED                       = _UxGT("Красный");                     // ProUI / JyersUI
  LSTR MSG_COLORS_GREEN                     = _UxGT("Зелёный");                     // ProUI / JyersUI
  LSTR MSG_COLORS_BLUE                      = _UxGT("Синий");                       // ProUI / JyersUI
  LSTR MSG_COLORS_WHITE                     = _UxGT("Белый");                       // ProUI
  LSTR MSG_UI_LANGUAGE                      = _UxGT("UI Language");                 // ProUI
  LSTR MSG_SOUND_ENABLE                     = _UxGT("Включить звук");               // ProUI
  LSTR MSG_LOCKSCREEN                       = _UxGT("Блокировать экран");           // ProUI
  LSTR MSG_LOCKSCREEN_LOCKED                = _UxGT("Принтер заблокирован,");       // ProUI
  LSTR MSG_LOCKSCREEN_UNLOCK                = _UxGT("Крутить для разблокировки.");  // ProUI
  LSTR MSG_PLEASE_WAIT_REBOOT               = _UxGT("Ждите перезагрузки.");         // ProUI
  LSTR MSG_MEDIA_NOT_INSERTED               = _UxGT("Нет носителя.");               // ProUI
  LSTR MSG_PLEASE_PREHEAT                   = _UxGT("Нагрейте сопло.");             // ProUI
  LSTR MSG_INFO_PRINT_COUNT_RESET           = _UxGT("Сбросить счетчик");            // ProUI
  LSTR MSG_FILAMENT_CHANGE_PURGE_CONTINUE   = _UxGT("Прочистить или продолжить?");  // ProUI
  LSTR MSG_EXTRUDER_MIN_TEMP                = _UxGT("Миню темп. экструдера.");      // ProUI

  LSTR MSG_FANCHECK                         = _UxGT("Пров.тахометра кулера");
  LSTR MSG_MMU2_REMOVE_AND_CLICK            = _UxGT("Уберите и кликните...");
  LSTR MSG_REHEATDONE                       = _UxGT("Нагрето");

  LSTR MSG_XATC                             = _UxGT("Помощник перекоса X");
  LSTR MSG_XATC_DONE                        = _UxGT("Перекос Х настроен!");
  LSTR MSG_XATC_UPDATE_Z_OFFSET             = _UxGT("Новое смещение Z-зонда ");
  LSTR MSG_HOST_SHUTDOWN                    = _UxGT("Выключить хост");

  // did not translate as there is no local terms/slang yet
  LSTR MSG_FIXED_TIME_MOTION                = _UxGT("FT Motion");
  LSTR MSG_FTM_CMPN_MODE                    = _UxGT("@ Режим комп.:");
  LSTR MSG_FTM_ZV                           = _UxGT("ZV");
  LSTR MSG_FTM_ZVD                          = _UxGT("ZVD");
  LSTR MSG_FTM_EI                           = _UxGT("EI");
  LSTR MSG_FTM_2HEI                         = _UxGT("2HEI");
  LSTR MSG_FTM_3HEI                         = _UxGT("3HEI");
  LSTR MSG_FTM_MZV                          = _UxGT("MZV");
  //LSTR MSG_FTM_ULENDO_FBS                 = _UxGT("Ulendo ФBС");
  //LSTR MSG_FTM_DISCTF                     = _UxGT("DISCTF");
  LSTR MSG_FTM_DYN_MODE                     = _UxGT("DF Mode:");
  LSTR MSG_FTM_Z_BASED                      = _UxGT("Z-based");
  LSTR MSG_FTM_MASS_BASED                   = _UxGT("Mass-based");
  LSTR MSG_FTM_BASE_FREQ_N                  = _UxGT("@ Base Freq.");
  LSTR MSG_FTM_DFREQ_K_N                    = _UxGT("@ Dyn. Freq.");
}

namespace LanguageWide_ru {
  using namespace LanguageNarrow_ru;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_MEDIA_INIT_FAIL                = _UxGT("Сбой инициализации SD");
    LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Расширенные настройки");
    LSTR MSG_KILL_SUBCALL_OVERFLOW          = _UxGT("Переполнение вызова");
    LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Установ. смещения дома");
    LSTR MSG_HOME_OFFSET_X                  = _UxGT("Смещение дома X"); // DWIN
    LSTR MSG_HOME_OFFSET_Y                  = _UxGT("Смещение дома Y"); // DWIN
    LSTR MSG_HOME_OFFSET_Z                  = _UxGT("Смещение дома Z"); // DWIN
    LSTR MSG_LAST_VALUE_SP                  = _UxGT("Последнее значение ");
    LSTR MSG_SPINDLE_MENU                   = _UxGT("Управлен.шпинделем");
    LSTR MSG_LASER_TOGGLE                   = _UxGT("Переключить лазер");
    LSTR MSG_SPINDLE_TOGGLE                 = _UxGT("Переключ. шпиндель");
    LSTR MSG_SPINDLE_POWER                  = _UxGT("Мощность шпинделя");
    LSTR MSG_LASER_POWER                    = _UxGT("Мощность лазера");
    LSTR MSG_LASER_PULSE_MS                 = _UxGT("Тестовый импульс мс");
    LSTR MSG_LASER_EVAC_TOGGLE              = _UxGT("Переключить обдув");
    LSTR MSG_SPINDLE_EVAC_TOGGLE            = _UxGT("Переключить вакуум");
    LSTR MSG_BED_TRAMMING_RAISE             = _UxGT("Вверх до срабатыв. зонда");
    LSTR MSG_BED_TRAMMING_IN_RANGE          = _UxGT("Углы в норме. Вырав.стола");
    LSTR MSG_MESH_EDITOR                    = _UxGT("Смещение по Z");
    LSTR MSG_EDITING_STOPPED                = _UxGT("Правка сетки окончена");
    LSTR MSG_UBL_BC_INSERT                  = _UxGT("Разместить шайбу,измерить");
    LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Убрать и замерить стол");
    LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Температура стола");
    LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Температура стола");
    LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Температура сопла");
    LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Температура сопла");
    LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Построить свою сетку");
    LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Правка сетки завершена");
    LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Построить холодную сетку");
    LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Проверить сетку $");
    LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Проверить свою сетку");
    LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("3-х точечное выравнивание");
    LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Сохранить сетку снаружи");
    LSTR MSG_UBL_INFO_UBL                   = _UxGT("Вывод информации UBL");
    LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Кол-во заполнителя");
    LSTR MSG_NEO2_PRESETS                   = _UxGT("Свет #2 предустановки");
    LSTR MSG_COOLER                         = _UxGT("Охлаждение лазера");
    LSTR MSG_COOLER_TOGGLE                  = _UxGT("Переключ. охлажд.");
    LSTR MSG_FLOWMETER_SAFETY               = _UxGT("Безопасн. потока");
    LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Отклонение угла");
    LSTR MSG_FILAMENT_DIAM                  = _UxGT("Диам. филамента");
    LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Диам. филамента *");
    LSTR MSG_RESTORE_DEFAULTS               = _UxGT("На базовые параметры");
    LSTR MSG_INIT_EEPROM                    = _UxGT("Инициализация EEPROM");
    LSTR MSG_CONTROL_RETRACT                = _UxGT("Откат, мм");
    LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Откат при смене, мм");
    LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Возврат при смене, мм");
    LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("Возврат при смене, V");
    LSTR MSG_AUTORETRACT                    = _UxGT("Автооткат");
    LSTR MSG_FILAMENT_SWAP_EXTRA            = _UxGT("Дополнительная длина");
    LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Начальная скор.");
    LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Скорость отката");
    LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Загрузить филамент *");
    LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Выгрузить филамент *");
    LSTR MSG_MOVE_NOZZLE_TO_BED             = _UxGT("Двигать сопло к столу");
    LSTR MSG_3POINT_LEVELING                = _UxGT("3-точечное выравнивание");
    LSTR MSG_LINEAR_LEVELING                = _UxGT("Линейное выравнивание");
    LSTR MSG_BILINEAR_LEVELING              = _UxGT("Билинейное выравнивание");
    LSTR MSG_MESH_DONE                      = _UxGT("Зондирование выполнено");
    LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Контроль утечки Т: Выкл");
    LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Контроль утечки Т: Вкл");
    LSTR MSG_HOTEND_IDLE_TIMEOUT            = _UxGT("Время простоя хотенда");
    LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Счётчик печати");
    LSTR MSG_INFO_PRINT_TIME                = _UxGT("Общее время печати");
    LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Наидольшее задание");
    LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Длина филамента");
    LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Датчик оконч. филамента");
    LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("Обновить прошивку MMU!");
    LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Извлечение филамента...");
    LSTR MSG_MIX                            = _UxGT("Смешивание");
    LSTR MSG_TOGGLE_MIX                     = _UxGT("Переключить смешивание");
    LSTR MSG_ACTIVE_VTOOL                   = _UxGT("Активация В-инструм.");
    LSTR MSG_START_VTOOL                    = _UxGT("Начало В-инструмента");
    LSTR MSG_END_VTOOL                      = _UxGT("Конец В-инструмента");
    LSTR MSG_GRADIENT_ALIAS                 = _UxGT("Псевдоним В-инструмента");
    LSTR MSG_RESET_VTOOLS                   = _UxGT("Сброс В-инструментов");
    LSTR MSG_COMMIT_VTOOL                   = _UxGT("Смешать В-инструменты");
    LSTR MSG_VTOOLS_RESET                   = _UxGT("В-инструменты сброшены");
    LSTR MSG_BAD_PAGE                       = _UxGT("Плохой индекс страницы");
    LSTR MSG_BAD_PAGE_SPEED                 = _UxGT("Плохая скорость страницы");
    LSTR MSG_REMINDER_SAVE_SETTINGS         = _UxGT("Не забудь сохранить!");
    LSTR MSG_PROBE_WIZARD_PROBING           = _UxGT("Зондиров. контр. точки Z");
    LSTR MSG_PROBE_WIZARD_MOVING            = _UxGT("Движение к точке зондиров.");
    LSTR MSG_FILAMENT_MAN                   = _UxGT("Управление филаментом");
  #endif
}

namespace LanguageTall_ru {
  using namespace LanguageWide_ru;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_3_LINE("Нажмите кнопку", "для продолжения", "печати"));
    LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_2_LINE("Ожидайте начала", "смены филамента"));
    LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_3_LINE("Вставьте филамент", "и нажмите кнопку", "для продолжения"));
    LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_3_LINE("Нажмите кнопку", "для нагрева", "сопла..."));
    LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_2_LINE("Нагрев сопла", "Ждите..."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_3_LINE("Ожидайте", "выгрузки", "филамента"));
    LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_3_LINE("Ожидайте", "загрузки", "филамента"));
    LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_3_LINE("Ожидайте", "экструзии", "филамента"));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_3_LINE("Нажмите кнопку", "для завершения", "прочистки филамента"));
    LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_3_LINE("Ожидайте", "возобновления", "печати"));
  #endif
}

namespace Language_ru {
  using namespace LanguageTall_ru;
}
