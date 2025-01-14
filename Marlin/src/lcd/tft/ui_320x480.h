/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../inc/MarlinConfigPre.h"

// -------------------------------- Core Parameters --------------------------------

#define MENU_TEXT_X                 16
#define MENU_TEXT_Y                 tft_string.vcenter(MENU_ITEM_HEIGHT)

#define MENU_ITEM_ICON_X            5
#define MENU_ITEM_ICON_Y            5
#define MENU_ITEM_ICON_SPACE        (5 + 32 + 5)

#define MENU_ITEM_HEIGHT            43
#define MENU_LINE_HEIGHT            (MENU_ITEM_HEIGHT + 2)
#define MENU_TOP_LINE_Y             4

#if TFT_FONT == NOTOSANS || TFT_FONT == HELVETICA
  #define FONT_SIZE                 19
#elif TFT_FONT == UNIFONT
  #define FONT_SIZE                 20
#endif

#define FONT_LINE_HEIGHT            34

// ---------------------------------------------------------------------------------

#define VCENTER tft_string.vcenter(FONT_LINE_HEIGHT)

// MarlinUI::show_bootscreen()
#ifndef BOOTSCREEN_LOGO
  #if ENABLED(BOOT_MARLIN_LOGO_SMALL)
    #define BOOTSCREEN_LOGO         MarlinLogo195x59x16
    #define BOOTSCREEN_LOGO_W       195
    #define BOOTSCREEN_LOGO_H       59
    #define BOOTSCREEN_SITE_URL_Y   (TFT_HEIGHT - 70)
  #else
    #define BOOTSCREEN_LOGO         MarlinLogo480x320x16
    #define BOOTSCREEN_LOGO_W       TFT_WIDTH
    #define BOOTSCREEN_LOGO_H       TFT_HEIGHT
    #define BOOTSCREEN_SITE_URL_Y   (TFT_HEIGHT - 90)
  #endif
#endif

#if !defined(NAVIGATION_PAGE_UP_X) && !defined(NAVIGATION_PAGE_UP_Y) && !defined(NAVIGATION_PAGE_DOWN_X) && !defined(NAVIGATION_PAGE_DOWN_Y) && !defined(NAVIGATION_BACK_X) && !defined(NAVIGATION_BACK_Y)
  #define NAVIGATION_PAGE_UP_X      48
  #define NAVIGATION_PAGE_UP_Y      TFT_HEIGHT - 38
  #define NAVIGATION_PAGE_DOWN_X    240
  #define NAVIGATION_PAGE_DOWN_Y    TFT_HEIGHT - 38
  #define NAVIGATION_BACK_X         144
  #define NAVIGATION_BACK_Y         TFT_HEIGHT - 38
#endif

#if !defined(BUTTON_DECREASE_X) && !defined(BUTTON_DECREASE_Y) && !defined(BUTTON_INCREASE_X) && !defined(BUTTON_INCREASE_Y) && !defined(BUTTON_CLICK_X) && !defined(BUTTON_CLICK_Y)
  #define BUTTON_DECREASE_X         32
  #define BUTTON_DECREASE_Y         TFT_HEIGHT - 64
  #define BUTTON_INCREASE_X         224
  #define BUTTON_INCREASE_Y         TFT_HEIGHT - 64
  #define BUTTON_CLICK_X            128
  #define BUTTON_CLICK_Y            TFT_HEIGHT - 64
#endif

#if !defined(BUTTON_CANCEL_X) && !defined(BUTTON_CANCEL_Y) && !defined(BUTTON_CONFIRM_X) && !defined(BUTTON_CONFIRM_Y)
  #define BUTTON_CANCEL_X           48
  #define BUTTON_CANCEL_Y           TFT_HEIGHT - 64
  #define BUTTON_CONFIRM_X          208
  #define BUTTON_CONFIRM_Y          TFT_HEIGHT - 64
#endif

// MarlinUI::draw_kill_screen()
#if !defined(KILL_SCREEN_STATUS_Y) && !defined(KILL_SCREEN_HALTED_Y) && !defined(KILL_SCREEN_RESET_Y)
  #define KILL_SCREEN_STATUS_Y      139
  #define KILL_SCREEN_HALTED_Y      229
  #define KILL_SCREEN_RESET_Y       274
#endif

// draw_heater_status() and draw_fan_status()
#if !defined(TEMP_FAN_CONTROL_W) && !defined(TEMP_FAN_CONTROL_H)
  #define TEMP_FAN_CONTROL_W        80
  #define TEMP_FAN_CONTROL_H        120
#endif
#if !defined(TEMP_ICON_X) && !defined(TEMP_ICON_Y)
  #define TEMP_ICON_X               0
  #define TEMP_ICON_Y               28
#endif
#if !defined(FAN_ICON_X) && !defined(FAN_ICON_Y)
  #define FAN_ICON_X                0
  #define FAN_ICON_Y                20
#endif

#if !defined(TEMP_CURRENT_TEXT_X) && !defined(TEMP_CURRENT_TEXT_Y) && !defined(TEMP_TARGET_TEXT_X) && !defined(TEMP_TARGET_TEXT_Y)
  #define TEMP_CURRENT_TEXT_X       tft_string.center(TEMP_FAN_CONTROL_W) + 2
  #define TEMP_CURRENT_TEXT_Y       80 + VCENTER
  #define TEMP_TARGET_TEXT_X        tft_string.center(TEMP_FAN_CONTROL_W) + 2
  #define TEMP_TARGET_TEXT_Y        6 + VCENTER
#endif

#if !defined(FAN_TEXT_X) && !defined(FAN_TEXT_Y)
  #define FAN_TEXT_X                tft_string.center(TEMP_FAN_CONTROL_W) + 6
  #define FAN_TEXT_Y                80 + VCENTER
#endif

// MarlinUI::draw_status_screen()
#if !defined(MAX_ITEMS) && !defined(ITEM_X) && !defined(ITEM_Y)
  #define MAX_ITEMS                 4
  #define ITEM_X(N)                 (TFT_WIDTH / _MIN(ITEMS_COUNT, MAX_ITEMS) - TEMP_FAN_CONTROL_W) / 2 + (TFT_WIDTH * N / _MIN(ITEMS_COUNT, MAX_ITEMS))
  #define ITEM_Y                    4
#endif

#if !defined(COORDINATES_W) && !defined(COORDINATES_H) && !defined(COORDINATES_X) && !defined(COORDINATES_Y)
  #define COORDINATES_W             TFT_WIDTH - 8
  #define COORDINATES_H             FONT_LINE_HEIGHT * 2
  #define COORDINATES_X             4
  #define COORDINATES_Y             132
#endif

#if !defined(E_MARK_X) && !defined(E_MARK_Y) && !defined(E_VALUE_X) && !defined(E_VALUE_Y)
  #define E_MARK_X                  10
  #define E_MARK_Y                  VCENTER + FONT_LINE_HEIGHT
  #define E_VALUE_X                 103 - tft_string.width()
  #define E_VALUE_Y                 VCENTER + FONT_LINE_HEIGHT
#endif

#if !defined(X_MARK_X) && !defined(X_MARK_Y) && !defined(X_VALUE_X) && !defined(X_VALUE_Y)
  #define X_MARK_X                  10
  #define X_MARK_Y                  VCENTER
  #define X_VALUE_X                 103 - tft_string.width()
  #define X_VALUE_Y                 VCENTER
#endif

#if !defined(Y_MARK_X) && !defined(Y_MARK_Y) && !defined(Y_VALUE_X) && !defined(Y_VALUE_Y)
  #define Y_MARK_X                  176
  #define Y_MARK_Y                  VCENTER
  #define Y_VALUE_X                 265 - tft_string.width()
  #define Y_VALUE_Y                 VCENTER
#endif

#if !defined(Z_MARK_X) && !defined(Z_MARK_Y) && !defined(Z_VALUE_X) && !defined(Z_VALUE_Y)
  #define Z_MARK_X                  176
  #define Z_MARK_Y                  VCENTER + FONT_LINE_HEIGHT
  #define Z_VALUE_X                 297 - tft_string.width()
  #define Z_VALUE_Y                 VCENTER + FONT_LINE_HEIGHT
  #define Z_VALUE_OFFSET            32
#endif

#if !defined(FEEDRATE_W) && !defined(FEEDRATE_H) && !defined(FEEDRATE_X) && !defined(FEEDRATE_Y)
  #define FEEDRATE_W                120
  #define FEEDRATE_H                32
  #define FEEDRATE_X                24
  #define FEEDRATE_Y                218
#endif
#if !defined(FEEDRATE_ICON_X) && !defined(FEEDRATE_ICON_Y) && !defined(FEEDRATE_ICON_Y) && !defined(FEEDRATE_TEXT_Y)
  #define FEEDRATE_ICON_X           0
  #define FEEDRATE_ICON_Y           0
  #define FEEDRATE_TEXT_X           36
  #define FEEDRATE_TEXT_Y           tft_string.vcenter(30)
#endif

#if !defined(FLOWRATE_W) && !defined(FLOWRATE_H) && !defined(FLOWRATE_X) && !defined(FLOWRATE_Y)
  #define FLOWRATE_W                120
  #define FLOWRATE_H                32
  #define FLOWRATE_X                176
  #define FLOWRATE_Y                218
#endif
#if !defined(FLOWRATE_ICON_X) && !defined(FLOWRATE_ICON_Y) && !defined(FLOWRATE_TEXT_X) && !defined(FLOWRATE_TEXT_Y)
  #define FLOWRATE_ICON_X           0
  #define FLOWRATE_ICON_Y           0
  #define FLOWRATE_TEXT_X           36
  #define FLOWRATE_TEXT_Y           tft_string.vcenter(30)
#endif

#if !defined(MENU_ICON_X) && !defined(MENU_ICON_Y)
  #define MENU_ICON_X               250
  #define MENU_ICON_Y               354
#endif
#if !defined(SDCARD_ICON_X) && !defined(SDCARD_ICON_Y)
  #define SDCARD_ICON_X             6
  #define SDCARD_ICON_Y             354
#endif

#if !defined(ELAPSED_TIME_W) && !defined(ELAPSED_TIME_H) && !defined(ELAPSED_TIME_X) && !defined(ELAPSED_TIME_Y)
  #define ELAPSED_TIME_W            168
  #define ELAPSED_TIME_H            29
  #define ELAPSED_TIME_X            (TFT_WIDTH - ELAPSED_TIME_W) / 2
  #define ELAPSED_TIME_Y            388
#endif

#if !defined(REMAINING_TIME_W) && !defined(REMAINING_TIME_H) && !defined(REMAINING_TIME_X) && !defined(REMAINING_TIME_Y)
  #if ALL(SHOW_ELAPSED_TIME, SHOW_REMAINING_TIME)
    #define REMAINING_TIME_Y        354
  #elif ENABLED(SHOW_REMAINING_TIME)
    #define REMAINING_TIME_Y        388
  #endif

  #define REMAINING_TIME_H          29
  #define REMAINING_TIME_W          168
  #define REMAINING_TIME_X          (TFT_WIDTH - REMAINING_TIME_W) / 2
#endif

#if !defined(ELAPSED_TIME_IMAGE_X) && !defined(ELAPSED_TIME_IMAGE_Y) && !defined(ELAPSED_TIME_TEXT_X) && !defined(ELAPSED_TIME_TEXT_Y)
  #define ELAPSED_TIME_IMAGE_X      tft_string.center(ELAPSED_TIME_W) - 18
  #define ELAPSED_TIME_IMAGE_Y      0
  #define ELAPSED_TIME_TEXT_X       tft_string.center(ELAPSED_TIME_W) + 18
  #define ELAPSED_TIME_TEXT_Y       tft_string.vcenter(29)
#endif

#if !defined(REMAINING_TIME_IMAGE_X) && !defined(REMAINING_TIME_IMAGE_Y) && !defined(REMAINING_TIME_TEXT_X) && !defined(REMAINING_TIME_TEXT_Y)
  #define REMAINING_TIME_IMAGE_X    tft_string.center(ELAPSED_TIME_W) - 18
  #define REMAINING_TIME_IMAGE_Y    0
  #define REMAINING_TIME_TEXT_X     tft_string.center(ELAPSED_TIME_W) + 18
  #define REMAINING_TIME_TEXT_Y     tft_string.vcenter(29)
#endif

#if !defined(PROGRESS_BAR_W) && !defined(PROGRESS_BAR_H) && !defined(PROGRESS_BAR_X) && !defined(PROGRESS_BAR_Y)
  #define PROGRESS_BAR_W            TFT_WIDTH - 8
  #define PROGRESS_BAR_H            12
  #define PROGRESS_BAR_X            4
  #define PROGRESS_BAR_Y            422
#endif

#if !defined(STATUS_MESSAGE_W) && !defined(STATUS_MESSAGE_H) && !defined(STATUS_MESSAGE_X) && !defined(STATUS_MESSAGE_Y)
  #define STATUS_MESSAGE_W          TFT_WIDTH
  #define STATUS_MESSAGE_H          FONT_LINE_HEIGHT
  #define STATUS_MESSAGE_X          0
  #define STATUS_MESSAGE_Y          440
#endif

#if !defined(STATUS_MESSAGE_TEXT_X) && !defined(STATUS_MESSAGE_TEXT_Y)
  #define STATUS_MESSAGE_TEXT_X     tft_string.center(STATUS_MESSAGE_W)
  #define STATUS_MESSAGE_TEXT_Y     VCENTER
#endif

// MenuEditItemBase::draw_edit_screen()
#if !defined(SLIDER_W) && !defined(SLIDER_Y)
  #define SLIDER_W                  288
  #define SLIDER_Y                  280
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)
  #if !defined(UBL_X_LABEL_X) && !defined(UBL_X_TEXT_X) && !defined(UBL_Y_LABEL_X) && !defined(UBL_Y_TEXT_X)
    #define UBL_X_LABEL_X           (TFT_WIDTH / 2 - 120)
    #define UBL_X_TEXT_X            (TFT_WIDTH / 2 - 16) - tft_string.width()
    #define UBL_Y_LABEL_X           (TFT_WIDTH / 2 + 16)
    #define UBL_Y_TEXT_X            (TFT_WIDTH / 2 + 120) - tft_string.width()
  #endif

  // MarlinUI::ubl_plot()
  #if !defined(UBL_GRID_W) && !defined(UBL_GRID_H) && !defined(UBL_GRID_X) && !defined(UBL_GRID_Y) && !defined(UBL_CONTROL_OFFSET)
    #define UBL_GRID_W              192
    #define UBL_GRID_H              192
    #define UBL_GRID_X              8
    #define UBL_GRID_Y              8
    #define UBL_CONTROL_OFFSET      16
  #endif

  #if !defined(UBL_COORDINATES_W) && !defined(UBL_COORDINATES_X) && !defined(UBL_X_COORDINATE_Y) && !defined(UBL_Y_COORDINATE_Y) && !defined(UBL_Z_COORDINATE_Y)
    #define UBL_COORDINATES_W       168
    #define UBL_COORDINATES_X       20

    #define UBL_X_COORDINATE_Y      283
    #define UBL_Y_COORDINATE_Y      326
    #define UBL_Z_COORDINATE_Y      369
  #endif

  #if !defined(UBL_ACTION_X) && !defined(UBL_ACTION_Y)
    #define UBL_ACTION_X            240
    #define UBL_ACTION_Y            332
  #endif
#endif
