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
#pragma once

#define MOVE_AXIS_SCREEN  // Special "Move Axis" screen activated by touching coordinates area

#ifndef COLOR_BACKGROUND
  #define COLOR_BACKGROUND        COLOR_MIDNIGHT_BLUE
#endif
#ifndef COLOR_SELECTION_BG
  #define COLOR_SELECTION_BG      COLOR_DARK_PURPLE
#endif
#ifndef COLOR_WEBSITE_URL
  #define COLOR_WEBSITE_URL       COLOR_OCEAN_BOAT_BLUE
#endif

#ifndef COLOR_INACTIVE
  #define COLOR_INACTIVE          COLOR_GREY
#endif
#ifndef COLOR_COLD
  #define COLOR_COLD              COLOR_CYAN
#endif
#ifndef COLOR_HOTEND
  #define COLOR_HOTEND            COLOR_SCARLET
#endif
#ifndef COLOR_HEATED_BED
  #define COLOR_HEATED_BED        COLOR_DARK_ORANGE
#endif
#ifndef COLOR_CHAMBER
  #define COLOR_CHAMBER           COLOR_DARK_ORANGE
#endif
#ifndef COLOR_COOLER
  #define COLOR_COOLER            COLOR_DARK_ORANGE
#endif
#ifndef COLOR_FAN
  #define COLOR_FAN               COLOR_CYAN
#endif

#ifndef COLOR_AXIS_HOMED
  #define COLOR_AXIS_HOMED        COLOR_WHITE
#endif
#ifndef COLOR_AXIS_NOT_HOMED
  #define COLOR_AXIS_NOT_HOMED    COLOR_YELLOW
#endif

#ifndef COLOR_RATE_100
  #define COLOR_RATE_100          COLOR_VIVID_GREEN
#endif
#ifndef COLOR_RATE_ALTERED
  #define COLOR_RATE_ALTERED      COLOR_YELLOW
#endif

#ifndef COLOR_PRINT_TIME
  #define COLOR_PRINT_TIME        COLOR_CYAN
#endif

#ifndef COLOR_PROGRESS_FRAME
  #define COLOR_PROGRESS_FRAME    COLOR_WHITE
#endif
#ifndef COLOR_PROGRESS_BAR
  #define COLOR_PROGRESS_BAR      COLOR_BLUE
#endif
#ifndef COLOR_PROGRESS_BG
  #define COLOR_PROGRESS_BG       COLOR_BLACK
#endif

#ifndef COLOR_STATUS_MESSAGE
  #define COLOR_STATUS_MESSAGE    COLOR_YELLOW
#endif

#ifndef COLOR_CONTROL_ENABLED
  #define COLOR_CONTROL_ENABLED   COLOR_WHITE
#endif
#ifndef COLOR_CONTROL_DISABLED
  #define COLOR_CONTROL_DISABLED  COLOR_GREY
#endif
#ifndef COLOR_CONTROL_CANCEL
  #define COLOR_CONTROL_CANCEL    COLOR_SCARLET
#endif
#ifndef COLOR_CONTROL_CONFIRM
  #define COLOR_CONTROL_CONFIRM   COLOR_VIVID_GREEN
#endif
#ifndef COLOR_BUSY
  #define COLOR_BUSY              COLOR_SILVER
#endif

#ifndef COLOR_MENU_TEXT
  #define COLOR_MENU_TEXT         COLOR_YELLOW
#endif
#ifndef COLOR_MENU_TEXT
  #define COLOR_MENU_STATIC_TEXT  COLOR_SILVER
#endif
#ifndef COLOR_MENU_BACK_TEXT
  #define COLOR_MENU_BACK_TEXT    COLOR_CYAN
#endif
#ifndef COLOR_MENU_EDIT_TEXT
  #define COLOR_MENU_EDIT_TEXT    COLOR_LIGHT_BLUE
#endif
#ifndef COLOR_MENU_VALUE
  #define COLOR_MENU_VALUE        COLOR_WHITE
#endif

#ifndef COLOR_SLIDER
  #define COLOR_SLIDER            COLOR_WHITE
#endif
#ifndef COLOR_SLIDER_INACTIVE
  #define COLOR_SLIDER_INACTIVE   COLOR_GREY
#endif

#ifndef COLOR_UBL
  #define COLOR_UBL               COLOR_WHITE
#endif

#ifndef COLOR_TOUCH_CALIBRATION
  #define COLOR_TOUCH_CALIBRATION COLOR_WHITE
#endif

#ifndef COLOR_KILL_SCREEN_BG
  #define COLOR_KILL_SCREEN_BG    COLOR_MAROON
#endif
#ifndef COLOR_KILL_SCREEN_TEXT
  #define COLOR_KILL_SCREEN_TEXT  COLOR_WHITE
#endif
