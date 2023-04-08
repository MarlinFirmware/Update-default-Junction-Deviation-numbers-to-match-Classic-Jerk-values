/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * pins_lcd.h - Define LCD pins based on the EXP connector / adapter
 */

#if defined(LCD_PINS_EN) || defined(DOGLCD_A0)

  // LCD pins were already defined in the board pins file

#else

  #include "lcd/adapters.h"

  #if ENABLED(CR10_STOCKDISPLAY)
    #include "lcd/CR10_STOCKDISPLAY.h"
  #elif ANY(ANET_FULL_GRAPHICS_LCD, CTC_A10S_A13)
    #include "lcd/ANET_FULL_GRAPHICS_LCD.h"
  #else

    // More displays to come
    #if HAS_LCD_SELECTED && !LCD_IS_SERIAL_HOST
      #define LCD_MIGRATION_WARNING
    #endif

  #endif

#endif

/**
 * Certain displays use LCD_PINS_RS as LCD_RESET_PIN
 */
#if !defined(LCD_RESET_PIN) && ANY(MKS_12864OLED, MKS_12864OLED_SSD1306, FYSETC_242_OLED_12864, ZONESTAR_12864OLED, K3D_242_OLED_CONTROLLER)
  #define LCD_RESET_PIN LCD_PINS_RS
#endif

/**
 * Make sure DOGLCD_SCK and DOGLCD_MOSI are defined.
 */
#if HAS_MARLINUI_U8GLIB
  #ifndef DOGLCD_SCK
    #define DOGLCD_SCK  SD_SCK_PIN
  #endif
  #ifndef DOGLCD_MOSI
    #define DOGLCD_MOSI SD_MOSI_PIN
  #endif
#endif
