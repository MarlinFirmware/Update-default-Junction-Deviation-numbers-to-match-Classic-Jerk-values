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
 * lcd/dogm/marlinui_DOGM.h
 */

#include "../../inc/MarlinConfigPre.h"

#include <U8glib-HAL.h>
#include "u8g/HAL_LCD_class_defines.h"

//#define ALTERNATIVE_LCD

#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

  // RepRapWorld Graphical LCD

  #if HAS_MEDIA
    #ifdef __SAMD21__
      #define U8G_CLASS U8GLIB_ST7920_128X64_4X_HAL
    #else
      // Hardware SPI on DUE
      #define U8G_CLASS U8GLIB_ST7920_128X64_4X
    #endif
    #define U8G_PARAM LCD_PINS_RS
  #elif (LCD_PINS_D4 == SD_SCK_PIN) && (LCD_PINS_EN == SD_MOSI_PIN)
    // Hardware SPI shared with SD Card
    #define U8G_CLASS U8GLIB_ST7920_128X64_4X_HAL
    #define U8G_PARAM LCD_PINS_RS
  #else
    // Software SPI
    #define U8G_CLASS U8GLIB_ST7920_128X64_4X
    #define U8G_PARAM LCD_PINS_D4, LCD_PINS_EN, LCD_PINS_RS
  #endif

#elif IS_U8GLIB_ST7920

  // RepRap Discount Full Graphics Smart Controller
  // and other variant LCDs using ST7920

  #if !HAS_MEDIA && (LCD_PINS_D4 == SD_SCK_PIN) && (LCD_PINS_EN == SD_MOSI_PIN)
    #define U8G_CLASS U8GLIB_ST7920_128X64_4X_HAL               // 2 stripes, HW SPI (Shared with SD card. Non-standard LCD adapter on AVR.)
    #define U8G_PARAM LCD_PINS_RS
  #else
    #if ENABLED(ALTERNATIVE_LCD)
      #define U8G_CLASS U8GLIB_ST7920_128X64_4X                 // 2 stripes, SW SPI (Original u8glib device)
    #else
      #define U8G_CLASS U8GLIB_ST7920_128X64_RRD                // Adjust stripes with PAGE_HEIGHT in ultralcd_st7920_u8glib_rrd.h
    #endif
    #define U8G_PARAM LCD_PINS_D4, LCD_PINS_EN, LCD_PINS_RS     // AVR version ignores these pin settings
                                                                // HAL version uses these pin settings
  #endif

#elif ENABLED(CARTESIO_UI)

  // CartesioUI LCD

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_DOGM128_2X                         // 4 stripes
    #define FORCE_SOFT_SPI                                      // SW-SPI
  #else
    #define U8G_CLASS U8GLIB_DOGM128_2X                         // 4 stripes (HW-SPI)
  #endif

#elif IS_U8GLIB_LM6059_AF

  // Based on the Adafruit ST7565 (https://www.adafruit.com/products/250)

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_LM6059                             // 8 stripes (HW-SPI)
  #else
    #define U8G_CLASS U8GLIB_LM6059_2X                          // 4 stripes (HW-SPI)
  #endif

#elif IS_U8GLIB_ST7565_64128N

  // MaKrPanel, Mini Viki, Viki 2.0, AZSMZ 12864 ST7565 controller

  #define SMART_RAMPS MB(RAMPS_SMART_EFB, RAMPS_SMART_EEB, RAMPS_SMART_EFF, RAMPS_SMART_EEF, RAMPS_SMART_SF)
  #define U8G_CLASS U8GLIB_64128N_2X_HAL                        // 4 stripes (HW-SPI)

  #if (SMART_RAMPS && defined(__SAM3X8E__)) || (defined(DOGLCD_SCK) && (DOGLCD_SCK != -1 && DOGLCD_SCK != SD_SCK_PIN)) || (defined(DOGLCD_MOSI) && (DOGLCD_MOSI != -1 && DOGLCD_MOSI != SD_MOSI_PIN))
    #define FORCE_SOFT_SPI                                      // SW-SPI
  #endif

#elif ANY(FYSETC_MINI_12864, MKS_MINI_12864, ENDER2_STOCKDISPLAY)

  // The FYSETC Mini 12864 display                              // "4 stripes"

  // The MKS_MINI_12864 V1/V2 aren't exact copies of the MiniPanel.
  // Panel management is in u8g_dev_uc1701_mini12864_HAL.cpp with
  // extra delays added to remove glitches seen with fast MCUs.

  #define U8G_CLASS U8GLIB_MINI12864_2X_HAL                     // 8 stripes (HW-SPI)

#elif ENABLED(MINIPANEL)

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_MINI12864
  #else
    #define U8G_CLASS U8GLIB_MINI12864_2X                       // 8 stripes (HW-SPI)
  #endif

#elif ENABLED(MKS_12864OLED_SSD1306)

  // MKS 128x64 (SSD1306) OLED I2C LCD

  #define FORCE_SOFT_SPI                                        // SW-SPI

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SSD1306_128X64_2X                  // 4 stripes
  #else
    #define U8G_CLASS U8GLIB_SSD1306_128X64                     // 8 stripes
  #endif

#elif ANY(FYSETC_242_OLED_12864, K3D_242_OLED_CONTROLLER)

  // FYSETC OLED 2.42" 128 × 64 Full Graphics Controller
  // or K3D OLED 2.42" 128 × 64 Full Graphics Controller

  #define FORCE_SOFT_SPI                                        // SW-SPI

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SSD1306_128X64_2X                  // 4 stripes
  #else
    #define U8G_CLASS U8GLIB_SSD1309_128X64_HAL
  #endif

#elif ENABLED(ZONESTAR_12864OLED_SSD1306)

  // Zonestar SSD1306 OLED SPI LCD

  #define FORCE_SOFT_SPI                                        // SW-SPI
  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SH1306_128X64_2X                   // 4 stripes
  #else
    #if ENABLED(U8GLIB_SSD1306_SPI)
      #define U8G_CLASS U8GLIB_SSD1306_128X64_SW_SPI_HAL
    #else
      #define U8G_CLASS U8GLIB_SH1306_128X64                    // 8 stripes
    #endif
  #endif

#elif ANY(MKS_12864OLED, ZONESTAR_12864OLED)

  // MKS 128x64 (SH1106) OLED I2C LCD
  // - or -
  // Zonestar SH1106 OLED SPI LCD

  #define FORCE_SOFT_SPI                                        // SW-SPI
  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SH1106_128X64_2X                   // 4 stripes
  #else
    #if ENABLED(U8GLIB_SH1106_SPI)
      #define U8G_CLASS U8GLIB_SH1106_128X64_SW_SPI_HAL
    #else
      #define U8G_CLASS U8GLIB_SH1106_128X64                    // 8 stripes
    #endif
  #endif

#elif ENABLED(U8GLIB_SH1106_EINSTART)

  // Connected via motherboard header

  #define U8G_CLASS U8GLIB_SH1106_128X64
  #define U8G_PARAM DOGLCD_SCK, DOGLCD_MOSI, DOGLCD_CS, LCD_PINS_DC, LCD_PINS_RS

#elif ENABLED(U8GLIB_SH1106)

  // Generic SH1106 OLED I2C LCD

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SH1106_128X64_2X_I2C_2_WIRE        // 4 stripes
  #else
    #define U8G_CLASS U8GLIB_SH1106_128X64_2X                   // 4 stripes
  #endif
  #define U8G_PARAM (U8G_I2C_OPT_NONE | U8G_I2C_OPT_FAST)       // I2C

#elif ENABLED(U8GLIB_SSD1309)

  // Generic support for SSD1309 OLED I2C LCDs

  #define U8G_CLASS TERN(USE_LCD_DOUBLE_BUFFERING, U8GLIB_SSD1309_128X64_F, U8GLIB_SSD1309_128X64)
  #define U8G_PARAM (U8G_I2C_OPT_NONE | U8G_I2C_OPT_FAST)       // I2C

#elif ENABLED(U8GLIB_SSD1306)

  // Generic SSD1306 OLED I2C LCD

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SSD1306_128X64_2X_I2C_2_WIRE       // 4 stripes
  #else
    #define U8G_CLASS U8GLIB_SSD1306_128X64_2X                  // 4 stripes
  #endif
  #define U8G_PARAM (U8G_I2C_OPT_NONE | U8G_I2C_OPT_FAST)

#elif TFT_SCALED_DOGLCD

  // Unspecified 320x240 TFT pre-initialized by built-in bootloader

  #define U8G_CLASS U8GLIB_TFT_320X240_UPSCALE_FROM_128X64
  #if HAS_FSMC_GRAPHICAL_TFT
    #define U8G_PARAM FSMC_CS_PIN, FSMC_RS_PIN
  #else
    #define U8G_PARAM -1, -1
  #endif

#else

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_DOGM128                            // 8 stripes (HW-SPI)
  #else
    #define U8G_CLASS U8GLIB_DOGM128_2X                         // 4 stripes (HW-SPI)
  #endif

#endif

// Use HW-SPI if no other option is specified
#ifndef U8G_PARAM
  #if ENABLED(FORCE_SOFT_SPI)
    #define U8G_PARAM DOGLCD_SCK, DOGLCD_MOSI, DOGLCD_CS, DOGLCD_A0 // SW-SPI
  #else
    #define U8G_PARAM DOGLCD_CS, DOGLCD_A0                          // HW-SPI
  #endif
#endif

// For selective rendering within a Y range
#define PAGE_OVER(ya)         ((ya) <= u8g.getU8g()->current_page.y1) // Does the current page follow a region top?
#define PAGE_UNDER(yb)        ((yb) >= u8g.getU8g()->current_page.y0) // Does the current page precede a region bottom?
#define PAGE_CONTAINS(ya, yb) ((yb) >= u8g.getU8g()->current_page.y0 && (ya) <= u8g.getU8g()->current_page.y1) // Do two vertical regions overlap?

extern U8G_CLASS u8g;
