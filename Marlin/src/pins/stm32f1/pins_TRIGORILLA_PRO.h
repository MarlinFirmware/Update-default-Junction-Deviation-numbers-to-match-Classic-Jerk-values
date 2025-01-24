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
 * ANYCUBIC Trigorilla Pro (STM32F103ZET6) board pin assignments.
 * It is the same used by the Tronxy X5SA thanks to ftoz1 for sharing it
 * https://github.com/MarlinFirmware/Marlin/issues/14655
 * https://github.com/MarlinFirmware/Marlin/files/3401484/x5sa-main_board-2.pdf
 */

#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Trigorilla Pro supports up to 2 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "Trigorilla Pro"

#define BOARD_NO_NATIVE_USB

#define DISABLE_JTAG

//#define SWAPPED_Z_PLUGS

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION
#endif
#if ENABLED(FLASH_EEPROM_EMULATION)
  // SoC Flash (framework-arduinoststm32-maple/STM32F1/libraries/EEPROM/EEPROM.h)
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE
#else
  #define MARLIN_EEPROM_SIZE              0x800U  // On SD, Limit to 2K, require this amount of RAM
#endif

//
// Limit Switches
//
#define X_STOP_PIN                          PG10
#define Y_STOP_PIN                          PA12
#ifndef Z_MIN_PIN
  #ifdef SWAPPED_Z_PLUGS
    #define Z_MIN_PIN                       PA14
  #else
    #define Z_MIN_PIN                       PA13
  #endif
#endif
#ifndef Z_MAX_PIN
  #ifdef SWAPPED_Z_PLUGS
    #define Z_MAX_PIN                       PA13
  #else
    #define Z_MAX_PIN                       PA14
  #endif
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PC13
#define X_STEP_PIN                          PE5
#define X_DIR_PIN                           PE6

#define Y_ENABLE_PIN                        PE4
#define Y_STEP_PIN                          PE2
#define Y_DIR_PIN                           PE3

#define Z_ENABLE_PIN                        PE1
#define Z_STEP_PIN                          PB9
#define Z_DIR_PIN                           PE0

#define E0_ENABLE_PIN                       PB8
#define E0_STEP_PIN                         PB4
#define E0_DIR_PIN                          PB5

#define E1_ENABLE_PIN                       PG8
#define E1_STEP_PIN                         PC7
#define E1_DIR_PIN                          PC6

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA1   // TH1
#define TEMP_BED_PIN                        PA0   // TB1

//
// Heaters
//
#define HEATER_0_PIN                        PG12  // HEATER1
#define HEATER_BED_PIN                      PG11  // HOT BED
#define HEATER_BED_INVERTING                true

//
// Fans
//
#define CONTROLLER_FAN_PIN                  PD6   // FAN
#define FAN0_PIN                            PG13  // FAN
#define FAN1_PIN                            PG14  // FAN

//
// Misc
//
#define BEEPER_PIN                          PB0
#define LED_PIN                             PD3
//#define POWER_LOSS_PIN                    PG2   // PG4 PW_DET
#define FIL_RUNOUT_PIN                      PA15  // MT_DET

//
// TFT with FSMC interface
//
#if HAS_FSMC_TFT
  /**
   * Note: MKS Robin TFT screens use various TFT controllers
   * Supported screens are based on the ILI9341, ST7789V and ILI9328 (320x240)
   * ILI9488 is not supported
   * Define init sequences for other screens in u8g_dev_tft_320x240_upscale_from_128x64.cpp
   *
   * If the screen stays white, disable 'LCD_RESET_PIN' to let the bootloader init the screen.
   *
   * Setting an 'LCD_RESET_PIN' may cause a flicker when switching menus
   * because Marlin uses the reset as a failsafe to revive a glitchy LCD.
   */
  #define TFT_RESET_PIN                     PF11
  #define TFT_BACKLIGHT_PIN                 PD13

  #define LCD_USE_DMA_FSMC
  #define FSMC_CS_PIN                       PD7   // NE4
  #define FSMC_RS_PIN                       PD11  // A0
  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN
#else
  #define LCD_RESET_PIN                     PF11
  #define LCD_BACKLIGHT_PIN                 PD13
#endif

// XPT2046 Touch Screen calibration
#if ANY(TFT_COLOR_UI, TFT_LVGL_UI, TFT_CLASSIC_UI)
  #ifndef TOUCH_CALIBRATION_X
    #define TOUCH_CALIBRATION_X           -17181
  #endif
  #ifndef TOUCH_CALIBRATION_Y
    #define TOUCH_CALIBRATION_Y            11434
  #endif
  #ifndef TOUCH_OFFSET_X
    #define TOUCH_OFFSET_X                   501
  #endif
  #ifndef TOUCH_OFFSET_Y
    #define TOUCH_OFFSET_Y                    -9
  #endif
#endif

#if NEED_TOUCH_PINS
  #define TOUCH_CS_PIN                      PB7   // SPI2_NSS
  #define TOUCH_SCK_PIN                     PA5   // SPI2_SCK
  #define TOUCH_MISO_PIN                    PA6   // SPI2_MISO
  #define TOUCH_MOSI_PIN                    PA7   // SPI2_MOSI
#endif

// SPI1(PA7) & SPI3(PB5) not available
#define SPI_DEVICE                             2  // Maple

//
// SD Card
//
#ifndef ONBOARD_SDIO
  #define ONBOARD_SDIO
#endif
#if ENABLED(ONBOARD_SDIO)
  #define SD_SCK_PIN                        PB13  // SPI2 ok
  #define SD_MISO_PIN                       PB14  // SPI2 ok
  #define SD_MOSI_PIN                       PB15  // SPI2 ok
  #define SD_SS_PIN                         PC11  // PB12 is X- ok
  #define SD_DETECT_PIN                     -1    // SD_CD ok
#else
  // SD as custom software SPI (SDIO pins)
  #define SD_SCK_PIN                        PC12
  #define SD_MISO_PIN                       PC8
  #define SD_MOSI_PIN                       PD2
  #define ONBOARD_SD_CS_PIN                 PC11
  #define SD_SS_PIN                         PD2
  #define SD_DETECT_PIN                     -1
#endif

#undef SWAPPED_Z_PLUGS
