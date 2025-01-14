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

#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS_ROBIN2 supports up to 2 hotends / E steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "MKS_ROBIN2"
#endif

#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME BOARD_INFO_NAME
#endif

#define SRAM_EEPROM_EMULATION

//
// Servos
//
#define SERVO0_PIN                          PB0   // XS2-5
#define SERVO1_PIN                          PF7   // XS1-5
#define SERVO2_PIN                          PF8   // XS1-6

//
// Limit Switches
//
#define X_MIN_PIN                           PG8
#define X_MAX_PIN                           PG7
#define Y_MIN_PIN                           PG6
#define Y_MAX_PIN                           PG5
#define Z_MIN_PIN                           PG4
#define Z_MAX_PIN                           PG3

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Steppers
//
#define X_STEP_PIN                          PE6
#define X_DIR_PIN                           PE5
#define X_ENABLE_PIN                        PC13

#define Y_STEP_PIN                          PE3
#define Y_DIR_PIN                           PE2
#define Y_ENABLE_PIN                        PE4

#define Z_STEP_PIN                          PE0
#define Z_DIR_PIN                           PB9
#define Z_ENABLE_PIN                        PE1

#define E0_STEP_PIN                         PG10
#define E0_DIR_PIN                          PG9
#define E0_ENABLE_PIN                       PB8

#define E1_STEP_PIN                         PD3
#define E1_DIR_PIN                          PA15
#define E1_ENABLE_PIN                       PD6

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // T1 <-> E0
#define TEMP_1_PIN                          PC2   // T2 <-> E1
#define TEMP_BED_PIN                        PC0   // T0 <-> Bed

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PF3   // Heater0
#define HEATER_1_PIN                        PF2   // Heater1
#define HEATER_BED_PIN                      PF4   // Hotbed
#define FAN0_PIN                            PA7   // Fan0

//
// Misc. Functions
//
#define SDSS                                -1    // PB12

#define SD_DETECT_PIN                       PF9
#define BEEPER_PIN                          PG2

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
   * If the screen stays white, disable 'TFT_RESET_PIN' to let the bootloader init the screen.
   *
   * Setting a 'TFT_RESET_PIN' may cause a flicker when switching menus
   * because Marlin uses the reset as a failsafe to revive a glitchy LCD.
   */
  #define TFT_RESET_PIN                     PD13
  #define TFT_BACKLIGHT_PIN                 PD12

  #define LCD_USE_DMA_FSMC
  #define FSMC_CS_PIN                       PG12  // NE4
  #define FSMC_RS_PIN                       PF12  // A0
  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define TFT_BUFFER_WORDS                 14400

  #define BEEPER_PIN                        PG2

  #if NEED_TOUCH_PINS
    #define TOUCH_BUTTONS_HW_SPI
    #define TOUCH_BUTTONS_HW_SPI_DEVICE        1
    #define TOUCH_CS_PIN                    PD11  // SPI1_NSS
    #define TOUCH_SCK_PIN                   PB3   // SPI1_SCK
    #define TOUCH_MISO_PIN                  PB4   // SPI1_MISO
    #define TOUCH_MOSI_PIN                  PB5   // SPI1_MOSI
  #endif

#endif
