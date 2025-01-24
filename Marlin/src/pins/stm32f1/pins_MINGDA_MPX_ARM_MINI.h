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
 * MPX ARM MINI (STM32F103ZET6) board pin assignments
 */

#if NOT_TARGET(STM32F1, STM32F1xx)
  #error "Oops! Select an STM32F1 board in 'Tools > Board.'"
#elif HOTENDS > 1 || E_STEPPERS > 1
  #error "MPX ARM Mini only supports 1 hotend / E stepper."
#endif

#define BOARD_INFO_NAME "Mingda MPX ARM Mini"

#define BOARD_NO_NATIVE_USB
#define DISABLE_DEBUG

//
// EEPROM
//

/*
//Mingda used an unknown EEPROM chip ATMLH753, so I turned on the emulation below.
//It is connected to EEPROM PB6 PB7

#define I2C_EEPROM
#undef NO_EEPROM_SELECTED
#define MARLIN_EEPROM_SIZE               0x1000U  // 4K
#define USE_SHARED_EEPROM                      1  // Use Platform-independent Arduino functions for I2C EEPROM
#define E2END                             0xFFFF  // EEPROM end address AT24C256 (32kB)
*/

#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

#define SPI_DEVICE                             2  // Maple

//
// Limit Switches
//
#define X_MIN_PIN                           PD6
#define X_MAX_PIN                           PG15  // To double check
#define Y_MIN_PIN                           PG9
#define Y_MAX_PIN                           PG14  // To double check
#define Z_MIN_PIN                           PG10
#define Z_MAX_PIN                           PG13

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PG11
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PD13
#define X_STEP_PIN                          PD12
#define X_DIR_PIN                           PD11

#define Y_ENABLE_PIN                        PG4
#define Y_STEP_PIN                          PG3
#define Y_DIR_PIN                           PG2

#define Z_ENABLE_PIN                        PG7
#define Z_STEP_PIN                          PG6
#define Z_DIR_PIN                           PG5

#define E0_ENABLE_PIN                       PC7
#define E0_STEP_PIN                         PC6
#define E0_DIR_PIN                          PG8

//
// Temperature Sensors
//
//#define TEMP_0_PIN                        PF6   // THERM_E0
//#define TEMP_0_PIN                        PB3   // E0 K+
#define TEMP_BED_PIN                        PF7   // THERM_BED

#define TEMP_0_CS_PIN                       PB5
#define TEMP_0_SCK_PIN                      PB3
#define TEMP_0_MISO_PIN                     PB4
#define TEMP_0_MOSI_PIN                     PA14

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB0
#define HEATER_BED_PIN                      PB1

#define FAN0_PIN                            PA0   // FAN

//
// SD Card
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#define ONBOARD_SDIO
#define SDIO_CLOCK                       4500000  // 4.5 MHz
#define SDIO_READ_RETRIES                     16

#define SD_DETECT_PIN                       PC5
#define ONBOARD_SPI_DEVICE                     1  // SPI1
#define ONBOARD_SD_CS_PIN                   PC10

//
// LCD / Controller
//
#define BEEPER_PIN                          PE4

//
// TFT with FSMC interface
//
#if HAS_FSMC_TFT
  #define LCD_USE_DMA_FSMC
  #define FSMC_CS_PIN                       PD7   // NE4
  #define FSMC_RS_PIN                       PG0   // A0
  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define TFT_RESET_PIN                     PF15
  #define TFT_BACKLIGHT_PIN                 PF11

  #define TOUCH_BUTTONS_HW_SPI
  #define TOUCH_BUTTONS_HW_SPI_DEVICE          1
#endif

#if NEED_TOUCH_PINS
  #define TOUCH_CS_PIN                      PA4   // SPI1_NSS
  #define TOUCH_SCK_PIN                     PA5   // SPI1_SCK
  #define TOUCH_MISO_PIN                    PA6   // SPI1_MISO
  #define TOUCH_MOSI_PIN                    PA7   // SPI1_MOSI
#endif
