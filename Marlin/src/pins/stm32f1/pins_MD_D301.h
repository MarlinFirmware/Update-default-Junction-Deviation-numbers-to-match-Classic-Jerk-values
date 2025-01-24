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

#include "env_validate.h"

/**
 * Mingda D2 (DZ301 V1.0 STM32F103ZET6) board pin assignments
 */

#define BOARD_INFO_NAME "MD DZ301 V1.0"

#define DISABLE_DEBUG
#define DISABLE_JTAG

#define FLASH_EEPROM_EMULATION
#define EEPROM_PAGE_SIZE                  0x800U  // 2K
#define EEPROM_START_ADDRESS   (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
#define MARLIN_EEPROM_SIZE      EEPROM_PAGE_SIZE  // 2K

/**
 * This board works with this SERIAL_PORT_* configuration
 * #define SERIAL_PORT 3
 * #define BAUDRATE 115200
 * #define SERIAL_PORT_2 -1
 */

//
// Servos
//
#define SERVO0_PIN                          PB0

//
// Z Probe must be this pin
//
#define Z_MIN_PROBE_PIN                     PB1

//
// Limit Switches
//
#define X_MIN_PIN                           PF12
#define X_MAX_PIN                           PF11
#define Y_MIN_PIN                           PF14
#define Y_MAX_PIN                           PF13
#ifdef Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
  #define Z_MIN_PIN                         PB1
#else
  #define Z_MIN_PIN                         PG0
#endif
#define Z_MAX_PIN                           PF15

//
// Filament Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PG1
#endif

//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PA1
#endif

//
// Backup Power Supply
//
#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PA0
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PD3
#define X_STEP_PIN                          PD2
#define X_DIR_PIN                           PC10

#define Y_ENABLE_PIN                        PC12
#define Y_STEP_PIN                          PA10
#define Y_DIR_PIN                           PA9

#define Z_ENABLE_PIN                        PC9
#define Z_STEP_PIN                          PG8
#define Z_DIR_PIN                           PG7

#define E0_ENABLE_PIN                       PD13
#define E0_STEP_PIN                         PD12
#define E0_DIR_PIN                          PD11

#define Z2_ENABLE_PIN                       PG5
#define Z2_STEP_PIN                         PG4
#define Z2_DIR_PIN                          PG3

/**
 * TMC2208/TMC2209 stepper drivers
 */
#if HAS_TMC_UART
  //
  // Software serial
  //
  #define X_SERIAL_TX_PIN                   PD6
  #define Y_SERIAL_TX_PIN                   PC11
  #define Z_SERIAL_TX_PIN                   PA8
  #define E0_SERIAL_TX_PIN                  PG2
  #define Z2_SERIAL_TX_PIN                  PG6

  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE 19200
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC4
#define TEMP_BED_PIN                        PC5

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC6
#define HEATER_BED_PIN                      PC8
#define FAN0_PIN                            PC7

//
// USB connect control
//
#define USB_CONNECT_PIN                     PA15
#define USB_CONNECT_INVERTING false

//
// SD Support
//
#if !defined(SDCARD_CONNECTION) && DISABLED(NO_LCD_SDCARD)
  #define SDCARD_CONNECTION                  LCD
#endif

#define SD_DETECT_PIN                       PE3

#define ONBOARD_SPI_DEVICE                     1  // SPI1
#define ONBOARD_SD_CS_PIN                   PA4   // Chip select for "System" SD card
#define SD_SS_PIN              ONBOARD_SD_CS_PIN

//
// Misc. Functions
//
#define LED_BLUE_PIN                        PF11
#define LED_RED_PIN                         PF13

#define BEEPER_PIN                          PG11
#define LED_PIN                             PG9

#define NEOPIXEL_PIN                        PB9

/**
 * Note: Mingda screens use various TFT controllers.
 * If the screen stays white, disable 'LCD_RESET_PIN'
 * to let the bootloader init the screen.
 */

//
// TFT with FSMC interface
//
#if HAS_FSMC_TFT
  #define TOUCH_CS_PIN                      PB6   // SPI2_NSS
  #define TOUCH_SCK_PIN                     PB3   // SPI2_SCK
  #define TOUCH_MOSI_PIN                    PB5   // SPI2_MOSI
  #define TOUCH_MISO_PIN                    PB4   // SPI2_MISO
  #define TOUCH_INT_PIN                     PB7   // PenIRQ coming from XPT2046

  #define LCD_USE_DMA_FSMC
  #define FSMC_CS_PIN                       PD7   // FSMC_NE1
  #define FSMC_RS_PIN                       PE2   // A23 Register. Only one address needed
  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define FSMC_DMA_DEV                      DMA1
  #define FSMC_DMA_CHANNEL               DMA_CH4

  #define LCD_BACKLIGHT_PIN                 PG10
  #define TFT_BACKLIGHT_PIN    LCD_BACKLIGHT_PIN

  // Buffer for Color UI
  #define TFT_BUFFER_WORDS                  3200
#endif

#define SPI_FLASH
#if ENABLED(SPI_FLASH)
  #define SPI_FLASH_SIZE               0x1000000  // 16MB
  #define SPI_FLASH_CS_PIN                  PB12
  #define SPI_FLASH_MOSI_PIN                PB15
  #define SPI_FLASH_MISO_PIN                PB14
  #define SPI_FLASH_SCK_PIN                 PB13
#endif
