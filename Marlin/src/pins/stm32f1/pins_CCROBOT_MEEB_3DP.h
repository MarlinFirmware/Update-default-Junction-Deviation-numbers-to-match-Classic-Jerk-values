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

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "CCROBOT-ONLINE MEEB_3DP only supports 1 hotend / E stepper."
#endif

// https://github.com/ccrobot-online/MEEB_3DP
// Pin assignments for 32-bit MEEB_3DP
#define BOARD_INFO_NAME "CCROBOT-ONLINE MEEB_3DP"
#define DEFAULT_MACHINE_NAME "STM32F103RCT6"
#define BOARD_WEBSITE_URL "ccrobot-online.com"

//
// Release PB4 from JTAG NRST role
//
#define DISABLE_JTAG

//
// EEPROM
//
#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE             0x1000U  // 4K
#endif

//
// Servos
//
#define SERVO0_PIN                          PA1

//
// Limit Switches
//
#define X_STOP_PIN                          PC0
#define Y_STOP_PIN                          PC1
#define Z_STOP_PIN                          PC2

//
// Z Probe must be this pin
//
#define Z_MIN_PROBE_PIN                     PC15  // "PROBE"

//
// TMC2208 stepper drivers
//
#define X_ENABLE_PIN                        PB4
#define X_STEP_PIN                          PC12
#define X_DIR_PIN                           PC11

#define Y_ENABLE_PIN                        PC10
#define Y_STEP_PIN                          PB14
#define Y_DIR_PIN                           PB13

#define Z_ENABLE_PIN                        PB12
#define Z_STEP_PIN                          PB2
#define Z_DIR_PIN                           PB1

#define E0_ENABLE_PIN                       PB0
#define E0_STEP_PIN                         PA6
#define E0_DIR_PIN                          PA5

#if HAS_TMC_UART
  // Stepper drivers Serial UART
  #define X_SERIAL_TX_PIN                   PB3
  #define X_SERIAL_RX_PIN                   PD2

  #define Y_SERIAL_TX_PIN                   PA15
  #define Y_SERIAL_RX_PIN                   PC6

  #define Z_SERIAL_TX_PIN                   PB11
  #define Z_SERIAL_RX_PIN                   PB10

  #define E0_SERIAL_TX_PIN                  PC5
  #define E0_SERIAL_RX_PIN                  PC4
#endif

// Reduce baud rate to improve software serial reliability
#ifndef TMC_BAUD_RATE
  #define TMC_BAUD_RATE                    19200
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA0   // TH0
#define TEMP_BED_PIN                        PC3   // THB

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC8   // HEATER0
#define HEATER_BED_PIN                      PC9   // HOT BED

#define FAN0_PIN                            PA7   // FAN  (fan2 on board) model cool fan
#define FAN1_PIN                            PA8   // FAN  (fan0 on board) e0 cool fan
#define FAN2_PIN                            PB9   // FAN  (fan1 on board) controller cool fan

// One NeoPixel onboard and a connector for other NeoPixels
#define BOARD_NEOPIXEL_PIN                  PC7   // The NEOPIXEL LED driving pin

/**
 *       ------
 *  PB5 | 1  2 | PB6
 *  PA2 | 3  4 | RESET
 *  PA3   5  6 | PB8
 *  PB7 | 7  8 | PA4
 *  GND | 9 10 | VCC5
 *       ------
 *        EXP1
 */
#define EXP1_01_PIN                         PB5
#define EXP1_02_PIN                         PB6
#define EXP1_03_PIN                         PA2
#define EXP1_04_PIN                         -1    // RESET
#define EXP1_05_PIN                         PA3
#define EXP1_06_PIN                         PB8
#define EXP1_07_PIN                         PB7
#define EXP1_08_PIN                         PA4

//
// LCD / Controller
//

#if ENABLED(CR10_STOCKDISPLAY)
  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_EN1                    EXP1_03_PIN
  #define BTN_EN2                    EXP1_05_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #define LCD_PINS_RS                EXP1_07_PIN  // CS -- SOFT SPI for ENDER3 LCD
  #define LCD_PINS_D4                EXP1_06_PIN  // SCLK
  #define LCD_PINS_EN                EXP1_08_PIN  // DATA MOSI
#endif

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1               125
  #define BOARD_ST7920_DELAY_2               125
  #define BOARD_ST7920_DELAY_3               125
#endif

//
// Camera
//
#define CHDK_PIN                            PB15

#if 0

//
// SD-NAND
//
#if SD_CONNECTION_IS(ONBOARD)
  #define SD_DETECT_PIN                     -1
  #define SD_SCK_PIN                        PA5
  #define SD_MISO_PIN                       PA6
  #define SD_MOSI_PIN                       PA7
  #define SD_SS_PIN                         PA4
#endif

#define ONBOARD_SPI_DEVICE                     1  // SPI1
#define ONBOARD_SD_CS_PIN                   PA4   // Chip select for SD-NAND

#endif
