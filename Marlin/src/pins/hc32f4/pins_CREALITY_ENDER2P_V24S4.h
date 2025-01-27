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

//
// Creality Ender 2 Pro v2.4.S4_170 (HC32f460kcta)
// Derived from https://github.com/Klipper3d/klipper/blob/master/config/printer-creality-ender2pro-hc32-2022.cfg
//
#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "Creality v2.4.S4 only supports one hotend and E-stepper"
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Creality v2.4.S4"
#endif
#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "Ender 2 Pro"
#endif

//
// Onboard crystal oscillator
//
#ifndef BOARD_XTAL_FREQUENCY
  #define BOARD_XTAL_FREQUENCY           8000000  // 8 MHz XTAL
#endif

//
// Release JTAG pins but keep SWD enabled
// - PA15 (JTDI / E0_DIR_PIN)
// - PB3 (JTDO / E0_STEP_PIN)
// - PB4 (NJTRST / E0_ENABLE_PIN)
//
//#define DISABLE_DEBUG
#define DISABLE_JTAG

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  #define IIC_BL24CXX_EEPROM
  //#define SDCARD_EEPROM_EMULATION
  #undef NO_EEPROM_SELECTED
#endif

#if ENABLED(IIC_BL24CXX_EEPROM)
  #define IIC_EEPROM_SDA                    PA12
  #define IIC_EEPROM_SCL                    PA11
  #define MARLIN_EEPROM_SIZE              0x800U // 2K (24C16)
#elif ENABLED(SDCARD_EEPROM_EMULATION)
  #define MARLIN_EEPROM_SIZE              0x800U // 2K
#endif

//
// Servos
//
#ifndef SERVO0_PIN
  #define SERVO0_PIN                        PB1
#endif

//
// Limit Switches
//
#define X_STOP_PIN                          PA5
#define Y_STOP_PIN                          PA6
#define Z_STOP_PIN                          PB0

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PB2
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA4   // "Pulled-high" *
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PC2
#define X_STEP_PIN                          PC1
#define X_DIR_PIN                           PC0

#define Y_ENABLE_PIN                        PH2
#define Y_STEP_PIN                          PB9
#define Y_DIR_PIN                           PB8

#define Z_ENABLE_PIN                        PB7
#define Z_STEP_PIN                          PB6
#define Z_DIR_PIN                           PB5

#define E0_ENABLE_PIN                       PB4
#define E0_STEP_PIN                         PB3
#define E0_DIR_PIN                          PA15

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC5   // HEATER1 ADC1_IN15
#define TEMP_BED_PIN                        PC4   // HOT BED ADC1_IN14

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA1   // HEATER1
#define HEATER_BED_PIN                      PB10  // HOT BED

#define FAN0_PIN                            PA0   // FAN0

//
// SD Card
//
#define SD_DETECT_PIN                       PA10
#define ONBOARD_SPI_DEVICE                     1
#define ONBOARD_SD_CS_PIN                   PC3   // SDSS
#define ONBOARD_SDIO
#define NO_SD_HOST_DRIVE                          // This board's SD is only seen by the printer

#ifndef LCD_SERIAL_PORT
  #define LCD_SERIAL_PORT                      1
#endif

//
// SDIO Pins
//
#define BOARD_SDIO_D0                       PC8
#define BOARD_SDIO_D1                       PC9
#define BOARD_SDIO_D2                       PC10
#define BOARD_SDIO_D3                       PC11
#define BOARD_SDIO_CLK                      PC12
#define BOARD_SDIO_CMD                      PD2
#define BOARD_SDIO_DET                      PA10

//
// USART Pins
//

// Display
#define BOARD_USART1_TX_PIN                 PA2   // LCD
#define BOARD_USART1_RX_PIN                 PA3

// Host
#define BOARD_USART2_TX_PIN                 PA8   // USB
#define BOARD_USART2_RX_PIN                 PA7

// Onboard LED (HIGH = off, LOW = on)
#ifndef LED_BUILTIN
  #define LED_BUILTIN                       PC3
#endif

#define BOARD_NO_NATIVE_USB

//
// SPI3 Pins (LCD)
//
#define SPI_NSS                             PB15  // SPI3_NSS
#define SPI_SCK                             PB14  // SPI3_CLK
#define SPI_MISO                            PB13  // SPI3_MISO
#define SPI_MOSI                            PB12  // SPI3_MOSI

//
// LCD / Controller
//

/**        ------
 *   PC6  | 1  2 | PC7
 *   PA2  | 3  4 | PA3
 *   PB13   5  6 | PB14
 *   PB15 | 7  8 | PB12
 *   GND  | 9 10 | 5V
 *         ------
 *          EXP1
 */
#define EXP1_01_PIN                         PC6
#define EXP1_02_PIN                         PC7
#define EXP1_03_PIN                         PA2
#define EXP1_04_PIN                         PA3
#define EXP1_05_PIN                         PB13
#define EXP1_06_PIN                         PB14
#define EXP1_07_PIN                         PB15
#define EXP1_08_PIN                         PB12

#if ENABLED(CR10_STOCKDISPLAY)                    // LCD used for C2
  /**          ------
   *   BEEPER | 1  2 | ENC
   *      EN1 | 3  4 | KILL
   *      EN2   5  6 | LCD_D4
   *   LCD_RS | 7  8 | LCD_EN
   *      GND | 9 10 | 5V
   *           ------
   */
  #define BEEPER_PIN                 EXP1_01_PIN

  #define BTN_ENC                    EXP1_02_PIN
  #define BTN_EN1                    EXP1_03_PIN
  #define BTN_EN2                    EXP1_05_PIN

  #define LCD_PINS_RS                EXP1_07_PIN
  #define LCD_PINS_EN                EXP1_08_PIN
  #define LCD_PINS_D4                EXP1_06_PIN

  #define KILL_PIN                   EXP1_04_PIN
#endif
