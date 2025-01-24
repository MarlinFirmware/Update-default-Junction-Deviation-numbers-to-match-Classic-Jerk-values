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

/**
 * Creality CREALITY_CR4NTXXC10 (STM32F401RET6) board pin assignments
 *
 * Sold as "E3 Free-runs Silent Motherboard"
 */
#include "env_validate.h"

#if HOTENDS > 1 || E_STEPPERS > 1
  #error "CR4NT220622C10 only supports one hotend / E-stepper."
#endif

#if !AXIS_DRIVER_TYPE_X(TMC2209) || !AXIS_DRIVER_TYPE_Y(TMC2209) || !AXIS_DRIVER_TYPE_Z(TMC2209) || !AXIS_DRIVER_TYPE_E0(TMC2209)
  #error "This board has onboard TMC2209 drivers for X, Y, Z, and E0."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME      "CR4NT220622C10"
#endif
#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "Ender-3 Series"
#endif
#define BOARD_WEBSITE_URL      "www.creality.com"

#define BOARD_NO_NATIVE_USB
//#undef DISABLE_DEBUG                            // DISABLE_(DEBUG|JTAG) is not supported for STM32F4.
//#define DISABLE_JTAG

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  #define IIC_BL24CXX_EEPROM                      // EEPROM on I2C-0
  //#define SDCARD_EEPROM_EMULATION
  #undef NO_EEPROM_SELECTED
#endif

#if ENABLED(IIC_BL24CXX_EEPROM)
  #define IIC_EEPROM_SDA                    PA11
  #define IIC_EEPROM_SCL                    PA12
  #define MARLIN_EEPROM_SIZE              0x800U  // 2K (24C16)
#elif ENABLED(SDCARD_EEPROM_EMULATION)
  #define MARLIN_EEPROM_SIZE              0x800U  // 2K
#endif

//
// Servos
//
#define SERVO0_PIN                          PB0   // BLTouch PWM-OUT PIN (TOUCH pins in the schematic have changed)

//
// Limit Switches
//
#define X_STOP_PIN                          PA5
#define Y_STOP_PIN                          PA6

#if ENABLED(BLTOUCH)
  #define Z_MIN_PIN                         PB1   // BLTouch IN PIN (TOUCH pins in the schematic have changed)
  #define Z_MAX_PIN                         PA7
#else
  #define Z_STOP_PIN                        PA7   // Z-axis limit switch
#endif

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PB1   // BLTouch IN
#endif

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA4   // "Pulled-high"
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PC3
#define X_STEP_PIN                          PC2
#define X_DIR_PIN                           PB9

#define Y_ENABLE_PIN                X_ENABLE_PIN
#define Y_STEP_PIN                          PB8
#define Y_DIR_PIN                           PB7

#define Z_ENABLE_PIN                X_ENABLE_PIN
#define Z_STEP_PIN                          PB6
#define Z_DIR_PIN                           PB5

#define E0_ENABLE_PIN               X_ENABLE_PIN
#define E0_STEP_PIN                         PB4
#define E0_DIR_PIN                          PB3

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC5   // TH1
#define TEMP_BED_PIN                        PC4   // TB1

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA1   // HEATER1
#define HEATER_BED_PIN                      PB10  // HOT BED

#ifndef FAN0_PIN
  #define FAN0_PIN                          PA0   // FAN
#endif
#if PIN_EXISTS(FAN0)
  #define FAN_SOFT_PWM
#endif

#if HAS_TMC_UART

  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE 19200

  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  #define X_HARDWARE_SERIAL  Serial6
  #define Y_HARDWARE_SERIAL  X_HARDWARE_SERIAL
  #define Z_HARDWARE_SERIAL  X_HARDWARE_SERIAL
  #define E0_HARDWARE_SERIAL X_HARDWARE_SERIAL

  // Default TMC slave addresses
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS                    0
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS                    1
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS                    2
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                   3
  #endif
  static_assert(X_SLAVE_ADDRESS == 0, "X_SLAVE_ADDRESS must be 0 for BOARD_CREALITY_CR4NTXXC10.");
  static_assert(Y_SLAVE_ADDRESS == 1, "Y_SLAVE_ADDRESS must be 1 for BOARD_CREALITY_CR4NTXXC10.");
  static_assert(Z_SLAVE_ADDRESS == 2, "Z_SLAVE_ADDRESS must be 2 for BOARD_CREALITY_CR4NTXXC10.");
  static_assert(E0_SLAVE_ADDRESS == 3, "E0_SLAVE_ADDRESS must be 3 for BOARD_CREALITY_CR4NTXXC10.");

  // Software serial

  #define X_SERIAL_TX_PIN                   PC6
  #define X_SERIAL_RX_PIN                   PC7

  #define Y_SERIAL_TX_PIN        X_SERIAL_TX_PIN
  #define Y_SERIAL_RX_PIN        X_SERIAL_RX_PIN

  #define Z_SERIAL_TX_PIN        X_SERIAL_TX_PIN
  #define Z_SERIAL_RX_PIN        X_SERIAL_RX_PIN

  #define E0_SERIAL_TX_PIN       X_SERIAL_TX_PIN
  #define E0_SERIAL_RX_PIN       X_SERIAL_RX_PIN

  #define E1_SERIAL_TX_PIN       X_SERIAL_TX_PIN
  #define E1_SERIAL_RX_PIN       X_SERIAL_RX_PIN

  #define X_DIAG_PIN                        PC13
  #define Y_DIAG_PIN                        PC14
  #define Z_DIAG_PIN                        PC15
  #define E0_DIAG_PIN                       PA15

#endif // HAS_TMC_UART

//
// SD Card
//
#define SD_DETECT_PIN                       PC1
#define SDCARD_CONNECTION                ONBOARD
#define ONBOARD_SDIO
#define SDIO_CLOCK                       4500000
#define SDIO_READ_RETRIES                     16
#define NO_SD_HOST_DRIVE                  // This board's SD is only seen by the printer

/**
 *                 ------
 *            PC0 | 1  2 | PB2
 *       TX2  PA2 | 3  4 | PA3  RX2
 * SPI2_MISO PB14 | 5  6 | PB13 SPI2_SCK
 * SPI2_NSS  PB12 | 7  8 | PB15 SPI2_MOSI
 *            GND | 9 10 | 5V
 *                 ------
 *                  EXP1
 */
#define EXP1_01_PIN                         PC0
#define EXP1_02_PIN                         PB2
#define EXP1_03_PIN                         PA2
#define EXP1_04_PIN                         PA3
#define EXP1_05_PIN                         PB14
#define EXP1_06_PIN                         PB13
#define EXP1_07_PIN                         PB12
#define EXP1_08_PIN                         PB15

#if ENABLED(CR10_STOCKDISPLAY)

  #define LCD_PINS_RS                EXP1_07_PIN
  #define LCD_PINS_EN                EXP1_08_PIN
  #define LCD_PINS_D4                EXP1_06_PIN

  #define BTN_ENC                    EXP1_02_PIN
  #define BTN_EN1                    EXP1_03_PIN
  #define BTN_EN2                    EXP1_05_PIN

  #ifndef HAS_PIN_27_BOARD
    #define BEEPER_PIN               EXP1_01_PIN
  #endif

#elif ENABLED(DWIN_CREALITY_LCD)

  // DWIN ENCODER LCD
  #define BTN_ENC                    EXP1_05_PIN
  #define BTN_EN1                    EXP1_08_PIN
  #define BTN_EN2                    EXP1_07_PIN

  //#define LCD_LED_PIN              EXP1_02_PIN
  #ifndef BEEPER_PIN
    #define BEEPER_PIN               EXP1_06_PIN
    #define NO_SPEAKER
  #endif

#endif
