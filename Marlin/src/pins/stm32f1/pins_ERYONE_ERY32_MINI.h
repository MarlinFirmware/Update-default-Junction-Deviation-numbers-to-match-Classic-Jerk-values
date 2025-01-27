/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Eryone Ery32 mini (STM32F103VET6) board pin assignments
 */

#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Eryone Ery32 mini supports up to 2 hotends / E steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "ERYONE Ery32 mini"
#endif

//#define DISABLE_DEBUG
#define DISABLE_JTAG
//#define ENABLE_SPI3

#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

//
// Servos
//
#define SERVO0_PIN                          PA12

//
// Limit Switches
//
#define X_STOP_PIN                          PD8
#define Y_STOP_PIN                          PD15
#define Z_MIN_PIN                           PA11
//#define Z_MAX_PIN                         PB1

//
// Steppers
//
#define X_STEP_PIN                          PB15
#define X_DIR_PIN                           PB14
#define X_ENABLE_PIN                        PD10

#define Y_STEP_PIN                          PD14
#define Y_DIR_PIN                           PD13
#define Y_ENABLE_PIN                        PC6

#define Z_STEP_PIN                          PC8
#define Z_DIR_PIN                           PC7
#define Z_ENABLE_PIN                        PA8

#define E0_STEP_PIN                         PE13
#define E0_DIR_PIN                          PE14
#define E0_ENABLE_PIN                       PB13

//#define E1_STEP_PIN                       PD13
//#define E1_DIR_PIN                        PD12
//#define E1_ENABLE_PIN                     PC6

//
// Heaters 0,1 / Fans / Bed
//
#define HEATER_0_PIN                        PD11

#if ENABLED(FET_ORDER_EFB)                        // Hotend, Fan, Bed
  #define HEATER_BED_PIN                    PD12
#elif ENABLED(FET_ORDER_EEF)                      // Hotend, Hotend, Fan
  #define HEATER_1_PIN                      PD4
#elif ENABLED(FET_ORDER_EEB)                      // Hotend, Hotend, Bed
  #define HEATER_1_PIN                      PD4
  #define HEATER_BED_PIN                    PD12
#elif ENABLED(FET_ORDER_EFF)                      // Hotend, Fan, Fan
  #define FAN1_PIN                          PD12
#elif DISABLED(FET_ORDER_SF)                      // Not Spindle, Fan (i.e., "EFBF" or "EFBE")
  #define HEATER_BED_PIN                    PD12
  #if ANY(HAS_MULTI_HOTEND, HEATERS_PARALLEL)
    #define HEATER_1_PIN                    PB9
  #else
    #define FAN1_PIN                        PB9
  #endif
#endif

#ifndef FAN0_PIN
  #if ANY(FET_ORDER_EFB, FET_ORDER_EFF)           // Hotend, Fan, Bed or Hotend, Fan, Fan
    #define FAN0_PIN                        PB5
  #elif ANY(FET_ORDER_EEF, FET_ORDER_SF)          // Hotend, Hotend, Fan or Spindle, Fan
    #define FAN0_PIN                        PD12
  #elif ENABLED(FET_ORDER_EEB)                    // Hotend, Hotend, Bed
    #define FAN0_PIN                        -1    // IO pin. Buffer needed
  #else                                           // Non-specific are "EFB" (i.e., "EFBF" or "EFBE")
    #define FAN0_PIN                        PB5
  #endif
#endif

#define FAN_SOFT_PWM_REQUIRED

//
// Misc. Functions
//
//#define PS_ON_PIN                         PB9

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  #define X_HARDWARE_SERIAL  MSerial4
  #define Y_HARDWARE_SERIAL  MSerial4
  #define Z_HARDWARE_SERIAL  MSerial4
  #define E0_HARDWARE_SERIAL MSerial4

  // Default TMC slave addresses
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS                    2
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS                    3
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS                    1
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                   0
  #endif
  static_assert(X_SLAVE_ADDRESS == 2, "X_SLAVE_ADDRESS must be 2 for BOARD_ERYONE_ERY32_MINI.");
  static_assert(Y_SLAVE_ADDRESS == 3, "Y_SLAVE_ADDRESS must be 3 for BOARD_ERYONE_ERY32_MINI.");
  static_assert(Z_SLAVE_ADDRESS == 1, "Z_SLAVE_ADDRESS must be 1 for BOARD_ERYONE_ERY32_MINI.");
  static_assert(E0_SLAVE_ADDRESS == 0, "E0_SLAVE_ADDRESS must be 0 for BOARD_ERYONE_ERY32_MINI.");
#endif

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PC2   // TB
#define TEMP_0_PIN                          PC1   // TH1
//#define TEMP_1_PIN                        PC3   // TH2
#define FIL_RUNOUT_PIN                      PA10  // MT_DET

#ifndef TEMP_BOARD_PIN
  #define TEMP_BOARD_PIN                    PC3
#endif
#if TEMP_BOARD_PIN == PC3 && TEMP_SENSOR_BOARD != 13
  #warning "The built-in TEMP_SENSOR_BOARD is 13 for ERYONE Ery32 mini."
#endif

//
// LCD Pins
//
#if HAS_WIRED_LCD
  #define BEEPER_PIN                        PE12
  #define BTN_ENC                           PE11
  #define LCD_PINS_EN                       PE10
  #define LCD_PINS_RS                       PE9
  #define BTN_EN1                           PE4
  #define BTN_EN2                           PE3
  #define LCD_PINS_D4                       PE8
  #define LCD_PINS_D5                       PE7
  #define LCD_PINS_D6                       PB2
  #define LCD_PINS_D7                       PB1

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

  #define BOARD_ST7920_DELAY_1                50
  #define BOARD_ST7920_DELAY_2                50
  #define BOARD_ST7920_DELAY_3                50

#endif // HAS_WIRED_LCD

//
// SD Card
//
#define ENABLE_SPI1
#define SD_DETECT_PIN                       PA4
#define SD_SCK_PIN                          PA5
#define SD_MISO_PIN                         PA6
#define SD_MOSI_PIN                         PA7
#define SD_SS_PIN                           PC4
