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

#define BOARD_INFO_NAME   "FYSETC AIO II"
#define BOARD_WEBSITE_URL "github.com/FYSETC/FYSETC-AIO_II"

#define BOARD_NO_NATIVE_USB
#define RESET_STEPPERS_ON_MEDIA_INSERT
#define DISABLE_JTAG

#define PINS_V2_20190128                          // new pins define

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

//
// Flash EEPROM Emulation
//
#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

//
// Limit Switches
//
#define X_STOP_PIN                          PA1
#define Y_STOP_PIN                          PA0
#define Z_STOP_PIN                          PB14

//
// Filament runout
//
#ifdef PINS_V2_20190128
  #define FIL_RUNOUT_PIN                    PB15
#else
  #define FIL_RUNOUT_PIN                    PB5
#endif

//
// Steppers
//
#define X_STEP_PIN                          PB8
#define X_DIR_PIN                           PB9
#define X_ENABLE_PIN                        PA8

#define Y_STEP_PIN                          PB2
#ifdef PINS_V2_20190128
  #define Y_DIR_PIN                         PB3
#else
  #define Y_DIR_PIN                         PB0
#endif
#define Y_ENABLE_PIN                        PB1

#define Z_STEP_PIN                          PC0
#define Z_DIR_PIN                           PC1
#define Z_ENABLE_PIN                        PC2

#define E0_STEP_PIN                         PC15
#define E0_DIR_PIN                          PC14
#define E0_ENABLE_PIN                       PC13

#if HAS_TMC_UART
  //
  // TMC2208/TMC2209 stepper drivers
  //

  // Hardware serial with switch
  #define X_HARDWARE_SERIAL  MSerial2
  #define Y_HARDWARE_SERIAL  MSerial2
  #define Z_HARDWARE_SERIAL  MSerial2
  #define E0_HARDWARE_SERIAL MSerial2

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
  static_assert(X_SLAVE_ADDRESS == 0, "X_SLAVE_ADDRESS must be 0 for BOARD_FYSETC_AIO_II.");
  static_assert(Y_SLAVE_ADDRESS == 1, "Y_SLAVE_ADDRESS must be 1 for BOARD_FYSETC_AIO_II.");
  static_assert(Z_SLAVE_ADDRESS == 2, "Z_SLAVE_ADDRESS must be 2 for BOARD_FYSETC_AIO_II.");
  static_assert(E0_SLAVE_ADDRESS == 3, "E0_SLAVE_ADDRESS must be 3 for BOARD_FYSETC_AIO_II.");

  #if HAS_DRIVER(TMC2208)
    #define TMC_SERIAL_MULTIPLEXER
    #define SERIAL_MUL_PIN1                 PB13
    #define SERIAL_MUL_PIN2                 PB12
  #endif

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Stepper current PWM
//
#ifndef MOTOR_CURRENT_PWM_RANGE
  #define MOTOR_CURRENT_PWM_RANGE           1500  // origin:2000
#endif
#define DEFAULT_PWM_MOTOR_CURRENT { 500, 500, 400 } // origin: {1300,1300,1250}

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC7
#define HEATER_BED_PIN                      PC6
#ifndef FAN0_PIN
  #define FAN0_PIN                          PC8
#endif

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PC5   // Analog Input
#define TEMP_0_PIN                          PC4   // Analog Input

//
// Misc. Functions
//
#define SDSS                                PA4

//
// LCD Pins
//
#if HAS_WIRED_LCD

  #define BEEPER_PIN                        PC9

  #if HAS_MARLINUI_U8GLIB

    #define DOGLCD_A0                       PA15
    #ifdef PINS_V2_20190128
      #define DOGLCD_CS                     PB5
    #else
      #define DOGLCD_CS                     PB7
    #endif

    //#define LCD_CONTRAST_INIT              190
    //#define LCD_SCREEN_ROTATE              180  // 0, 90, 180, 270

  #endif

  // not connected to a pin
  #define SD_DETECT_PIN                     PC3

  #if IS_NEWPANEL
    // The encoder and click button
    #define BTN_EN1                         PC10
    #define BTN_EN2                         PC11
    #define BTN_ENC                         PC12
  #endif

  #ifdef PINS_V2_20190128
    #define LCD_RESET_PIN                   PB4
    #ifndef RGB_LED_R_PIN
      #define RGB_LED_R_PIN                 PB0
    #endif
    #ifndef RGB_LED_G_PIN
      #define RGB_LED_G_PIN                 PB6
    #endif
    #ifndef RGB_LED_B_PIN
      #define RGB_LED_B_PIN                 PB7
    #endif
  #else
    #define LCD_RESET_PIN                   PB6
    #ifndef RGB_LED_R_PIN
      #define RGB_LED_R_PIN                 PB3
    #endif
    #ifndef RGB_LED_G_PIN
      #define RGB_LED_G_PIN                 PB4
    #endif
    #ifndef RGB_LED_B_PIN
      #define RGB_LED_B_PIN                 PB5
    #endif
  #endif

#endif
