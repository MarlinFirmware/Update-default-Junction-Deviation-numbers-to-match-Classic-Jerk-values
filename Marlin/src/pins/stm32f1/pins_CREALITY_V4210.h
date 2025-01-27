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
 * Creality 4.2.10 (STM32F103RE / STM32F103RC) board pin assignments
 */

#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "Creality v4.2.10 only supports 1 hotend / E stepper."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME      "Creality V4.2.10"
#endif
#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "3DPrintMill"
#endif

#define BOARD_NO_NATIVE_USB

//
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role
//
#define DISABLE_DEBUG

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  #define IIC_BL24CXX_EEPROM                      // EEPROM on I2C-0
#endif
#if ENABLED(IIC_BL24CXX_EEPROM)
  #define IIC_EEPROM_SDA                    PA11
  #define IIC_EEPROM_SCL                    PA12
  #define MARLIN_EEPROM_SIZE              0x800U  // 2K (24C16)
#else
  #define SDCARD_EEPROM_EMULATION                 // SD EEPROM until all EEPROM is BL24CXX
  #define MARLIN_EEPROM_SIZE              0x800U  // 2K
#endif

//
// Servos
//
#define SERVO0_PIN                          PB0   // BLTouch OUT

//
// Limit Switches
//
#define X_STOP_PIN                          PA3
#define Y_STOP_PIN                          PA7
#define Z_STOP_PIN                          PA5

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PA5   // BLTouch IN
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
  #define FIL_RUNOUT_PIN                    PA6   // "Pulled-high"
#endif

//
// Steppers
//
#ifndef X_STEP_PIN
  #define X_STEP_PIN                        PC2
#endif
#ifndef X_DIR_PIN
  #define X_DIR_PIN                         PB9
#endif
#define X_ENABLE_PIN                        PC3

#ifndef Y_STEP_PIN
  #define Y_STEP_PIN                        PB8
#endif
#ifndef Y_DIR_PIN
  #define Y_DIR_PIN                         PB7
#endif
#define Y_ENABLE_PIN                X_ENABLE_PIN

#ifndef Z_STEP_PIN
  #define Z_STEP_PIN                        PB6
#endif
#ifndef Z_DIR_PIN
  #define Z_DIR_PIN                         PB5
#endif
#define Z_ENABLE_PIN                X_ENABLE_PIN

#ifndef E0_STEP_PIN
  #define E0_STEP_PIN                       PB4
#endif
#ifndef E0_DIR_PIN
  #define E0_DIR_PIN                        PB3
#endif
#define E0_ENABLE_PIN               X_ENABLE_PIN

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC5   // TH1
#define TEMP_BED_PIN                        PC4   // TB1

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA0   // HEATER1
#define HEATER_BED_PIN                      PA1   // HOT BED

#define FAN0_PIN                            PA2   // FAN
#define FAN_SOFT_PWM_REQUIRED

//
// SD Card
//
#define SD_DETECT_PIN                       PC7
#define SDCARD_CONNECTION                ONBOARD
#define ONBOARD_SPI_DEVICE                     1
#define ONBOARD_SD_CS_PIN                   PA4   // SDSS
#define ONBOARD_SDIO
#define NO_SD_HOST_DRIVE                          // This board's SD is only seen by the printer

#if ANY(RET6_12864_LCD, HAS_DWIN_E3V2, IS_DWIN_MARLINUI)

  /**
   *    RET6 12864 LCD
   *        ------
   *  PC6  | 1  2 | PB2
   *  PB10 | 3  4 | PE8
   *  PB14   5  6 | PB13
   *  PB12 | 7  8 | PB15
   *   GND | 9 10 | 5V
   *        ------
   */
  #define EXP3_01_PIN                       PC6
  #define EXP3_02_PIN                       PB2
  #define EXP3_03_PIN                       PB10
  #define EXP3_04_PIN                       PE8
  #define EXP3_05_PIN                       PB14
  #define EXP3_06_PIN                       PB13
  #define EXP3_07_PIN                       PB12
  #define EXP3_08_PIN                       PB15

#elif ANY(VET6_12864_LCD, DWIN_VET6_CREALITY_LCD)

  /**
   *    VET6 12864 LCD
   *        ------
   *  ?    | 1  2 | PC5
   *  PB10 | 3  4 | ?
   *  PA6    5  6 | PA5
   *  PA4  | 7  8 | PA7
   *   GND | 9 10 | 5V
   *        ------
   */
  #define EXP3_01_PIN                       -1
  #define EXP3_02_PIN                       PC5
  #define EXP3_03_PIN                       PB10
  #define EXP3_04_PIN                       -1
  #define EXP3_05_PIN                       PA6
  #define EXP3_06_PIN                       PA5
  #define EXP3_07_PIN                       PA4
  #define EXP3_08_PIN                       PA7

#endif

#if ENABLED(CR10_STOCKDISPLAY)
  #if NONE(RET6_12864_LCD, VET6_12864_LCD)
    #error "Define RET6_12864_LCD or VET6_12864_LCD to select pins for CR10_STOCKDISPLAY with the Creality V4 controller."
  #endif

  #define LCD_PINS_RS                EXP3_07_PIN
  #define LCD_PINS_EN                EXP3_08_PIN
  #define LCD_PINS_D4                EXP3_06_PIN

  #define BTN_ENC                    EXP3_02_PIN
  #define BTN_EN1                    EXP3_03_PIN
  #define BTN_EN2                    EXP3_05_PIN

  #define BEEPER_PIN                 EXP3_01_PIN

#elif ANY(DWIN_VET6_CREALITY_LCD, HAS_DWIN_E3V2, IS_DWIN_MARLINUI)

  // RET6 / VET6 DWIN ENCODER LCD
  #define BTN_ENC                    EXP3_05_PIN
  #define BTN_EN1                    EXP3_08_PIN
  #define BTN_EN2                    EXP3_07_PIN

  //#define LCD_LED_PIN              EXP3_02_PIN
  #ifndef BEEPER_PIN
    #define BEEPER_PIN               EXP3_06_PIN
  #endif

#endif
