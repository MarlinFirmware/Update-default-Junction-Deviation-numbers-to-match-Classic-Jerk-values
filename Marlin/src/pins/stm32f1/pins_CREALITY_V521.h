/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Creality 5.2.1 (STM32F103RE) board pin assignments
 */

#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Creality v5.2.1 supports up to 2 hotends / E steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME      "Creality V521"
#endif
#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "Creality V5.2.1"
#endif

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  // FLASH
  //#define FLASH_EEPROM_EMULATION

  // I2C
  #define IIC_BL24CXX_EEPROM                      // EEPROM on I2C-0 used only for display settings
  #if ENABLED(IIC_BL24CXX_EEPROM)
    #define IIC_EEPROM_SDA                  PC2
    #define IIC_EEPROM_SCL                  PC3
    #define MARLIN_EEPROM_SIZE            0x800U  // 2K (24C16)
  #else
    #define SDCARD_EEPROM_EMULATION               // SD EEPROM until all EEPROM is BL24CXX
    #define MARLIN_EEPROM_SIZE            0x800U  // 2K
  #endif

  #undef NO_EEPROM_SELECTED
#endif

//
// Servos
//
#define SERVO0_PIN                          PD13  // BLTouch OUT

//
// Limit Switches
//
#define X_STOP_PIN                          PD10  // X
#define X2_STOP_PIN                         PE15  // X2
#define Y_STOP_PIN                          PE0   // Y
#define Z_STOP_PIN                          PE1   // Z
#define Z2_STOP_PIN                         PE2   // Z2

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PD12  // BLTouch IN
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
#define FIL_RUNOUT_PIN                      PE5   // "Pulled-high"
#define FIL_RUNOUT2_PIN                     PE6   // "Pulled-high"

//
// Steppers
//
#define X_ENABLE_PIN                        PC7
#define X_STEP_PIN                          PD15
#define X_DIR_PIN                           PD14

#define Y_ENABLE_PIN                        PB9
#define Y_STEP_PIN                          PB7
#define Y_DIR_PIN                           PB6

#define Z_ENABLE_PIN                        PB5
#define Z_STEP_PIN                          PB3
#define Z_DIR_PIN                           PD7

#define E0_ENABLE_PIN                       PD4
#define E0_STEP_PIN                         PD1
#define E0_DIR_PIN                          PD0

#define E1_ENABLE_PIN                       PE7
#define E1_STEP_PIN                         PB1
#define E1_DIR_PIN                          PB0

#define X2_ENABLE_PIN                       PE11
#define X2_STEP_PIN                         PE9
#define X2_DIR_PIN                          PE8

#define Z2_ENABLE_PIN                       PC5
#define Z2_STEP_PIN                         PA7
#define Z2_DIR_PIN                          PA6

//
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role
//
#define DISABLE_JTAG

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA4   // TH0
#define TEMP_1_PIN                          PA5   // TH1
#define TEMP_BED_PIN                        PA3   // TB1

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA1   // HEATER0
#define HEATER_1_PIN                        PA0   // HEATER1
#define HEATER_BED_PIN                      PA2   // HOT BED

#define FAN0_PIN                            PB14  // FAN
#define FAN1_PIN                            PB12  // FAN
#define FAN_SOFT_PWM_REQUIRED

//
// SD Card
//
#define SD_DETECT_PIN                       PA8
#define SDCARD_CONNECTION                ONBOARD
#define ONBOARD_SPI_DEVICE                     1
#define ONBOARD_SD_CS_PIN                   PC11  // SDSS
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
