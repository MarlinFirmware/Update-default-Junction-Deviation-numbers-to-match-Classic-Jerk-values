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

/**
 * 2017 Victor Perez Marlin for stm32f1 test
 */

#define BOARD_INFO_NAME      "Beast STM32"
#define DEFAULT_MACHINE_NAME "STM32F103RET6"

// Enable I2C_EEPROM for testing
#define I2C_EEPROM

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

//
// Limit Switches
//
#define X_STOP_PIN                          PD5
#define Y_STOP_PIN                          PD6
#define Z_STOP_PIN                          PD7

//
// Steppers
//
#define X_STEP_PIN                          PE0
#define X_DIR_PIN                           PE1
#define X_ENABLE_PIN                        PC0

#define Y_STEP_PIN                          PE2
#define Y_DIR_PIN                           PE3
#define Y_ENABLE_PIN                        PC1

#define Z_STEP_PIN                          PE4
#define Z_DIR_PIN                           PE5
#define Z_ENABLE_PIN                        PC2

#define E0_STEP_PIN                         PE6
#define E0_DIR_PIN                          PE7
#define E0_ENABLE_PIN                       PC3

/**
 * TODO: Currently using same Enable pin to all steppers.
 */

#define E1_STEP_PIN                         PE8
#define E1_DIR_PIN                          PE9
#define E1_ENABLE_PIN                       PC4

#define E2_STEP_PIN                         PE10
#define E2_DIR_PIN                          PE11
#define E2_ENABLE_PIN                       PC5

//
// Misc. Functions
//
#define SD_SS_PIN                           PA15
#define LED_PIN                             PB2

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PD12  // EXTRUDER 1
#define HEATER_1_PIN                        PD13
#define HEATER_2_PIN                        PD14

#define HEATER_BED_PIN                      PB9   // BED

#ifndef FAN0_PIN
  #define FAN0_PIN                          PB10
#endif

#define FAN_SOFT_PWM_REQUIRED

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PA0   // Analog Input
#define TEMP_0_PIN                          PA1   // Analog Input
#define TEMP_1_PIN                          PA2   // Analog Input
#define TEMP_2_PIN                          PA3   // Analog Input

//
// LCD Pins
//
#if HAS_WIRED_LCD

  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
    #error "REPRAPWORLD_GRAPHICAL_LCD is not supported."
  #else
    #define LCD_PINS_RS                     PB8
    #define LCD_PINS_EN                     PD2
    #define LCD_PINS_D4                     PB12
    #define LCD_PINS_D5                     PB13
    #define LCD_PINS_D6                     PB14
    #define LCD_PINS_D7                     PB15
    #if !IS_NEWPANEL
      #error "Non-NEWPANEL LCD is not supported."
    #endif
  #endif

  #if IS_NEWPANEL
    #if IS_RRD_SC
      #error "RRD Smart Controller is not supported."
    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
      #error "REPRAPWORLD_GRAPHICAL_LCD is not supported."
    #elif ENABLED(LCD_I2C_PANELOLU2)
      #error "LCD_I2C_PANELOLU2 is not supported."
    #elif ENABLED(LCD_I2C_VIKI)
      #error "LCD_I2C_VIKI is not supported."
    #elif ANY(VIKI2, miniVIKI)
      #error "VIKI2 / miniVIKI is not supported."
    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #error "ELB_FULL_GRAPHIC_CONTROLLER is not supported."
    #elif ENABLED(MINIPANEL)
      #error "MINIPANEL is not supported."
    #else
      #error "Other generic NEWPANEL LCD is not supported."
    #endif
  #endif // IS_NEWPANEL

#endif // HAS_WIRED_LCD
