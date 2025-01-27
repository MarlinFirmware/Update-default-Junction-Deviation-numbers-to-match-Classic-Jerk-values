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

#define BOARD_INFO_NAME      "Misc. STM32F1R"
#define DEFAULT_MACHINE_NAME "STM32F103RET6"

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

//
// Limit Switches
//
#define X_STOP_PIN                          PB3
#define Y_STOP_PIN                          PB4
#define Z_STOP_PIN                          PB5

//
// Steppers
//
#define X_STEP_PIN                          PC0
#define X_DIR_PIN                           PC1
#define X_ENABLE_PIN                        PA8

#define Y_STEP_PIN                          PC2
#define Y_DIR_PIN                           PC3
#define Y_ENABLE_PIN                        PA8

#define Z_STEP_PIN                          PC4
#define Z_DIR_PIN                           PC5
#define Z_ENABLE_PIN                        PA8

#define E0_STEP_PIN                         PC6
#define E0_DIR_PIN                          PC7
#define E0_ENABLE_PIN                       PA8

/**
 * TODO: Currently using same Enable pin to all steppers.
 */

#define E1_STEP_PIN                         PC8
#define E1_DIR_PIN                          PC9
#define E1_ENABLE_PIN                       PA8

#define E2_STEP_PIN                         PC10
#define E2_DIR_PIN                          PC11
#define E2_ENABLE_PIN                       PA8

//
// Misc. Functions
//
#define SD_SS_PIN                           PA4
#define LED_PIN                             PD2

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB0   // EXTRUDER 1
#define HEATER_1_PIN                        PB1

#define HEATER_BED_PIN                      PA3   // BED

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PA0   // Analog Input
#define TEMP_0_PIN                          PA1   // Analog Input
#define TEMP_1_PIN                          PA2   // Analog Input

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
  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

#endif // HAS_WIRED_LCD
