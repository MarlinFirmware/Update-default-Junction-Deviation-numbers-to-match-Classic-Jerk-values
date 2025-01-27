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

#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "Artillery Ruby only supports 1 hotend / E stepper."
#endif

#define BOARD_INFO_NAME      "Artillery Ruby"

#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION
  //#define I2C_EEPROM
#endif
#define MARLIN_EEPROM_SIZE               0x1000U  // 4K

#define HAL_TIMER_RATE                     F_CPU

//
// Limit Switches
//
#if (X_HOME_DIR == 1)
  #define X_MIN_PIN                         -1
  #define X_MAX_PIN                         PA2
#else
  #define X_MIN_PIN                         PA2
  #define X_MAX_PIN                         -1
#endif
#if (Y_HOME_DIR == 1)
  #define Y_MIN_PIN                         -1
  #define Y_MAX_PIN                         PA1
#else
  #define Y_MIN_PIN                         PA1
  #define Y_MAX_PIN                         -1
#endif
#if (Z_HOME_DIR == 1)
  #define Z_MIN_PIN                         PC2
  #define Z_MAX_PIN                         PA0
#else
  #define Z_MIN_PIN                         PA0
  #define Z_MAX_PIN                         PC2
#endif

//
// Steppers
//
#define X_STEP_PIN                          PB14
#define X_DIR_PIN                           PB13
#define X_ENABLE_PIN                        PB15

#define Y_STEP_PIN                          PB10
#define Y_DIR_PIN                           PB2
#define Y_ENABLE_PIN                        PB12

#define Z_STEP_PIN                          PB0
#define Z_DIR_PIN                           PC5
#define Z_ENABLE_PIN                        PB1

#define E0_STEP_PIN                         PA7
#define E0_DIR_PIN                          PA6
#define E0_ENABLE_PIN                       PC4

#define E1_STEP_PIN                         PA4
#define E1_DIR_PIN                          PA3
#define E1_ENABLE_PIN                       PA5

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC0
#define TEMP_BED_PIN                        PC1

//
// Heaters / Fans
//
#ifndef HEATER_0_PIN
  #define HEATER_0_PIN                      PC9   // Heater0
#endif
#ifndef HEATER_BED_PIN
  #define HEATER_BED_PIN                    PA8   // Hotbed
#endif
#ifndef FAN0_PIN
  #define FAN0_PIN                          PC8   // Fan0
#endif
#ifndef FAN1_PIN
  #define FAN1_PIN                          PC7   // Fan1
#endif
#ifndef FAN2_PIN
  #define FAN2_PIN                          PC6   // Fan2
#endif

//
// Servos
//
#define SERVO0_PIN                          PC3

//
// LCD / Controller
//

#if HAS_WIRED_LCD
  #if ANY(MKS_12864OLED, MKS_12864OLED_SSD1306)
    #define LCD_PINS_DC                     PB8   // Set as output on init
    #define LCD_PINS_RS                     PB9   // Pull low for 1s to init
    // DOGM SPI LCD Support
    #define DOGLCD_CS                       PC15
    #define DOGLCD_MOSI                     PB6
    #define DOGLCD_SCK                      PB5
    #define DOGLCD_A0                LCD_PINS_DC
  #elif ENABLED(FYSETC_MINI_12864)
    #define DOGLCD_CS                       PB6
    #define DOGLCD_A0                       PC15

    //#define FORCE_SOFT_SPI                      // Use this if default of hardware SPI causes display problems
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0

    #define LCD_RESET_PIN                   PB5   // Must be high or open for LCD to operate normally.

    #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
      #ifndef RGB_LED_R_PIN
        #define RGB_LED_R_PIN               PB9
      #endif
      #ifndef RGB_LED_G_PIN
        #define RGB_LED_G_PIN               PB8
      #endif
      #ifndef RGB_LED_B_PIN
        #define RGB_LED_B_PIN               PB7
      #endif
    #elif ENABLED(FYSETC_MINI_12864_2_1)
      #define NEOPIXEL_PIN                  PB9
    #endif

    #define LCD_CONTRAST_INIT                255
  #else
    #define LCD_PINS_RS                     PC15
    #define LCD_PINS_EN                     PB6
    #define LCD_PINS_D4                     PB5
    #define LCD_PINS_D5                     PB9
    #define LCD_PINS_D6                     PB8
  #endif

  #define LCD_PINS_D7                       PB7

  //
  // Beeper, SD Card, Encoder
  //
  #define BEEPER_PIN                        PC13

  #if HAS_MEDIA
    #define SD_SS_PIN                       PA15
    #define SD_DETECT_PIN                   PD2
  #endif

  #define BTN_EN1                           PB4
  #define BTN_EN2                           PB3
  #define BTN_ENC                           PC14
#endif
