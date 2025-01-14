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

#define BOARD_INFO_NAME      "Chitu3D"
#define DEFAULT_MACHINE_NAME "STM32F103RET6"

#define BOARD_NO_NATIVE_USB

// Enable I2C_EEPROM for testing
//#define I2C_EEPROM

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

//
// Limit Switches
//
#define X_STOP_PIN                          PG10
#define Y_STOP_PIN                          PA12
#define Z_STOP_PIN                          PA14

//
// Steppers
//
#define X_STEP_PIN                          PE5
#define X_DIR_PIN                           PE6
#define X_ENABLE_PIN                        PC13

#define Y_STEP_PIN                          PE2
#define Y_DIR_PIN                           PE3
#define Y_ENABLE_PIN                        PE4

#define Z_STEP_PIN                          PB9
#define Z_DIR_PIN                           PE0
#define Z_ENABLE_PIN                        PE1

#define E0_STEP_PIN                         PB4
#define E0_DIR_PIN                          PB5
#define E0_ENABLE_PIN                       PB8

//
// Misc. Functions
//
#define SDSS                                -1
#define LED_PIN                             -1

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                    PA8   // 8
#endif

#define PS_ON_PIN                           -1
#define KILL_PIN                            PD6   // LED strip 24v

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PD12  // HOT-END
#define HEATER_BED_PIN                      PG11  // HOT-BED

#ifndef FAN0_PIN
  #define FAN0_PIN                          PG14  // MAIN BOARD FAN
#endif

#define FAN_SOFT_PWM_REQUIRED

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA1   // Analog Input
#define TEMP_BED_PIN                        PA0   // Analog Input

//
// LCD Pins
//
#if HAS_WIRED_LCD

  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
    #define LCD_PINS_RS                     PD1   // 49  // CS chip select /SS chip slave select
    #define LCD_PINS_EN                     PD3   // 51  // SID (MOSI)
    #define LCD_PINS_D4                     PD4   // 52  // SCK (CLK) clock
  #elif ALL(IS_NEWPANEL, PANEL_ONE)
    #define LCD_PINS_RS                     PB8
    #define LCD_PINS_EN                     PD2
    #define LCD_PINS_D4                     PB12
    #define LCD_PINS_D5                     PB13
    #define LCD_PINS_D6                     PB14
    #define LCD_PINS_D7                     PB15
  #else
    #define LCD_PINS_RS                     PB8
    #define LCD_PINS_EN                     PD2
    #define LCD_PINS_D4                     PB12
    #define LCD_PINS_D5                     PB13
    #define LCD_PINS_D6                     PB14
    #define LCD_PINS_D7                     PB15
    #if !IS_NEWPANEL
      #define BEEPER_PIN                    PC1   // 33
      // Buttons attached to a shift register
      // Not wired yet
      //#define SHIFT_CLK_PIN               PC6   // 38
      //#define SHIFT_LD_PIN                PC10  // 42
      //#define SHIFT_OUT_PIN               PC8   // 40
      //#define SHIFT_EN_PIN                PA1   // 17
    #endif
  #endif

  #if IS_NEWPANEL

    #if IS_RRD_SC

      #define BEEPER_PIN                    PC5

      #define BTN_EN1                       PB15  // 31
      #define BTN_EN2                       PC1   // 33
      #define BTN_ENC                       PC3   // 35

      #define SD_DETECT_PIN                 PD1   // 49
      #define KILL_PIN                      PC9   // 41

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        #define LCD_BACKLIGHT_PIN           PC7   // 39
      #endif

    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

      #define BTN_EN1                       PE0   // 64
      #define BTN_EN2                       PD11  // 59
      #define BTN_ENC                       PD15  // 63
      #define SD_DETECT_PIN                 PC10  // 42

    #elif ENABLED(LCD_I2C_PANELOLU2)

      #define BTN_EN1                       PC15  // 47
      #define BTN_EN2                       PC11  // 43
      #define BTN_ENC                       PC0   // 32
      #define LCD_SDSS                      PD5   // 53
      #define SD_DETECT_PIN                 -1
      #define KILL_PIN                      PC9   // 41
      #undef LCD_PINS_EN                          // not used, causes false pin conflict report

    #elif ENABLED(LCD_I2C_VIKI)

      #define BTN_EN1                       PB6   // 22   // https://files.panucatt.com/datasheets/viki_wiring_diagram.pdf explains 40/42.
      #define BTN_EN2                       PA7   //  7   // 22/7 are unused on RAMPS_14. 22 is unused and 7 the SERVO0_PIN on RAMPS_13.

      #define BTN_ENC                       -1
      #define LCD_SDSS                      PD5   // 53
      #define SD_DETECT_PIN                 PD1   // 49

    #elif ANY(VIKI2, miniVIKI)

      #define BEEPER_PIN                    PC1   // 33

      // Pins for DOGM SPI LCD Support
      #define DOGLCD_A0                     PC12  // 44
      #define DOGLCD_CS                     PC13  // 45

      #define BTN_EN1                       PB6   // 22
      #define BTN_EN2                       PA7   //  7
      #define BTN_ENC                       PC7   // 39

      #define SDSS                          PD5   // 53
      #define SD_DETECT_PIN                 -1    // Pin 49 for display sd interface, 72 for easy adapter board

      #define KILL_PIN                      PB15  // 31

      #define STAT_LED_RED_PIN              PC0   // 32
      #define STAT_LED_BLUE_PIN             PC3   // 35

      #define LCD_SCREEN_ROTATE              180  // 0, 90, 180, 270

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #define BTN_EN1                       PC3   // 35
      #define BTN_EN2                       PC5   // 37
      #define BTN_ENC                       PB15  // 31
      #define SD_DETECT_PIN                 PD1   // 49
      #define LCD_SDSS                      PD5   // 53
      #define KILL_PIN                      PC9   // 41
      #define BEEPER_PIN                    PB7   // 23
      #define DOGLCD_CS                     PB13  // 29
      #define DOGLCD_A0                     PB11  // 27
      #define LCD_BACKLIGHT_PIN             PC1   // 33

    #elif ENABLED(MINIPANEL)

      #define BEEPER_PIN                    PC10  // 42
      // Pins for DOGM SPI LCD Support
      #define DOGLCD_A0                     PC12  // 44
      #define DOGLCD_CS                     PE2   // 66
      #define LCD_BACKLIGHT_PIN             PE1   // 65   // backlight LED on A11/D65
      #define SDSS                          PD5   // 53

      #define KILL_PIN                      PE0   // 64

      // The encoder and click button
      #define BTN_EN1                       PC8   // 40
      #define BTN_EN2                       PD15  // 63
      #define BTN_ENC                       PD11  // 59
      // not connected to a pin
      #define SD_DETECT_PIN                 PD1   // 49

      //#define LCD_SCREEN_ROTATE            180  // 0, 90, 180, 270

    #else

      // Beeper on AUX-4
      #define BEEPER_PIN                    PC1   // 33

      // Buttons directly attached to AUX-2
      #if IS_RRW_KEYPAD
        #define BTN_EN1                     PE0   // 64
        #define BTN_EN2                     PD11  // 59
        #define BTN_ENC                     PD15  // 63
        #define SHIFT_OUT_PIN               PC8   // 40
        #define SHIFT_CLK_PIN               PC12  // 44
        #define SHIFT_LD_PIN                PC10  // 42
      #elif ENABLED(PANEL_ONE)
        #define BTN_EN1                     PD11  // 59   // AUX2 PIN 3
        #define BTN_EN2                     PD15  // 63   // AUX2 PIN 4
        #define BTN_ENC                     PD1   // 49   // AUX3 PIN 7
      #else
        #define BTN_EN1                     PC5   // 37
        #define BTN_EN2                     PC3   // 35
        #define BTN_ENC                     PB15  // 31
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN               PD1   // 49
        #define KILL_PIN                    PC9   // 41
      #else
        //#define SD_DETECT_PIN             -1    // Ramps doesn't use this
      #endif

    #endif
  #endif // IS_NEWPANEL

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

#endif // HAS_WIRED_LCD
