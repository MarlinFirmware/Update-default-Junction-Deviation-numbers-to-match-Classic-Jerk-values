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

/**
 * STM32F407VET6 with RAMPS-like shield
 * 'Black' STM32F407VET6 board - https://www.stm32duino.com/viewtopic.php?t=485
 * Shield - https://github.com/jmz52/Hardware
 */

#define ALLOW_STM32DUINO
#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Black STM32F4VET6 supports up to 2 hotends / E steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Black STM32F4VET6"
#endif

#define DEFAULT_MACHINE_NAME "STM32F407VET6"

//#define I2C_EEPROM
#define SRAM_EEPROM_EMULATION
#define MARLIN_EEPROM_SIZE               0x2000U  // 8K

//
// Servos
//
#define SERVO0_PIN                          PC6
#define SERVO1_PIN                          PC7

//
// Limit Switches
//
#define X_MIN_PIN                           PC13
#define X_MAX_PIN                           PA15
#define Y_MIN_PIN                           PA5
#define Y_MAX_PIN                           PD12
#define Z_MIN_PIN                           PD14
#define Z_MAX_PIN                           PD15

//
// Steppers
//
#define X_STEP_PIN                          PC4
#define X_DIR_PIN                           PA4
#define X_ENABLE_PIN                        PE7

#define Y_STEP_PIN                          PE5
#define Y_DIR_PIN                           PE2
#define Y_ENABLE_PIN                        PE6

#define Z_STEP_PIN                          PD5
#define Z_DIR_PIN                           PD3
#define Z_ENABLE_PIN                        PD6

#define E0_STEP_PIN                         PD7
#define E0_DIR_PIN                          PD0
#define E0_ENABLE_PIN                       PB9

#define E1_STEP_PIN                         PE0
#define E1_DIR_PIN                          PE1
#define E1_ENABLE_PIN                       PB8

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC0   // T0
#define TEMP_1_PIN                          PC1   // T1
#define TEMP_BED_PIN                        PC2   // TB

#ifndef TEMP_CHAMBER_PIN
  #define TEMP_CHAMBER_PIN                  PC3   // TC
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA2   // Heater0
#define HEATER_1_PIN                        PA3   // Heater1
#define HEATER_BED_PIN                      PA1   // Hotbed

#define FAN0_PIN                            PE9   // Fan0
#define FAN1_PIN                            PE11  // Fan1
#define FAN2_PIN                            PE13  // Fan2
#define FAN3_PIN                            PE14  // Fan3

//
// Misc. Functions
//
#define LED_PIN                             PA6
//#define LED_PIN                           PA7
#define KILL_PIN                            PB1

//
// LCD / Controller
//
//#define SD_DETECT_PIN                     PC5
//#define SD_DETECT_PIN                     PA8   // SDIO SD_DETECT_PIN, external SDIO card reader only

#define BEEPER_PIN                          PD10
#define LCD_PINS_RS                         PE15
#define LCD_PINS_EN                         PD8
#define LCD_PINS_D4                         PE10
#define LCD_PINS_D5                         PE12
#define LCD_PINS_D6                         PD1
#define LCD_PINS_D7                         PE8
#define BTN_ENC                             PD9
#define BTN_EN1                             PD4
#define BTN_EN2                             PD13

#define DOGLCD_CS                    LCD_PINS_D5
#define DOGLCD_A0                    LCD_PINS_D6

#if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
  #define BTN_ENC_EN                 LCD_PINS_D7  // Detect the presence of the encoder
#endif

//
// Onboard SD support
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define ONBOARD_SDIO                            // Use SDIO for onboard SD
  #if DISABLED(ONBOARD_SDIO)
    #define SOFTWARE_SPI                          // Use soft SPI for onboard SD
    #define SD_SS_PIN                       PC11
    #define SD_SCK_PIN                      PC12
    #define SD_MISO_PIN                     PC8
    #define SD_MOSI_PIN                     PD2
  #endif
#endif
