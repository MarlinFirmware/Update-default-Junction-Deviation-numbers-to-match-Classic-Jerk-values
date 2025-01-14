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

#define ALLOW_STM32DUINO
#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "LERDGE X only supports 1 hotend / E stepper."
#endif

#define BOARD_INFO_NAME      "Lerdge X"
#define DEFAULT_MACHINE_NAME "LERDGE"

#define STEP_TIMER  4
#define TEMP_TIMER  2

#define I2C_EEPROM
#define I2C_SCL_PIN                         PB8
#define I2C_SDA_PIN                         PB9
#define MARLIN_EEPROM_SIZE               0x10000  // FM24CL64 F-RAM 64K (8Kx8)

#define HAS_OTG_USB_HOST_SUPPORT                  // USB Flash Drive support

//
// Servos
//
//#define SERVO0_PIN                        PD13

//
// Limit Switches
//
#define X_STOP_PIN                          PB12
#define Y_STOP_PIN                          PB13
#define Z_STOP_PIN                          PB14

//
// Filament runout
//
#define FIL_RUNOUT_PIN                      PE1

//
// Z Probe (when not Z_MIN_PIN)
//
//#ifndef Z_MIN_PROBE_PIN
//  #define Z_MIN_PROBE_PIN                 PB15
//#endif

//
// Steppers
//
#define X_STEP_PIN                          PB10
#define X_DIR_PIN                           PB2
#define X_ENABLE_PIN                        PB11

#define Y_STEP_PIN                          PB0
#define Y_DIR_PIN                           PC5
#define Y_ENABLE_PIN                        PB1

#define Z_STEP_PIN                          PA7
#define Z_DIR_PIN                           PA6
#define Z_ENABLE_PIN                        PC4

#define E0_STEP_PIN                         PA4
#define E0_DIR_PIN                          PA3
#define E0_ENABLE_PIN                       PA5

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC0   // Analog Input
#define TEMP_1_PIN                          -1    // Analog Input
#define TEMP_BED_PIN                        PC1   // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA1
#define HEATER_1_PIN                        -1
#define HEATER_BED_PIN                      PA2

//#ifndef FAN0_PIN
//  #define FAN0_PIN                        PC15
//#endif
#define FAN1_PIN                            PC15
#define FAN2_PIN                            PA0

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                   PC15  // FAN1_PIN
#endif

//
// LED / Lighting
//
//#define CASE_LIGHT_PIN_CI                 -1
//#define CASE_LIGHT_PIN_DO                 -1

//
// SD support (On board)
//
#define ONBOARD_SDIO
#define SD_DETECT_PIN                       PA8
#define SDIO_CLOCK                       4800000
#if DISABLED(ONBOARD_SDIO)
  #define SOFTWARE_SPI
  #define SD_SCK_PIN                        PC12
  #define SD_MISO_PIN                       PC8
  #define SD_MOSI_PIN                       PD2
  #define SD_SS_PIN                         PC11
  #define SDSS                              PC11
#endif

//
// Misc. Functions
//
#define LED_PIN                             PC7   // Alive
#define PS_ON_PIN                           -1
#define KILL_PIN                            -1

// Lerdge supports auto-power off and power loss sense through a single pin.
#define POWER_LOSS_PIN                      PC14  // Power-loss / nAC_FAULT

//
// TFT with FSMC interface
//
#if HAS_FSMC_TFT
  #ifndef TFT_DRIVER
    #define TFT_DRIVER                    ST7796
  #endif
  #define ST7796S_INVERTED

  // TODO: See if FSMC DMA can work
  #define FSMC_CS_PIN                       PD7
  #define FSMC_RS_PIN                       PD11
  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define TFT_RESET_PIN                     PD6
  #define TFT_BACKLIGHT_PIN                 PD3

  #define TOUCH_CS_PIN                      PB6
  #define TOUCH_SCK_PIN                     PB3
  #define TOUCH_MOSI_PIN                    PB5
  #define TOUCH_MISO_PIN                    PB4
#endif

#if IS_NEWPANEL
  #define BEEPER_PIN                        PD12
  #define BTN_EN1                           PE4
  #define BTN_EN2                           PE3
  #define BTN_ENC                           PE2
  #ifndef ENCODER_STEPS_PER_MENU_ITEM
    #define ENCODER_STEPS_PER_MENU_ITEM 2
  #endif
#endif
