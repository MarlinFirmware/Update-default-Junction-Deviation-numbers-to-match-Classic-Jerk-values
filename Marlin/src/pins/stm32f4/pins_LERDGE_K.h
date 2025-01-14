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

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "LERDGE K supports up to 2 hotends / E steppers."
#endif

#define BOARD_INFO_NAME      "Lerdge K"
#define DEFAULT_MACHINE_NAME "LERDGE"

// EEPROM
#if NO_EEPROM_SELECTED
  #define I2C_EEPROM
  #define SOFT_I2C_EEPROM                         // Force the use of Software I2C
  #define I2C_SCL_PIN                       PG14
  #define I2C_SDA_PIN                       PG13
  #define MARLIN_EEPROM_SIZE             0x10000
#endif

#define HAS_OTG_USB_HOST_SUPPORT                  // USB Flash Drive support

//
// Servos
//
#define SERVO0_PIN                          PB11

//
// Limit Switches
//
#define X_STOP_PIN                          PG3
#define Y_STOP_PIN                          PG4
#define Z_STOP_PIN                          PG5

//
// Z Probe (when not Z_MIN_PIN)
//
//#ifndef Z_MIN_PROBE_PIN
//  #define Z_MIN_PROBE_PIN                 PG6
//#endif

//
// Filament runout
//
#define FIL_RUNOUT_PIN                      PE5
#define FIL_RUNOUT2_PIN                     PE6

//
// Steppers
//
#define X_STEP_PIN                          PG1
#define X_DIR_PIN                           PB10
#define X_ENABLE_PIN                        PG0
//#ifndef X_CS_PIN
//  #define X_CS_PIN                        PB2
//#endif

#define Y_STEP_PIN                          PF14
#define Y_DIR_PIN                           PF15
#define Y_ENABLE_PIN                        PF13
//#ifndef Y_CS_PIN
//  #define Y_CS_PIN                        PE2
//#endif

#define Z_STEP_PIN                          PF11
#define Z_DIR_PIN                           PF12
#define Z_ENABLE_PIN                        PC5
//#ifndef Z_CS_PIN
//  #define Z_CS_PIN                        PE3
//#endif

#define E0_STEP_PIN                         PC14
#define E0_DIR_PIN                          PC13
#define E0_ENABLE_PIN                       PC15
//#ifndef E0_CS_PIN
//  #define E0_CS_PIN                       PE4
//#endif

#define E1_STEP_PIN                         PF1
#define E1_DIR_PIN                          PF0
#define E1_ENABLE_PIN                       PF2
//#ifndef E1_CS_PIN
//  #define E1_CS_PIN                       PE1
//#endif

//#define Z2_STEP_PIN                       PF4
//#define Z2_DIR_PIN                        PF3
//#define Z2_ENABLE_PIN                     PF5
//#define Z2_STOP_PIN                       PG2
//#ifndef Z2_CS_PIN
//  #define Z2_CS_PIN                       PE0
//#endif

#if HAS_TMC_UART
  //
  // TMC2208/TMC2209 stepper drivers
  //
  #ifndef X_SERIAL_TX_PIN
    #define X_SERIAL_TX_PIN                 PB2
  #endif
  #ifndef Y_SERIAL_TX_PIN
    #define Y_SERIAL_TX_PIN                 PE2
  #endif
  #ifndef Z_SERIAL_TX_PIN
    #define Z_SERIAL_TX_PIN                 PE3
  #endif
  #ifndef E0_SERIAL_TX_PIN
    #define E0_SERIAL_TX_PIN                PE4
  #endif
  #ifndef E1_SERIAL_TX_PIN
    #define E1_SERIAL_TX_PIN                PE1
  #endif
  // Ex-motor can be any... X2/Y2/Z2 or E2
  #ifndef EX_SERIAL_TX_PIN
    #define EX_SERIAL_TX_PIN                PE0
  #endif
  //#define Z2_SERIAL_TX_PIN    EX_SERIAL_TX_PIN
  //#define E2_SERIAL_TX_PIN    EX_SERIAL_TX_PIN

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // Analog Input
#define TEMP_1_PIN                          PC2   // Analog Input
#define TEMP_BED_PIN                        PC0   // Analog Input

// Lergde-K can choose thermocouple/thermistor mode in software.
// For use with thermistors, these pins must be OUT/LOW.
// This is done automatically.
#define TEMP_0_TR_ENABLE_PIN                PF10
#define TEMP_1_TR_ENABLE_PIN                PF9

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA1
#define HEATER_1_PIN                        PA0
#define HEATER_BED_PIN                      PA2

#ifndef FAN0_PIN
  #define FAN0_PIN                          PF7
#endif

#define FAN1_PIN                            PF6

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                   PB1
#endif

#ifndef E1_AUTO_FAN_PIN
  #define E1_AUTO_FAN_PIN                   PB0
#endif

#define CONTROLLER_FAN_PIN                  PF8

//
// LED / Lighting
//
#define LED_PIN                             PA15  // Status LED
//#define CASE_LIGHT_PIN                    PB6   // LED Ribbon Connector (PWM TIM4_CH1)
#ifndef RGB_LED_R_PIN
  #define RGB_LED_R_PIN                     PB8   // swap R and G pin for compatibility with real wires
#endif
#ifndef RGB_LED_G_PIN
  #define RGB_LED_G_PIN                     PB7
#endif
#ifndef RGB_LED_B_PIN
  #define RGB_LED_B_PIN                     PB9
#endif

//
// SD support
//
#define ONBOARD_SDIO
#define SDIO_CLOCK                       4800000
#define SD_DETECT_PIN                       PA8
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
#define PS_ON_PIN                           PA4
#define KILL_PIN                            -1
#define POWER_LOSS_PIN                      PA4   // Power-loss / nAC_FAULT

//
// TFT with FSMC interface
//
#if HAS_FSMC_TFT
  #ifndef TFT_DRIVER
    #define TFT_DRIVER                    ST7796
  #endif
  #define ST7796S_INVERTED

  #define TFT_RESET_PIN                     PD6
  #define TFT_BACKLIGHT_PIN                 PD3

  // TODO: See if FSMC DMA can work
  #define FSMC_CS_PIN                       PD7
  #define FSMC_RS_PIN                       PD11
  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define TOUCH_CS_PIN                      PG15
  #define TOUCH_SCK_PIN                     PB3
  #define TOUCH_MOSI_PIN                    PB5
  #define TOUCH_MISO_PIN                    PB4
  #define TOUCH_INT_PIN                     PG12
#endif

#if IS_NEWPANEL
  #define BEEPER_PIN                        PC7
  #define BTN_EN1                           PG11
  #define BTN_EN2                           PG10
  #define BTN_ENC                           PG9
  #ifndef ENCODER_STEPS_PER_MENU_ITEM
    #define ENCODER_STEPS_PER_MENU_ITEM 2
  #endif
#endif
