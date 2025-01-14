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

// https://github.com/ktand/Armed

#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Arm'ed supports up to 2 hotends / E steppers."
#endif

#ifndef ARMED_V1_0
  #define ARMED_V1_1
#endif

#undef BOARD_INFO_NAME                            // Defined on the command line by Arduino Core STM32
#define BOARD_INFO_NAME      "Arm'ed"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

#if NO_EEPROM_SELECTED
  #define I2C_EEPROM
  #define MARLIN_EEPROM_SIZE              0x1000  // 4K
#endif

//
// Limit Switches
//
#define X_STOP_PIN                          PE0
#define Y_STOP_PIN                          PE1
#define Z_STOP_PIN                          PE14

//
// Z Probe (when not Z_MIN_PIN)
//
//#ifndef Z_MIN_PROBE_PIN
//  #define Z_MIN_PROBE_PIN  PA4
//#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA3
#endif

//
// Steppers
//

#ifdef ARMED_SWAP_X_E1
  #define X_STEP_PIN                        PE4
  #define X_DIR_PIN                         PE2
  #define X_ENABLE_PIN                      PE3
  #define X_CS_PIN                          PE5
#else
  #define X_STEP_PIN                        PD3
  #define X_DIR_PIN                         PD2
  #define X_ENABLE_PIN                      PD0
  #define X_CS_PIN                          PD1
#endif

#define Y_STEP_PIN                          PE11
#define Y_DIR_PIN                           PE10
#define Y_ENABLE_PIN                        PE13
#define Y_CS_PIN                            PE12

#define Z_STEP_PIN                          PD6
#define Z_DIR_PIN                           PD7
#define Z_ENABLE_PIN                        PD4
#define Z_CS_PIN                            PD5

#define E0_STEP_PIN                         PB5
#define E0_DIR_PIN                          PB6
#ifdef ARMED_V1_1
  #define E0_ENABLE_PIN                     PC12
#else
  #define E0_ENABLE_PIN                     PB3
#endif
#define E0_CS_PIN                           PB4

#ifdef ARMED_SWAP_X_E1
  #define E1_STEP_PIN                       PD3
  #define E1_DIR_PIN                        PD2
  #define E1_ENABLE_PIN                     PD0
  #define E1_CS_PIN                         PD1
#else
  #define E1_STEP_PIN                       PE4
  #define E1_DIR_PIN                        PE2
  #define E1_ENABLE_PIN                     PE3
  #define E1_CS_PIN                         PE5
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC0   // Analog Input
#define TEMP_1_PIN                          PC1   // Analog Input
#define TEMP_BED_PIN                        PC2   // Analog Input

#if HOTENDS == 1 && TEMP_SENSOR_PROBE
  #define TEMP_PROBE_PIN                    PC1
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA1   // Hardware PWM
#define HEATER_1_PIN                        PA2   // Hardware PWM
#define HEATER_BED_PIN                      PA0   // Hardware PWM

#define FAN0_PIN                            PC6   // Hardware PWM, Part cooling fan
#define FAN1_PIN                            PC7   // Hardware PWM, Extruder fan
#define FAN2_PIN                            PC8   // Hardware PWM, Controller fan

//
// Misc functions
//
#define SDSS                                PE7
#define LED_PIN                             PB7   // Heart beat
#define PS_ON_PIN                           PA10
#define KILL_PIN                            PA8
#define PWR_LOSS                            PA4   // Power loss / nAC_FAULT

//
// LCD / Controller
//
#define SD_DETECT_PIN                       PA15
#define BEEPER_PIN                          PC9

#if ENABLED(FYSETC_MINI_12864)
  //
  // See https://wiki.fysetc.com/Mini12864_Panel/
  //
  #define DOGLCD_A0                         PE9
  #define DOGLCD_CS                         PE8

  #define LCD_BACKLIGHT_PIN                 -1

  #define LCD_RESET_PIN                     PB12  // Must be high or open for LCD to operate normally.

  #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
    #ifndef RGB_LED_R_PIN
      #define RGB_LED_R_PIN                 PB13
    #endif
    #ifndef RGB_LED_G_PIN
      #define RGB_LED_G_PIN                 PB14
    #endif
    #ifndef RGB_LED_B_PIN
      #define RGB_LED_B_PIN                 PB15
    #endif
  #elif ENABLED(FYSETC_MINI_12864_2_1)
    #define NEOPIXEL_PIN                    PB13
  #endif
#else
  #define LCD_PINS_RS                       PE9
  #define LCD_PINS_EN                       PE8
  #define LCD_PINS_D4                       PB12
  #define LCD_PINS_D5                       PB13
  #define LCD_PINS_D6                       PB14
  #define LCD_PINS_D7                       PB15

  #if ENABLED(MKS_MINI_12864)
    #define DOGLCD_CS                       PB13
    #define DOGLCD_A0                       PB14
  #endif
#endif

#define BTN_EN1                             PC4
#define BTN_EN2                             PC5
#define BTN_ENC                             PC3

//
// Extension pins
//
#define EXT0_PIN                            PB0
#define EXT1_PIN                            PB1
#define EXT2_PIN                            PB2
#define EXT3_PIN                            PD8
#define EXT4_PIN                            PD9
#define EXT5_PIN                            PD10
#define EXT6_PIN                            PD11
#define EXT7_PIN                            PD12
#define EXT8_PIN                            PB10
#define EXT9_PIN                            PB11

#if HAS_TMC_UART
  // TMC2208/TMC2209 stepper drivers
  #define X_SERIAL_TX_PIN               EXT0_PIN
  #define Y_SERIAL_TX_PIN               EXT1_PIN
  #define Z_SERIAL_TX_PIN               EXT2_PIN
  #define E0_SERIAL_TX_PIN              EXT3_PIN
  #define E1_SERIAL_TX_PIN              EXT4_PIN
  #define Z2_SERIAL_TX_PIN              EXT4_PIN

  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART
