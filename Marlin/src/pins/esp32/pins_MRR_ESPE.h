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
 * MRR ESPE pin assignments
 *
 * 3D printer control board based on the ESP32 microcontroller.
 * Supports 5 stepper drivers (using I2S stepper stream), heated bed,
 * single hotend, and LCD controller.
 */

#include "env_validate.h"

#if EXTRUDERS > 2 || E_STEPPERS > 2
  #error "MRR ESPE supports up to 2 E steppers."
#elif HAS_MULTI_HOTEND
  #error "MRR ESPE only supports 1 hotend / E stepper."
#endif

#define BOARD_INFO_NAME      "MRR ESPE"
#define BOARD_WEBSITE_URL    "github.com/maplerainresearch/MRR_ESPE"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

//
// Limit Switches
//
#define X_STOP_PIN                            35
#define Y_STOP_PIN                            32
#define Z_STOP_PIN                            33

//
// Enable I2S stepper stream
//
#ifndef I2S_STEPPER_STREAM
  #define I2S_STEPPER_STREAM
#endif
#if ENABLED(I2S_STEPPER_STREAM)
  #define I2S_WS                              26
  #define I2S_BCK                             25
  #define I2S_DATA                            27
#endif

//
// Steppers
//
#define X_STEP_PIN                           129
#define X_DIR_PIN                            130
#define X_ENABLE_PIN                         128
//#define X_CS_PIN                            21

#define Y_STEP_PIN                           132
#define Y_DIR_PIN                            133
#define Y_ENABLE_PIN                         131
//#define Y_CS_PIN                            22

#define Z_STEP_PIN                           135
#define Z_DIR_PIN                            136
#define Z_ENABLE_PIN                         134
//#define Z_CS_PIN                             5  // SS_PIN

#define E0_STEP_PIN                          138
#define E0_DIR_PIN                           139
#define E0_ENABLE_PIN                        137
//#define E0_CS_PIN                           21

#define E1_STEP_PIN                          141
#define E1_DIR_PIN                           142
#define E1_ENABLE_PIN                        140
//#define E1_CS_PIN                           22

#define Z2_STEP_PIN                          141
#define Z2_DIR_PIN                           142
#define Z2_ENABLE_PIN                        140
//#define Z2_CS_PIN                            5

//
// Temperature Sensors
//
#define TEMP_0_PIN                            36  // Analog Input
#define TEMP_1_PIN                            34  // Analog Input
#define TEMP_BED_PIN                          39  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                         145  // 2
#define FAN0_PIN                             146  // 15
#define HEATER_BED_PIN                       144  // 4

#define CONTROLLER_FAN_PIN                   147
//#define E0_AUTO_FAN_PIN                    148  // need to update Configuration_adv.h @section extruder
//#define E1_AUTO_FAN_PIN                    149  // need to update Configuration_adv.h @section extruder
#define FAN1_PIN                             149

//
// MicroSD card
//
#define SD_MOSI_PIN                           23
#define SD_MISO_PIN                           19
#define SD_SCK_PIN                            18
#define SD_SS_PIN                              5
#define USES_SHARED_SPI                           // SPI is shared by SD card with TMC SPI drivers

//
// LCD / Controller
//

#if HAS_WIRED_LCD

  #define LCD_PINS_RS                         13
  #define LCD_PINS_EN                         17
  #define LCD_PINS_D4                         16

  #if ENABLED(CR10_STOCKDISPLAY)

    #define BEEPER_PIN                       151

  #elif IS_RRD_FG_SC

    #define BEEPER_PIN                       151

    //#define LCD_PINS_D5                    150
    //#define LCD_PINS_D6                    152
    //#define LCD_PINS_D7                    153

  #else

    #error "Only CR10_STOCKDISPLAY and REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER are currently supported. Comment out this line to continue."

  #endif

  #define BTN_EN1                              0
  #define BTN_EN2                             12
  #define BTN_ENC                             14

#endif // HAS_MARLINUI_U8GLIB

// Hardware serial pins
// Add the following to Configuration.h or Configuration_adv.h to assign
// specific pins to hardware Serial1 and Serial2.
// Note: Serial2 can be defined using HARDWARE_SERIAL2_RX and HARDWARE_SERIAL2_TX but
// MRR ESPA does not have enough spare pins for such reassignment.
//#define HARDWARE_SERIAL1_RX                 21
//#define HARDWARE_SERIAL1_TX                 22
//#define HARDWARE_SERIAL2_RX                  2
//#define HARDWARE_SERIAL2_TX                  4
