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
 * TH3D EZBoard pin assignments
 */

#include "env_validate.h"

//#define V3_EZABL_ON_SERVO                       // As in TH3D Firmware Config

#define BOARD_INFO_NAME   "TH3D EZBoard"
#define BOARD_WEBSITE_URL "th3dstudio.com"

//
// Servos
//
#if ENABLED(V3_EZABL_ON_SERVO)
  #define SERVO0_PIN                       -1
#else
  #define SERVO0_PIN                       P2_04
#endif

//
// Limit Switches
//
#define X_STOP_PIN                         P1_24
#define Y_STOP_PIN                         P1_25
#if ENABLED(V3_EZABL_ON_SERVO)
  #define Z_STOP_PIN                       P2_04
#else
  #define Z_STOP_PIN                       P1_26
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                   P1_27
#endif

//
// Steppers
//
#define X_STEP_PIN                         P2_00
#define X_DIR_PIN                          P1_16
#define X_ENABLE_PIN                       P1_17

#define Y_STEP_PIN                         P2_01
#define Y_DIR_PIN                          P1_10
#define Y_ENABLE_PIN                       P1_09

#define Z_STEP_PIN                         P2_02
#define Z_DIR_PIN                          P1_15
#define Z_ENABLE_PIN                       P1_14

#define E0_STEP_PIN                        P2_03
#define E0_DIR_PIN                         P1_04
#define E0_ENABLE_PIN                      P1_08

#define E1_STEP_PIN                        P2_08
#define E1_DIR_PIN                         P2_13
#define E1_ENABLE_PIN                      P4_29

#if HAS_TMC_UART
  //
  // TMC220x stepper drivers
  //
  #define X_SERIAL_TX_PIN                  P0_04
  #define X_SERIAL_RX_PIN                  P0_05

  #define Y_SERIAL_TX_PIN                  P0_10
  #define Y_SERIAL_RX_PIN                  P0_11

  #define Z_SERIAL_TX_PIN                  P0_19
  #define Z_SERIAL_RX_PIN                  P0_20

  #define E0_SERIAL_TX_PIN                 P0_22
  #define E0_SERIAL_RX_PIN                 P0_21

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temp Sensors
//  3.3V max when defined as an Analog Input!
//
#if TEMP_SENSOR_0 == 20                           // PT100 Adapter
  #define TEMP_0_PIN                    P0_02_A7  // Analog Input
#else
  #define TEMP_0_PIN                    P0_23_A0  // Analog Input P0_23
#endif

#define TEMP_BED_PIN                    P0_24_A1  // Analog Input P0_24

//
// Heaters / Fans
//
#define HEATER_BED_PIN                     P2_05
#define HEATER_0_PIN                       P2_07
#ifndef FAN0_PIN
  #define FAN0_PIN                         P2_06
#endif
#define FAN1_PIN                           P1_22

//
// Auto fans
//
#define AUTO_FAN_PIN                       P1_22  // FET 3
#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif

//
// SD Card
//

#define SDCARD_CONNECTION                ONBOARD

#define SD_DETECT_PIN                      P0_27  // SD_CD
#define SD_SCK_PIN                         P0_07
#define SD_MISO_PIN                        P0_08
#define SD_MOSI_PIN                        P0_09
#define ONBOARD_SD_CS_PIN                  P0_06
#define SD_SS_PIN              ONBOARD_SD_CS_PIN

//
// LCD / Controller
//

/**
 *         ------
 *  P1_31 | 1  2 | P1_30
 *  P3_26 | 3  4 | P2_11
 *  P3_25   5  6 | P0_15
 *  P0_16 | 7  8 | P0_18
 *    GND | 9 10 | 5V
 *         ------
 *          EXP1
 *
 * LCD_PINS_D5, D6, and D7 are not present in the EXP1 connector, and will need to be
 * defined to use the REPRAP_DISCOUNT_SMART_CONTROLLER.
 *
 * A remote SD card is currently not supported because the pins routed to the EXP2
 * connector are shared with the onboard SD card.
 */
#define EXP1_01_PIN                        P1_31
#define EXP1_02_PIN                        P1_30
#define EXP1_03_PIN                        P3_26
#define EXP1_04_PIN                        P2_11
#define EXP1_05_PIN                        P3_25
#define EXP1_06_PIN                        P0_15
#define EXP1_07_PIN                        P0_16
#define EXP1_08_PIN                        P0_18

#if ENABLED(CR10_STOCKDISPLAY)

  // Migrated to pins/lcd

#elif ENABLED(MKS_MINI_12864)

  // Migrated to pins/lcd

  #define ADAPTER_TH3D_DUAL
  #define LCD_CONTRAST_INIT                  160
  #define LCD_CONTRAST_MIN                   120
  #define LCD_CONTRAST_MAX                   180
  #define FORCE_SOFT_SPI

#elif HAS_WIRED_LCD

  #error "Only CR10_STOCKDISPLAY or MKS_MINI_12864 are supported with TH3D EZBoard."

#endif
