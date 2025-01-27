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
 * Cohesion3D ReMix pin assignments
 * Schematic: https://github.com/Cohesion3D/Cohesion3D-ReMix/blob/master/C3D%20ReMix%20rev2.sch
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "Cohesion3D ReMix"

//
// Servos
//
#define SERVO0_PIN                         P2_04

//
// Limit Switches
//
#define X_MIN_PIN                          P1_24  // 10k pullup to 3.3V
#define X_MAX_PIN                          P1_25  // 10k pullup to 3.3V
#define Y_MIN_PIN                          P1_26  // 10k pullup to 3.3V
#define Y_MAX_PIN                          P1_27  // 10k pullup to 3.3V
#define Z_MIN_PIN                          P1_28  // 10k pullup to 3.3V
#define Z_MAX_PIN                          P1_29  // 10k pullup to 3.3V

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                  P1_29
#endif

//
// Steppers
//
#define X_STEP_PIN                         P2_00
#define X_DIR_PIN                          P0_05
#define X_ENABLE_PIN                       P0_04
#define X_CS_PIN                           P1_10  // Ethernet Expansion - Pin 9

#define Y_STEP_PIN                         P2_01
#define Y_DIR_PIN                          P0_11
#define Y_ENABLE_PIN                       P0_10
#define Y_CS_PIN                           P1_09  // Ethernet Expansion - Pin 10

#define Z_STEP_PIN                         P2_02
#define Z_DIR_PIN                          P0_20
#define Z_ENABLE_PIN                       P0_19
#define Z_CS_PIN                           P1_00  // Ethernet Expansion - Pin 11

#define E0_STEP_PIN                        P2_03
#define E0_DIR_PIN                         P0_22
#define E0_ENABLE_PIN                      P0_21
#define E0_CS_PIN                          P1_04  // Ethernet Expansion - Pin 12

#define E1_STEP_PIN                        P2_08
#define E1_DIR_PIN                         P2_13
#define E1_ENABLE_PIN                      P4_29
#define E1_CS_PIN                          P1_01  // Ethernet Expansion - Pin 14

#define E2_STEP_PIN                        P1_20
#define E2_DIR_PIN                         P1_19
#define E2_ENABLE_PIN                      P1_21
#define E2_CS_PIN                          P1_18  // FET 6

//
// Default pins for TMC software SPI
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                     P1_16  // Ethernet Expansion - Pin 5
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                     P1_17  // Ethernet Expansion - Pin 6
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                      P1_08  // Ethernet Expansion - Pin 7
#endif

//
// Analog Inputs
//  3.3V max when defined as an analog input
//
#define TEMP_0_PIN                      P0_23_A0
#define TEMP_BED_PIN                    P0_24_A1
#define TEMP_1_PIN                      P0_25_A2
#if ENABLED(FILAMENT_WIDTH_SENSOR)
  #define FILWIDTH_PIN                  P0_26_A3
#else
  #define TEMP_2_PIN                    P0_26_A3
#endif

//
// Heaters / Fans
//
#define HEATER_BED_PIN                     P2_05
#define HEATER_0_PIN                       P2_07  // FET 1
#define HEATER_1_PIN                       P1_23  // FET 2
#define HEATER_2_PIN                       P1_22  // FET 3
#ifndef FAN0_PIN
  #define FAN0_PIN                         P2_06  // FET 4
#endif

//
// Auto fans
//
#if HOTENDS == 3
  #define AUTO_FAN_PIN                     P1_18  // FET 6
#else
  #define AUTO_FAN_PIN                     P1_22  // FET 3
#endif
#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E1_AUTO_FAN_PIN
  #define E1_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E2_AUTO_FAN_PIN
  #define E2_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif

//
// Misc. Functions
//
#define LED_PIN                            P4_28  // Play LED

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  #undef HEATER_0_PIN
  #undef HEATER_BED_PIN
  #undef FAN0_PIN
  #define SPINDLE_LASER_PWM_PIN            P2_05  // Bed FET
  #define SPINDLE_LASER_ENA_PIN            P2_07  // FET 1
  #define SPINDLE_DIR_PIN                  P2_06  // FET 4
#endif

/**               ------                                        ------
 * (BEEPER) 1.31 | 1  2 | 1.30 (BTN_ENC)         (MISO)    0.8 | 1  2 | 0.7  (SD_SCK)
 * (EN)     0.18 | 3  4 | 0.16 (RS)              (EN1)    3.26 | 3  4 | 0.28 (SD_CS2)
 * (D4)     0.15   5  6 | --                     (EN2)    3.25   5  6 | 0.9  (SD_MOSI)
 *            -- | 7  8 | 0.27 (D7)              (SD_DET) 0.27 | 7  8 | 2.11
 *           GND | 9 10 | 5V                               GND | 9 10 | --
 *                ------                                        ------
 *                 EXP1                                          EXP2
 */
#define EXP1_01_PIN                        P1_31
#define EXP1_02_PIN                        P1_30
#define EXP1_03_PIN                        P0_18
#define EXP1_04_PIN                        P0_16
#define EXP1_05_PIN                        P0_15
#define EXP1_06_PIN                        -1
#define EXP1_07_PIN                        -1
#define EXP1_08_PIN                        P0_27  // (also on EXP2-7)
#define EXP1_09_PIN                        -1
#define EXP1_10_PIN                        -1

#define EXP2_01_PIN                        P0_08
#define EXP2_02_PIN                        P0_07
#define EXP2_03_PIN                        P3_26
#define EXP2_04_PIN                        P0_28
#define EXP2_05_PIN                        P3_25
#define EXP2_06_PIN                        P0_09
#define EXP2_07_PIN                        P0_27  // (also on EXP1-8)
#define EXP2_08_PIN                        P2_11
#define EXP2_09_PIN                        -1
#define EXP2_10_PIN                        -1

//
// LCD / Controller
//
// LCD_PINS_D5, D6, and D7 are not present in the EXP1 connector, and will need to be
// defined to use the REPRAP_DISCOUNT_SMART_CONTROLLER.
//
// A remote SD card is currently not supported because the pins routed to the EXP2
// connector are shared with the onboard SD card, and Marlin does not support that
// hardware configuration.
//

#if ENABLED(FYSETC_MINI_12864)

  #define FORCE_SOFT_SPI                          // REQUIRED. Results in LCD Software SPI mode 3

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN
  #define DOGLCD_CS                  EXP1_03_PIN
  #define DOGLCD_A0                  EXP1_04_PIN
  #define LCD_RESET_PIN              EXP1_05_PIN

  // A custom cable is REQUIRED for EXP2 cable because the SCK & MOSI on the card's EXP2 are dedicated
  // to the onboard SD card.  All required EXP2 signals come from the Ethernet connector.  Pin 1 of this
  // connector is the one nearest the motor power connector.
  #define DOGLCD_SCK                       P1_17  // LCD2-2  =>  Ethernet pin  5 (bottom, 3 from left)
  #define BTN_EN2                          P1_09  // LCD2-3  =>  Ethernet pin  9 (bottom, 5 from left)
  #define BTN_EN1                          P1_04  // LCD2-5  =>  Ethernet pin 11 (bottom, 6 from left)
  #define DOGLCD_MOSI                      P1_01  // LCD2-6  =>  Ethernet pin 13 (bottom, 7 from left)

  // A custom EXP1 cable is required colored LEDs. Pins 1-5, 9, 10 of the cable go to pins 1-5, 9, 10
  // on the board's EXP1 connector. Pins 6, 7, and 8 of the EXP1 cable go to the Ethernet connector.
  // Rev 1.2 displays do NOT require the RGB LEDs. 2.0 and 2.1 displays do require RGB.
  #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
    #ifndef RGB_LED_R_PIN
      #define RGB_LED_R_PIN                P1_16  // LCD1-6  =>  Ethernet pin  6 (top row, 3 from left)
    #endif
    #ifndef RGB_LED_G_PIN
      #define RGB_LED_G_PIN                P1_10  // LCD1-7  =>  Ethernet pin 10 (top row, 5 from left)
    #endif
    #ifndef RGB_LED_B_PIN
      #define RGB_LED_B_PIN                P1_00  // LCD1-8  =>  Ethernet pin 12 (top row, 6 from left)
    #endif
  #elif ENABLED(FYSETC_MINI_12864_2_1)
    #define NEOPIXEL_PIN                   P1_16  // LCD1-6  =>  Ethernet pin  6 (top row, 3 from left)
  #endif

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN
  //#define SD_DETECT_PIN            EXP2_07_PIN

  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #define LCD_PINS_RS                EXP1_04_PIN
  #define LCD_SDSS_PIN               EXP2_04_PIN
  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_D4                EXP1_05_PIN

  #define KILL_PIN                   EXP2_08_PIN

#endif // HAS_WIRED_LCD

//
// SD Support
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(LCD) || SD_CONNECTION_IS(ONBOARD)
  #define SD_SCK_PIN                 EXP2_02_PIN  // (52) System-defined J3-9 & AUX-3
  #define SD_MISO_PIN                EXP2_01_PIN  // (50) System-defined J3-10 & AUX-3
  #define SD_MOSI_PIN                EXP2_06_PIN  // (51) System-defined J3-10 & AUX-3
  #if SD_CONNECTION_IS(LCD)
    #define SD_SS_PIN                      P1_23  // (53) System-defined J3-5 & AUX-3 (Sometimes called SDSS) - CS used by Marlin
  #else
    #undef SD_DETECT_PIN
    #define ONBOARD_SD_CS_PIN              P0_06  // Chip select for "System" SD card
    #define SD_SS_PIN          ONBOARD_SD_CS_PIN
  #endif
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "No custom SD drive cable defined for this board."
#endif

//
// Ethernet pins
//
//#define ENET_MDIO                        P1_17  // Ethernet pin  5 (bottom, 3 from left)
//#define ENET_RX_ER                       P1_14
//#define ENET_RXD1                        P1_10  // Ethernet pin 10 (top row, 5 from left)
//#define ENET_MOC                         P1_16  // Ethernet pin  6 (top row, 3 from left)
//#define REF_CLK                          P1_15
//#define ENET_RXD0                        P1_09  // Ethernet pin  9 (bottom, 5 from left)
//#define ENET_CRS                         P1_08  // Ethernet pin  8 (top row, 4 from left) - INPUT ONLY
//#define ENET_TX_EN                       P1_04  // Ethernet pin 11 (bottom, 6 from left)
//#define ENET_TXD0                        P1_00  // Ethernet pin 12 (top row, 6 from left)
//#define ENET_TXD1                        P1_01  // Ethernet pin 13 (bottom, 7 from left)
