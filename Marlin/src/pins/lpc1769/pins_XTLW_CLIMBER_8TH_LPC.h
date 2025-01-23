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

#define BOARD_INFO_NAME   "XTLW Climber-8th-LPC"

//
// Servos
//
#define SERVO0_PIN                         P1_26

//
// Limit Switches
//
#define X_MIN_PIN                          P1_29
#define X_MAX_PIN                          P1_28
#define Y_STOP_PIN                         P1_25
#define Z_STOP_PIN                         P1_22

//
// Steppers
//
#define X_STEP_PIN                         P2_00
#define X_DIR_PIN                          P1_01
#define X_ENABLE_PIN                       P1_00
#ifndef X_CS_PIN
  #define X_CS_PIN                         P1_04
#endif

#define X2_STEP_PIN                        P2_05
#define X2_DIR_PIN                         P0_11
#define X2_ENABLE_PIN                      P2_13
#ifndef X2_CS_PIN
  #define X2_CS_PIN                        P0_10
#endif

#define Y_STEP_PIN                         P2_01
#define Y_DIR_PIN                          P1_09
#define Y_ENABLE_PIN                       P1_08
#ifndef Y_CS_PIN
  #define Y_CS_PIN                         P1_10
#endif

#define Z_STEP_PIN                         P2_02
#define Z_DIR_PIN                          P1_15
#define Z_ENABLE_PIN                       P1_14
#ifndef Z_CS_PIN
  #define Z_CS_PIN                         P1_16
#endif

#define E0_STEP_PIN                        P2_03
#define E0_DIR_PIN                         P4_29
#define E0_ENABLE_PIN                      P1_17
#ifndef E0_CS_PIN
  #define E0_CS_PIN                        P4_28
#endif

#define E1_STEP_PIN                        P2_04
#define E1_DIR_PIN                         P2_11
#define E1_ENABLE_PIN                      P0_04
#ifndef E1_CS_PIN
  #define E1_CS_PIN                        P2_12
#endif

//
// Software SPI pins for TMC2130 stepper drivers
//
#if ENABLED(TMC_USE_SW_SPI)
  #ifndef TMC_SW_MOSI
    #define TMC_SW_MOSI                    P0_20
  #endif
  #ifndef TMC_SW_MISO
    #define TMC_SW_MISO                    P0_19
  #endif
  #ifndef TMC_SW_SCK
    #define TMC_SW_SCK                     P0_21
  #endif
#endif

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                  P1_04
  #define X2_SERIAL_TX_PIN                 P0_10
  #define Y_SERIAL_TX_PIN                  P1_10
  #define Z_SERIAL_TX_PIN                  P1_16
  #define E0_SERIAL_TX_PIN                 P4_28
  #define E1_SERIAL_TX_PIN                 P2_12
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                      P0_26_A3  // (T4)
#define TEMP_1_PIN                      P0_25_A2  // (T3)
#define TEMP_BED_PIN                    P0_23_A0  // (T1)

//
// Heaters / Fans
//
#define HEATER_BED_PIN                     P3_26
#define HEATER_0_PIN                       P3_25
#define HEATER_1_PIN                       P1_20

#define FAN0_PIN                           P1_18
#define FAN1_PIN                           P1_23
#define FAN2_PIN                           P1_21
#define FAN3_PIN                           P0_24
#define FAN4_PIN                           P1_24

//
// Filament sensor
//
#define FIL_RUNOUT_PIN                     P1_27
#define FIL_RUNOUT2_PIN                    P0_27

//
// SD Connection
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define SD_SS_PIN                        P0_06
  #define SD_SCK_PIN                       P0_07
  #define SD_MISO_PIN                      P0_08
  #define SD_MOSI_PIN                      P0_09
  #define SD_DETECT_PIN                    P0_05
#elif SD_CONNECTION_IS(LCD)
  #define SD_SCK_PIN                       P0_15
  #define SD_MISO_PIN                      P0_17
  #define SD_MOSI_PIN                      P0_18
  #define SD_SS_PIN                        P0_16
  #define SD_DETECT_PIN                    P2_06
#endif

/**
 *               -----                                             -----
 *           NC | 1 2 | GND                                    5V | 1 2 | GND
 *        RESET | 3 4 | P2_06 (SD_DETECT)          (LCD_D7) P1_31 | 3 4 | P1_30 (LCD_D6)
 * (MOSI) P0_18 | 5 6   P0_01 (BTN_EN2)            (LCD_D5) P2_08 | 5 6   P1_19 (LCD_D4)
 *(SD_SS) P0_16 | 7 8 | P0_00 (BTN_EN1)            (LCD_RS) P2_10 | 7 8 | P0_22 (LCD_EN)
 *  (SCK) P0_15 | 9 10| P0_17 (MISO)              (BTN_ENC) P0_28 | 9 10| P2_07 (BEEPER)
 *               -----                                             -----
 *               EXP2                                              EXP1
 */

//
// LCDs and Controllers
//
#if IS_TFTGLCD_PANEL

  #if ENABLED(TFTGLCD_PANEL_SPI)
    #define TFTGLCD_CS                     P0_00
  #endif

  #define SD_DETECT_PIN                    P2_06

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                       P2_07
  #define BTN_ENC                          P0_28

  #if ENABLED(CR10_STOCKDISPLAY)

    #define LCD_PINS_RS                    P1_30

    #define BTN_EN1                        P0_22
    #define BTN_EN2                        P1_19

    #define LCD_PINS_EN                    P1_31
    #define LCD_PINS_D4                    P2_08

  #else

    #define BTN_EN1                        P0_00
    #define BTN_EN2                        P0_01

    #define LCD_SDSS_PIN                   P0_16

    #if ENABLED(MKS_12864OLED_SSD1306)

      #define LCD_PINS_DC                  P2_08
      #define DOGLCD_CS                    P2_10
      #define DOGLCD_A0              LCD_PINS_DC
      #define DOGLCD_SCK                   P1_19
      #define DOGLCD_MOSI                  P0_22

      #define LCD_PINS_RS                  P1_30
      #define LCD_PINS_D7                  P1_31
      #define KILL_PIN                     -1     // NC

    #else                                         // !MKS_12864OLED_SSD1306

      #define LCD_PINS_RS                  P2_10

      #define LCD_PINS_EN                  P0_22
      #define LCD_PINS_D4                  P1_19

      #if ENABLED(FYSETC_MINI_12864)

        #define DOGLCD_CS                  P0_22
        #define DOGLCD_A0                  P2_10
        #define DOGLCD_SCK                 P0_15
        #define DOGLCD_MOSI                P0_18

        #define LCD_BACKLIGHT_PIN          -1

        #define FORCE_SOFT_SPI                    // Use this if default of hardware SPI causes display problems
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0

        #define LCD_RESET_PIN              P1_19  // Must be high or open for LCD to operate normally.

        #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
          #ifndef RGB_LED_R_PIN
            #define RGB_LED_R_PIN          P2_08
          #endif
          #ifndef RGB_LED_G_PIN
            #define RGB_LED_G_PIN          P1_30
          #endif
          #ifndef RGB_LED_B_PIN
            #define RGB_LED_B_PIN          P1_31
          #endif
        #elif ENABLED(FYSETC_MINI_12864_2_1)
          #define NEOPIXEL_PIN             P2_08
        #endif

      #else                                       // !FYSETC_MINI_12864

        #if ENABLED(MKS_MINI_12864)
          #define DOGLCD_CS                P2_08
          #define DOGLCD_A0                P1_30
        #endif

        #if ENABLED(ULTIPANEL)
          #define LCD_PINS_D5              P2_08
          #define LCD_PINS_D6              P1_30
          #define LCD_PINS_D7              P1_31
        #endif

      #endif // !FYSETC_MINI_12864

    #endif // !MKS_12864OLED_SSD1306

  #endif // !CR10_STOCKDISPLAY

#endif // HAS_WIRED_LCD
