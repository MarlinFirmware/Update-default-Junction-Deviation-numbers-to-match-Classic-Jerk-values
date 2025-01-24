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
 * BigTreeTech SKR 1.4 pin assignments
 * Schematic: https://github.com/bigtreetech/BIGTREETECH-SKR-V1.3/blob/master/BTT%20SKR%20V1.4/Hardware/BTT%20SKR%20V1.4-SCH.pdf
 */

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "BTT SKR V1.4"
#endif

#ifndef BOARD_CUSTOM_BUILD_FLAGS
  #define BOARD_CUSTOM_BUILD_FLAGS -DLPC_PINCFG_UART3_P4_28
#endif

#define USES_DIAG_PINS

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  //#define I2C_EEPROM                            // EEPROM on I2C-0
  //#define SDCARD_EEPROM_EMULATION
  //#undef NO_EEPROM_SELECTED
#endif

#if ENABLED(I2C_EEPROM)
  #define MARLIN_EEPROM_SIZE             0x8000U  // 32K
#elif ENABLED(SDCARD_EEPROM_EMULATION)
  #define MARLIN_EEPROM_SIZE              0x800U  // 2K
#endif

//
// Servos
//
#define SERVO0_PIN                         P2_00

//
// TMC StallGuard DIAG pins
//
#define X_DIAG_PIN                         P1_29  // X-STOP
#define Y_DIAG_PIN                         P1_28  // Y-STOP
#define Z_DIAG_PIN                         P1_27  // Z-STOP
#define E0_DIAG_PIN                        P1_26  // E0DET
#define E1_DIAG_PIN                        P1_25  // E1DET

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
  #if X_HOME_TO_MIN
    #define X_MAX_PIN                      P1_26  // E0DET
  #else
    #define X_MIN_PIN                      P1_26  // E0DET
  #endif
#elif ENABLED(X_DUAL_ENDSTOPS)
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                      P1_29  // X-STOP
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                      P1_26  // E0DET
  #endif
#else
  #define X_STOP_PIN                       P1_29  // X-STOP
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
  #if Y_HOME_TO_MIN
    #define Y_MAX_PIN                      P1_25  // E1DET
  #else
    #define Y_MIN_PIN                      P1_25  // E1DET
  #endif
#elif ENABLED(Y_DUAL_ENDSTOPS)
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                      P1_28  // Y-STOP
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                      P1_25  // E1DET
  #endif
#else
  #define Y_STOP_PIN                       P1_28  // Y-STOP
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
  #if Z_HOME_TO_MIN
    #define Z_MAX_PIN                      P1_00  // PWRDET
  #else
    #define Z_MIN_PIN                      P1_00  // PWRDET
  #endif
#elif ENABLED(Z_MULTI_ENDSTOPS)
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                      P1_27  // Z-STOP
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                      P1_00  // PWRDET
  #endif
#else
  #ifndef Z_STOP_PIN
    #define Z_STOP_PIN                     P1_27  // Z-STOP
  #endif
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                  P0_10
#endif

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Filament Runout Sensor
//
#define FIL_RUNOUT_PIN                     P1_26  // E0DET
#define FIL_RUNOUT2_PIN                    P1_25  // E1DET

//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                        P1_00  // PWRDET
#endif

//
// Power Loss Detection
//
#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                   P1_00  // PWRDET
#endif

//
// Steppers
//
#define X_STEP_PIN                         P2_02
#define X_DIR_PIN                          P2_06
#define X_ENABLE_PIN                       P2_01
#ifndef X_CS_PIN
  #define X_CS_PIN                         P1_10
#endif

#define Y_STEP_PIN                         P0_19
#define Y_DIR_PIN                          P0_20
#define Y_ENABLE_PIN                       P2_08
#ifndef Y_CS_PIN
  #define Y_CS_PIN                         P1_09
#endif

#define Z_STEP_PIN                         P0_22
#define Z_DIR_PIN                          P2_11
#define Z_ENABLE_PIN                       P0_21
#ifndef Z_CS_PIN
  #define Z_CS_PIN                         P1_08
#endif

#define E0_STEP_PIN                        P2_13
#define E0_DIR_PIN                         P0_11
#define E0_ENABLE_PIN                      P2_12
#ifndef E0_CS_PIN
  #define E0_CS_PIN                        P1_04
#endif

#define E1_STEP_PIN                        P1_15
#define E1_DIR_PIN                         P1_14
#define E1_ENABLE_PIN                      P1_16
#ifndef E1_CS_PIN
  #define E1_CS_PIN                        P1_01
#endif

#define TEMP_1_PIN                      P0_23_A0  // A0 (T0) - (67) - TEMP_1_PIN
#define TEMP_BED_PIN                    P0_25_A2  // A2 (T2) - (69) - TEMP_BED_PIN

//
// Default pins for TMC software SPI
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                     P1_17
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                     P0_05
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                      P0_04
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1
  //#define E4_HARDWARE_SERIAL Serial1

  #define X_SERIAL_TX_PIN                  P1_10
  #define Y_SERIAL_TX_PIN                  P1_09
  #define Z_SERIAL_TX_PIN                  P1_08
  #define E0_SERIAL_TX_PIN                 P1_04
  #define E1_SERIAL_TX_PIN                 P1_01

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

/**       ------                ------
 *  1.30 | 1  2 | 0.28    0.17 | 1  2 | 0.15
 *  1.18 | 3  4 | 1.19    3.26 | 3  4 | 0.16
 *  1.20   5  6 | 1.21    3.25   5  6 | 0.18
 *  1.22 | 7  8 | 1.23    1.31 | 7  8 | RESET
 *   GND | 9 10 | 5V       GND | 9 10 | --
 *        ------                ------
 *         EXP1                  EXP2
 */
#define EXP1_01_PIN                        P1_30
#define EXP1_02_PIN                        P0_28
#define EXP1_03_PIN                        P1_18
#define EXP1_04_PIN                        P1_19
#define EXP1_05_PIN                        P1_20
#define EXP1_06_PIN                        P1_21
#define EXP1_07_PIN                        P1_22
#define EXP1_08_PIN                        P1_23

#define EXP2_01_PIN                        P0_17
#define EXP2_02_PIN                        P0_15
#define EXP2_03_PIN                        P3_26
#define EXP2_04_PIN                        P0_16
#define EXP2_05_PIN                        P3_25
#define EXP2_06_PIN                        P0_18
#define EXP2_07_PIN                        P1_31
#define EXP2_08_PIN                        -1     // RESET

#if HAS_DWIN_E3V2 || IS_DWIN_MARLINUI

  // RET6 DWIN ENCODER LCD
  #define BTN_ENC                    EXP1_05_PIN
  #define BTN_EN1                    EXP1_08_PIN
  #define BTN_EN2                    EXP1_07_PIN

  #ifndef BEEPER_PIN
    #define BEEPER_PIN               EXP1_06_PIN
  #endif

#elif HAS_WIRED_LCD

  #if ENABLED(CTC_A10S_A13)
    CONTROLLER_WARNING("BTT_SKR_V1_4", "CTC_A10S_A13")

    /**
     * 1. Cut the tab off the LCD connector so it can be plugged into the "EXP1" connector the other way.
     * 2. Swap the LCD's +5V (Pin2) and GND (Pin1) wires. (This is the critical part!)
     *
     * !!! If you are unsure, ask for help! Your motherboard may be damaged in some circumstances !!!
     *
     * The CTC_A10S_A13 connector plug:
     *
     *                BEFORE                     AFTER
     *                ------                     ------
     *      (BEEPER) | 1  2 | (CLK)    (BEEPER) |10  9 | (CLK)
     *     (BTN_ENC) | 3  4 | --      (BTN_ENC) | 8  7 | --
     *     (BTN_EN1)   5  6 | (SID)   (BTN_EN1)   6  5 | (SID)
     *     (BTN_EN2) | 7  8 | (CS)    (BTN_EN2) | 4  3 | (CS)
     *            5V | 9 10 | GND           GND | 2  1 | 5V
     *                ------                     ------
     *                 LCD                        LCD
     */

    #define LCD_PINS_RS              EXP1_04_PIN

    #define BTN_EN1                  EXP1_06_PIN
    #define BTN_EN2                  EXP1_07_PIN
    #define BTN_ENC                  EXP1_01_PIN

    #define LCD_PINS_EN              EXP1_03_PIN
    #define LCD_PINS_D4              EXP1_05_PIN
    #define BEEPER_PIN               EXP1_08_PIN

  #elif ENABLED(ANET_FULL_GRAPHICS_LCD)
    CONTROLLER_WARNING("BTT_SKR_V1_4", "ANET_FULL_GRAPHICS_LCD")

   /**
    * 1. Cut the tab off the LCD connector so it can be plugged into the "EXP1" connector the other way.
    * 2. Swap the LCD's +5V (Pin2) and GND (Pin1) wires. (This is the critical part!)
    * 3. Rewire the CLK Signal (LCD Pin9) to LCD Pin7. (LCD Pin9 remains open because it is open drain.)
    * 4. A wire is needed to connect the Reset switch at J3 (LCD Pin7) to EXP2 (Pin3) on the board.
    *
    * !!! If you are unsure, ask for help! Your motherboard may be damaged in some circumstances !!!
    *
    * The ANET_FULL_GRAPHICS_LCD connector plug:
    *
    *            BEFORE                     AFTER
    *            ------                     ------
    *  (BEEPER) | 1  2 | (CLK)    (BEEPER) |10  9 | --
    * (BTN_ENC) | 3  4 | --      (BTN_ENC) | 8  7 | (CLK)
    * (BTN_EN1)   5  6 | (SID)   (BTN_EN1)   6  5 | (SID)
    * (BTN_EN2) | 7  8 | (CS)    (BTN_EN2) | 4  3 | (CS)
    *        5V | 9 10 | GND           GND | 2  1 | 5V
    *            ------                     ------
    *             LCD                        LCD
    */

    #define LCD_PINS_RS              EXP1_08_PIN

    #define BTN_EN1                  EXP1_05_PIN
    #define BTN_EN2                  EXP1_07_PIN
    #define BTN_ENC                  EXP1_03_PIN

    #define LCD_PINS_EN              EXP1_06_PIN
    #define LCD_PINS_D4              EXP1_04_PIN

    #define BEEPER_PIN               EXP1_01_PIN

  #elif ENABLED(CR10_STOCKDISPLAY)
    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN
    #define BTN_ENC                  EXP1_02_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #elif ENABLED(ENDER2_STOCKDISPLAY)

    /** Creality Ender-2 display pinout
     *                   ------
     *   (SCK)     1.30 | 1  2 | 0.28 (BTN_ENC)
     *   (BTN_EN1) 1.18 | 3  4 | 1.19 (RESET)
     *   (BTN_EN2) 1.20   5  6 | 1.21 (LCD_A0)
     *   (LCD_RS)  1.22 | 7  8 | 1.23 (MOSI)
     *              GND | 9 10 | 5V
     *                   ------
     *                    EXP1
     */

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN
    #define BTN_ENC                  EXP1_02_PIN

    #define DOGLCD_CS                EXP1_07_PIN
    #define DOGLCD_A0                EXP1_06_PIN
    #define DOGLCD_SCK               EXP1_01_PIN
    #define DOGLCD_MOSI              EXP1_08_PIN
    #define FORCE_SOFT_SPI
    #define LCD_BACKLIGHT_PIN              -1

  #elif HAS_SPI_TFT                               // Config for Classic UI (emulated DOGM) and Color UI

    #define SDCARD_CONNECTION            ONBOARD

    #define BEEPER_PIN               EXP1_01_PIN

    #define BTN_ENC                  EXP1_02_PIN
    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

    #define TFT_A0_PIN                TFT_DC_PIN

    #ifndef TFT_WIDTH
      #define TFT_WIDTH                      480
    #endif
    #ifndef TFT_HEIGHT
      #define TFT_HEIGHT                     320
    #endif

    #if ENABLED(BTT_TFT35_SPI_V1_0)

      /**
       *            ------                       ------
       *    BEEPER | 1  2 | LCD-BTN        MISO | 1  2 | CLK
       *    T_MOSI | 3  4 | T_CS       LCD-ENCA | 3  4 | TFTCS
       *     T_CLK | 5  6   T_MISO     LCD-ENCB | 5  6   MOSI
       *    PENIRQ | 7  8 | F_CS             RS | 7  8 | RESET
       *       GND | 9 10 | VCC             GND | 9 10 | NC
       *            ------                       ------
       *             EXP1                         EXP2
       *
       * 480x320, 3.5", SPI Display with Rotary Encoder.
       * Stock Display for the BIQU B1 SE Series.
       * Schematic: https://github.com/bigtreetech/TFT35-SPI/blob/master/v1/Hardware/BTT%20TFT35-SPI%20V1-SCH.pdf
       */
      #define TFT_CS_PIN             EXP2_04_PIN
      #define TFT_DC_PIN             EXP2_07_PIN

      #define TFT_SCK_PIN            EXP2_02_PIN
      #define TFT_MISO_PIN           EXP2_01_PIN
      #define TFT_MOSI_PIN           EXP2_06_PIN

      #define TOUCH_CS_PIN           EXP1_04_PIN
      #define TOUCH_SCK_PIN          EXP1_05_PIN
      #define TOUCH_MISO_PIN         EXP1_06_PIN
      #define TOUCH_MOSI_PIN         EXP1_03_PIN
      #define TOUCH_INT_PIN          EXP1_07_PIN

    #elif ENABLED(MKS_TS35_V2_0)

      CONTROLLER_WARNING("BTT_SKR_V1_4", "MKS_TS35_V2_0", " The SKR 1.4 EXP ports are rotated 180°.")

      /**                      ------                                   ------
       *               BEEPER | 1  2 | BTN_ENC               SPI1_MISO | 1  2 | SPI1_SCK
       *     TFT_BKL / LCD_EN | 3  4 | TFT_RESET / LCD_RS      BTN_EN1 | 3  4 | SPI1_CS
       *    TOUCH_CS / LCD_D4 | 5  6   TOUCH_INT / LCD_D5      BTN_EN2 | 5  6   SPI1_MOSI
       *     SPI1_CS / LCD_D6 | 7  8 | SPI1_RS / LCD_D7       SPI1_RS  | 7  8 | RESET
       *                  GND | 9 10 | VCC                         GND | 9 10 | VCC
       *                       ------                                   ------
       *                        EXP1                                     EXP2
       */
      #define TFT_CS_PIN             EXP1_07_PIN
      #define TFT_DC_PIN             EXP1_08_PIN

      #define TFT_RESET_PIN          EXP1_04_PIN
      #define TFT_BACKLIGHT_PIN      EXP1_03_PIN

      //#define TFT_RST_PIN          EXP2_07_PIN
      #define TFT_SCK_PIN            EXP2_02_PIN
      #define TFT_MISO_PIN           EXP2_01_PIN
      #define TFT_MOSI_PIN           EXP2_06_PIN

      #define LCD_USE_DMA_SPI

      #define TFT_BUFFER_WORDS              2400

      #define TOUCH_CS_PIN           EXP1_05_PIN
      #define TOUCH_INT_PIN          EXP1_06_PIN
      #define TOUCH_BUTTONS_HW_SPI
      #define TOUCH_BUTTONS_HW_SPI_DEVICE      1

    #endif

    #if ENABLED(TFT_CLASSIC_UI)
      #ifndef TOUCH_CALIBRATION_X
        #define TOUCH_CALIBRATION_X       -16794
      #endif
      #ifndef TOUCH_CALIBRATION_Y
        #define TOUCH_CALIBRATION_Y        11000
      #endif
      #ifndef TOUCH_OFFSET_X
        #define TOUCH_OFFSET_X              1024
      #endif
      #ifndef TOUCH_OFFSET_Y
        #define TOUCH_OFFSET_Y              -352
      #endif

    #elif ENABLED(TFT_COLOR_UI)
      #ifndef TOUCH_CALIBRATION_X
        #define TOUCH_CALIBRATION_X       -16741
      #endif
      #ifndef TOUCH_CALIBRATION_Y
        #define TOUCH_CALIBRATION_Y        11258
      #endif
      #ifndef TOUCH_OFFSET_X
        #define TOUCH_OFFSET_X              1024
      #endif
      #ifndef TOUCH_OFFSET_Y
        #define TOUCH_OFFSET_Y              -367
      #endif
      #define TFT_BUFFER_WORDS              2400
    #endif

  #elif IS_TFTGLCD_PANEL

    #if ENABLED(TFTGLCD_PANEL_SPI)
      #define TFTGLCD_CS             EXP2_03_PIN
    #endif

    #define SD_DETECT_PIN            EXP2_07_PIN

  #else

    #define BTN_ENC                  EXP1_02_PIN  // (58) open-drain
    #define LCD_PINS_RS              EXP1_04_PIN

    #define BTN_EN1                  EXP2_03_PIN  // (31) J3-2 & AUX-4
    #define BTN_EN2                  EXP2_05_PIN  // (33) J3-4 & AUX-4

    #define LCD_PINS_EN              EXP1_03_PIN
    #define LCD_PINS_D4              EXP1_05_PIN

    #define LCD_SDSS_PIN             EXP2_04_PIN  // (16) J3-7 & AUX-4

    #if ENABLED(FYSETC_MINI_12864)
      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_A0              EXP1_04_PIN
      #define DOGLCD_SCK             EXP2_02_PIN
      #define DOGLCD_MOSI            EXP2_06_PIN

      #define LCD_BACKLIGHT_PIN            -1

      #define FORCE_SOFT_SPI                      // Use this if default of hardware SPI causes display problems
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0

      #define LCD_RESET_PIN          EXP1_05_PIN  // Must be high or open for LCD to operate normally.

      #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
        #ifndef RGB_LED_R_PIN
          #define RGB_LED_R_PIN      EXP1_06_PIN
        #endif
        #ifndef RGB_LED_G_PIN
          #define RGB_LED_G_PIN      EXP1_07_PIN
        #endif
        #ifndef RGB_LED_B_PIN
          #define RGB_LED_B_PIN      EXP1_08_PIN
        #endif
      #elif ENABLED(FYSETC_MINI_12864_2_1)
        #define NEOPIXEL_PIN         EXP1_06_PIN
      #endif

    #else // !FYSETC_MINI_12864

      #if ENABLED(MKS_MINI_12864)
        #define DOGLCD_CS            EXP1_06_PIN
        #define DOGLCD_A0            EXP1_07_PIN
        #define DOGLCD_SCK           EXP2_02_PIN
        #define DOGLCD_MOSI          EXP2_06_PIN
        #define FORCE_SOFT_SPI
      #endif

      #if IS_ULTIPANEL
        #define LCD_PINS_D5          EXP1_06_PIN
        #define LCD_PINS_D6          EXP1_07_PIN
        #define LCD_PINS_D7          EXP1_08_PIN

        #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
          #define BTN_ENC_EN         EXP1_08_PIN  // Detect the presence of the encoder
        #endif

      #endif

    #endif // !FYSETC_MINI_12864

  #endif // HAS_MARLINUI_U8GLIB

#endif // HAS_WIRED_LCD

#if HAS_ADC_BUTTONS
  #error "ADC BUTTONS do not work unmodified on SKR 1.4, The ADC ports cannot take more than 3.3v."
#endif

//
// NeoPixel LED
//
#ifndef BOARD_NEOPIXEL_PIN
  #define BOARD_NEOPIXEL_PIN               P1_24
#endif

/**
 * Special pins
 *   P1_30  (37) (NOT 5V tolerant)
 *   P1_31  (49) (NOT 5V tolerant)
 *   P0_27  (57) (Open collector)
 *   P0_28  (58) (Open collector)
 */

//
// Include common SKR pins
//
#include "pins_BTT_SKR_common.h"
