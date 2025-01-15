/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if HAS_MULTI_HOTEND
  #error "BOARD_LDO_LEVIATHAN_V1_2 only supports 1 hotend."
#endif

#define BOARD_INFO_NAME "LDO Leviathan V1.2"
#define BOARD_WEBSITE_URL "github.com/MotorDynamicsLab/Leviathan"

// Ignore temp readings during development
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

// Use one of these or SDCard-based Emulation will be used
#if NO_EEPROM_SELECTED
  //#define SRAM_EEPROM_EMULATION                 // Use BackSRAM-based EEPROM emulation
  #define FLASH_EEPROM_EMULATION                  // Use Flash-based EEPROM emulation
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  // Decrease delays and flash wear by spreading writes across the
  // 128 kB sector allocated for EEPROM emulation.
  #define FLASH_EEPROM_LEVELING
#endif

// Avoid conflict with TIMER_TONE
#define STEP_TIMER 8

#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE              0x1000  // 4K
#endif

//
// Filament Runout Sensor
//
#define FIL_RUNOUT_PIN                      PC0   // FILAMENT-SENSOR

//
// Misc. Functions
//
#define LED_PIN                             PE6   // LED-Strip

//
// Steppers
//
#define X_STEP_PIN                          PB10  // HV-STEPPER-0
#define X_DIR_PIN                           PB11
#define X_ENABLE_PIN                        PG0
#ifndef X_CS_PIN
  #define X_CS_PIN                          PE15
#endif

#define Y_STEP_PIN                          PF15  // HV-STEPPER-1
#define Y_DIR_PIN                           PF14
#define Y_ENABLE_PIN                        PE9
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PE11
#endif

#define Z_STEP_PIN                          PD4   // STEPPER-0
#define Z_DIR_PIN                           PD3
#define Z_ENABLE_PIN                        PD7

#define E0_STEP_PIN                         PC12  // STEPPER-1
#define E0_DIR_PIN                          PC11
#define E0_ENABLE_PIN                       PD2

#define E1_STEP_PIN                         PC9   // STEPPER-2
#define E1_DIR_PIN                          PC8
#define E1_ENABLE_PIN                       PC10

#define E2_STEP_PIN                         PG7   // STEPPER-3
#define E2_DIR_PIN                          PG6
#define E2_ENABLE_PIN                       PC7

#define E3_STEP_PIN                         PD10  // STEPPER-4
#define E3_DIR_PIN                          PD9
#define E3_ENABLE_PIN                       PD13

//
// Integrated TMC5160/TMC2209 driver defaults
//
#if  (HAS_X_AXIS && !AXIS_DRIVER_TYPE_X(TMC5160)) \
  || (HAS_Y_AXIS && !AXIS_DRIVER_TYPE_Y(TMC5160)) \
  || (NUM_Z_STEPPERS >= 1 && !AXIS_DRIVER_TYPE_Z(TMC2209)) \
  || (NUM_Z_STEPPERS >= 2 && !AXIS_DRIVER_TYPE_Z2(TMC2209)) \
  || (NUM_Z_STEPPERS >= 3 && !AXIS_DRIVER_TYPE_Z3(TMC2209)) \
  || (NUM_Z_STEPPERS >= 4 && !AXIS_DRIVER_TYPE_Z4(TMC2209)) \
  || (EXTRUDERS >= 1 && !AXIS_DRIVER_TYPE_E0(TMC2209)) \
  || (EXTRUDERS >= 2 && !AXIS_DRIVER_TYPE_E1(TMC2209)) \
  || (EXTRUDERS >= 3 && !AXIS_DRIVER_TYPE_E2(TMC2209)) \
  || (EXTRUDERS >= 4 && !AXIS_DRIVER_TYPE_E3(TMC2209))
  #error "X and Y DRIVER TYPE must be TMC5160 and all others should be TMC2209 for BOARD_LDO_LEVIATHAN_V1_2."
#endif

// RSENSE defaults
#if HAS_X_AXIS
  static_assert(X_RSENSE == 0.075, "X_RSENSE (HV-STEPPER-0) must be 0.075 for BOARD_LDO_LEVIATHAN_V1_2.");
#endif
#if HAS_Y_AXIS
  static_assert(Y_RSENSE == 0.075, "Y_RSENSE (HV-STEPPER-1) must be 0.075 for BOARD_LDO_LEVIATHAN_V1_2.");
#endif
#if NUM_Z_STEPPERS >= 1
  static_assert(Z_RSENSE == 0.11, "Z_RSENSE (STEPPER-0) must be 0.11 for BOARD_LDO_LEVIATHAN_V1_2.");
#endif
#if NUM_Z_STEPPERS >= 2
  static_assert(Z2_RSENSE == 0.11, "Z2_RSENSE (STEPPER-2) must be 0.11 for BOARD_LDO_LEVIATHAN_V1_2.");
#endif
#if NUM_Z_STEPPERS >= 3
  static_assert(Z3_RSENSE == 0.11, "Z3_RSENSE (STEPPER-3) must be 0.11 for BOARD_LDO_LEVIATHAN_V1_2.");
#endif
#if NUM_Z_STEPPERS >= 4
  static_assert(Z4_RSENSE == 0.11, "Z4_RSENSE (STEPPER-4) must be 0.11 for BOARD_LDO_LEVIATHAN_V1_2.");
#endif
#if EXTRUDERS >= 1
  static_assert(E0_RSENSE == 0.11, "E0_RSENSE (STEPPER-1) must be 0.11 for BOARD_LDO_LEVIATHAN_V1_2.");
#endif
#if EXTRUDERS >= 2
  static_assert(E1_RSENSE == 0.11, "E1_RSENSE (STEPPER-2) must be 0.11 for BOARD_LDO_LEVIATHAN_V1_2.");
#endif
#if EXTRUDERS >= 3
  static_assert(E2_RSENSE == 0.11, "E2_RSENSE (STEPPER-3) must be 0.11 for BOARD_LDO_LEVIATHAN_V1_2.");
#endif
#if EXTRUDERS >= 4
  static_assert(E3_RSENSE == 0.11, "E3_RSENSE (STEPPER-4) must be 0.11 for BOARD_LDO_LEVIATHAN_V1_2.");
#endif

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PG1   // HV-STEPPER-0
#define Y_DIAG_PIN                          PE10  // HV-STEPPER-1
#define Z_DIAG_PIN                          PD6   // STEPPER-0
#define E0_DIAG_PIN                         PD1   // STEPPER-1
#define E1_DIAG_PIN                         PA15  // STEPPER-2
#define E2_DIAG_PIN                         PC6   // STEPPER-3
#define E3_DIAG_PIN                         PD12  // STEPPER-4

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
#else
  #define X_STOP_PIN                        PC1   // X-ENDSTOP
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
#else
  #define Y_STOP_PIN                        PC2   // Y-ENDSTOP
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
#else
  #define Z_STOP_PIN                        PC3   // Z-ENDSTOP
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PF1   // Z-PROBE
#endif

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PA1   // TH0
#define TEMP_0_PIN                          PA2   // TH1
#define TEMP_1_PIN                          PA0   // TH2
#define TEMP_2_PIN                          PA3   // TH3

//
// Heaters / Fans
//
#define HEATER_BED_PIN                      PG11  // HEATBED
#define HEATER_0_PIN                        PG10  // HEATER

#define FAN0_PIN                            PB7   // FAN0
#define FAN1_PIN                            PB3   // FAN1
#define FAN2_PIN                            PF7   // FAN2
#define FAN3_PIN                            PF9   // FAN3

#ifndef E0_FAN_TACHO_PIN
  //#define E0_FAN_TACHO_PIN                PB4   // FAN1
#endif

//
// SPI pins for TMC5160 HV-STEPPER drivers
//
#if HAS_TMC_SPI
  #define TMC_USE_SW_SPI
  #ifndef TMC_SPI_MOSI
    #define TMC_SPI_MOSI                    PE14
  #endif
  #ifndef TMC_SPI_MISO
    #define TMC_SPI_MISO                    PE13
  #endif
  #ifndef TMC_SPI_SCK
    #define TMC_SPI_SCK                     PE12
  #endif
#endif

#if HAS_TMC_UART
/**
   * TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */

  #define Z_SERIAL_TX_PIN                   PD5
  #define Z_SERIAL_RX_PIN        Z_SERIAL_TX_PIN

  #define E0_SERIAL_TX_PIN                  PD0
  #define E0_SERIAL_RX_PIN      E0_SERIAL_TX_PIN

  #define E1_SERIAL_TX_PIN                  PA8
  #define E1_SERIAL_RX_PIN      E1_SERIAL_TX_PIN

  #define E2_SERIAL_TX_PIN                  PG8
  #define E2_SERIAL_RX_PIN      E2_SERIAL_TX_PIN

  #define E3_SERIAL_TX_PIN                  PD11
  #define E3_SERIAL_RX_PIN      E3_SERIAL_TX_PIN

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

/**               ------                                      ------
 * (BEEPER) PG9  | 1  2 | PE7  (BTN_ENC)         (MISO) PA6  | 1  2 | PA5  (SCK)
 * (LCD_EN) PG13 | 3  4 | PG14 (LCD_RS)       (BTN_EN1) PE2  | 3  4 | PA4  (SD_SS)
 * (LCD_D4) PC13   5  6 | PC14 (LCD_D5)       (BTN_EN2) PE3    5  6 | PA7  (MOSI)
 * (LCD_D6) PC15 | 7  8 | PF0  (LCD_D7)     (SD_DETECT) PE5  | 7  8 | RESET
 *           GND | 9 10 | 5V                             GND | 9 10 | PE4 (KILL)
 *                ------                                      ------
 *                 EXP1                                        EXP2
 */
#define EXP1_01_PIN                         PG9
#define EXP1_02_PIN                         PG12
#define EXP1_03_PIN                         PG13
#define EXP1_04_PIN                         PG14
#define EXP1_05_PIN                         PC13
#define EXP1_06_PIN                         PC14
#define EXP1_07_PIN                         PC15
#define EXP1_08_PIN                         PF0

#define EXP2_01_PIN                         PA6
#define EXP2_02_PIN                         PA5
#define EXP2_03_PIN                         PE2
#define EXP2_04_PIN                         PA4
#define EXP2_05_PIN                         PE3
#define EXP2_06_PIN                         PA7
#define EXP2_07_PIN                         PE5
#define EXP2_08_PIN                         -1
#define EXP2_10_PIN                         PE4

//
// SD Support
//
#if !defined(SDCARD_CONNECTION) && DISABLED(NO_LCD_SDCARD)
  #define SDCARD_CONNECTION                  LCD
#endif

#if SD_CONNECTION_IS(LCD)

  #define SDSS                       EXP2_04_PIN
  #define SD_SS_PIN                         SDSS
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN
#elif SD_CONNECTION_IS(ONBOARD)
  #error "ONBOARD is not a supported SDCARD_CONNECTION for BOARD_LDO_LEVIATHAN_V1_2."
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "CUSTOM_CABLE is not a supported SDCARD_CONNECTION for BOARD_LDO_LEVIATHAN_V1_2."
#endif

//
// LCDs and Controllers
//
#if HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #if ENABLED(CR10_STOCKDISPLAY)

    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #else

    #define LCD_PINS_RS              EXP1_04_PIN

    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

    #define LCD_PINS_EN              EXP1_03_PIN
    #define LCD_PINS_D4              EXP1_05_PIN

    #if ENABLED(FYSETC_MINI_12864)
      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_A0              EXP1_04_PIN
      //#define LCD_BACKLIGHT_PIN           -1
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
    #endif // !FYSETC_MINI_12864

    #if IS_ULTIPANEL
      #define LCD_PINS_D5            EXP1_06_PIN
      #define LCD_PINS_D6            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN

      #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
        #define BTN_ENC_EN           LCD_PINS_D7  // Detect the presence of the encoder
      #endif

    #endif

  #endif
#endif // HAS_WIRED_LCD

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1               120
  #define BOARD_ST7920_DELAY_2                80
  #define BOARD_ST7920_DELAY_3               580
#endif

#if HAS_SPI_TFT                                   // Config for Classic UI (emulated DOGM) and Color UI

  #define TFT_SCK_PIN                EXP2_02_PIN
  #define TFT_MISO_PIN               EXP2_01_PIN
  #define TFT_MOSI_PIN               EXP2_06_PIN

  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN

  #ifndef TFT_WIDTH
    #define TFT_WIDTH                        480
  #endif
  #ifndef TFT_HEIGHT
    #define TFT_HEIGHT                       320
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
    #define TFT_CS_PIN               EXP2_04_PIN
    #define TFT_DC_PIN               EXP2_07_PIN
    #define TFT_A0_PIN                TFT_DC_PIN

    #define TOUCH_CS_PIN             EXP1_04_PIN
    #define TOUCH_SCK_PIN            EXP1_05_PIN
    #define TOUCH_MISO_PIN           EXP1_06_PIN
    #define TOUCH_MOSI_PIN           EXP1_03_PIN
    #define TOUCH_INT_PIN            EXP1_07_PIN

    #ifndef TOUCH_CALIBRATION_X
      #define TOUCH_CALIBRATION_X          17540
    #endif
    #ifndef TOUCH_CALIBRATION_Y
      #define TOUCH_CALIBRATION_Y         -11388
    #endif
    #ifndef TOUCH_OFFSET_X
      #define TOUCH_OFFSET_X                 -21
    #endif
    #ifndef TOUCH_OFFSET_Y
      #define TOUCH_OFFSET_Y                 337
    #endif
    #ifndef TOUCH_ORIENTATION
      #define TOUCH_ORIENTATION TOUCH_LANDSCAPE
    #endif

  #elif ENABLED(MKS_TS35_V2_0)

    /**                      ------                                   ------
     *               BEEPER | 1  2 | BTN_ENC               SPI1_MISO | 1  2 | SPI1_SCK
     *     TFT_BKL / LCD_EN | 3  4 | TFT_RESET / LCD_RS      BTN_EN1 | 3  4 | SPI1_CS
     *    TOUCH_CS / LCD_D4 | 5  6   TOUCH_INT / LCD_D5      BTN_EN2 | 5  6   SPI1_MOSI
     *     SPI1_CS / LCD_D6 | 7  8 | SPI1_RS / LCD_D7       SPI1_RS  | 7  8 | RESET
     *                  GND | 9 10 | VCC                         GND | 9 10 | VCC
     *                       ------                                   ------
     *                        EXP1                                     EXP2
     */
    #define TFT_CS_PIN               EXP1_07_PIN  // SPI1_CS
    #define TFT_DC_PIN               EXP1_08_PIN  // SPI1_RS
    #define TFT_A0_PIN                TFT_DC_PIN

    #define TFT_RESET_PIN            EXP1_04_PIN

    #define LCD_BACKLIGHT_PIN        EXP1_03_PIN
    #define TFT_BACKLIGHT_PIN  LCD_BACKLIGHT_PIN

    #define TOUCH_BUTTONS_HW_SPI
    #define TOUCH_BUTTONS_HW_SPI_DEVICE        1

    #define TOUCH_CS_PIN             EXP1_05_PIN  // SPI1_NSS
    #define TOUCH_SCK_PIN            EXP2_02_PIN  // SPI1_SCK
    #define TOUCH_MISO_PIN           EXP2_01_PIN  // SPI1_MISO
    #define TOUCH_MOSI_PIN           EXP2_06_PIN  // SPI1_MOSI

    #define LCD_READ_ID                   0xD3
    #define LCD_USE_DMA_SPI

    #define TFT_BUFFER_WORDS               14400

    #ifndef TOUCH_CALIBRATION_X
      #define TOUCH_CALIBRATION_X         -17253
    #endif
    #ifndef TOUCH_CALIBRATION_Y
      #define TOUCH_CALIBRATION_Y          11579
    #endif
    #ifndef TOUCH_OFFSET_X
      #define TOUCH_OFFSET_X                 514
    #endif
    #ifndef TOUCH_OFFSET_Y
      #define TOUCH_OFFSET_Y                 -24
    #endif
    #ifndef TOUCH_ORIENTATION
      #define TOUCH_ORIENTATION TOUCH_LANDSCAPE
    #endif

  #endif

#endif

//
// NeoPixel LED
// The FYSETC_MINI_12864_2_1 uses one of the EXP pins
//
#if DISABLED(FYSETC_MINI_12864_2_1) && !defined(NEOPIXEL_PIN)
  #define NEOPIXEL_PIN                      PF10  // NEOPIXEL
#endif
