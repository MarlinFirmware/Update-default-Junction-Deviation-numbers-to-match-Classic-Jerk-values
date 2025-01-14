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

#define USES_DIAG_PINS

// If you have the BigTreeTech driver expansion module, enable BTT_MOTOR_EXPANSION
// https://github.com/bigtreetech/BTT-Expansion-module/tree/master/BTT%20EXP-MOT
//#define BTT_MOTOR_EXPANSION

#if ALL(HAS_WIRED_LCD, BTT_MOTOR_EXPANSION)
  #if ANY(CR10_STOCKDISPLAY, ENDER2_STOCKDISPLAY)
    #define EXP_MOT_USE_EXP2_ONLY 1
  #else
    #error "You can't use both an LCD and a Motor Expansion Module on EXP1/EXP2 at the same time."
  #endif
#endif

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

#define HAS_OTG_USB_HOST_SUPPORT                  // USB Flash Drive support

//
// Servos
//
#define SERVO0_PIN                          PA1
#define SERVO1_PIN                          PC9

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PB10  // X-
#define Y_DIAG_PIN                          PE12  // Y-
#define Z_DIAG_PIN                          PG8   // Z-
#define E0_DIAG_PIN                         PE15  // E0
#define E1_DIAG_PIN                         PE10  // E1
#define E2_DIAG_PIN                         PG5   // E2

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
  #if X_HOME_TO_MIN
    #define X_MAX_PIN                       PE15  // E0
  #else
    #define X_MIN_PIN                       PE15  // E0
  #endif
#else
  #define X_MIN_PIN                         PB10  // X-
  #define X_MAX_PIN                         PE15  // E0
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
  #if Y_HOME_TO_MIN
    #define Y_MAX_PIN                       PE10  // E1
  #else
    #define Y_MIN_PIN                       PE10  // E1
  #endif
#else
  #define Y_MIN_PIN                         PE12  // Y-
  #define Y_MAX_PIN                         PE10  // E1
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
  #if Z_HOME_TO_MIN
    #define Z_MAX_PIN                       PG5   // E2
  #else
    #define Z_MIN_PIN                       PG5   // E2
  #endif
#else
  #define Z_MIN_PIN                         PG8   // Z-
  #define Z_MAX_PIN                         PG5   // E2
#endif

//
// Z Probe must be this pin
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PA2
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
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PE15
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN                   PE10
#endif
#ifndef FIL_RUNOUT3_PIN
  #define FIL_RUNOUT3_PIN                   PG5
#endif

//
// Steppers
//
#define X_STEP_PIN                          PE9
#define X_DIR_PIN                           PF1
#define X_ENABLE_PIN                        PF2
#ifndef X_CS_PIN
  #define X_CS_PIN                          PA15
#endif

#define Y_STEP_PIN                          PE11
#define Y_DIR_PIN                           PE8
#define Y_ENABLE_PIN                        PD7
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PB8
#endif

#define Z_STEP_PIN                          PE13
#define Z_DIR_PIN                           PC2
#define Z_ENABLE_PIN                        PC0
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PB9
#endif

#define E0_STEP_PIN                         PE14
#define E0_DIR_PIN                          PA0
#define E0_ENABLE_PIN                       PC3
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PB3
#endif

#define E1_STEP_PIN                         PD15
#define E1_DIR_PIN                          PE7
#define E1_ENABLE_PIN                       PA3
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PG15
#endif

#define E2_STEP_PIN                         PD13
#define E2_DIR_PIN                          PG9
#define E2_ENABLE_PIN                       PF0
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PG12
#endif

//
// SPI pins for TMC2130 stepper drivers
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PC12
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PC11
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PC10
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

  #define X_SERIAL_TX_PIN                   PC13
  #define Y_SERIAL_TX_PIN                   PE3
  #define Z_SERIAL_TX_PIN                   PE1
  #define E0_SERIAL_TX_PIN                  PD4
  #define E1_SERIAL_TX_PIN                  PD1
  #define E2_SERIAL_TX_PIN                  PD6

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temperature Sensors
// Use ADC pins without pullup for sensors that don't need a pullup.
//
#if TEMP_SENSOR_0_IS_AD8495 || TEMP_SENSOR_0 == 20
  #define TEMP_0_PIN                        PF8
#else
  #define TEMP_0_PIN                        PF4   // T1 <-> E0
#endif
#if TEMP_SENSOR_1_IS_AD8495 || TEMP_SENSOR_1 == 20
  #define TEMP_1_PIN                        PF9
#else
  #define TEMP_1_PIN                        PF5   // T2 <-> E1
#endif
#if TEMP_SENSOR_2_IS_AD8495 || TEMP_SENSOR_2 == 20
  #define TEMP_2_PIN                        PF10
#else
  #define TEMP_2_PIN                        PF6   // T3 <-> E2
#endif
#if TEMP_SENSOR_BED_IS_AD8495 || TEMP_SENSOR_BED == 20
  #define TEMP_BED_PIN                      PF7
#else
  #define TEMP_BED_PIN                      PF3   // T0 <-> Bed
#endif

#if TEMP_SENSOR_PROBE && !defined(TEMP_PROBE_PIN)
  #if TEMP_SENSOR_PROBE_IS_AD8495 || TEMP_SENSOR_PROBE == 20
    #if HOTENDS == 2
      #define TEMP_PROBE_PIN                PF10
    #elif HOTENDS < 2
      #define TEMP_PROBE_PIN                PF9
    #endif
  #else
    #if HOTENDS == 2
      #define TEMP_PROBE_PIN          TEMP_2_PIN
    #elif HOTENDS < 2
      #define TEMP_PROBE_PIN          TEMP_1_PIN
    #endif
  #endif
#endif

#if TEMP_SENSOR_CHAMBER && !defined(TEMP_CHAMBER_PIN)
  #if TEMP_SENSOR_CHAMBER_IS_AD8495 || TEMP_SENSOR_CHAMBER == 20
    #define TEMP_CHAMBER_PIN                PF10
  #else
    #define TEMP_CHAMBER_PIN          TEMP_2_PIN
  #endif
#endif

//
// Heaters
//
#define HEATER_0_PIN                        PB1   // Heater0
#define HEATER_1_PIN                        PD14  // Heater1
#if TEMP_SENSOR_CHAMBER && HOTENDS < 3
  #define HEATER_CHAMBER_PIN                PB0   // Heater2
#else
  #define HEATER_2_PIN                      PB0   // Heater2
#endif
#define HEATER_BED_PIN                      PD12  // Hotbed

//
// Fans
//
#define FAN0_PIN                            PC8   // Fan0
#define FAN1_PIN                            PE5   // Fan1

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN               FAN1_PIN
#endif

#if !defined(CONTROLLER_FAN_PIN) && ENABLED(USE_CONTROLLER_FAN) && HOTENDS < 2
  #define CONTROLLER_FAN_PIN                PE6   // Fan2
#else
  #define FAN2_PIN                          PE6   // Fan2
#endif

//
// Misc. Functions
//

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

/**               ------                                      ------
 * (BEEPER) PG4  | 1  2 | PA8  (BTN_ENC)         (MISO) PB14 | 1  2 | PB13 (SCK)
 * (LCD_EN) PD11 | 3  4 | PD10 (LCD_RS)       (BTN_EN1) PG10 | 3  4 | PB12 (SD_SS)
 * (LCD_D4) PG2    5  6 | PG3  (LCD_D5)       (BTN_EN2) PF11   5  6 | PB15 (MOSI)
 * (LCD_D6) PG6  | 7  8 | PG7  (LCD_D7)     (SD_DETECT) PF12 | 7  8 | RESET
 *           GND | 9 10 | 5V                             GND | 9 10 | --
 *                ------                                      ------
 *                 EXP1                                        EXP2
 */
#define EXP1_01_PIN                         PG4
#define EXP1_02_PIN                         PA8
#define EXP1_03_PIN                         PD11
#define EXP1_04_PIN                         PD10
#define EXP1_05_PIN                         PG2
#define EXP1_06_PIN                         PG3
#define EXP1_07_PIN                         PG6
#define EXP1_08_PIN                         PG7

#define EXP2_01_PIN                         PB14
#define EXP2_02_PIN                         PB13
#define EXP2_03_PIN                         PG10
#define EXP2_04_PIN                         PB12
#define EXP2_05_PIN                         PF11
#define EXP2_06_PIN                         PB15
#define EXP2_07_PIN                         PF12
#define EXP2_08_PIN                         -1

//
// Onboard SD card
// Must use soft SPI because Marlin's default hardware SPI is tied to LCD's EXP2
//
#if SD_CONNECTION_IS(LCD)

  #define SD_DETECT_PIN              EXP2_07_PIN
  #define SDSS                       EXP2_04_PIN

#elif SD_CONNECTION_IS(ONBOARD)

  // The SKR Pro's ONBOARD SD interface is on SPI1.
  // Due to a pull resistor on the clock line, it needs to use SPI Data Mode 3 to
  // function with Hardware SPI. This is not currently configurable in the HAL,
  // so force Software SPI to work around this issue.
  #define SOFTWARE_SPI
  #define SDSS                              PA4
  #define SD_SCK_PIN                        PA5
  #define SD_MISO_PIN                       PA6
  #define SD_MOSI_PIN                       PB5
  #define SD_DETECT_PIN                     PB11

#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "CUSTOM_CABLE is not a supported SDCARD_CONNECTION for this board"
#endif

#if ENABLED(BTT_MOTOR_EXPANSION)
  /**       -----                        -----
   *    -- | . . | GND               -- | . . | GND
   *    -- | . . | M1EN            M2EN | . . | M3EN
   * M1STP | . .   M1DIR           M1RX | . .   M1DIAG
   * M2DIR | . . | M2STP           M2RX | . . | M2DIAG
   * M3DIR | . . | M3STP           M3RX | . . | M3DIAG
   *        -----                        -----
   *        EXP2                         EXP1
   *
   * NB In EXP_MOT_USE_EXP2_ONLY mode EXP1 is not used and M2EN and M3EN need to be jumpered to M1EN
   */

  // M1 on Driver Expansion Module
  #define E3_STEP_PIN                EXP2_06_PIN
  #define E3_DIR_PIN                 EXP2_05_PIN
  #define E3_ENABLE_PIN              EXP2_07_PIN
  #if !EXP_MOT_USE_EXP2_ONLY
    #define E3_DIAG_PIN              EXP1_05_PIN
    #define E3_CS_PIN                EXP1_06_PIN
    #if HAS_TMC_UART
      #define E3_SERIAL_TX_PIN       EXP1_06_PIN
    #endif
  #endif

  // M2 on Driver Expansion Module
  #define E4_STEP_PIN                EXP2_03_PIN
  #define E4_DIR_PIN                 EXP2_04_PIN
  #if !EXP_MOT_USE_EXP2_ONLY
    #define E4_ENABLE_PIN            EXP1_08_PIN
    #define E4_DIAG_PIN              EXP1_03_PIN
    #define E4_CS_PIN                EXP1_04_PIN
    #if HAS_TMC_UART
      #define E4_SERIAL_TX_PIN       EXP1_04_PIN
    #endif
  #else
    #define E4_ENABLE_PIN            EXP2_07_PIN
  #endif

  // M3 on Driver Expansion Module
  #define E5_STEP_PIN                EXP2_01_PIN
  #define E5_DIR_PIN                 EXP2_02_PIN
  #if !EXP_MOT_USE_EXP2_ONLY
    #define E5_ENABLE_PIN            EXP1_07_PIN
    #define E5_DIAG_PIN              EXP1_01_PIN
    #define E5_CS_PIN                EXP1_02_PIN
    #if HAS_TMC_UART
      #define E5_SERIAL_TX_PIN       EXP1_02_PIN
    #endif
  #else
    #define E5_ENABLE_PIN            EXP2_07_PIN
  #endif

#endif // BTT_MOTOR_EXPANSION

//
// LCD / Controller
//

#if IS_TFTGLCD_PANEL

  #if ENABLED(TFTGLCD_PANEL_SPI)
    #define TFTGLCD_CS               EXP2_03_PIN
  #endif

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #if ENABLED(CR10_STOCKDISPLAY)

    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #elif ENABLED(MKS_MINI_12864)

    #define DOGLCD_A0                EXP1_07_PIN
    #define DOGLCD_CS                EXP1_06_PIN
    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

  #elif ENABLED(WYH_L12864)

    CONTROLLER_WARNING("BTT_SKR_PRO_common", "WYH_L12864")

    /**
     * 1. Cut the tab off the LCD connector so it can be plugged into the "EXP1" connector the other way.
     * 2. Swap the LCD's +5V (Pin2) and GND (Pin1) wires.
     *
     * !!! If you are unsure, ask for help! Your motherboard may be damaged in some circumstances !!!
     *
     * The WYH_L12864 connector plug:
     *
     *                  BEFORE                     AFTER
     *                  ------                     ------
     *              -- | 1  2 | MOSI           -- | 1  2 | MOSI
     *         BTN_ENC | 3  4 | SCK       BTN_ENC | 3  4 | SCK
     *         BTN_EN1 | 5  6   SID       BTN_EN1 | 5  6   SID
     *         BTN_EN2 | 7  8 | CS        BTN_EN2 | 7  8 | CS
     *              5V | 9 10 | GND           GND | 9 10 | 5V
     *                  ------                     ------
     *                   LCD                        LCD
     */
    #undef BEEPER_PIN
    #undef BTN_ENC
    #define BTN_EN1                  EXP1_05_PIN
    #define BTN_EN2                  EXP1_07_PIN
    #define BTN_ENC                  EXP1_03_PIN
    #define DOGLCD_CS                EXP1_08_PIN
    #define DOGLCD_A0                EXP1_06_PIN
    #define DOGLCD_SCK               EXP1_04_PIN
    #define DOGLCD_MOSI              EXP1_02_PIN
    #define LCD_BACKLIGHT_PIN               -1

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
  #ifndef BOARD_ST7920_DELAY_1
    #define BOARD_ST7920_DELAY_1             125
  #endif
  #ifndef BOARD_ST7920_DELAY_2
    #define BOARD_ST7920_DELAY_2              90
  #endif
  #ifndef BOARD_ST7920_DELAY_3
    #define BOARD_ST7920_DELAY_3             600
  #endif
#endif

#if ENABLED(WIFISUPPORT)
  //
  // WIFI
  //

  /**
   *          ------
   *      RX | 8  7 | 3.3V      GPIO0  PF14 ... Leave as unused (ESP3D software configures this with a pullup so OK to leave as floating)
   *   GPIO0 | 6  5 | Reset     GPIO2  PF15 ... must be high (ESP3D software configures this with a pullup so OK to leave as floating)
   *   GPIO2 | 4  3 | Enable    Reset  PG0  ... active low, probably OK to leave floating
   *     GND | 2  1 | TX        Enable PG1  ... Must be high for module to run
   *          ------
   *            W1
   */
  #define ESP_WIFI_MODULE_COM                  6  // Must also set either SERIAL_PORT or SERIAL_PORT_2 to this
  #define ESP_WIFI_MODULE_BAUDRATE      BAUDRATE  // Must use same BAUDRATE as SERIAL_PORT & SERIAL_PORT_2
  #define ESP_WIFI_MODULE_RESET_PIN         PG0
  #define ESP_WIFI_MODULE_ENABLE_PIN        PG1
  #define ESP_WIFI_MODULE_GPIO0_PIN         PF14
  #define ESP_WIFI_MODULE_GPIO2_PIN         PF15
#endif
