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

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "BIGTREE BTT002 V1.0 only supports 1 hotend / E stepper."
#endif

#define BOARD_INFO_NAME "BTT BTT002 V1.0"

//#define MK3_FAN_PINS

#define USES_DIAG_PINS

// Ignore temp readings during development.
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

//
// Limit Switches
//
#define X_STOP_PIN                          PD3
#define Y_STOP_PIN                          PD2
#define Z_STOP_PIN                          PD1   // Shares J4 connector with PC3

//
// Z Probe must be this pin
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PD1
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA15
#endif

//
// Power Loss Detection
//
#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PD4
#endif

//
// Steppers
//
#define X_STEP_PIN                          PA9
#define X_DIR_PIN                           PA10
#define X_ENABLE_PIN                        PA8
#ifndef X_CS_PIN
  #define X_CS_PIN                          PE2
#endif

#define Y_STEP_PIN                          PC8
#define Y_DIR_PIN                           PC9
#define Y_ENABLE_PIN                        PC7
 #ifndef Y_CS_PIN
  #define Y_CS_PIN                          PE3
#endif

#define Z_STEP_PIN                          PD15
#define Z_DIR_PIN                           PC6
#define Z_ENABLE_PIN                        PD14
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PE4
#endif

#define E0_STEP_PIN                         PD12
#define E0_DIR_PIN                          PD13
#define E0_ENABLE_PIN                       PD11
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD7
#endif

//
// SPI pins for TMC2130 stepper drivers
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PB15
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PB14
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PB13
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL Serial1
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

  #define X_SERIAL_TX_PIN                   PE2
  #define Y_SERIAL_TX_PIN                   PE3
  #define Z_SERIAL_TX_PIN                   PE4
  #define E0_SERIAL_TX_PIN                  PD7

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA2   // T0 <-> E0
#define TEMP_1_PIN                          PA0   // T1 <-> E1
#define TEMP_BED_PIN                        PA1   // T2 <-> Bed
#define TEMP_PROBE_PIN                      PC3   // Shares J4 connector with PD1

#ifndef TEMP_BOARD_PIN
  #define TEMP_BOARD_PIN                    PC2   // Onboard thermistor, NTC100K
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PE6   // Heater0
#define HEATER_BED_PIN                      PE5   // Hotbed

#ifndef FAN0_PIN
  #ifdef MK3_FAN_PINS
    #define FAN0_PIN                        PB8   // Fan1
  #else
    #define FAN0_PIN                        PB9   // Fan0
  #endif
#endif

#ifndef FAN1_PIN
  #ifdef MK3_FAN_PINS
    #define FAN1_PIN                        PB9   // Fan0
  #else
    #define FAN1_PIN                        PB8   // Fan1
  #endif
#endif

#ifndef E0_FAN_TACHO_PIN
  #ifdef MK3_FAN_PINS
    #define E0_FAN_TACHO_PIN                PE1   // Fan1
  #else
    #define E0_FAN_TACHO_PIN                PE0   // Fan0
  #endif
#endif

/**
 *                ------                                    ------
 * (BEEPER) PE7  | 1  2 | PB1  (BTN_ENC)     (MISO)    PA6 | 1  2 | PA5 (SCK)
 * (LCD_EN) PE9  | 3  4 | PE8  (LCD_RS)      (BTN_EN1) PC5 | 3  4 | PA4 (SD_SS)
 * (LCD_D4) PE10   5  6 | PE11 (LCD_D5)      (BTN_EN2) PB0   5  6 | PA7 (MOSI)
 * (LCD_D6) PE12 | 7  8 | PE13 (LCD_D7)      (SD_DET)  PC4 | 7  8 | RESET
 *           GND | 9 10 | 5V                           GND | 9 10 | PA3
 *                ------                                    ------
 *                 EXP1                                      EXP2
 */
#define EXP1_01_PIN                         PE7
#define EXP1_02_PIN                         PB1
#define EXP1_03_PIN                         PE9
#define EXP1_04_PIN                         PE8
#define EXP1_05_PIN                         PE10
#define EXP1_06_PIN                         PE11
#define EXP1_07_PIN                         PE12
#define EXP1_08_PIN                         PE13

#define EXP2_01_PIN                         PA6
#define EXP2_02_PIN                         PA5
#define EXP2_03_PIN                         PC5
#define EXP2_04_PIN                         PA4
#define EXP2_05_PIN                         PB0
#define EXP2_06_PIN                         PA7
#define EXP2_07_PIN                         PC4
#define EXP2_08_PIN                         -1
#define EXP2_10_PIN                         PA3

// HAL SPI1 pins
#define SD_SCK_PIN                   EXP2_02_PIN  // SPI1 SCLK
#define SD_SS_PIN                    EXP2_04_PIN  // SPI1 SSEL
#define SD_MISO_PIN                  EXP2_01_PIN  // SPI1 MISO
#define SD_MOSI_PIN                  EXP2_06_PIN  // SPI1 MOSI

#define SDSS                         EXP2_04_PIN

//
// LCD / Controller
//

#if HAS_WIRED_LCD
  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #define SD_DETECT_PIN              EXP2_07_PIN

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

  #else

    #define LCD_PINS_RS              EXP1_04_PIN

    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

    #define LCD_PINS_EN              EXP1_03_PIN
    #define LCD_PINS_D4              EXP1_05_PIN

    #if ENABLED(FYSETC_MINI_12864)
      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_A0              EXP1_04_PIN
      #define DOGLCD_MOSI            EXP2_06_PIN
      #define DOGLCD_MISO            EXP2_01_PIN
      #define DOGLCD_SCK             EXP2_02_PIN

      #define LCD_BACKLIGHT_PIN             -1

      #define FORCE_SOFT_SPI

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

  // Alter timing for graphical display
  #if IS_U8GLIB_ST7920
    #define BOARD_ST7920_DELAY_1              96
    #define BOARD_ST7920_DELAY_2              48
    #define BOARD_ST7920_DELAY_3             600
  #endif

#endif // HAS_WIRED_LCD

//
// RGB LEDs
//
#ifndef RGB_LED_R_PIN
  #define RGB_LED_R_PIN                     PB5
#endif
#ifndef RGB_LED_G_PIN
  #define RGB_LED_G_PIN                     PB4
#endif
#ifndef RGB_LED_B_PIN
  #define RGB_LED_B_PIN                     PB3
#endif
#ifndef RGB_LED_W_PIN
  #define RGB_LED_W_PIN                     -1
#endif
