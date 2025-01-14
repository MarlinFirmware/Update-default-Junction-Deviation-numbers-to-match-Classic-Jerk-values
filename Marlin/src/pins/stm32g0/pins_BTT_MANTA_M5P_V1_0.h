/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "BTT Manta M5P V1.0"
#endif

#define USES_DIAG_JUMPERS

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

//
// EEPROM
//
#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #undef NO_EEPROM_SELECTED
  #ifndef FLASH_EEPROM_EMULATION
    #define FLASH_EEPROM_EMULATION
  #endif
  #define EEPROM_PAGE_SIZE      (0x800UL) // 2K
  #define EEPROM_START_ADDRESS  (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE
#endif

//
// Servos
//
#define SERVO0_PIN                          PC15  // PROBE

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Trinamic StallGuard pins
//
#define X_DIAG_PIN                          PD3   // MIN1
#define Y_DIAG_PIN                          PD2   // MIN2
#define Z_DIAG_PIN                          PC3   // MIN3
#define E0_DIAG_PIN                         PC2   // MIN4
#define E1_DIAG_PIN                         -1

//
// Limit Switches
//
#define X_STOP_PIN                    X_DIAG_PIN  // MIN1
#define Y_STOP_PIN                    Y_DIAG_PIN  // MIN1
#define Z_STOP_PIN                    Z_DIAG_PIN  // MIN3

//
// Z Probe (when not Z_STOP_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PC13  // PROBE
  //#define Z_MIN_PROBE_PIN                 PC15  // IND-DET (with adjustable pullup set via jumper)
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN             E0_DIAG_PIN  // MIN4
#endif

//
// Steppers
//
#define X_STEP_PIN                          PC8
#define X_DIR_PIN                           PC9
#define X_ENABLE_PIN                        PA15
#ifndef X_CS_PIN
  #define X_CS_PIN                          PD9
#endif

#define Y_STEP_PIN                          PA10
#define Y_DIR_PIN                           PA14
#define Y_ENABLE_PIN                        PA13
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PD8
#endif

#define Z_STEP_PIN                          PC6
#define Z_DIR_PIN                           PC7
#define Z_ENABLE_PIN                        PA9
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PB10
#endif

#define E0_STEP_PIN                         PB12
#define E0_DIR_PIN                          PB11
#define E0_ENABLE_PIN                       PA8
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PB2
#endif

#define E1_STEP_PIN                         PB0
#define E1_DIR_PIN                          PB1
#define E1_ENABLE_PIN                       PC4
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PA6
#endif

//
// Default pins for TMC software SPI
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PB15  // Shared with SPI header, Pin 5 (SPI2)
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PB14  // Shared with SPI header, Pin 6 (SPI2)
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PB13  // Shared with SPI header, Pin 4 (SPI2)
#endif

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   PD9   // X_CS_PIN
  #define Y_SERIAL_TX_PIN                   PD8   // Y_CS_PIN
  #define Z_SERIAL_TX_PIN                   PB10  // Z_CS_PIN
  #define E0_SERIAL_TX_PIN                  PB2   // E0_CS_PIN
  #define E1_SERIAL_TX_PIN                  PA6   // E1_CS_PIN

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA1   // Analog Input "TH0"
#define TEMP_1_PIN                          PA2   // Analog Input "TH1"
#define TEMP_BED_PIN                        PA0   // Analog Input "TB"

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC5   // "HE0"
#define HEATER_1_PIN                        PA7   // "HE1"
#define HEATER_BED_PIN                      PA5   // "HB"

#define FAN0_PIN                            PA4   // "FAN0"
#define FAN1_PIN                            PA3   // "FAN1"

//
// Auto fans
//
#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN               FAN1_PIN
#endif

/**
 *                ------                                   ------
 *  (BEEPER) PD5 | 1  2 | PD4 (BTN_ENC)       (MISO) PB14 | 1  2 | PB13 (SCK)
 *  (LCD_EN) PB3 | 3  4 | PD6 (LCD_RS)     (BTN_EN1)  PB8 | 3  4 | PB9  (SD_SS)
 *  (LCD_D4) PB5 | 5  6   PB4 (LCD_D5)     (BTN_EN2) PC10 | 5  6   PB15 (MOSI)
 *  (LCD_D6) PB7 | 7  8 | PB6 (LCD_D7)   (SD_DETECT) PC12 | 7  8 | PF2
 *           GND | 9 10 | 5V                          GND | 9 10 | --
 *                ------                                   ------
 *                 EXP1                                     EXP2
 */
#define EXP1_01_PIN                         PD5
#define EXP1_02_PIN                         PD4
#define EXP1_03_PIN                         PB3
#define EXP1_04_PIN                         PD6
#define EXP1_05_PIN                         PB5
#define EXP1_06_PIN                         PB4
#define EXP1_07_PIN                         PB7
#define EXP1_08_PIN                         PB6

#define EXP2_01_PIN                         PB14
#define EXP2_02_PIN                         PB13
#define EXP2_03_PIN                         PB8
#define EXP2_04_PIN                         PB9
#define EXP2_05_PIN                         PC10
#define EXP2_06_PIN                         PB15
#define EXP2_07_PIN                         PC12
#define EXP2_08_PIN                         -1

//
// Onboard SD card
// Must use soft SPI because Marlin's default hardware SPI is tied to LCD's EXP2
//
#if SD_CONNECTION_IS(LCD)
  #define SDSS                       EXP2_04_PIN
  #define SD_SS_PIN                         SDSS
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN
#elif SD_CONNECTION_IS(ONBOARD)
  #define SD_SCK_PIN                        PB13
  #define SD_MISO_PIN                       PB14
  #define SD_MOSI_PIN                       PB15
  #define ONBOARD_SD_CS_PIN                 PC1   // Chip select for "System" SD card
  #define SD_SS_PIN            ONBOARD_SD_CS_PIN
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "No custom SD drive cable defined for this board."
#endif

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
    #define BOARD_ST7920_DELAY_1             120
  #endif
  #ifndef BOARD_ST7920_DELAY_2
    #define BOARD_ST7920_DELAY_2              80
  #endif
  #ifndef BOARD_ST7920_DELAY_3
    #define BOARD_ST7920_DELAY_3             580
  #endif
#endif

//
// NeoPixel LED
//
#ifndef BOARD_NEOPIXEL_PIN
  #define BOARD_NEOPIXEL_PIN                PC11  // RGB1
#endif

#ifndef NEOPIXEL2_PIN
  #define NEOPIXEL2_PIN                     PC14  // RGB2
#endif
