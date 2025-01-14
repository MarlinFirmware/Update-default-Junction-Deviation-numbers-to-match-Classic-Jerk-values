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

#define BOARD_INFO_NAME "BTT SKR Mini V1.1"

//#define DISABLE_DEBUG
#define DISABLE_JTAG

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

//
// Limit Switches
//
#define X_MIN_PIN                           PC2
#define X_MAX_PIN                           PA2
#define Y_MIN_PIN                           PC1
#define Y_MAX_PIN                           PA1
#define Z_MIN_PIN                           PC0
#define Z_MAX_PIN                           PC3

//
// Steppers
//

#define X_STEP_PIN                          PC6
#define X_DIR_PIN                           PC7
#define X_ENABLE_PIN                        PB15

#define Y_STEP_PIN                          PB13
#define Y_DIR_PIN                           PB14
#define Y_ENABLE_PIN                        PB12

#define Z_STEP_PIN                          PB10
#define Z_DIR_PIN                           PB11
#define Z_ENABLE_PIN                        PB2

#define E0_STEP_PIN                         PC5
#define E0_DIR_PIN                          PB0
#define E0_ENABLE_PIN                       PC4

// Shared with EXP2
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PB3
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PB4
#endif
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PB5
#endif

#if HAS_TMC_UART                                  // Shared with EXP1
  #define X_SERIAL_TX_PIN                   PC10
  #define Y_SERIAL_TX_PIN                   PC11
  #define Z_SERIAL_TX_PIN                   PC12
  #define E0_SERIAL_TX_PIN                  PC14

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA8
#define FAN0_PIN                            PC8
#define HEATER_BED_PIN                      PC9

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PB1   // Analog Input
#define TEMP_0_PIN                          PA0   // Analog Input

/**                ------                                    ------
 *  (BEEPER) PC10 | 1  2 | PC11 (BTN_ENC)        (MISO) PB4 | 1  2 | PB3   (SCK)
 *  (LCD_EN) PB6  | 3  4 | PC12  (LCD_RS)     (BTN_EN1) PD2 | 3  4 | PA15  (SD_SS)
 *  (LCD_D4) PC13   5  6 | PB7   (LCD_D5)     (BTN_EN2) PB8   5  6 | PB5   (MOSI)
 *  (LCD_D6) PC15 | 7  8 | PC14  (LCD_D7)   (SD_DETECT) PB9 | 7  8 | RESET
 *            GND | 9 10 | 5V                           GND | 9 10 | --
 *                 ------                                    ------
 *                  EXP1                                      EXP2
 */
#define EXP1_01_PIN                         PC10
#define EXP1_02_PIN                         PC11
#define EXP1_03_PIN                         PB6
#define EXP1_04_PIN                         PC12
#define EXP1_05_PIN                         PC13
#define EXP1_06_PIN                         PB7
#define EXP1_07_PIN                         PC15
#define EXP1_08_PIN                         PC14

#define EXP2_01_PIN                         PB4
#define EXP2_02_PIN                         PB3
#define EXP2_03_PIN                         PD2
#define EXP2_04_PIN                         PA15
#define EXP2_05_PIN                         PB8
#define EXP2_06_PIN                         PB5
#define EXP2_07_PIN                         PB9
#define EXP2_08_PIN                         -1    // RESET

//
// LCD / Controller
//

#if ANY(TFT_COLOR_UI, TFT_CLASSIC_UI)
  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN
  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN

  #define TFT_CS_PIN                 EXP1_07_PIN
  #define TFT_SCK_PIN                EXP2_02_PIN
  #define TFT_MISO_PIN               EXP2_01_PIN
  #define TFT_MOSI_PIN               EXP2_06_PIN
  #define TFT_DC_PIN                 EXP1_08_PIN
  #define TFT_A0_PIN                  TFT_DC_PIN

  #define TFT_RESET_PIN              EXP1_04_PIN

  #define LCD_BACKLIGHT_PIN          EXP1_03_PIN
  #define TFT_BACKLIGHT_PIN    LCD_BACKLIGHT_PIN

  #define TOUCH_BUTTONS_HW_SPI
  #define TOUCH_BUTTONS_HW_SPI_DEVICE          3

  #define TOUCH_CS_PIN               EXP1_05_PIN  // SPI3_NSS
  #define TOUCH_SCK_PIN              EXP2_02_PIN  // SPI3_SCK
  #define TOUCH_MISO_PIN             EXP2_01_PIN  // SPI3_MISO
  #define TOUCH_MOSI_PIN             EXP2_06_PIN  // SPI3_MOSI

  #define LCD_READ_ID                       0xD3
  #define LCD_USE_DMA_SPI

  #define TFT_BUFFER_WORDS                  9600

#elif HAS_WIRED_LCD
  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #if ENABLED(CR10_STOCKDISPLAY)
    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #elif IS_TFTGLCD_PANEL

    #undef BEEPER_PIN
    #undef BTN_ENC

    #if ENABLED(TFTGLCD_PANEL_SPI)
      #define TFTGLCD_CS             EXP2_03_PIN
    #endif

    #define SD_DETECT_PIN            EXP2_07_PIN

  #else

    #define LCD_PINS_RS              EXP1_04_PIN

    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

    #define LCD_PINS_EN              EXP1_03_PIN

    #if ENABLED(FYSETC_MINI_12864)

      #define LCD_BACKLIGHT_PIN             -1
      #define LCD_RESET_PIN          EXP1_05_PIN
      #define DOGLCD_A0              EXP1_04_PIN
      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_SCK             EXP2_02_PIN
      #define DOGLCD_MOSI            EXP2_06_PIN

      #define FORCE_SOFT_SPI                      // SPI MODE3

      #define LED_PIN                EXP1_06_PIN  // red pwm
      //#define LED_PIN              EXP1_07_PIN  // green
      //#define LED_PIN              EXP1_08_PIN  // blue

      //#if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
      //  #ifndef RGB_LED_R_PIN
      //    #define RGB_LED_R_PIN    EXP1_06_PIN
      //  #endif
      //  #ifndef RGB_LED_G_PIN
      //    #define RGB_LED_G_PIN    EXP1_07_PIN
      //  #endif
      //  #ifndef RGB_LED_B_PIN
      //    #define RGB_LED_B_PIN    EXP1_08_PIN
      //  #endif
      //#elif ENABLED(FYSETC_MINI_12864_2_1)
      //  #define NEOPIXEL_PIN       EXP1_06_PIN
      //#endif

    #else // !FYSETC_MINI_12864

      #define LCD_PINS_D4            EXP1_05_PIN
      #if IS_ULTIPANEL
        #define LCD_PINS_D5          EXP1_06_PIN
        #define LCD_PINS_D6          EXP1_07_PIN
        #define LCD_PINS_D7          EXP1_08_PIN

        #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
          #define BTN_ENC_EN         LCD_PINS_D7  // Detect the presence of the encoder
        #endif

      #endif

    #endif // !FYSETC_MINI_12864

    // Alter timing for graphical display
    #if IS_U8GLIB_ST7920
      #define BOARD_ST7920_DELAY_1           125
      #define BOARD_ST7920_DELAY_2           125
      #define BOARD_ST7920_DELAY_3           125
    #endif

  #endif

#endif // HAS_WIRED_LCD

//
// SD Card
//

// By default the onboard SD is enabled.
// Change SDCARD_CONNECTION from 'ONBOARD' to 'LCD' for an external (LCD module) SD
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(LCD)
  #define SPI_DEVICE                           3  // Maple
  #define SD_DETECT_PIN              EXP2_07_PIN
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_SS_PIN                  EXP2_04_PIN
#elif SD_CONNECTION_IS(ONBOARD)
  #define SD_DETECT_PIN                     PA3
  #define SD_SCK_PIN                        PA5
  #define SD_MISO_PIN                       PA6
  #define SD_MOSI_PIN                       PA7
  #define SD_SS_PIN                         PA4
#endif

#define ONBOARD_SPI_DEVICE                     1  // SPI1
#define ONBOARD_SD_CS_PIN                   PA4   // Chip select for "System" SD card
#define SDSS                           SD_SS_PIN
