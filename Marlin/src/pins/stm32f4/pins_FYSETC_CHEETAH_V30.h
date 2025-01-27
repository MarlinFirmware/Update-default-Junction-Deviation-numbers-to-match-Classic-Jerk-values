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

#define BOARD_INFO_NAME   "FYSETC Cheetah V3.0"
#define BOARD_WEBSITE_URL "github.com/FYSETC/Cheetah_V3.0"

// USB Flash Drive support
//#define HAS_OTG_USB_HOST_SUPPORT

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define FLASH_EEPROM_LEVELING

#endif

//
// Z Probe
//
#if ENABLED(BLTOUCH)
  #error "You need to set jumper to 5V for BLTouch, then comment out this line to proceed."
  #define SERVO0_PIN                        PA0   // PROBE
#elif !defined(Z_MIN_PROBE_PIN)
  #define Z_MIN_PROBE_PIN                   PA0
#endif

//
// Limit Switches
//
#define X_STOP_PIN                          PA2
#define Y_STOP_PIN                          PA3
#define Z_STOP_PIN                          PC4

//
// Filament runout
//
#define FIL_RUNOUT_PIN                      PA1   // FIL-D

//
// Steppers
//
#define X_STEP_PIN                          PC0
#define X_DIR_PIN                           PC1
#define X_ENABLE_PIN                        PC3

#define Y_STEP_PIN                          PC14
#define Y_DIR_PIN                           PC13
#define Y_ENABLE_PIN                        PC15

#define Z_STEP_PIN                          PB4
#define Z_DIR_PIN                           PB5
#define Z_ENABLE_PIN                        PC2

#define E0_STEP_PIN                         PB2
#define E0_DIR_PIN                          PA15
#define E0_ENABLE_PIN                       PD2

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial2
  //#define Y_HARDWARE_SERIAL  Serial2
  //#define Z_HARDWARE_SERIAL  Serial2
  //#define E0_HARDWARE_SERIAL Serial2

  #ifndef X_SERIAL_TX_PIN
    #define X_SERIAL_TX_PIN                 PB3
  #endif

  #ifndef Y_SERIAL_TX_PIN
    #define Y_SERIAL_TX_PIN                 PB3
  #endif

  #ifndef Z_SERIAL_TX_PIN
    #define Z_SERIAL_TX_PIN                 PB3
  #endif
  #ifndef E0_SERIAL_TX_PIN
    #define E0_SERIAL_TX_PIN                PB3
  #endif

  // Default TMC slave addresses
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS                    0
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS                    2
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS                    1
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                   3
  #endif
  static_assert(X_SLAVE_ADDRESS == 0, "X_SLAVE_ADDRESS must be 0 for BOARD_FYSETC_CHEETAH_V30.");
  static_assert(Y_SLAVE_ADDRESS == 2, "Y_SLAVE_ADDRESS must be 2 for BOARD_FYSETC_CHEETAH_V30.");
  static_assert(Z_SLAVE_ADDRESS == 1, "Z_SLAVE_ADDRESS must be 1 for BOARD_FYSETC_CHEETAH_V30.");
  static_assert(E0_SLAVE_ADDRESS == 3, "E0_SLAVE_ADDRESS must be 3 for BOARD_FYSETC_CHEETAH_V30.");
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC7   // PWM
#define HEATER_BED_PIN                      PC8   // PWM

#ifndef FAN0_PIN
  #define FAN0_PIN                          PA14
#endif
#define FAN1_PIN                            PA13
#define FAN2_PIN                            PA8

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC5   // Analog Input T0 ADC
#define TEMP_BED_PIN                        PB1   // Analog Input    ADC
#define TEMP_CHAMBER_PIN                    PB0   // Analog Input T1 ADC

//
// Misc. Functions
//
#if 0
  #define SD_SS_PIN                         PA4
  #define SD_DETECT_PIN                     PC3
#endif

#ifndef RGB_LED_R_PIN
  #define RGB_LED_R_PIN                     PB0
#endif
#ifndef RGB_LED_G_PIN
  #define RGB_LED_G_PIN                     PB7
#endif
#ifndef RGB_LED_B_PIN
  #define RGB_LED_B_PIN                     PB6
#endif

/**
 *                    -----                                                      -----
 *  (KILL?)       5V | 1 2 | GND                                             5V | 1 2 | GND
 *  (RST)      RESET | 3 4 | PC12  ((SD_DET?)CD)             (LCD_D7 BLUE)  PB7 | 3 4 | PB6  (LCD_D6 GREEN)
 *  ((SD?)MOSI   PA7 | 5 6 | PC11 (BTN_EN2)               (LCD_D5 RED DIN) PB14 | 5 6 | PB13 (LCD_D4 LCD_RST)
 *  ((SD?)SS)    PA4 | 7 8 | PC10 (BTN_EN1)                (LCD_RS LCD_A0) PB12 | 7 8 | PB15 (LCD_EN LCD CS)
 *  ((SD?)SCK)   PA5 | 9 10| PA6  ((SD?)MISO)                    (BTN_ENC) PC6  | 9 10| PB10  (BEEPER)
 *                    -----                                                      -----
 *                    EXP2                                                       EXP1
 */

/**
 *                 -----
 *  (BEEPER) PC9  | 1 2 | PC12 (BTN_ENC)
 * (BTN_EN1) PC10 | 3 4 | PB14 (LCD_D5/MISO)
 * (BTN_EN2) PC11   5 6 | PB13 (LCD_D4/SCK)
 *  (LCD_RS) PB12 | 7 8 | PB15 (LCD_EN/MOSI)
 *            GND | 9 10| 5V
 *                 -----
 *                 EXP3
 */

#define EXP1_03_PIN                         PB7
#define EXP1_04_PIN                         PB6
#define EXP1_05_PIN                         PB14
#define EXP1_06_PIN                         PB13
#define EXP1_07_PIN                         PB12
#define EXP1_08_PIN                         PB15
#define EXP1_09_PIN                         PC6
#define EXP1_10_PIN                         PB10

#define EXP2_03_PIN                         -1    // RESET
#define EXP2_04_PIN                         PC12
#define EXP2_05_PIN                         PA7
#define EXP2_06_PIN                         PC11
#define EXP2_07_PIN                         PA4
#define EXP2_08_PIN                         PC10
#define EXP2_09_PIN                         PA5
#define EXP2_10_PIN                         PA6

//
// SPI / SD Card
//
#define SD_SCK_PIN                   EXP2_09_PIN
#define SD_MISO_PIN                  EXP2_10_PIN
#define SD_MOSI_PIN                  EXP2_05_PIN

#define SD_SS_PIN                    EXP2_07_PIN
#define SD_DETECT_PIN                EXP2_04_PIN

#if HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_10_PIN
  #define BTN_ENC                    EXP1_09_PIN

  #if ENABLED(CR10_STOCKDISPLAY)

    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP2_08_PIN
    #define BTN_EN2                  EXP2_06_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #elif ENABLED(MKS_MINI_12864)

    #define DOGLCD_A0                EXP1_04_PIN
    #define DOGLCD_CS                EXP1_05_PIN
    #define BTN_EN1                  EXP2_08_PIN
    #define BTN_EN2                  EXP2_06_PIN

  #else

    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP2_06_PIN
    #define BTN_EN2                  EXP2_08_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

    #if ENABLED(FYSETC_MINI_12864)
      #define DOGLCD_CS              EXP1_08_PIN
      #define DOGLCD_A0              EXP1_07_PIN
      //#define LCD_BACKLIGHT_PIN           -1
      #define LCD_RESET_PIN          EXP1_06_PIN  // Must be high or open for LCD to operate normally.
      #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
        #ifndef RGB_LED_R_PIN
          #define RGB_LED_R_PIN      EXP1_05_PIN
        #endif
        #ifndef RGB_LED_G_PIN
          #define RGB_LED_G_PIN      EXP1_04_PIN
        #endif
        #ifndef RGB_LED_B_PIN
          #define RGB_LED_B_PIN      EXP1_03_PIN
        #endif
      #elif ENABLED(FYSETC_MINI_12864_2_1)
        #define NEOPIXEL_PIN         EXP1_05_PIN
      #endif
    #endif // !FYSETC_MINI_12864

    #if IS_ULTIPANEL
      #define LCD_PINS_D5            EXP1_05_PIN
      #define LCD_PINS_D6            EXP1_04_PIN
      #define LCD_PINS_D7            EXP1_03_PIN

      #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
        #define BTN_ENC_EN           LCD_PINS_D7  // Detect the presence of the encoder
      #endif

    #endif

  #endif

#endif // HAS_WIRED_LCD

// Alter timing for graphical display
#if ENABLED(U8GLIB_ST7920)
  #define BOARD_ST7920_DELAY_1                96
  #define BOARD_ST7920_DELAY_2                48
  #define BOARD_ST7920_DELAY_3               600
#endif

#if ENABLED(TOUCH_UI_FTDI_EVE)
  #define BEEPER_PIN                 EXP1_10_PIN
  #define CLCD_MOD_RESET             EXP2_08_PIN
  #define CLCD_SPI_CS                EXP2_06_PIN
#endif

#define NEOPIXEL2_PIN                       PC9
