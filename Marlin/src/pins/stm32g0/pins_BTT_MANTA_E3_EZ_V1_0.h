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
  #define BOARD_INFO_NAME "BTT Manta E3 EZ V1.0"
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
#define SERVO0_PIN                          PA7   // BLTouch

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Trinamic StallGuard pins
//
#define X_DIAG_PIN                          PC4   // X-STOP
#define Y_DIAG_PIN                          PB0   // Y-STOP
#define Z_DIAG_PIN                          PC6   // Z-STOP
#define E0_DIAG_PIN                         PC5   // E0-DET
#define E1_DIAG_PIN                         PB1   // E1-DET

//
// Limit Switches
//
#define X_STOP_PIN                    X_DIAG_PIN  // X-STOP
#define Y_STOP_PIN                    Y_DIAG_PIN  // Y-STOP
#define Z_STOP_PIN                    Z_DIAG_PIN  // Z-STOP

//
// Z Probe (when not Z_STOP_PIN)
//
#define Z_MIN_PROBE_PIN                     PA6   // BLTouch

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN             E0_DIAG_PIN  // E0-DET
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN            E1_DIAG_PIN  // E1-DET
#endif

//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PA9   // PS-ON
#endif

//
// Power Loss Detection
//
#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PB9   // PWRDET
#endif

//
// Steppers
//
#define X_STEP_PIN                          PA14
#define X_DIR_PIN                           PA10
#define X_ENABLE_PIN                        PA13
#ifndef X_CS_PIN
  #define X_CS_PIN                          PB8
#endif

#define Y_STEP_PIN                          PC8
#define Y_DIR_PIN                           PA15
#define Y_ENABLE_PIN                        PC14
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PC9
#endif

#define Z_STEP_PIN                          PD2
#define Z_DIR_PIN                           PD4
#define Z_ENABLE_PIN                        PD3
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PD0
#endif

#define E0_STEP_PIN                         PD5
#define E0_DIR_PIN                          PD6
#define E0_ENABLE_PIN                       PB3
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD1
#endif

#define E1_STEP_PIN                         PB7
#define E1_DIR_PIN                          PB6
#define E1_ENABLE_PIN                       PB4
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PB5
#endif

//
// Default pins for TMC software SPI
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PC12  // Shared with SPI header, Pin 5 (SPI3)
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PC11  // Shared with SPI header, Pin 6 (SPI3)
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PC10  // Shared with SPI header, Pin 4 (SPI3)
#endif

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   PB8   // X_CS_PIN
  #define Y_SERIAL_TX_PIN                   PC9   // Y_CS_PIN
  #define Z_SERIAL_TX_PIN                   PD0   // Z_CS_PIN
  #define E0_SERIAL_TX_PIN                  PD1   // E0_CS_PIN
  #define E1_SERIAL_TX_PIN                  PB5   // E1_CS_PIN

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA4   // Analog Input "TH0"
#define TEMP_1_PIN                          PA5   // Analog Input "TH1"
#define TEMP_BED_PIN                        PA3   // Analog Input "TB"

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB11  // "HE0"
#define HEATER_1_PIN                        PB10  // "HE1"
#define HEATER_BED_PIN                      PB2   // "HB"

#define FAN0_PIN                            PA8   // "FAN0"
#define FAN1_PIN                            PB15  // "FAN1"
#define FAN2_PIN                            PB14  // "FAN2"

//
// Auto fans
//
#if HOTENDS == 2
  #ifndef E0_AUTO_FAN_PIN
    #define E0_AUTO_FAN_PIN             FAN1_PIN
  #endif
  #ifndef E1_AUTO_FAN_PIN
    #define E1_AUTO_FAN_PIN             FAN2_PIN
  #endif
#else
  #ifndef E0_AUTO_FAN_PIN
    #define E0_AUTO_FAN_PIN             FAN1_PIN
  #endif
  #ifndef CONTROLLER_FAN_PIN
    #define CONTROLLER_FAN_PIN          FAN2_PIN
  #endif
#endif

/**
 *           Manta E3 EZ V1.0
 *                ------
 * (BEEPER)  PC1 | 1  2 | PC2  (BTN_ENC)
 * (BTN_EN1) PC3 | 3  4 | RESET
 * (BTN_EN2) PC0   5  6 | PA0  (LCD_D4)
 * (LCD_RS)  PA2 | 7  8 | PA1  (LCD_EN)
 *           GND | 9 10 | 5V
 *                ------
 *                 EXP1
 */
#define EXP1_01_PIN                         PC1
#define EXP1_02_PIN                         PC2
#define EXP1_03_PIN                         PC3
#define EXP1_04_PIN                         -1
#define EXP1_05_PIN                         PC0
#define EXP1_06_PIN                         PA0
#define EXP1_07_PIN                         PA2
#define EXP1_08_PIN                         PA1
#define EXP1_09_PIN                         -1
#define EXP1_10_PIN                         -1

#if HAS_DWIN_E3V2 || IS_DWIN_MARLINUI
  /**
   *              ------                   ------            ---
   *  (PC1) BEEP | 1  2 |                 | 1  2 |          | 1 |    (5V)
   *             | 3  4 |              RX | 3  4 | TX       | 2 |    (GND)
   *  (PC0)  ENT   5  6 |             ENT   5  6 | BEEP     | 3 | RX (PD8)
   *  (PA2)    B | 7  8 | A  (PA1)      B | 7  8 | A        | 4 | TX (PD9)
   *         GND | 9 10 | 5V          GND | 9 10 | VCC      | 5 |    (RST)
   *              ------                   ------            ---
   *               EXP1                     DWIN             TFT
   *
   * DWIN pins are labeled as printed on DWIN PCB. GND, VCC, A, B, ENT & BEEP can be connected in the same orientation as the
   * existing plug/DWIN to EXP1. DWIN TX/RX need to be connected to the Manta E3 EZ's TFT port, with DWIN TX->PD9, DWIN RX->PD8.
   *
   * Needs custom cable:
   *
   *     Board        Adapter       Display
   * ------------------------------------------
   *  (EXP1-1) PC1 <-----------> BEEP (DWIN-6)
   *  (EXP1-5) PC0 <-----------> ENT  (DWIN-5)
   *   (TFT-4) PD9 <-----------> RX   (DWIN-3)
   *   (TFT-3) PD8 <-----------> TX   (DWIN-4)
   *  (EXP1-7) PA2 <-----------> B    (DWIN-7)
   *  (EXP1-9) GND <-----------> GND  (DWIN-9)
   *  (EXP1-8) PA1 <-----------> A    (DWIN-8)
   * (EXP1-10)  5V <-----------> VCC  (DWIN-10)
   */
  CONTROLLER_WARNING("BTT_MANTA_E3_EZ_V1_0", "Ender-3 V2 display", " Requires a custom cable with TX = PA0, RX = PC2.")

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_EN1                    EXP1_08_PIN
  #define BTN_EN2                    EXP1_07_PIN
  #define BTN_ENC                    EXP1_05_PIN

#elif HAS_WIRED_LCD

  #if ENABLED(CR10_STOCKDISPLAY)

    #define BEEPER_PIN               EXP1_01_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN
    #define BTN_ENC                  EXP1_02_PIN

    #define LCD_PINS_RS              EXP1_07_PIN
    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #elif ENABLED(ZONESTAR_LCD)                     // ANET A8 LCD Controller - Must convert to 3.3V - CONNECTING TO 5V WILL DAMAGE THE BOARD!

    CONTROLLER_WARNING("BTT_MANTA_E3_EZ_V1_0", "ZONESTAR_LCD")

    #define LCD_PINS_RS              EXP1_06_PIN
    #define LCD_PINS_EN              EXP1_02_PIN
    #define LCD_PINS_D4              EXP1_07_PIN
    #define LCD_PINS_D5              EXP1_05_PIN
    #define LCD_PINS_D6              EXP1_03_PIN
    #define LCD_PINS_D7              EXP1_01_PIN
    #define ADC_KEYPAD_PIN                  PA7   // Repurpose default SERVO0_PIN for ADC - CONNECTING TO 5V WILL DAMAGE THE BOARD!

  #elif ANY(MKS_MINI_12864, ENDER2_STOCKDISPLAY)

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN
    #define BTN_ENC                  EXP1_02_PIN

    #define DOGLCD_CS                EXP1_07_PIN
    #define DOGLCD_A0                EXP1_06_PIN
    #define DOGLCD_SCK               EXP1_01_PIN
    #define DOGLCD_MOSI              EXP1_08_PIN
    #define FORCE_SOFT_SPI
    #define LCD_BACKLIGHT_PIN               -1

  #else

    #error "Only ZONESTAR_LCD, MKS_MINI_12864, ENDER2_STOCKDISPLAY, and CR10_STOCKDISPLAY are currently supported on the BTT_MANTA_E3_EZ_V1_0."

  #endif

#endif // HAS_WIRED_LCD

//
// SD Support
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#define SD_DETECT_PIN                       -1

#if SD_CONNECTION_IS(LCD) && (ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050) || IS_TFTGLCD_PANEL)
  #define SD_SS_PIN                  EXP1_05_PIN
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "SD CUSTOM_CABLE is not compatible with Manta E3 EZ."
#endif

#define ONBOARD_SPI_DEVICE                     3  // SPI3 -> used only by HAL/STM32F1...
#define ONBOARD_SD_CS_PIN                   PC13  // Chip select for "System" SD card

#define ENABLE_SPI3
#define SDSS                   ONBOARD_SD_CS_PIN
#define SD_SS_PIN              ONBOARD_SD_CS_PIN
#define SD_SCK_PIN                          PC10
#define SD_MISO_PIN                         PC11
#define SD_MOSI_PIN                         PC12

//
// NeoPixel LED
//
#ifndef BOARD_NEOPIXEL_PIN
  #define BOARD_NEOPIXEL_PIN                PC7   // RGB
#endif
