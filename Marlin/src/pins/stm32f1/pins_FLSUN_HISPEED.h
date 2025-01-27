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
 * FLSUN HiSpeed V1 (STM32F103VET6) board pin assignments
 * FLSun Hispeed (clone MKS_Robin_miniV2) board.
 *
 * MKS Robin Mini USB uses UART3 (PB10-TX, PB11-RX)
 * #define SERIAL_PORT_2 3
 */

#if NOT_TARGET(__STM32F1__, STM32F1xx)
  #error "Oops! Select an STM32F1 board in 'Tools > Board.'"
#elif HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "FLSUN HiSpeedV1 only supports 1 hotend / E stepper."
#endif

#define BOARD_INFO_NAME      "FLSun HiSpeedV1"
#define BOARD_WEBSITE_URL    "github.com/Foxies-CSTL"

#define BOARD_NO_NATIVE_USB

// Avoid conflict with TIMER_SERVO when using the STM32 HAL
#define TEMP_TIMER                             5

//
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role
//
#define DISABLE_DEBUG

//
// EEPROM
//
#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

//
// SPI
//
#define SPI_DEVICE                             2  // Maple

//
// SD Card SPI
//
#define SD_SCK_PIN                          PB13  // SPI2
#define SD_MISO_PIN                         PB14  // SPI2
#define SD_MOSI_PIN                         PB15  // SPI2

//
// SPI Flash
//
#define SPI_FLASH
#if ENABLED(SPI_FLASH)
  #define SPI_FLASH_SIZE               0x1000000  // 16MB
  #define SPI_FLASH_CS_PIN                  PB12  // SPI2_NSS / Flash chip-select
  #define SPI_FLASH_SCK_PIN                 PB13
  #define SPI_FLASH_MISO_PIN                PB14
  #define SPI_FLASH_MOSI_PIN                PB15
#endif

//
// Servos
//
//#define SERVO0_PIN                        PA8   // use IO0 to enable BLTOUCH support/remove Mks_Wifi

//
// Limit Switches
//
#define X_STOP_PIN                          PA15  // -X
#define Y_STOP_PIN                          PA12  // -Y
#define Z_MIN_PIN                           PA11  // -Z
#define Z_MAX_PIN                           PC4   // +Z

//
// Steppers
//
#define X_ENABLE_PIN                        PE4   // X_EN
#define X_STEP_PIN                          PE3   // X_STEP
#define X_DIR_PIN                           PE2   // X_DIR

#define Y_ENABLE_PIN                        PE1   // Y_EN
#define Y_STEP_PIN                          PE0   // Y_STEP
#define Y_DIR_PIN                           PB9   // Y_DIR

#define Z_ENABLE_PIN                        PB8   // Z_EN
#define Z_STEP_PIN                          PB5   // Z_STEP
#define Z_DIR_PIN                           PB4   // Z_DIR

#define E0_ENABLE_PIN                       PB3   // E0_EN
#define E0_STEP_PIN                         PD6   // E0_STEP
#define E0_DIR_PIN                          PD3   // E0_DIR

/**
 * FLSUN Hi-Speed has no hard-wired UART pins for TMC drivers.
 * Several wiring options are provided below, defaulting to
 * to the most compatible.
 */
#if HAS_TMC_UART
  // SoftwareSerial with one pin per driver
  // Compatible with TMC2208/TMC2209 drivers
  #define X_SERIAL_TX_PIN                   PA10  // RXD1
  #define Y_SERIAL_TX_PIN                   PA9   // TXD1
  #define Z_SERIAL_TX_PIN                   PC7   // IO1
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif
#else
  // Motor current PWM pins
  #define MOTOR_CURRENT_PWM_XY_PIN          PA6   // VREF2/3 CONTROL XY
  #define MOTOR_CURRENT_PWM_Z_PIN           PA7   // VREF4 CONTROL Z
  #define MOTOR_CURRENT_PWM_RANGE           1500  // (255 * (1000mA / 65535)) * 257 = 1000 is equal 1.6v Vref in turn equal 1Amp
  #ifndef DEFAULT_PWM_MOTOR_CURRENT
    #define DEFAULT_PWM_MOTOR_CURRENT { 800, 800, 800 }
  #endif

  #if ENABLED(WIFISUPPORT)
    /**
     * MKS Robin_Wifi or another ESP8266 module
     *
     *      __ESP(M1)__       -J1-
     *  GND| 15 | | 08 |+3v3  (22)  RXD1      (PA10)
     *     | 16 | | 07 |MOSI  (21)  TXD1      (PA9)   Active LOW, probably OK to leave floating
     *  IO2| 17 | | 06 |MISO  (19)  IO1       (PC7)   Leave as unused (ESP3D software configures this with a pullup so OK to leave as floating)
     *  IO0| 18 | | 05 |CLK   (18)  IO0       (PA8)   Must be HIGH (ESP3D software configures this with a pullup so OK to leave as floating)
     *  IO1| 19 | | 03 |EN    (03)  WIFI_EN           Must be HIGH for module to run
     *     | nc | | nc |      (01)  WIFI_CTRL (PA5)
     *   RX| 21 | | nc |
     *   TX| 22 | | 01 |RST
     *       ￣￣ AE￣￣
     */
    // Module ESP-WIFI
    #define ESP_WIFI_MODULE_COM                2  // Must also set either SERIAL_PORT or SERIAL_PORT_2 to this
    #define ESP_WIFI_MODULE_BAUDRATE    BAUDRATE  // Must use same BAUDRATE as SERIAL_PORT & SERIAL_PORT_2
    #define ESP_WIFI_MODULE_RESET_PIN       PA5   // WIFI CTRL/RST
    #define ESP_WIFI_MODULE_ENABLE_PIN      -1
    #define ESP_WIFI_MODULE_TXD_PIN         PA9   // MKS or ESP WIFI RX PIN
    #define ESP_WIFI_MODULE_RXD_PIN         PA10  // MKS or ESP WIFI TX PIN
  #endif
#endif

//
// EXTRUDER
//
#if AXIS_HAS_UART(E0)
  #define E0_SERIAL_TX_PIN                  PA8   // IO0
#else
  // Motor current PWM pins
  #define MOTOR_CURRENT_PWM_E_PIN           PB0   // VREF1 CONTROL E
  #define MOTOR_CURRENT_PWM_RANGE           1500  // (255 * (1000mA / 65535)) * 257 = 1000 is equal 1.6v Vref in turn equal 1Amp
  #ifndef DEFAULT_PWM_MOTOR_CURRENT
   #define DEFAULT_PWM_MOTOR_CURRENT { 800, 800, 800 }
  #endif
#endif

//
// Temperature Sensors (THM)
//
#define TEMP_0_PIN                          PC1   // TEMP_E0
#define TEMP_BED_PIN                        PC0   // TEMP_BED

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC3   // HEATER_E0
#define HEATER_BED_PIN                      PA0   // HEATER_BED-WKUP

#define FAN0_PIN                            PB1   // E_FAN

//
// Misc. Functions
//
#if ENABLED(BACKUP_POWER_SUPPLY)
  #define POWER_LOSS_PIN                    PA2   // PW_DET (UPS) MKSPWC
#else
  //#define POWER_LOSS_PIN                  PA1   // PW_SO
#endif

/**
 *    Connector J2
 *      -------
 * DIO O|1   2|O  3v3
 * CSK O|3   5|O  GND
 * RST O|5   6|O  GND
 *      -------
 */
//#define SW_DIO                            PA13
//#define SW_CLK                            PA14
//#define SW_RST                            NRST  // (14)

//
// Power Supply Control
//
#if ENABLED(PSU_CONTROL)
  #define KILL_PIN                          PA2   // PW_DET
  #define KILL_PIN_STATE                    HIGH
  //#define PS_ON_PIN                       PA3   // PW_CN /PW_OFF
#endif

#if HAS_TFT_LVGL_UI
  #ifndef FIL_RUNOUT_PIN
    #define FIL_RUNOUT_PIN                  PA4   // MT_DET_1
  #endif
  #ifndef FIL_RUNOUT2_PIN
    #define FIL_RUNOUT2_PIN                 PE6   // MT_DET_2
  #endif
  #ifndef FIL_RUNOUT_STATE
    #define FIL_RUNOUT_STATE                LOW
  #endif
#endif

//
// LED / NEOPixel
//
//#define LED_PIN                           PB2   // BOOT1

#if ENABLED(NEOPIXEL_LED)
  #define LED_PWM                           PC7   // IO1
  #ifndef BOARD_NEOPIXEL_PIN
    #define BOARD_NEOPIXEL_PIN           LED_PWM  // USED WIFI IO0/IO1 PIN
  #endif
#endif

//
// SD Card
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

// Use the on-board card socket labeled SD_Extender
#if SD_CONNECTION_IS(CUSTOM_CABLE)
  #define SD_SCK_PIN                        PC12
  #define SD_MISO_PIN                       PC8
  #define SD_MOSI_PIN                       PD2
  #define SD_DETECT_PIN                     PD12  // SD_CD (if -1 no detection)
#else
  #define ONBOARD_SDIO
  #define SDIO_CLOCK                     4500000  // 4.5 MHz
  #define SDIO_READ_RETRIES                   16
  #define ONBOARD_SPI_DEVICE                   1  // SPI1
  #define ONBOARD_SD_CS_PIN                 PC11
  #define SD_DETECT_PIN                     -1    // SD_CD (-1 active refresh)
#endif

//
// LCD / Controller
//

#ifndef BEEPER_PIN
  #define BEEPER_PIN                        PC5
#endif

#if ENABLED(SPEAKER) && BEEPER_PIN == PC5
  #error "FLSun HiSpeed default BEEPER_PIN is not a SPEAKER."
#endif

//
// TFT with FSMC interface
//
#if HAS_FSMC_TFT
  /**
   * Note: MKS Robin TFT screens use various TFT controllers
   * Supported screens are based on the ILI9341, ST7789V and ILI9328 (320x240)
   * ILI9488 is not supported
   * Define init sequences for other screens in u8g_dev_tft_320x240_upscale_from_128x64.cpp
   *
   * If the screen stays white, disable 'TFT_RESET_PIN' to let the bootloader init the screen.
   *
   * Setting a 'TFT_RESET_PIN' may cause a flicker when switching menus
   * because Marlin uses the reset as a failsafe to revive a glitchy LCD.
   */
  //#define TFT_RESET_PIN                   PC6   // FSMC_RST
  #define TFT_BACKLIGHT_PIN                 PD13

  #define LCD_USE_DMA_FSMC
  #define FSMC_CS_PIN                       PD7   // NE4
  #define FSMC_RS_PIN                       PD11  // A0
  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #if ENABLED(TFT_CLASSIC_UI)
    #define TFT_MARLINBG_COLOR            0x3186  // Grey
    #define TFT_MARLINUI_COLOR            0xC7B6  // Green
    #define TFT_BTARROWS_COLOR            0xDEE6  // Yellow
    #define TFT_BTOKMENU_COLOR            0x145F  // Cyan
  #endif
  #define TFT_BUFFER_WORDS                 14400

#elif HAS_GRAPHICAL_TFT

  #define TFT_RESET_PIN                     PC6
  #define TFT_BACKLIGHT_PIN                 PD13
  #define TFT_CS_PIN                        PD7   // NE4
  #define TFT_RS_PIN                        PD11  // A0

#endif

#if NEED_TOUCH_PINS
  #define TOUCH_CS_PIN                      PC2   // SPI2_NSS
  #define TOUCH_SCK_PIN                     PB13  // SPI2_SCK
  #define TOUCH_MISO_PIN                    PB14  // SPI2_MISO
  #define TOUCH_MOSI_PIN                    PB15  // SPI2_MOSI
  #define TOUCH_INT_PIN                     -1
#endif
