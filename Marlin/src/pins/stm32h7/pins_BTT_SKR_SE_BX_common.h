/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#define BOARD_WEBSITE_URL    "github.com/bigtreetech/BIQU-BX"
#define DEFAULT_MACHINE_NAME "BIQU BX"

#define USES_DIAG_JUMPERS

// Onboard I2C EEPROM
#define I2C_EEPROM
#define MARLIN_EEPROM_SIZE                0x1000  // 4K (24C32)

#define HAS_OTG_USB_HOST_SUPPORT                  // USB Flash Drive support
//#define SWD_DEBUG                               // Use pins PA13 and PA14 on STM32H7 for the SWD debugger

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PB11  // X-
#define Y_DIAG_PIN                          PB12  // Y-
#define Z_DIAG_PIN                          PD12  // Z-
#define E0_DIAG_PIN                         PD13  // X+
#define E1_DIAG_PIN                         PB13  // Y+

//
// Limit Switches
//
#define X_MIN_PIN                     X_DIAG_PIN  // X-
#define X_MAX_PIN                    E0_DIAG_PIN  // X+
#define Y_MIN_PIN                     Y_DIAG_PIN  // Y-
#define Y_MAX_PIN                    E1_DIAG_PIN  // Y+
#define Z_MIN_PIN                     Z_DIAG_PIN  // Z-
#define Z_MAX_PIN                           PD11  // Z+

#define FIL_RUNOUT_PIN               E0_DIAG_PIN  // X+
#define FIL_RUNOUT2_PIN              E1_DIAG_PIN  // Y+

#ifndef SWD_DEBUG
  #define LED_PIN                           PA13
  #define BEEPER_PIN                        PA14
#endif

#define POWER_MONITOR_PIN                   PB0
#define RPI_POWER_PIN                       PE5

#define SERVO0_PIN                          PA2   // PROBE

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PH2   // PROBE
#endif

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Steppers
//
#define X_STEP_PIN                          PG13  // X
#define X_DIR_PIN                           PG12
#define X_ENABLE_PIN                        PG14
#define X_CS_PIN                            PG10

#define Y_STEP_PIN                          PB3   // Y
#define Y_DIR_PIN                           PD3
#define Y_ENABLE_PIN                        PB4
#define Y_CS_PIN                            PD4

#ifdef BX_SWAP_ZM_E1M
  #define Z_STEP_PIN                        PA8   // E1
  #define Z_DIR_PIN                         PC9
  #define Z_ENABLE_PIN                      PD2
  #define Z_CS_PIN                          PC8
#else
  #define Z_STEP_PIN                        PD7   // Z1
  #define Z_DIR_PIN                         PD6
  #define Z_ENABLE_PIN                      PG9
  #define Z_CS_PIN                          PD5
#endif

#define E0_STEP_PIN                         PC14  // E0
#define E0_DIR_PIN                          PC13
#define E0_ENABLE_PIN                       PC15
#define E0_CS_PIN                           PI8

#ifdef BX_SWAP_ZM_E1M
  #define E1_STEP_PIN                       PD7   // Z1
  #define E1_DIR_PIN                        PD6
  #define E1_ENABLE_PIN                     PG9
  #define E1_CS_PIN                         PD5
#else
  #define E1_STEP_PIN                       PA8   // E1
  #define E1_DIR_PIN                        PC9
  #define E1_ENABLE_PIN                     PD2
  #define E1_CS_PIN                         PC8
#endif

//
// SPI pins for TMC2130 stepper drivers
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PC6
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PG3
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PC7
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial
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
  //#define E5_HARDWARE_SERIAL Serial1
  //#define E6_HARDWARE_SERIAL Serial1
  //#define E7_HARDWARE_SERIAL Serial1

  #define X_SERIAL_TX_PIN                   PG10
  #define Y_SERIAL_TX_PIN                   PD4
  #ifdef BX_SWAP_ZM_E1M
    #define Z_SERIAL_TX_PIN                 PC8
  #else
    #define Z_SERIAL_TX_PIN                 PD5
  #endif

  #define E0_SERIAL_TX_PIN                  PI8
  #ifdef BX_SWAP_ZM_E1M
    #define E1_SERIAL_TX_PIN                PD5
  #else
    #define E1_SERIAL_TX_PIN                PC8
  #endif

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PH4   // TH0
#define TEMP_1_PIN                          PA3   // TH1
#define TEMP_BED_PIN                        PH5   // TB

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC4   // HE0
#define HEATER_1_PIN                        PC5   // HE1
#define HEATER_BED_PIN                      PA4   // BED

#define FAN0_PIN                            PA5   // FAN0
#define FAN1_PIN                            PA6   // FAN1
#define FAN2_PIN                            PA7   // FAN2 / DCOT

#define BOARD_NEOPIXEL_PIN                  PH3   // RGB
#define NEOPIXEL2_PIN                       PB1

#if HAS_LTDC_TFT

  // LTDC_LCD Timing
  #define LTDC_LCD_CLK                        50  // LTDC clock frequency = 50Mhz
  #define LTDC_LCD_HSYNC                      30  // Horizontal synchronization
  #define LTDC_LCD_HBP                       114  // Horizontal back porch
  #define LTDC_LCD_HFP                        16  // Horizontal front porch
  #define LTDC_LCD_VSYNC                       3  // Vertical synchronization
  #define LTDC_LCD_VBP                        32  // Vertical back porch
  #define LTDC_LCD_VFP                        10  // Vertical front porch

  #define TFT_BACKLIGHT_PIN                 PB5
  #define LCD_DE_PIN                        PF10
  #define LCD_CLK_PIN                       PG7
  #define LCD_VSYNC_PIN                     PI9
  #define LCD_HSYNC_PIN                     PI10
  #define LCD_R7_PIN                        PG6   // R5
  #define LCD_R6_PIN                        PH12
  #define LCD_R5_PIN                        PH11
  #define LCD_R4_PIN                        PH10
  #define LCD_R3_PIN                        PH9
  #define LCD_G7_PIN                        PI2   // G6
  #define LCD_G6_PIN                        PI1
  #define LCD_G5_PIN                        PI0
  #define LCD_G4_PIN                        PH15
  #define LCD_G3_PIN                        PH14
  #define LCD_G2_PIN                        PH13
  #define LCD_B7_PIN                        PI7   // B5
  #define LCD_B6_PIN                        PI6
  #define LCD_B5_PIN                        PI5
  #define LCD_B4_PIN                        PI4
  #define LCD_B3_PIN                        PG11

  // GT911 Capacitive Touch Sensor
  #if ENABLED(TFT_TOUCH_DEVICE_GT911)
    #define GT911_RST_PIN                   PE4
    #define GT911_INT_PIN                   PE3
    #define GT911_SW_I2C_SCL_PIN            PE2
    #define GT911_SW_I2C_SDA_PIN            PE6
  #endif

#endif

#if IS_NEWPANEL
  #define BTN_EN1                           PH6
  #define BTN_EN2                           PH7
  #define BTN_ENC                           PH8
#endif

//
// SD card
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#define SOFTWARE_SPI
#define SDSS                                PA15
#define SD_SS_PIN                           SDSS
#define SD_SCK_PIN                          PC10
#define SD_MISO_PIN                         PC11
#define SD_MOSI_PIN                         PC12
#define SD_DETECT_PIN                       PI3

#if ENABLED(WIFISUPPORT)
  //
  // WIFI
  //

  /**
   *                 -----
   * (ESP-RX4)  PA1 | 1 8 | GND
   *  (ESP-EN)   -- | 2 7 | --  (ESP-IO2)
   * (ESP-RST)   -- | 3 6 | --  (ESP-IO0)
   *           3.3V | 4 5 | PA0 (ESP-TX4)
   *                 -----
   *               WIFI-ESP01
   */
  #define ESP_WIFI_MODULE_COM                  4  // Must also set either SERIAL_PORT / SERIAL_PORT_2 / SERIAL_PORT_3 to this
  #define ESP_WIFI_MODULE_BAUDRATE      BAUDRATE  // Must use same BAUDRATE as SERIAL_PORT / SERIAL_PORT_2 / SERIAL_PORT_3
  #define ESP_WIFI_MODULE_RESET_PIN         -1
  #define ESP_WIFI_MODULE_ENABLE_PIN        -1
  #define ESP_WIFI_MODULE_GPIO0_PIN         -1
  #define ESP_WIFI_MODULE_GPIO2_PIN         -1
#endif
