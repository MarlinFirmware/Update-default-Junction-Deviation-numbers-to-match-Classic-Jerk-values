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
 * MKS Robin (STM32F103ZET6) board pin assignments
 * https://github.com/makerbase-mks/MKS-Robin/tree/master/MKS%20Robin/Hardware
 */

#if NOT_TARGET(STM32F1, STM32F1xx)
  #error "Oops! Select an STM32F1 board in 'Tools > Board.'"
#elif HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS Robin supports up to 2 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "MKS Robin"

#define BOARD_NO_NATIVE_USB

//
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role
//
#define DISABLE_JTAG

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  #ifdef ARDUINO_ARCH_STM32
    #define FLASH_EEPROM_EMULATION
  #else
    #define SDCARD_EEPROM_EMULATION
  #endif
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

//
// Servos
//
#define SERVO0_PIN                          PC3   // XS1 - 5
#define SERVO1_PIN                          PA1   // XS1 - 6
#define SERVO2_PIN                          PF9   // XS2 - 5
#define SERVO3_PIN                          PF8   // XS2 - 6

//
// Limit Switches
//
#define X_MIN_PIN                           PB12
#define X_MAX_PIN                           PB0
#define Y_MIN_PIN                           PC5
#define Y_MAX_PIN                           PC4
#define Z_MIN_PIN                           PA4
#define Z_MAX_PIN                           PF7

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PB9
#define X_STEP_PIN                          PB8
#define X_DIR_PIN                           PB5

#define Y_ENABLE_PIN                        PB4
#define Y_STEP_PIN                          PG15
#define Y_DIR_PIN                           PG10

#define Z_ENABLE_PIN                        PD7
#define Z_STEP_PIN                          PD3
#define Z_DIR_PIN                           PG14

#define E0_ENABLE_PIN                       PG13
#define E0_STEP_PIN                         PG8
#define E0_DIR_PIN                          PA15

#define E1_ENABLE_PIN                       PA12
#define E1_STEP_PIN                         PA11
#define E1_DIR_PIN                          PA8

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // TH1
#define TEMP_1_PIN                          PC2   // TH2
#define TEMP_BED_PIN                        PC0   // TB1

//
// Heaters
//
#define HEATER_0_PIN                        PC7   // HEATER1
#define HEATER_1_PIN                        PA6   // HEATER2
#define HEATER_BED_PIN                      PC6   // HOT BED

//
// Fan
//
#define FAN0_PIN                            PA7   // FAN

//
// Thermocouples
//
//#define TEMP_0_CS_PIN                     PE5   // TC1 - CS1
//#define TEMP_0_CS_PIN                     PE6   // TC2 - CS2

//
// Filament Runout Sensor
//
#define FIL_RUNOUT_PIN                      PF11  // MT_DET

//
// Power loss detection
//
#define POWER_LOSS_PIN                      PA2   // PW_DET

//
// Power supply control
//
#define PS_ON_PIN                           PA3   // PW_OFF

//
// Piezzoelectric speaker
//
#define BEEPER_PIN                          PC13

//
// Activity LED
//
#define LED_PIN                             PB2

//
// ESP12-S Wi-Fi module
//
#define WIFI_IO0_PIN                        PG1

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
  #define LCD_USE_DMA_FSMC
  #define FSMC_CS_PIN                       PG12  // NE4
  #define FSMC_RS_PIN                       PF0   // A0
  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define TFT_RESET_PIN                     PF6
  #define TFT_BACKLIGHT_PIN                 PG11

  #define TOUCH_BUTTONS_HW_SPI
  #define TOUCH_BUTTONS_HW_SPI_DEVICE          2
  #define TFT_BUFFER_WORDS                 14400
#endif

#if NEED_TOUCH_PINS
  #define TOUCH_CS_PIN                      PB1   // SPI2_NSS
  #define TOUCH_SCK_PIN                     PB13  // SPI2_SCK
  #define TOUCH_MISO_PIN                    PB14  // SPI2_MISO
  #define TOUCH_MOSI_PIN                    PB15  // SPI2_MOSI
  #define TOUCH_INT_PIN                     -1
#endif

// SPI2 is shared by LCD touch driver and flash
// SPI1(PA7) & SPI3(PB5) not available
#define SPI_DEVICE                             2  // Maple

#define ONBOARD_SDIO
#define SDIO_CLOCK                       4500000
#define SDIO_READ_RETRIES                     16
#if ENABLED(ONBOARD_SDIO)
  #define SD_SCK_PIN                        PB13  // SPI2
  #define SD_MISO_PIN                       PB14  // SPI2
  #define SD_MOSI_PIN                       PB15  // SPI2
  /**
   * MKS Robin has a few hardware revisions
   * https://github.com/makerbase-mks/MKS-Robin/tree/master/MKS%20Robin/Hardware
   *
   * MKS Robin <= V2.3 have no SD_DETECT_PIN.
   * MKS Robin >= V2.4 have SD_DETECT_PIN on PF12.
   *
   * Uncomment here or add SD_DETECT_PIN to Configuration.h.
   */
  //#define SD_DETECT_PIN                   -1
  //#define SD_DETECT_PIN                   PF12  // SD_CD
#else
  // SD as custom software SPI (SDIO pins)
  #define SD_SCK_PIN                        PC12
  #define SD_MISO_PIN                       PC8
  #define SD_MOSI_PIN                       PD2
  #define ONBOARD_SD_CS_PIN                 PC11
  #define SD_SS_PIN                         PD2
  #define SD_DETECT_PIN                     -1
#endif

//
// Trinamic TMC2208/TMC2209 UART
//
#if HAS_TMC_UART
  /**
   * This board does not have dedicated TMC UART pins. Custom wiring is needed.
   * You may uncomment one of the options below, or add it to your Configuration.h.
   *
   * When using up to four TMC2209 drivers, hardware serial is recommended on
   * MSerial0 or MSerial1.
   *
   * When using TMC2208 or more than four drivers, SoftwareSerial will be needed,
   * to provide dedicated pins for each drier.
   */

  //#define TMC_HARDWARE_SERIAL
  #if ENABLED(TMC_HARDWARE_SERIAL)
    #define X_HARDWARE_SERIAL  MSerial0
    #define X2_HARDWARE_SERIAL MSerial0
    #define Y_HARDWARE_SERIAL  MSerial0
    #define Y2_HARDWARE_SERIAL MSerial0
    #define Z_HARDWARE_SERIAL  MSerial0
    #define Z2_HARDWARE_SERIAL MSerial0
    #define E0_HARDWARE_SERIAL MSerial0
    #define E1_HARDWARE_SERIAL MSerial0
  #endif

  //#define TMC_SOFTWARE_SERIAL
  #if ENABLED(TMC_SOFTWARE_SERIAL)
    #define X_SERIAL_TX_PIN                 PF8   // SERVO3_PIN -- XS2 - 6
    #define Y_SERIAL_TX_PIN                 PF9   // SERVO2_PIN -- XS2 - 5
    #define Z_SERIAL_TX_PIN                 PA1   // SERVO1_PIN -- XS1 - 6
    #define E0_SERIAL_TX_PIN                PC3   // SERVO0_PIN -- XS1 - 5
    #define TMC_BAUD_RATE                  19200
  #endif
#endif

//
// W25Q64 64Mb (8MB) SPI flash
//
#define SPI_FLASH
#if ENABLED(SPI_FLASH)
  #define SPI_FLASH_SIZE                0x800000  // 8MB
  #define SPI_FLASH_CS_PIN                  PG9
  #define SPI_FLASH_SCK_PIN                 PB13
  #define SPI_FLASH_MISO_PIN                PB14
  #define SPI_FLASH_MOSI_PIN                PB15
#endif
