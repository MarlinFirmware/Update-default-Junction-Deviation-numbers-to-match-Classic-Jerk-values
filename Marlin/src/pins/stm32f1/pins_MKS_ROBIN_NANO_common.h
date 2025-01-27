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

/**
 * MKS Robin Nano board common pin assignments
 */

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS Robin nano boards support up to 2 hotends / E steppers."
#endif

#ifndef USB_MOD
  #define BOARD_NO_NATIVE_USB
#endif

// Avoid conflict with TIMER_SERVO when using the STM32 HAL
#define TEMP_TIMER  5

//
// EEPROM
//
#if ENABLED(SRAM_EEPROM_EMULATION)
  #undef NO_EEPROM_SELECTED
#endif
#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

#define SPI_DEVICE                             2  // Maple

//
// Servos
//
#define SERVO0_PIN                          PA8   // Enable BLTOUCH

//
// Limit Switches
//
#define X_STOP_PIN                          PA15
#define Z_MAX_PIN                           PC4
#ifndef USB_MOD
  #define Y_STOP_PIN                        PA12
  #define Z_MIN_PIN                         PA11
#else
  #define Y_STOP_PIN                        PB10
  #define Z_MIN_PIN                         PB11
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
#define X_ENABLE_PIN                        PE4
#define X_STEP_PIN                          PE3
#define X_DIR_PIN                           PE2

#define Y_ENABLE_PIN                        PE1
#define Y_STEP_PIN                          PE0
#define Y_DIR_PIN                           PB9

#define Z_ENABLE_PIN                        PB8
#define Z_STEP_PIN                          PB5
#define Z_DIR_PIN                           PB4

#define E0_ENABLE_PIN                       PB3
#define E0_STEP_PIN                         PD6
#define E0_DIR_PIN                          PD3

#define E1_ENABLE_PIN                       PA3
#define E1_STEP_PIN                         PA6
#define E1_DIR_PIN                          PA1

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // TH1
#define TEMP_1_PIN                          PC2   // TH2
#define TEMP_BED_PIN                        PC0   // TB1

//
// Heaters / Fans
//
#ifndef HEATER_0_PIN
  #define HEATER_0_PIN                      PC3
#endif
#ifndef FAN0_PIN
  #define FAN0_PIN                          PB1   // FAN
#endif
#ifndef HEATER_BED_PIN
  #define HEATER_BED_PIN                    PA0
#endif

#if HOTENDS == 1 && DISABLED(HEATERS_PARALLEL)
  #ifndef FAN1_PIN
    #define FAN1_PIN                        PB0
  #endif
#elif !defined(HEATER_1_PIN)
  #define HEATER_1_PIN                      PB0
#endif

//
// Power Supply Control
//
#if ENABLED(MKS_PWC)
  #if ENABLED(TFT_LVGL_UI)
    #if ENABLED(PSU_CONTROL)
      #error "PSU_CONTROL is incompatible with MKS_PWC plus TFT_LVGL_UI."
    #endif
    #undef MKS_PWC
    #define SUICIDE_PIN                     PB2
    #define SUICIDE_PIN_STATE               LOW
  #else
    #define PS_ON_PIN                       PB2   // PW_OFF
  #endif
  #define KILL_PIN                          PA2
  #define KILL_PIN_STATE                    HIGH
#endif

//
// Misc. Functions
//
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

  #define WIFI_IO0_PIN                      PC13
  #define WIFI_IO1_PIN                      PC7
  #define WIFI_RESET_PIN                    PA5
#else
  //#define POWER_LOSS_PIN                  PA2   // PW_DET
  //#define PS_ON_PIN                       PB2   // PW_OFF
  #define FIL_RUNOUT_PIN                    PA4
  #define FIL_RUNOUT2_PIN                   PE6
#endif

//
// SD Card
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#define ONBOARD_SDIO
#define SDIO_CLOCK                       4500000  // 4.5 MHz
#define SD_DETECT_PIN                       PD12
#define ONBOARD_SD_CS_PIN                   PC11

//
// LCD / Controller
//
#define BEEPER_PIN                          PC5

//
// TFT with FSMC interface
//
#if HAS_FSMC_TFT
  /**
   * Note: MKS Robin TFT screens use various TFT controllers.
   * If the screen stays white, disable 'TFT_RESET_PIN' to let the bootloader init the screen.
   */
  #define LCD_USE_DMA_FSMC
  #define FSMC_CS_PIN                       PD7
  #define FSMC_RS_PIN                       PD11
  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define TFT_RESET_PIN                     PC6   // FSMC_RST
  #define TFT_BACKLIGHT_PIN                 PD13

  #define TOUCH_CS_PIN                      PA7   // SPI2_NSS
  #define TOUCH_SCK_PIN                     PB13  // SPI2_SCK
  #define TOUCH_MISO_PIN                    PB14  // SPI2_MISO
  #define TOUCH_MOSI_PIN                    PB15  // SPI2_MOSI

  #define TOUCH_BUTTONS_HW_SPI
  #define TOUCH_BUTTONS_HW_SPI_DEVICE          2

  #define TFT_BUFFER_WORDS                 14400
#endif

#define SPI_FLASH
#if ENABLED(SPI_FLASH)
  #define SPI_FLASH_SIZE               0x1000000  // 16MB
  #define SPI_FLASH_CS_PIN                  PB12
  #define SPI_FLASH_SCK_PIN                 PB13
  #define SPI_FLASH_MISO_PIN                PB14
  #define SPI_FLASH_MOSI_PIN                PB15
#endif
