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
 * MKS Robin mini (STM32F103VET6) board pin assignments
 */

#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "MKS Robin mini only supports 1 hotend / E stepper."
#endif

#define BOARD_INFO_NAME "MKS Robin Mini"

#define BOARD_NO_NATIVE_USB

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

#define SPI_DEVICE                             2  // Maple

//
// Servos
//
#ifndef SERVO0_PIN
  #define SERVO0_PIN                        PA8   // Enable BLTOUCH support on IO0 (WIFI connector)
#endif

//
// Limit Switches
//
#define X_STOP_PIN                          PA15
#define Y_STOP_PIN                          PA12
#define Z_MIN_PIN                           PA11
#define Z_MAX_PIN                           PC4

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA4   // MT_DET
#endif
#ifndef FIL_RUNOUT_STATE
  #define FIL_RUNOUT_STATE                  LOW
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

// Motor current PWM pins
#define MOTOR_CURRENT_PWM_XY_PIN            PA6
#define MOTOR_CURRENT_PWM_Z_PIN             PA7
#define MOTOR_CURRENT_PWM_E_PIN             PB0
#define MOTOR_CURRENT_PWM_RANGE             1500  // (255 * (1000mA / 65535)) * 257 = 1000 is equal 1.6v Vref in turn equal 1Amp
#ifndef DEFAULT_PWM_MOTOR_CURRENT
  #define DEFAULT_PWM_MOTOR_CURRENT { 800, 800, 800 }
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // TH1
#define TEMP_BED_PIN                        PC0   // TB1

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC3
#define HEATER_BED_PIN                      PA0

#define FAN0_PIN                            PB1   // FAN

//
// Misc. Functions
//
#define POWER_LOSS_PIN                      PA2   // PW_DET
#define PS_ON_PIN                           PA3   // PW_OFF

#define WIFI_IO0_PIN                        PC13

//
// SD Card
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#define ONBOARD_SDIO
#define SDIO_CLOCK                       4500000  // 4.5 MHz
#define SD_DETECT_PIN                       PD12
#define ONBOARD_SPI_DEVICE                     1  // SPI1
#define ONBOARD_SD_CS_PIN                   PC11

//
// LCD / Controller
//
#define BEEPER_PIN                          PC5

/**
 * Note: MKS Robin TFT screens use various TFT controllers.
 * If the screen stays white, disable 'LCD_RESET_PIN' to let the bootloader init the screen.
 */
#if HAS_FSMC_TFT
  #define LCD_USE_DMA_FSMC
  #define FSMC_CS_PIN                       PD7   // NE4
  #define FSMC_RS_PIN                       PD11  // A0
  #define TFT_CS_PIN                 FSMC_CS_PIN  // NE4
  #define TFT_RS_PIN                 FSMC_RS_PIN  // A0

  #define LCD_RESET_PIN                     PC6   // FSMC_RST
  #define LCD_BACKLIGHT_PIN                 PD13

  #if NEED_TOUCH_PINS
    #define TOUCH_CS_PIN                    PC2   // SPI2_NSS
    #define TOUCH_SCK_PIN                   PB13  // SPI2_SCK
    #define TOUCH_MISO_PIN                  PB14  // SPI2_MISO
    #define TOUCH_MOSI_PIN                  PB15  // SPI2_MOSI
  #endif
#endif

#if ENABLED(TFT_320x240)                          // TFT32/28
  #define TFT_DRIVER                     ILI9341
  #define TFT_BUFFER_WORDS                 14400
  #define ILI9341_COLOR_RGB
  // YV for normal screen mounting
  #define ILI9341_ORIENTATION  ILI9341_MADCTL_MY | ILI9341_MADCTL_MV
  // XV for 180° rotated screen mounting
  //#define ILI9341_ORIENTATION  ILI9341_MADCTL_MX | ILI9341_MADCTL_MV
#endif

#if ENABLED(TOUCH_SCREEN)
  #ifndef TOUCH_CALIBRATION_X
    #define TOUCH_CALIBRATION_X            12033
  #endif
  #ifndef TOUCH_CALIBRATION_Y
    #define TOUCH_CALIBRATION_Y            -9047
  #endif
  #ifndef TOUCH_OFFSET_X
    #define TOUCH_OFFSET_X                   -30
  #endif
  #ifndef TOUCH_OFFSET_Y
    #define TOUCH_OFFSET_Y                   254
  #endif
#endif

#define SPI_FLASH
#if ENABLED(SPI_FLASH)
  #define SPI_FLASH_SIZE               0x1000000  // 16MB
  #define SPI_FLASH_CS_PIN                  PB12  // Flash chip-select
  #define SPI_FLASH_SCK_PIN                 PB13
  #define SPI_FLASH_MISO_PIN                PB14
  #define SPI_FLASH_MOSI_PIN                PB15
#endif
