/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * BOARD_TRONXY_CXY_446_V10
 *
 * CXY-V6-191121 / CXY-446-V10-220413
 */

#pragma once

#include "env_validate.h"

#if EXTRUDERS > 2 || E_STEPPERS > 2 || NUM_RUNOUT_SENSORS > 2
  #error "TRONXY CXY 446 V10 only supports 2 Extruders / E steppers / Filament Runout sensors."
#endif

#define BOARD_INFO_NAME                       "BOARD_TRONXY_CXY_446_V10"
#define DEFAULT_MACHINE_NAME                  "TRONXY CXY 446 V10"

#define STEP_TIMER                            6
#define TEMP_TIMER                            14

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  #define I2C_EEPROM
  //#define FLASH_EEPROM_EMULATION
  #undef NO_EEPROM_SELECTED
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  #define EEPROM_START_ADDRESS                (0x8000000UL + (512 * 1024) - 2 * EEPROM_PAGE_SIZE)
  #define EEPROM_PAGE_SIZE                    (0x800U)  // 2K, but will use 2x more (4K)
  #define MARLIN_EEPROM_SIZE                  EEPROM_PAGE_SIZE
#else
  #define MARLIN_EEPROM_SIZE                  0x800  // 2K (FT24C16A)
#endif

//
// SPI Flash
//
#define SPI_FLASH                                 // W25Q16
#if ENABLED(SPI_FLASH)
  #define SPI_FLASH_SIZE                      0x1000000  // 16MB
  #define SPI_FLASH_CS_PIN                  PG15
  #define SPI_FLASH_MOSI_PIN                PB5
  #define SPI_FLASH_MISO_PIN                PB4
  #define SPI_FLASH_SCK_PIN                 PB3
#endif

//
// SD Card / Flash Drive
//
#define HAS_OTG_USB_HOST_SUPPORT  // USB Flash Drive Support

//
// SD Card
//
#define ONBOARD_SDIO
#define SD_DETECT_PIN                       -1
#define SDIO_CLOCK                            4500000
#define SDIO_READ_RETRIES                     16

#define SDIO_D0_PIN                         PC8
#define SDIO_D1_PIN                         PC9
#define SDIO_D2_PIN                         PC10
#define SDIO_D3_PIN                         PC11
#define SDIO_CK_PIN                         PC12
#define SDIO_CMD_PIN                        PD2

//
// Limit Switches
//
#define X_STOP_PIN                          PC15
#define Y_STOP_PIN                          PC14

#if ENABLED(FIX_MOUNTED_PROBE)
  #define Z_STOP_PIN                        PE3
#else
  #define Z_STOP_PIN                        PC13
#endif

//
// Filament Sensors
//
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define FIL_RUNOUT_PIN                    PE6
  #define FIL_RUNOUT2_PIN                   PF12
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PF0
#define X_STEP_PIN                          PE5
#define X_DIR_PIN                           PF1

#define Y_ENABLE_PIN                        PF5
#define Y_STEP_PIN                          PF9
#define Y_DIR_PIN                           PF3

#define Z_ENABLE_PIN                        PA5
#define Z_STEP_PIN                          PA6
#define Z_DIR_PIN                           PF15

#define E0_ENABLE_PIN                       PF14
#define E0_STEP_PIN                         PB1
#define E0_DIR_PIN                          PF13

#define E1_ENABLE_PIN                       PG5
#define E1_STEP_PIN                         PD12
#define E1_DIR_PIN                          PG4

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC3
#define TEMP_1_PIN                          PC0
#define TEMP_BED_PIN                        PC2

//
// Heaters
//
#define HEATER_0_PIN                        PG7   // Hotend #1 Heater
#define HEATER_1_PIN                        PA15  // Hotend #2 Heater
#define HEATER_BED_PIN                      PE2

//
// Fans
//
#define FAN_SOFT_PWM_REQUIRED

#define FAN0_PIN                            PG0   // Part Cooling Fan #1
#define FAN1_PIN                            PB6   // Part Cooling Fan #2
#define FAN2_PIN                            PG9   // Extruder/Hotend #1 Heatsink Fan
#define FAN3_PIN                            PF10  // Extruder/Hotend #2 Heatsink Fan
#define CONTROLLER_FAN_PIN                  PD7

//
// Laser / Servos
//
#define SPINDLE_LASER_ENA_PIN               PB11  // WiFi Module TXD (Pin5)
#define SPINDLE_LASER_PWM_PIN               PB10  // WiFi Module RXD (Pin4)

//
// NOTE: The PWM pin definition const PinMap PinMap_PWM[] in PeripheralPins.c must be augmented here.
// See PWM_PIN(x) definition for details.
//

//
// TFT with FSMC interface
//
#if HAS_FSMC_TFT
  #define LCD_USE_DMA_FSMC
  //#define FSMC_DMA_DEV                    DMA2 // Maple
  //#define FSMC_DMA_CHANNEL             DMA_CH5 // Maple
  #define FSMC_CS_PIN                       PG12
  #define FSMC_RS_PIN                       PG2
  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define TFT_RESET_PIN                     PB12
  #define TFT_BACKLIGHT_PIN                 PG8

  #define TOUCH_CS_PIN                      PD11
  #define TOUCH_SCK_PIN                     PB13
  #define TOUCH_MISO_PIN                    PB14
  #define TOUCH_MOSI_PIN                    PB15

  #if ENABLED(TFT_LVGL_UI)
    #define HAS_SPI_FLASH_FONT                    1
    #define HAS_GCODE_PREVIEW                     1
    #define HAS_GCODE_DEFAULT_VIEW_IN_FLASH       0
    #define HAS_LANG_SELECT_SCREEN                1
    #define HAS_BAK_VIEW_IN_FLASH                 0
    #define HAS_LOGO_IN_FLASH                     0
  #elif ANY(TFT_CLASSIC_UI, TFT_COLOR_UI)
    //#define TFT_DRIVER                 ILI9488
    #define TFT_BUFFER_WORDS               14400
  #endif

  // Touch Screen calibration
  #if ENABLED(TFT_TRONXY_X5SA)
    #ifndef TOUCH_CALIBRATION_X
      #define TOUCH_CALIBRATION_X         -17181
    #endif
    #ifndef TOUCH_CALIBRATION_Y
      #define TOUCH_CALIBRATION_Y          11434
    #endif
    #ifndef TOUCH_OFFSET_X
      #define TOUCH_OFFSET_X                 501
    #endif
    #ifndef TOUCH_OFFSET_Y
      #define TOUCH_OFFSET_Y                  -9
    #endif
    #ifndef TOUCH_ORIENTATION
      #define TOUCH_ORIENTATION  TOUCH_LANDSCAPE
    #endif
  #endif

  #if ENABLED(MKS_ROBIN_TFT43)
    #ifndef TOUCH_CALIBRATION_X
      #define TOUCH_CALIBRATION_X          17184
    #endif
    #ifndef TOUCH_CALIBRATION_Y
      #define TOUCH_CALIBRATION_Y          10604
    #endif
    #ifndef TOUCH_OFFSET_X
      #define TOUCH_OFFSET_X                 -31
    #endif
    #ifndef TOUCH_OFFSET_Y
      #define TOUCH_OFFSET_Y                 -29
    #endif
    #ifndef TOUCH_ORIENTATION
      #define TOUCH_ORIENTATION  TOUCH_LANDSCAPE
    #endif
  #endif
#else
  #error "TRONXY CXY 446 V10 only supports TFT with FSMC interface."
#endif

//
// Power Loss
//
#if ENABLED(PSU_CONTROL)
  #define PS_ON_PIN                         PG10
  #define POWER_LOSS_PIN                    PE1
#endif

//
// Misc. Functions
//
//#define LED_PIN                           PG10
#define BEEPER_PIN                          PA8
