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
 * BigTreeTech SKR 1.1 pin assignments
 * Schematic: https://github.com/bigtreetech/BIGTREETECH-SKR-V1.1/blob/master/hardware/SKR-V1.1SchDoc.pdf
 */

#define BOARD_INFO_NAME "BTT SKR V1.1"

//
// Limit Switches
//
#define X_MIN_PIN                          P1_29
#define X_MAX_PIN                          P1_28
#define Y_MIN_PIN                          P1_27
#define Y_MAX_PIN                          P1_26
#define Z_MIN_PIN                          P1_25
#define Z_MAX_PIN                          P1_24

//
// Steppers
//
#define X_STEP_PIN                         P0_04
#define X_DIR_PIN                          P0_05
#define X_ENABLE_PIN                       P4_28

#define Y_STEP_PIN                         P2_01
#define Y_DIR_PIN                          P2_02
#define Y_ENABLE_PIN                       P2_00

#define Z_STEP_PIN                         P0_20
#define Z_DIR_PIN                          P0_21
#define Z_ENABLE_PIN                       P0_19

#define E0_STEP_PIN                        P0_11
#define E0_DIR_PIN                         P2_13
#define E0_ENABLE_PIN                      P2_12

/**        ------                  ------
 *   1.30 | 1  2 | 2.11      0.17 | 1  2 | 0.15
 *   0.18 | 3  4 | 0.16      3.26 | 3  4 | 1.23
 *   0.15   5  6 | --        3.25   5  6 | 0.18
 *     -- | 7  8 | --        1.31 | 7  8 | RESET
 *    GND | 9 10 | 5V         GND | 9 10 | --
 *         ------                  ------
 *          EXP1                    EXP2
 */
#define EXP1_01_PIN                        P1_30
#define EXP1_02_PIN                        P2_11
#define EXP1_03_PIN                        P0_18
#define EXP1_04_PIN                        P0_16
#define EXP1_05_PIN                        P0_15
#define EXP1_06_PIN                        -1     // NC
#define EXP1_07_PIN                        -1     // NC
#define EXP1_08_PIN                        -1     // NC

#define EXP2_01_PIN                        P0_17
#define EXP2_02_PIN                        P0_15
#define EXP2_03_PIN                        P3_26
#define EXP2_04_PIN                        P1_23
#define EXP2_05_PIN                        P3_25
#define EXP2_06_PIN                        P0_18
#define EXP2_07_PIN                        P1_31
#define EXP2_08_PIN                        -1     // RESET

/**
 * LCD / Controller
 *
 * As of 20 JAN 2019 only the REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER display has
 * been tested with these settings. It can be connected to the SKR using standard cables
 * via the EXP1 and EXP2 ports. Other displays may need a custom cable and/or changes to
 * the pins defined below.
 *
 * The SD card on the LCD controller uses the same SPI signals as the LCD, resulting in
 * garbage/lines on the LCD display during SD card access. The LCD code mitigates this
 * by redrawing the screen after SD card accesses.
 */

#if IS_TFTGLCD_PANEL

  #if ENABLED(TFTGLCD_PANEL_SPI)
    #define TFTGLCD_CS               EXP2_03_PIN
  #endif

#elif HAS_WIRED_LCD

  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #define LCD_SDSS_PIN               EXP2_04_PIN
  #define LCD_PINS_RS                EXP1_04_PIN
  #define LCD_PINS_EN                EXP2_06_PIN
  #define LCD_PINS_D4                EXP2_02_PIN

  #if ENABLED(MKS_MINI_12864)
    #define DOGLCD_CS                      P2_06
    #define DOGLCD_A0                EXP1_04_PIN
  #endif

#endif // HAS_WIRED_LCD

//
// SD Support
//
// MKS_MINI_12864 strongly prefers the SD card on the display and
// requires jumpers on the SKR V1.1 board as documented here:
// https://www.facebook.com/groups/505736576548648/permalink/630639874058317/
#ifndef SDCARD_CONNECTION
  #if ANY(MKS_MINI_12864, ENDER2_STOCKDISPLAY, IS_TFTGLCD_PANEL)
    #define SDCARD_CONNECTION                LCD
  #else
    #define SDCARD_CONNECTION            ONBOARD
  #endif
#endif

// Trinamic driver support

#if HAS_TRINAMIC_CONFIG
  // Using TMC devices in intelligent mode requires extra connections to each device. Unfortunately
  // the SKR does not have many free pins (especially if a display is in use). The SPI-based devices
  // will require 3 connections (clock, mosi, miso), plus a chip select line (CS) for each driver.
  // The UART-based devices require 2 pis per driver (one of which must be interrupt capable).
  // The same SPI pins can be shared with the display/SD card reader, meaning SPI-based devices are
  // probably a good choice for this board.
  //
  // SOFTWARE_DRIVER_ENABLE is a good option. It uses SPI to control the driver enable and allows the
  // hardware ENABLE pins for each driver to be repurposed as SPI chip select. To use this mode the
  // driver modules will probably need to be modified, removing the pin used for the enable line from
  // the module and wiring this connection directly to GND (as is the case for TMC2130).
  // Using this option and sharing all of the SPI pins allows 5 TMC2130 drivers to be used along with
  // a REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER without requiring the use of any extra pins.
  //
  // Other options will probably require the use of any free pins and the TFT serial port or a
  // different type of display (like the TFT), using the pins normally used for the display and encoder.
  // Unfortunately, tests show it's not possible to use endstop and thermistor pins for chip-select.
  // Sample settings are provided below, but only some have been tested.
  //
  // Another option is to share the enable and chip-select pins when using SPI. Several users have
  // reported that this works. However, it's unlikely that this configuration will allow SPI communi-
  // cation with the device when the drivers are active, meaning that some of the more advanced TMC
  // options may not be available.

  // When using any TMC SPI-based drivers, software SPI is used
  // because pins may be shared with the display or SD card.
  #define TMC_USE_SW_SPI
  #define TMC_SPI_MOSI               EXP2_06_PIN
  #define TMC_SPI_MISO               EXP2_01_PIN
  // To minimize pin usage use the same clock pin as the display/SD card reader. (May generate LCD noise.)
  #define TMC_SPI_SCK                EXP2_02_PIN
  // If pin 2_06 is unused, it can be used for the clock to avoid the LCD noise.
  //#define TMC_SPI_SCK                    P2_06

  #if ENABLED(SOFTWARE_DRIVER_ENABLE)

    // Software enable allows the enable pins to be repurposed as chip-select pins.
    // Note: Requires the driver modules to be modified to always be enabled with the enable pin removed.
    #if AXIS_DRIVER_TYPE_X(TMC2130)
      #define X_CS_PIN                     P4_28
      #undef X_ENABLE_PIN
    #endif

    #if AXIS_DRIVER_TYPE_Y(TMC2130)
      #define Y_CS_PIN                     P2_00
      #undef Y_ENABLE_PIN
    #endif

    #if AXIS_DRIVER_TYPE_Z(TMC2130)
      #define Z_CS_PIN                     P0_19
      #undef Z_ENABLE_PIN
    #endif

    #if AXIS_DRIVER_TYPE_E0(TMC2130)
      #define E0_CS_PIN                    P2_12
      #undef E0_ENABLE_PIN
    #endif

    #if AXIS_DRIVER_TYPE_E1(TMC2130)
      #define E1_CS_PIN                    P0_10
      #undef E1_ENABLE_PIN
    #endif

  #else // !SOFTWARE_DRIVER_ENABLE

    // A chip-select pin is needed for each driver.

    // EXAMPLES

    // Example 1: No LCD attached or a TFT style display using the AUX header RX/TX pins.
    //            SDCARD_CONNECTION must not be 'LCD'. Nothing should be connected to EXP1/EXP2.
    //#define SKR_USE_LCD_PINS_FOR_CS
    #if ENABLED(SKR_USE_LCD_PINS_FOR_CS)
      #define X_CS_PIN               EXP2_04_PIN
      #define Y_CS_PIN               EXP2_03_PIN
      #define Z_CS_PIN               EXP1_02_PIN
      #define E0_CS_PIN              EXP2_05_PIN
      #define E1_CS_PIN              EXP2_07_PIN
    #endif

    // Example 2: A REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
    //            The SD card reader attached to the LCD (if present) can't be used because
    //            the pins will be in use. So SDCARD_CONNECTION must not be 'LCD'.
    //#define SKR_USE_LCD_SD_CARD_PINS_FOR_CS
    #if ENABLED(SKR_USE_LCD_SD_CARD_PINS_FOR_CS)
      #define X_CS_PIN                     P0_02
      #define Y_CS_PIN                     P0_03
      #define Z_CS_PIN                     P2_06
      // We use SD_DETECT_PIN for E0
      #undef SD_DETECT_PIN
      #define E0_CS_PIN              EXP2_07_PIN
      // We use LCD_SDSS_PIN pin for E1
      #undef LCD_SDSS_PIN
      #define LCD_SDSS_PIN                 -1
      #define E1_CS_PIN              EXP2_04_PIN
    #endif

    // Example 3: Use the driver enable pins for chip-select.
    //            Commands must not be sent to the drivers when enabled. So certain
    //            advanced features (like driver monitoring) will not be available.
    //#define SKR_USE_ENABLE_CS
    #if ENABLED(SKR_USE_ENABLE_FOR_CS)
      #define X_CS_PIN              X_ENABLE_PIN
      #define Y_CS_PIN              Y_ENABLE_PIN
      #define Z_CS_PIN              Z_ENABLE_PIN
      #define E0_CS_PIN            E0_ENABLE_PIN
      #define E1_CS_PIN            E1_ENABLE_PIN
    #endif

  #endif // SOFTWARE_DRIVER_ENABLE

#endif

// Include common SKR pins
#include "pins_BTT_SKR_common.h"
