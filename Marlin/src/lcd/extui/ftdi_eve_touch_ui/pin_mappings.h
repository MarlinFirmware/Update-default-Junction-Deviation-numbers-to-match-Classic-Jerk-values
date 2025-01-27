/******************
 * pin_mappings.h *
 ******************/

/****************************************************************************
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#pragma once

/* This file defines mappings from the ULTRA_LCD pins functions to new
 * functions for the FTDI display. These mappings allows any board that
 * support ULTRA_LCD via EXP1 and EXP2 connectors to use FTDI modules
 * without adding new pin definitions to the board.
 */

#include "../../../inc/MarlinConfig.h"

#if ENABLED(F6_TFT_PINMAP)              // FYSETC F6 - ATmega2560

  #define CLCD_SPI_CS                    33
  #define CLCD_MOD_RESET                 31

#elif ENABLED(S6_TFT_PINMAP)            // FYSETC S6 - STM32F4

  #define CLCD_SPI_CS                    PC7
  #define CLCD_MOD_RESET                 PC6

#elif ENABLED(CR10_TFT_PINMAP)          // FYSETC S6 - STM32F4 - with TOUCH_UI_ULTIPANEL

  #define CLCD_USE_SOFT_SPI
  #define CLCD_SOFT_SPI_SCLK             LCD_PINS_D4  // PORTA1               Pin 6
  #define CLCD_SOFT_SPI_MOSI             LCD_PINS_EN  // PORTC1               Pin 8
  #define CLCD_SPI_CS                    LCD_PINS_RS  // PORTA3               Pin 7
  #define CLCD_SOFT_SPI_MISO             16           // PORTC0   BTN_ENC     Pin 2
  #define CLCD_MOD_RESET                 11           // PORTD3   BTN_EN1     Pin 3
  #define CLCD_AUX_0                     10           // PORTD2   BTN_EN2     Pin 5
  #define CLCD_AUX_1                     BEEPER_PIN   // PORTA4               Pin 1

#elif ENABLED(AO_EXP1_DEPRECATED_PINMAP)

  /**
   * This LulzBot pinout re-purposes the UltraLCD
   * connector EXP1 for Software SPI (rev B, obsolete)
   */

  #define CLCD_MOD_RESET                 LCD_PINS_D4
  #define CLCD_SPI_CS                    LCD_PINS_D5

  #define CLCD_AUX_0                     LCD_PINS_EN
  #define CLCD_AUX_1                     BTN_ENC
  #define CLCD_AUX_2                     BEEPER_PIN

  #define CLCD_USE_SOFT_SPI
  #define CLCD_SOFT_SPI_SCLK             LCD_PINS_D7
  #define CLCD_SOFT_SPI_MOSI             LCD_PINS_D6
  #define CLCD_SOFT_SPI_MISO             LCD_PINS_RS

#elif ENABLED(AO_EXP1_PINMAP)

  /**
   * AO_EXP1_PINMAP with TOUCH_UI_ULTIPANEL
   *
   * This LulzBot mapping re-purposes the UltraLCD
   * connector EXP1 for Software SPI for display (rev C):
   *
   *     EXP2:      FTDI:   SD -or- USB [1]:     ULTRA_LCD:
   *      1         MISO    MISO    MISO    -->  BEEPER
   *      2         SCLK    SCLK    SCLK    -->  BTN_ENC
   *      3         PD_N      -      -      -->  LCDE
   *      4          -      CS_N    CS_N    -->  LCDRS
   *      5         CS_N      -      -      -->  LCD4
   *      6         MOSI    MOSI    MOSI    -->  LCD5
   *      7          -      SD_DET  INT     -->  LCD6
   *      8         RESET     -     RESET   -->  LCD4
   *      9         GND     GND     GND     -->  GND
   *     10         5V      5V      5V      -->  5V
   *
   *     [1] At the moment, Marlin does not support SD or USB
   *         functionality over software SPI.
   */

  #define CLCD_MOD_RESET                 LCD_PINS_EN
  #define CLCD_SPI_CS                    LCD_PINS_D4

  #define CLCD_USE_SOFT_SPI
  #define CLCD_SOFT_SPI_SCLK             BTN_ENC
  #define CLCD_SOFT_SPI_MOSI             LCD_PINS_D5
  #define CLCD_SOFT_SPI_MISO             BEEPER_PIN

#elif ENABLED(AO_EXP2_PINMAP)

  /**
   * AO_EXP2_PINMAP with TOUCH_UI_ULTIPANEL
   *
   * The LulzBot mapping for re-purposing the UltraLCD
   * connector EXP2 for hardware SPI for display and SD card
   * or USB (rev C):
   *
   *     EXP2:      FTDI:   SD -or- USB:         ULTRA_LCD:
   *      1         MISO    MISO    MISO    -->  MISO
   *      2         SCLK    SCLK    SCLK    -->  SCLK
   *      3         PD_N      -      -      -->  BTN_EN2
   *      4          -      CS_N    CS_N    -->  SD_CSEL
   *      5         CS_N      -      -      -->  BTN_EN1
   *      6         MOSI    MOSI    MOSI    -->  MOSI
   *      7          -      SD_DET  INT     -->  SD_DET
   *      8         RESET     -     RESET   -->  RESET
   *      9         GND     GND     GND     -->  GND
   *     10         5V      5V      5V      -->  KILL [3]
   *
   * [1] This configuration allows daisy-chaining of the
   *     display and SD/FD on EXP2, except for [2]
   *
   * [2] The Ultimachine Einsy boards have a level shifter
   *     on MISO enabled by SD_CSEL chip select, hence it
   *     is not possible to run both the display and the
   *     SD/FD on EXP2.
   *
   * [3] Archim Rambo provides 5V on this pin. On any other
   *     board, divert this wire from the ribbon cable and
   *     connect it to 5V at an endstop.
   */

  #define CLCD_SPI_CS                    BTN_EN1
  #define CLCD_MOD_RESET                 BTN_EN2
  #if MB(EINSY_RAMBO, EINSY_RETRO) && !HAS_MEDIA
    #define CLCD_SPI_EXTRA_CS            SD_SS_PIN
  #endif

#endif
