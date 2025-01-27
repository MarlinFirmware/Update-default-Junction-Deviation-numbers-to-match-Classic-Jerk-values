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
 * Conditionals-5-post.h
 * Internal defines that depend on Configurations and Pins but are not user-editable.
 */

//========================================================
// Get requirements for the benefit of IntelliSense, etc.
//
#include "MarlinConfigPre-5-post.h"
//========================================================

#ifdef GITHUB_ACTIONS
  // Extras for CI testing
#endif

// Arduino IDE with Teensy Additions
#ifdef TEENSYDUINO
  #undef max
  #define max(a,b) ((a)>(b)?(a):(b))
  #undef min
  #define min(a,b) ((a)<(b)?(a):(b))
  #undef NOT_A_PIN    // Override Teensyduino legacy CapSense define work-around
  #define NOT_A_PIN 0 // For PINS_DEBUGGING
#endif

// ADC
#ifdef BOARD_ADC_VREF_MV
  #define ADC_VREF_MV BOARD_ADC_VREF_MV
#else
  #define ADC_VREF_MV HAL_ADC_VREF_MV
#endif

// Determine which type of 'EEPROM' is in use
#if ENABLED(EEPROM_SETTINGS)
  // EEPROM type may be defined by compile flags, configs, HALs, or pins
  // Set additional flags to let HALs choose in their Conditionals-5-post.h
  #if ANY(FLASH_EEPROM_EMULATION, SRAM_EEPROM_EMULATION, SDCARD_EEPROM_EMULATION, QSPI_EEPROM)
    #define USE_EMULATED_EEPROM 1
  #elif ANY(I2C_EEPROM, SPI_EEPROM)
    #define USE_WIRED_EEPROM    1
  #elif ENABLED(IIC_BL24CXX_EEPROM)
    // nothing
  #else
    #define USE_FALLBACK_EEPROM 1
  #endif
#else
  #undef I2C_EEPROM
  #undef SPI_EEPROM
  #undef QSPI_EEPROM
  #undef SDCARD_EEPROM_EMULATION
  #undef SRAM_EEPROM_EMULATION
  #undef FLASH_EEPROM_EMULATION
  #undef IIC_BL24CXX_EEPROM
#endif

#if DISABLED(IIC_BL24CXX_EEPROM)
  #undef OTA_FIRMWARE_UPDATE
#endif

/**
 * Axis lengths and center
 */
#if HAS_I_AXIS && !defined(AXIS4_NAME)
  #define AXIS4_NAME 'A'
#endif
#if HAS_J_AXIS && !defined(AXIS5_NAME)
  #define AXIS5_NAME 'B'
#endif
#if HAS_K_AXIS && !defined(AXIS6_NAME)
  #define AXIS6_NAME 'C'
#endif
#if HAS_U_AXIS && !defined(AXIS7_NAME)
  #define AXIS7_NAME 'U'
#endif
#if HAS_V_AXIS && !defined(AXIS8_NAME)
  #define AXIS8_NAME 'V'
#endif
#if HAS_W_AXIS && !defined(AXIS9_NAME)
  #define AXIS9_NAME 'W'
#endif

#if HAS_X_AXIS
  #define X_MAX_LENGTH (X_MAX_POS - (X_MIN_POS))
#endif
#if HAS_Y_AXIS
  #define Y_MAX_LENGTH (Y_MAX_POS - (Y_MIN_POS))
#endif
#if HAS_Z_AXIS
  #define Z_MAX_LENGTH (Z_MAX_POS - (Z_MIN_POS))
#else
  #undef CONTROLLER_FAN_USE_Z_ONLY
#endif
#if HAS_I_AXIS
  #define I_MAX_LENGTH (I_MAX_POS - (I_MIN_POS))
#endif
#if HAS_J_AXIS
  #define J_MAX_LENGTH (J_MAX_POS - (J_MIN_POS))
#endif
#if HAS_K_AXIS
  #define K_MAX_LENGTH (K_MAX_POS - (K_MIN_POS))
#endif
#if HAS_U_AXIS
  #define U_MAX_LENGTH (U_MAX_POS - (U_MIN_POS))
#endif
#if HAS_V_AXIS
  #define V_MAX_LENGTH (V_MAX_POS - (V_MIN_POS))
#endif
#if HAS_W_AXIS
  #define W_MAX_LENGTH (W_MAX_POS - (W_MIN_POS))
#endif

// Defined only if the sanity-check is bypassed
#if HAS_X_AXIS && !defined(X_BED_SIZE)
  #define X_BED_SIZE X_MAX_LENGTH
#endif
#if HAS_Y_AXIS && !defined(Y_BED_SIZE)
  #define Y_BED_SIZE Y_MAX_LENGTH
#endif
#if HAS_I_AXIS && !defined(I_BED_SIZE)
  #define I_BED_SIZE I_MAX_LENGTH
#endif
#if HAS_J_AXIS && !defined(J_BED_SIZE)
  #define J_BED_SIZE J_MAX_LENGTH
#endif
#if HAS_K_AXIS && !defined(K_BED_SIZE)
  #define K_BED_SIZE K_MAX_LENGTH
#endif
#if HAS_U_AXIS && !defined(U_BED_SIZE)
  #define U_BED_SIZE U_MAX_LENGTH
#endif
#if HAS_V_AXIS && !defined(V_BED_SIZE)
  #define V_BED_SIZE V_MAX_LENGTH
#endif
#if HAS_W_AXIS && !defined(W_BED_SIZE)
  #define W_BED_SIZE W_MAX_LENGTH
#endif

// Require 0,0 bed center for Delta, SCARA, and Polargraph
#if IS_KINEMATIC
  #define BED_CENTER_AT_0_0
#endif

// Define center values for future use
#if HAS_X_AXIS
  #define _X_HALF_BED ((X_BED_SIZE) / 2)
#endif
#if HAS_Y_AXIS
  #define _Y_HALF_BED ((Y_BED_SIZE) / 2)
#endif
#if HAS_I_AXIS
  #define _I_HALF_IMAX ((I_BED_SIZE) / 2)
#endif
#if HAS_J_AXIS
  #define _J_HALF_JMAX ((J_BED_SIZE) / 2)
#endif
#if HAS_K_AXIS
  #define _K_HALF_KMAX ((K_BED_SIZE) / 2)
#endif
#if HAS_U_AXIS
  #define _U_HALF_UMAX ((U_BED_SIZE) / 2)
#endif
#if HAS_V_AXIS
  #define _V_HALF_VMAX ((V_BED_SIZE) / 2)
#endif
#if HAS_W_AXIS
  #define _W_HALF_WMAX ((W_BED_SIZE) / 2)
#endif

#if HAS_X_AXIS
  #define X_CENTER TERN(BED_CENTER_AT_0_0, 0, _X_HALF_BED)
#endif
#if HAS_Y_AXIS
  #define Y_CENTER TERN(BED_CENTER_AT_0_0, 0, _Y_HALF_BED)
  #define XY_CENTER { X_CENTER, Y_CENTER }
#endif
#if HAS_I_AXIS
  #define I_CENTER _I_HALF_BED
#endif
#if HAS_J_AXIS
  #define J_CENTER _J_HALF_BED
#endif
#if HAS_K_AXIS
  #define K_CENTER _K_HALF_BED
#endif
#if HAS_U_AXIS
  #define U_CENTER _U_HALF_BED
#endif
#if HAS_V_AXIS
  #define V_CENTER _V_HALF_BED
#endif
#if HAS_W_AXIS
  #define W_CENTER _W_HALF_BED
#endif

// Get the linear boundaries of the bed
#if HAS_X_AXIS
  #define X_MIN_BED (X_CENTER - _X_HALF_BED)
  #define X_MAX_BED (X_MIN_BED + X_BED_SIZE)
#endif
#if HAS_Y_AXIS
  #define Y_MIN_BED (Y_CENTER - _Y_HALF_BED)
  #define Y_MAX_BED (Y_MIN_BED + Y_BED_SIZE)
#endif
#if HAS_I_AXIS
  #define I_MINIM (I_CENTER - _I_HALF_BED_SIZE)
  #define I_MAXIM (I_MINIM + I_BED_SIZE)
#endif
#if HAS_J_AXIS
  #define J_MINIM (J_CENTER - _J_HALF_BED_SIZE)
  #define J_MAXIM (J_MINIM + J_BED_SIZE)
#endif
#if HAS_K_AXIS
  #define K_MINIM (K_CENTER - _K_HALF_BED_SIZE)
  #define K_MAXIM (K_MINIM + K_BED_SIZE)
#endif
#if HAS_U_AXIS
  #define U_MINIM (U_CENTER - _U_HALF_BED_SIZE)
  #define U_MAXIM (U_MINIM + U_BED_SIZE)
#endif
#if HAS_V_AXIS
  #define V_MINIM (V_CENTER - _V_HALF_BED_SIZE)
  #define V_MAXIM (V_MINIM + V_BED_SIZE)
#endif
#if HAS_W_AXIS
  #define W_MINIM (W_CENTER - _W_HALF_BED_SIZE)
  #define W_MAXIM (W_MINIM + W_BED_SIZE)
#endif

/**
 * Dual X Carriage
 */
#if ENABLED(DUAL_X_CARRIAGE)
  #ifndef X1_MIN_POS
    #define X1_MIN_POS X_MIN_POS
  #endif
  #ifndef X1_MAX_POS
    #define X1_MAX_POS X_BED_SIZE
  #endif
#endif

// Calibration codes only for non-core axes
#if ANY(BACKLASH_GCODE, CALIBRATION_GCODE)
  #if ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX)
    #define CAN_CALIBRATE(A,B) TERN0(HAS_##A##_AXIS, (_AXIS(A) == B))
  #else
    #define CAN_CALIBRATE(A,B) ENABLED(HAS_##A##_AXIS)
  #endif
  #define AXIS_CAN_CALIBRATE(A) CAN_CALIBRATE(A,NORMAL_AXIS)
#else
  #define AXIS_CAN_CALIBRATE(A) false
#endif

/**
 * No adjustable bed on non-cartesians
 */
#if IS_KINEMATIC
  #undef LCD_BED_TRAMMING
  #undef SLOWDOWN
#endif

/**
 * SCARA cannot use SLOWDOWN and requires QUICKHOME
 * Printable radius assumes joints can fully extend
 */
#if IS_SCARA
  #if ENABLED(AXEL_TPARA)
    #define PRINTABLE_RADIUS (TPARA_LINKAGE_1 + TPARA_LINKAGE_2)
  #else
    #define QUICK_HOME
    #define PRINTABLE_RADIUS (SCARA_LINKAGE_1 + SCARA_LINKAGE_2)
  #endif
#endif

/**
 * Set the home position based on settings or manual overrides
 */
#if HAS_X_AXIS
  #ifdef MANUAL_X_HOME_POS
    #define X_HOME_POS MANUAL_X_HOME_POS
  #else
    #define X_END_POS TERN(X_HOME_TO_MIN, X_MIN_POS, X_MAX_POS)
    #if ENABLED(BED_CENTER_AT_0_0)
      #define X_HOME_POS TERN(DELTA, 0, X_END_POS)
    #else
      #define X_HOME_POS TERN(DELTA, X_MIN_POS + (X_BED_SIZE) * 0.5, X_END_POS)
    #endif
  #endif
#endif

#if HAS_Y_AXIS
  #ifdef MANUAL_Y_HOME_POS
    #define Y_HOME_POS MANUAL_Y_HOME_POS
  #else
    #define Y_END_POS TERN(Y_HOME_TO_MIN, Y_MIN_POS, Y_MAX_POS)
    #if ENABLED(BED_CENTER_AT_0_0)
      #define Y_HOME_POS TERN(DELTA, 0, Y_END_POS)
    #else
      #define Y_HOME_POS TERN(DELTA, Y_MIN_POS + (Y_BED_SIZE) * 0.5, Y_END_POS)
    #endif
  #endif
#endif

#ifdef MANUAL_Z_HOME_POS
  #define Z_HOME_POS MANUAL_Z_HOME_POS
#else
  #define Z_HOME_POS TERN(Z_HOME_TO_MIN, Z_MIN_POS, Z_MAX_POS)
#endif

#if HAS_I_AXIS
  #ifdef MANUAL_I_HOME_POS
    #define I_HOME_POS MANUAL_I_HOME_POS
  #else
    #define I_HOME_POS TERN(I_HOME_TO_MIN, I_MIN_POS, I_MAX_POS)
  #endif
#endif
#if HAS_J_AXIS
  #ifdef MANUAL_J_HOME_POS
    #define J_HOME_POS MANUAL_J_HOME_POS
  #else
    #define J_HOME_POS TERN(J_HOME_TO_MIN, J_MIN_POS, J_MAX_POS)
  #endif
#endif
#if HAS_K_AXIS
  #ifdef MANUAL_K_HOME_POS
    #define K_HOME_POS MANUAL_K_HOME_POS
  #else
    #define K_HOME_POS TERN(K_HOME_TO_MIN, K_MIN_POS, K_MAX_POS)
  #endif
#endif
#if HAS_U_AXIS
  #ifdef MANUAL_U_HOME_POS
    #define U_HOME_POS MANUAL_U_HOME_POS
  #else
    #define U_HOME_POS TERN(U_HOME_TO_MIN, U_MIN_POS, U_MAX_POS)
  #endif
#endif
#if HAS_V_AXIS
  #ifdef MANUAL_V_HOME_POS
    #define V_HOME_POS MANUAL_V_HOME_POS
  #else
    #define V_HOME_POS TERN(V_HOME_TO_MIN, V_MIN_POS, V_MAX_POS)
  #endif
#endif
#if HAS_W_AXIS
  #ifdef MANUAL_W_HOME_POS
    #define W_HOME_POS MANUAL_W_HOME_POS
  #else
    #define W_HOME_POS TERN(W_HOME_TO_MIN, W_MIN_POS, W_MAX_POS)
  #endif
#endif

/**
 * If DELTA_HEIGHT isn't defined use the old setting
 */
#if ENABLED(DELTA) && !defined(DELTA_HEIGHT)
  #define DELTA_HEIGHT Z_HOME_POS
#endif

/**
 * Z Sled Probe requires Z_SAFE_HOMING
 */
#if ENABLED(Z_PROBE_SLED)
  #define Z_SAFE_HOMING
#endif

/**
 * DELTA should ignore Z_SAFE_HOMING and SLOWDOWN
 */
#if ENABLED(DELTA)
  #undef Z_SAFE_HOMING
#endif

#ifdef MESH_INSET
  #define _MESH_INSET (MESH_INSET)
#else
  #define _MESH_INSET 0
#endif

/**
 * Safe Homing Options
 */
#if ENABLED(Z_SAFE_HOMING)
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    // Home close to center so grid points have z heights very close to 0
    #define _SAFE_POINT(A) (((GRID_MAX_POINTS_##A) / 2) * (A##_BED_SIZE - 2 * _MESH_INSET) / (GRID_MAX_POINTS_##A - 1) + _MESH_INSET)
  #else
    #define _SAFE_POINT(A) A##_CENTER
  #endif
  #ifndef Z_SAFE_HOMING_X_POINT
    #define Z_SAFE_HOMING_X_POINT _SAFE_POINT(X)
  #endif
  #ifndef Z_SAFE_HOMING_Y_POINT
    #define Z_SAFE_HOMING_Y_POINT _SAFE_POINT(Y)
  #endif
#endif

#ifdef GRID_MAX_POINTS_X
  #define GRID_MAX_CELLS_X (GRID_MAX_POINTS_X - 1)
  #define GRID_MAX_CELLS_Y (GRID_MAX_POINTS_Y - 1)
#endif

/**
 * Host keep alive
 */
#ifndef DEFAULT_KEEPALIVE_INTERVAL
  #define DEFAULT_KEEPALIVE_INTERVAL 2
#endif

/**
 * Provide a MAX_AUTORETRACT for older configs
 */
#if ENABLED(FWRETRACT) && !defined(MAX_AUTORETRACT)
  #define MAX_AUTORETRACT 99
#endif

/**
 * LCD Contrast for Graphical Displays
 */
#if ENABLED(CARTESIO_UI)
  #define _LCD_CONTRAST_MIN   60
  #define _LCD_CONTRAST_INIT  90
  #define _LCD_CONTRAST_MAX  140
#elif ENABLED(miniVIKI)
  #define _LCD_CONTRAST_MIN   75
  #define _LCD_CONTRAST_INIT  95
  #define _LCD_CONTRAST_MAX  115
#elif ENABLED(VIKI2)
  #define _LCD_CONTRAST_INIT 140
#elif ENABLED(WYH_L12864)
  #define _LCD_CONTRAST_INIT 190
#elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
  #define _LCD_CONTRAST_MIN   90
  #define _LCD_CONTRAST_INIT 110
  #define _LCD_CONTRAST_MAX  130
#elif ENABLED(AZSMZ_12864)
  #define _LCD_CONTRAST_MIN  120
  #define _LCD_CONTRAST_INIT 190
#elif ANY(MKS_LCD12864A, MKS_LCD12864B)
  #define _LCD_CONTRAST_MIN  120
  #define _LCD_CONTRAST_INIT 205
#elif ANY(MKS_MINI_12864, ENDER2_STOCKDISPLAY)
  #define _LCD_CONTRAST_MIN  120
  #define _LCD_CONTRAST_INIT 195
#elif ENABLED(FYSETC_MINI_12864_2_1)
  #define _LCD_CONTRAST_MIN  230
  #define _LCD_CONTRAST_INIT 255
#elif ENABLED(FYSETC_MINI_12864)
  #define _LCD_CONTRAST_MIN  180
  #define _LCD_CONTRAST_INIT 220
#elif ENABLED(ULTI_CONTROLLER)
  #define _LCD_CONTRAST_INIT 127
  #define _LCD_CONTRAST_MAX  254
#elif ENABLED(MAKRPANEL)
  #define _LCD_CONTRAST_INIT  17
#elif ENABLED(MINIPANEL)
  #define _LCD_CONTRAST_INIT 150
#elif ENABLED(ZONESTAR_12864OLED)
  #define _LCD_CONTRAST_MIN   64
  #define _LCD_CONTRAST_INIT 128
#elif ENABLED(EMOTION_TECH_LCD)
  #define _LCD_CONTRAST_INIT 140
#elif IS_TFTGLCD_PANEL
  #define _LCD_CONTRAST_INIT 250
#endif

#ifdef _LCD_CONTRAST_INIT
  #define HAS_LCD_CONTRAST 1
  #ifndef LCD_CONTRAST_MIN
    #ifdef _LCD_CONTRAST_MIN
      #define LCD_CONTRAST_MIN _LCD_CONTRAST_MIN
    #else
      #define LCD_CONTRAST_MIN 0
    #endif
  #endif
  #ifndef LCD_CONTRAST_INIT
    #define LCD_CONTRAST_INIT _LCD_CONTRAST_INIT
  #endif
  #ifndef LCD_CONTRAST_MAX
    #ifdef _LCD_CONTRAST_MAX
      #define LCD_CONTRAST_MAX _LCD_CONTRAST_MAX
    #elif _LCD_CONTRAST_INIT > 63
      #define LCD_CONTRAST_MAX 255
    #else
      #define LCD_CONTRAST_MAX 63   // ST7567 6-bits contrast
    #endif
  #endif
  #ifndef LCD_CONTRAST_DEFAULT
    #define LCD_CONTRAST_DEFAULT LCD_CONTRAST_INIT
  #endif
#endif

/**
 * Override the SD_DETECT_STATE set in Configuration_adv.h
 * and enable sharing of onboard SD host drives (all platforms but AGCM4)
 */
#if HAS_MEDIA

  #if HAS_SD_HOST_DRIVE && SD_CONNECTION_IS(ONBOARD) && DISABLED(KEEP_SD_DETECT)
    //
    // The external SD card is not used. Hardware SPI is used to access the card.
    // When sharing the SD card with a PC we want the menu options to
    // mount/unmount the card and refresh it. So we disable card detect.
    //
    #undef SD_DETECT_PIN
    #define HAS_SHARED_MEDIA 1
  #endif

  // Extender cable doesn't support SD_DETECT_PIN
  #if ENABLED(NO_SD_DETECT)
    #undef SD_DETECT_PIN
  #endif

  // Not onboard or custom cable
  #if SD_CONNECTION_IS(LCD) || !defined(SDCARD_CONNECTION)
    #define SD_CONNECTION_TYPICAL 1
  #endif

  // Set SD_DETECT_STATE based on hardware if not overridden
  #if PIN_EXISTS(SD_DETECT)
    #define HAS_SD_DETECT 1
    #ifndef SD_DETECT_STATE
      #if ALL(SD_CONNECTION_TYPICAL, HAS_MARLINUI_MENU, ELB_FULL_GRAPHIC_CONTROLLER)
        #define SD_DETECT_STATE HIGH
      #else
        #define SD_DETECT_STATE LOW
      #endif
    #endif
  #endif

  #if !HAS_USB_FLASH_DRIVE || ALL(HAS_MULTI_VOLUME, VOLUME_SD_ONBOARD)
    #if ENABLED(ONBOARD_SDIO)
      #define NEED_SD2CARD_SDIO 1
    #else
      #define NEED_SD2CARD_SPI 1
    #endif
  #endif

  #if HAS_SD_DETECT && NONE(HAS_GRAPHICAL_TFT, LCD_USE_DMA_FSMC, HAS_FSMC_GRAPHICAL_TFT, HAS_SPI_GRAPHICAL_TFT, IS_DWIN_MARLINUI, EXTENSIBLE_UI, HAS_DWIN_E3V2, HAS_U8GLIB_I2C_OLED)
    #define REINIT_NOISY_LCD 1  // Have the LCD re-init on SD insertion
  #endif

#endif // HAS_MEDIA

/**
 * Power Supply
 */
#ifndef PSU_NAME
  #if DISABLED(PSU_CONTROL)
    #define PSU_NAME "Generic"  // No control
  #elif PSU_ACTIVE_STATE
    #define PSU_NAME "XBox"     // X-Box 360 (203W)
  #else
    #define PSU_NAME "ATX"      // ATX style
  #endif
#endif

#if ENABLED(PSU_CONTROL)
  #ifndef PSU_POWERUP_DELAY
    #define PSU_POWERUP_DELAY 250
  #endif
  #ifndef POWER_OFF_DELAY
    #define POWER_OFF_DELAY   0
  #endif
#endif

/**
 * Temp Sensor defines; set up pins as needed.
 */

// Usurp a sensor to do redundant readings
#if TEMP_SENSOR_REDUNDANT
  #ifndef TEMP_SENSOR_REDUNDANT_SOURCE
    #define TEMP_SENSOR_REDUNDANT_SOURCE E1
  #endif
  #ifndef TEMP_SENSOR_REDUNDANT_TARGET
    #define TEMP_SENSOR_REDUNDANT_TARGET E0
  #endif
  #if !PIN_EXISTS(TEMP_REDUNDANT)
    #ifndef TEMP_SENSOR_REDUNDANT_MAX_DIFF
      #define TEMP_SENSOR_REDUNDANT_MAX_DIFF 10
    #endif
    #if REDUNDANT_TEMP_MATCH(SOURCE, BOARD)
      #if !PIN_EXISTS(TEMP_BOARD)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to BOARD requires TEMP_BOARD_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_BOARD_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, COOLER)
      #if !PIN_EXISTS(TEMP_COOLER)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to COOLER requires TEMP_COOLER_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_COOLER_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, PROBE)
      #if !PIN_EXISTS(TEMP_PROBE)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to PROBE requires TEMP_PROBE_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_PROBE_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, CHAMBER)
      #if !PIN_EXISTS(TEMP_CHAMBER)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to CHAMBER requires TEMP_CHAMBER_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_CHAMBER_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, BED)
      #if !PIN_EXISTS(TEMP_BED)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to BED requires TEMP_BED_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_BED_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, E0)
      #if !PIN_EXISTS(TEMP_0)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to E0 requires TEMP_0_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_0_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, E1)
      #if !PIN_EXISTS(TEMP_1)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to E1 requires TEMP_1_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_1_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, E2)
      #if !PIN_EXISTS(TEMP_2)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to E2 requires TEMP_2_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_2_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, E3)
      #if !PIN_EXISTS(TEMP_3)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to E3 requires TEMP_3_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_3_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, E4)
      #if !PIN_EXISTS(TEMP_4)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to E4 requires TEMP_4_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_4_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, E5)
      #if !PIN_EXISTS(TEMP_5)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to E5 requires TEMP_5_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_5_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, E6)
      #if !PIN_EXISTS(TEMP_6)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to E6 requires TEMP_6_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_6_PIN
      #endif
    #elif REDUNDANT_TEMP_MATCH(SOURCE, E7)
      #if !PIN_EXISTS(TEMP_7)
        #error "TEMP_SENSOR_REDUNDANT_SOURCE set to E7 requires TEMP_7_PIN."
      #else
        #define TEMP_REDUNDANT_PIN TEMP_7_PIN
      #endif
    #endif
  #endif
#endif

/**
 * Compatibility layer for MAX (SPI) temp boards
 */

#define TEMP_SENSOR_IS_ANY_MAX_TC(n) (TEMP_SENSOR_IS_MAX_TC(n) || (TEMP_SENSOR_IS_MAX_TC(REDUNDANT) && REDUNDANT_TEMP_MATCH(SOURCE, E##n)))

#if HAS_MAX_TC

  // Software SPI - enable if MISO/SCK are defined.
  #if TEMP_SENSOR_IS_ANY_MAX_TC(0) && DISABLED(TEMP_SENSOR_0_FORCE_HW_SPI) && PINS_EXIST(TEMP_0_MISO, TEMP_0_SCK)
    #if TEMP_SENSOR_0_IS_MAX31865 && !PIN_EXISTS(TEMP_0_MOSI)
      #error "TEMP_SENSOR_0 MAX31865 requires TEMP_0_MOSI_PIN defined for Software SPI. To use Hardware SPI instead, undefine MISO/SCK or enable TEMP_SENSOR_0_FORCE_HW_SPI."
    #else
      #define TEMP_SENSOR_0_HAS_SPI_PINS 1
    #endif
  #endif

  #if TEMP_SENSOR_IS_ANY_MAX_TC(1) && DISABLED(TEMP_SENSOR_1_FORCE_HW_SPI) && PINS_EXIST(TEMP_1_MISO, TEMP_1_SCK)
    #if TEMP_SENSOR_1_IS_MAX31865 && !PIN_EXISTS(TEMP_1_MOSI)
      #error "TEMP_SENSOR_1 MAX31865 requires TEMP_1_MOSI_PIN defined for Software SPI. To use Hardware SPI instead, undefine MISO/SCK or enable TEMP_SENSOR_1_FORCE_HW_SPI."
    #else
      #define TEMP_SENSOR_1_HAS_SPI_PINS 1
    #endif
  #endif

  #if TEMP_SENSOR_IS_ANY_MAX_TC(2) && DISABLED(TEMP_SENSOR_2_FORCE_HW_SPI) && PINS_EXIST(TEMP_2_MISO, TEMP_2_SCK)
    #if TEMP_SENSOR_2_IS_MAX31865 && !PIN_EXISTS(TEMP_2_MOSI)
      #error "TEMP_SENSOR_2 MAX31865 requires TEMP_2_MOSI_PIN defined for Software SPI. To use Hardware SPI instead, undefine MISO/SCK or enable TEMP_SENSOR_2_FORCE_HW_SPI."
    #else
      #define TEMP_SENSOR_2_HAS_SPI_PINS 1
    #endif
  #endif

  #if (TEMP_SENSOR_IS_MAX_TC(BED)) && DISABLED(TEMP_SENSOR_BED_FORCE_HW_SPI) && PINS_EXIST(TEMP_BED_MISO, TEMP_BED_SCK)
    #if TEMP_SENSOR_BED_IS_MAX31865 && !PIN_EXISTS(TEMP_BED_MOSI)
      #error "TEMP_SENSOR_BED MAX31865 requires TEMP_BED_MOSI_PIN defined for Software SPI. To use Hardware SPI instead, undefine MISO/SCK or enable TEMP_SENSOR_BED_FORCE_HW_SPI."
    #else
      #define TEMP_SENSOR_BED_HAS_SPI_PINS 1
    #endif
  #endif

  //
  // User-defined thermocouple libraries
  //
  // Add LIB_MAX6675 / LIB_MAX31855 / LIB_MAX31865 to the build_flags
  // to select a USER library for MAX6675, MAX31855, MAX31865
  //
  #if ALL(HAS_MAX6675, LIB_MAX6675)
    #define USE_LIB_MAX6675 1
  #endif
  #if ALL(HAS_MAX31855, LIB_MAX31855)
    #define USE_ADAFRUIT_MAX31855 1
  #endif
  #if ALL(HAS_MAX31865, LIB_MAX31865)
    #define USE_ADAFRUIT_MAX31865 1
  #elif HAS_MAX31865
    #define LIB_INTERNAL_MAX31865 1
  #endif

#endif // HAS_MAX_TC

/**
 * X_DUAL_ENDSTOPS endstop reassignment
 */
#if ENABLED(X_DUAL_ENDSTOPS)
  #if X_HOME_TO_MAX
    #ifndef X2_MAX_ENDSTOP_HIT_STATE
      #if PIN_EXISTS(X2_STOP)
        #if X2_STOP_PIN == X_MIN_PIN
          #define X2_MAX_ENDSTOP_HIT_STATE X_MIN_ENDSTOP_HIT_STATE
        #elif X2_STOP_PIN == X_MAX_PIN
          #define X2_MAX_ENDSTOP_HIT_STATE X_MAX_ENDSTOP_HIT_STATE
        #elif X2_STOP_PIN == Y_MIN_PIN
          #define X2_MAX_ENDSTOP_HIT_STATE Y_MIN_ENDSTOP_HIT_STATE
        #elif X2_STOP_PIN == Y_MAX_PIN
          #define X2_MAX_ENDSTOP_HIT_STATE Y_MAX_ENDSTOP_HIT_STATE
        #elif X2_STOP_PIN == Z_MIN_PIN
          #define X2_MAX_ENDSTOP_HIT_STATE Z_MIN_ENDSTOP_HIT_STATE
        #elif X2_STOP_PIN == Z_MAX_PIN
          #define X2_MAX_ENDSTOP_HIT_STATE Z_MAX_ENDSTOP_HIT_STATE
        #endif
      #endif
    #endif
    #if !PIN_EXISTS(X2_MAX)
      #undef X2_MAX_PIN
      #if PIN_EXISTS(X2_STOP)
        #define X2_MAX_PIN X2_STOP_PIN
      #endif
    #endif
  #else
    #ifndef X2_MIN_ENDSTOP_HIT_STATE
      #if PIN_EXISTS(X2_STOP)
        #if X2_STOP_PIN == X_MIN_PIN
          #define X2_MIN_ENDSTOP_HIT_STATE X_MIN_ENDSTOP_HIT_STATE
        #elif X2_STOP_PIN == X_MAX_PIN
          #define X2_MIN_ENDSTOP_HIT_STATE X_MAX_ENDSTOP_HIT_STATE
        #elif X2_STOP_PIN == Y_MIN_PIN
          #define X2_MIN_ENDSTOP_HIT_STATE Y_MIN_ENDSTOP_HIT_STATE
        #elif X2_STOP_PIN == Y_MAX_PIN
          #define X2_MIN_ENDSTOP_HIT_STATE Y_MAX_ENDSTOP_HIT_STATE
        #elif X2_STOP_PIN == Z_MIN_PIN
          #define X2_MIN_ENDSTOP_HIT_STATE Z_MIN_ENDSTOP_HIT_STATE
        #elif X2_STOP_PIN == Z_MAX_PIN
          #define X2_MIN_ENDSTOP_HIT_STATE Z_MAX_ENDSTOP_HIT_STATE
        #endif
      #endif
    #endif
    #if !PIN_EXISTS(X2_MIN)
      #undef X2_MIN_PIN
      #if PIN_EXISTS(X2_STOP)
        #define X2_MIN_PIN X2_STOP_PIN
      #endif
    #endif
  #endif
  #ifndef X2_MAX_ENDSTOP_HIT_STATE
    #define X2_MAX_ENDSTOP_HIT_STATE HIGH
  #endif
  #ifndef X2_MIN_ENDSTOP_HIT_STATE
    #define X2_MIN_ENDSTOP_HIT_STATE HIGH
  #endif
#endif

/**
 * Y_DUAL_ENDSTOPS endstop reassignment
 */
#if ENABLED(Y_DUAL_ENDSTOPS)
  #if Y_HOME_TO_MAX
    #ifndef Y2_MAX_ENDSTOP_HIT_STATE
      #if PIN_EXISTS(Y2_STOP)
        #if Y2_STOP_PIN == X_MIN_PIN
          #define Y2_MAX_ENDSTOP_HIT_STATE X_MIN_ENDSTOP_HIT_STATE
        #elif Y2_STOP_PIN == X_MAX_PIN
          #define Y2_MAX_ENDSTOP_HIT_STATE X_MAX_ENDSTOP_HIT_STATE
        #elif Y2_STOP_PIN == Y_MIN_PIN
          #define Y2_MAX_ENDSTOP_HIT_STATE Y_MIN_ENDSTOP_HIT_STATE
        #elif Y2_STOP_PIN == Y_MAX_PIN
          #define Y2_MAX_ENDSTOP_HIT_STATE Y_MAX_ENDSTOP_HIT_STATE
        #elif Y2_STOP_PIN == Z_MIN_PIN
          #define Y2_MAX_ENDSTOP_HIT_STATE Z_MIN_ENDSTOP_HIT_STATE
        #elif Y2_STOP_PIN == Z_MAX_PIN
          #define Y2_MAX_ENDSTOP_HIT_STATE Z_MAX_ENDSTOP_HIT_STATE
        #endif
      #endif
    #endif
    #if !PIN_EXISTS(Y2_MAX)
      #undef Y2_MAX_PIN
      #if PIN_EXISTS(Y2_STOP)
        #define Y2_MAX_PIN Y2_STOP_PIN
      #endif
    #endif
  #else
    #ifndef Y2_MIN_ENDSTOP_HIT_STATE
      #if PIN_EXISTS(Y2_STOP)
        #if Y2_STOP_PIN == X_MIN_PIN
          #define Y2_MIN_ENDSTOP_HIT_STATE X_MIN_ENDSTOP_HIT_STATE
        #elif Y2_STOP_PIN == X_MAX_PIN
          #define Y2_MIN_ENDSTOP_HIT_STATE X_MAX_ENDSTOP_HIT_STATE
        #elif Y2_STOP_PIN == Y_MIN_PIN
          #define Y2_MIN_ENDSTOP_HIT_STATE Y_MIN_ENDSTOP_HIT_STATE
        #elif Y2_STOP_PIN == Y_MAX_PIN
          #define Y2_MIN_ENDSTOP_HIT_STATE Y_MAX_ENDSTOP_HIT_STATE
        #elif Y2_STOP_PIN == Z_MIN_PIN
          #define Y2_MIN_ENDSTOP_HIT_STATE Z_MIN_ENDSTOP_HIT_STATE
        #elif Y2_STOP_PIN == Z_MAX_PIN
          #define Y2_MIN_ENDSTOP_HIT_STATE Z_MAX_ENDSTOP_HIT_STATE
        #endif
      #endif
    #endif
    #if !PIN_EXISTS(Y2_MIN)
      #undef Y2_MIN_PIN
      #if PIN_EXISTS(Y2_STOP)
        #define Y2_MIN_PIN Y2_STOP_PIN
      #endif
    #endif
  #endif
  #ifndef Y2_MAX_ENDSTOP_HIT_STATE
    #define Y2_MAX_ENDSTOP_HIT_STATE HIGH
  #endif
  #ifndef Y2_MIN_ENDSTOP_HIT_STATE
    #define Y2_MIN_ENDSTOP_HIT_STATE HIGH
  #endif
#endif

/**
 * Z_MULTI_ENDSTOPS endstop reassignment
 */
#if ENABLED(Z_MULTI_ENDSTOPS)

  #if Z_HOME_TO_MAX
    #ifndef Z2_MAX_ENDSTOP_HIT_STATE
      #if PIN_EXISTS(Z2_STOP)
        #if Z2_STOP_PIN == X_MIN_PIN
          #define Z2_MAX_ENDSTOP_HIT_STATE X_MIN_ENDSTOP_HIT_STATE
        #elif Z2_STOP_PIN == X_MAX_PIN
          #define Z2_MAX_ENDSTOP_HIT_STATE X_MAX_ENDSTOP_HIT_STATE
        #elif Z2_STOP_PIN == Y_MIN_PIN
          #define Z2_MAX_ENDSTOP_HIT_STATE Y_MIN_ENDSTOP_HIT_STATE
        #elif Z2_STOP_PIN == Y_MAX_PIN
          #define Z2_MAX_ENDSTOP_HIT_STATE Y_MAX_ENDSTOP_HIT_STATE
        #elif Z2_STOP_PIN == Z_MIN_PIN
          #define Z2_MAX_ENDSTOP_HIT_STATE Z_MIN_ENDSTOP_HIT_STATE
        #elif Z2_STOP_PIN == Z_MAX_PIN
          #define Z2_MAX_ENDSTOP_HIT_STATE Z_MAX_ENDSTOP_HIT_STATE
        #endif
      #endif
    #endif
    #if !PIN_EXISTS(Z2_MAX)
      #undef Z2_MAX_PIN
      #if PIN_EXISTS(Z2_STOP)
        #define Z2_MAX_PIN Z2_STOP_PIN
      #endif
    #endif
  #else
    #ifndef Z2_MIN_ENDSTOP_HIT_STATE
      #if PIN_EXISTS(Z2_STOP)
        #if Z2_STOP_PIN == X_MIN_PIN
          #define Z2_MIN_ENDSTOP_HIT_STATE X_MIN_ENDSTOP_HIT_STATE
        #elif Z2_STOP_PIN == X_MAX_PIN
          #define Z2_MIN_ENDSTOP_HIT_STATE X_MAX_ENDSTOP_HIT_STATE
        #elif Z2_STOP_PIN == Y_MIN_PIN
          #define Z2_MIN_ENDSTOP_HIT_STATE Y_MIN_ENDSTOP_HIT_STATE
        #elif Z2_STOP_PIN == Y_MAX_PIN
          #define Z2_MIN_ENDSTOP_HIT_STATE Y_MAX_ENDSTOP_HIT_STATE
        #elif Z2_STOP_PIN == Z_MIN_PIN
          #define Z2_MIN_ENDSTOP_HIT_STATE Z_MIN_ENDSTOP_HIT_STATE
        #elif Z2_STOP_PIN == Z_MAX_PIN
          #define Z2_MIN_ENDSTOP_HIT_STATE Z_MAX_ENDSTOP_HIT_STATE
        #endif
      #endif
    #endif
    #if !PIN_EXISTS(Z2_MIN)
      #undef Z2_MIN_PIN
      #if PIN_EXISTS(Z2_STOP)
        #define Z2_MIN_PIN Z2_STOP_PIN
      #endif
    #endif
  #endif
  #ifndef Z2_MAX_ENDSTOP_HIT_STATE
    #define Z2_MAX_ENDSTOP_HIT_STATE HIGH
  #endif
  #ifndef Z2_MIN_ENDSTOP_HIT_STATE
    #define Z2_MIN_ENDSTOP_HIT_STATE HIGH
  #endif

  #if NUM_Z_STEPPERS >= 3
    #if Z_HOME_TO_MAX
      #ifndef Z3_MAX_ENDSTOP_HIT_STATE
        #if PIN_EXISTS(Z3_STOP)
          #if Z3_STOP_PIN == X_MIN_PIN
            #define Z3_MAX_ENDSTOP_HIT_STATE X_MIN_ENDSTOP_HIT_STATE
          #elif Z3_STOP_PIN == X_MAX_PIN
            #define Z3_MAX_ENDSTOP_HIT_STATE X_MAX_ENDSTOP_HIT_STATE
          #elif Z3_STOP_PIN == Y_MIN_PIN
            #define Z3_MAX_ENDSTOP_HIT_STATE Y_MIN_ENDSTOP_HIT_STATE
          #elif Z3_STOP_PIN == Y_MAX_PIN
            #define Z3_MAX_ENDSTOP_HIT_STATE Y_MAX_ENDSTOP_HIT_STATE
          #elif Z3_STOP_PIN == Z_MIN_PIN
            #define Z3_MAX_ENDSTOP_HIT_STATE Z_MIN_ENDSTOP_HIT_STATE
          #elif Z3_STOP_PIN == Z_MAX_PIN
            #define Z3_MAX_ENDSTOP_HIT_STATE Z_MAX_ENDSTOP_HIT_STATE
          #endif
        #endif
      #endif
      #if !PIN_EXISTS(Z3_MAX)
        #undef Z3_MAX_PIN
        #if PIN_EXISTS(Z3_STOP)
          #define Z3_MAX_PIN Z3_STOP_PIN
        #endif
      #endif
    #else
      #ifndef Z3_MIN_ENDSTOP_HIT_STATE
        #if PIN_EXISTS(Z3_STOP)
          #if Z3_STOP_PIN == X_MIN_PIN
            #define Z3_MIN_ENDSTOP_HIT_STATE X_MIN_ENDSTOP_HIT_STATE
          #elif Z3_STOP_PIN == X_MAX_PIN
            #define Z3_MIN_ENDSTOP_HIT_STATE X_MAX_ENDSTOP_HIT_STATE
          #elif Z3_STOP_PIN == Y_MIN_PIN
            #define Z3_MIN_ENDSTOP_HIT_STATE Y_MIN_ENDSTOP_HIT_STATE
          #elif Z3_STOP_PIN == Y_MAX_PIN
            #define Z3_MIN_ENDSTOP_HIT_STATE Y_MAX_ENDSTOP_HIT_STATE
          #elif Z3_STOP_PIN == Z_MIN_PIN
            #define Z3_MIN_ENDSTOP_HIT_STATE Z_MIN_ENDSTOP_HIT_STATE
          #elif Z3_STOP_PIN == Z_MAX_PIN
            #define Z3_MIN_ENDSTOP_HIT_STATE Z_MAX_ENDSTOP_HIT_STATE
          #endif
        #endif
      #endif
      #if !PIN_EXISTS(Z3_MIN)
        #undef Z3_MIN_PIN
        #if PIN_EXISTS(Z3_STOP)
          #define Z3_MIN_PIN Z3_STOP_PIN
        #endif
      #endif
    #endif
    #ifndef Z3_MAX_ENDSTOP_HIT_STATE
      #define Z3_MAX_ENDSTOP_HIT_STATE HIGH
    #endif
    #ifndef Z3_MIN_ENDSTOP_HIT_STATE
      #define Z3_MIN_ENDSTOP_HIT_STATE HIGH
    #endif
  #endif

  #if NUM_Z_STEPPERS >= 4
    #if Z_HOME_TO_MAX
      #ifndef Z4_MAX_ENDSTOP_HIT_STATE
        #if PIN_EXISTS(Z4_STOP)
          #if Z4_STOP_PIN == X_MIN_PIN
            #define Z4_MAX_ENDSTOP_HIT_STATE X_MIN_ENDSTOP_HIT_STATE
          #elif Z4_STOP_PIN == X_MAX_PIN
            #define Z4_MAX_ENDSTOP_HIT_STATE X_MAX_ENDSTOP_HIT_STATE
          #elif Z4_STOP_PIN == Y_MIN_PIN
            #define Z4_MAX_ENDSTOP_HIT_STATE Y_MIN_ENDSTOP_HIT_STATE
          #elif Z4_STOP_PIN == Y_MAX_PIN
            #define Z4_MAX_ENDSTOP_HIT_STATE Y_MAX_ENDSTOP_HIT_STATE
          #elif Z4_STOP_PIN == Z_MIN_PIN
            #define Z4_MAX_ENDSTOP_HIT_STATE Z_MIN_ENDSTOP_HIT_STATE
          #elif Z4_STOP_PIN == Z_MAX_PIN
            #define Z4_MAX_ENDSTOP_HIT_STATE Z_MAX_ENDSTOP_HIT_STATE
          #endif
        #endif
      #endif
      #if !PIN_EXISTS(Z4_MAX)
        #undef Z4_MAX_PIN
        #if PIN_EXISTS(Z4_STOP)
          #define Z4_MAX_PIN Z4_STOP_PIN
        #endif
      #endif
    #else
      #ifndef Z4_MIN_ENDSTOP_HIT_STATE
        #if PIN_EXISTS(Z4_STOP)
          #if Z4_STOP_PIN == X_MIN_PIN
            #define Z4_MIN_ENDSTOP_HIT_STATE X_MIN_ENDSTOP_HIT_STATE
          #elif Z4_STOP_PIN == X_MAX_PIN
            #define Z4_MIN_ENDSTOP_HIT_STATE X_MAX_ENDSTOP_HIT_STATE
          #elif Z4_STOP_PIN == Y_MIN_PIN
            #define Z4_MIN_ENDSTOP_HIT_STATE Y_MIN_ENDSTOP_HIT_STATE
          #elif Z4_STOP_PIN == Y_MAX_PIN
            #define Z4_MIN_ENDSTOP_HIT_STATE Y_MAX_ENDSTOP_HIT_STATE
          #elif Z4_STOP_PIN == Z_MIN_PIN
            #define Z4_MIN_ENDSTOP_HIT_STATE Z_MIN_ENDSTOP_HIT_STATE
          #elif Z4_STOP_PIN == Z_MAX_PIN
            #define Z4_MIN_ENDSTOP_HIT_STATE Z_MAX_ENDSTOP_HIT_STATE
          #endif
        #endif
      #endif
      #if !PIN_EXISTS(Z4_MIN)
        #undef Z4_MIN_PIN
        #if PIN_EXISTS(Z4_STOP)
          #define Z4_MIN_PIN Z4_STOP_PIN
        #endif
      #endif
    #endif
    #ifndef Z4_MAX_ENDSTOP_HIT_STATE
      #define Z4_MAX_ENDSTOP_HIT_STATE HIGH
    #endif
    #ifndef Z4_MIN_ENDSTOP_HIT_STATE
      #define Z4_MIN_ENDSTOP_HIT_STATE HIGH
    #endif
  #endif

#endif // Z_MULTI_ENDSTOPS

/**
 * Shorthand for pin tests, used wherever needed
 */

// Steppers
#if HAS_X_AXIS
  #if PIN_EXISTS(X_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(X))
    #define HAS_X_ENABLE 1
  #endif
  #if PIN_EXISTS(X_DIR)
    #define HAS_X_DIR 1
  #endif
  #if PIN_EXISTS(X_STEP)
    #define HAS_X_STEP 1
  #endif
  #if PIN_EXISTS(X_MS1)
    #define HAS_X_MS_PINS 1
  #endif

  #if PIN_EXISTS(X2_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(X2))
    #define HAS_X2_ENABLE 1
  #endif
  #if PIN_EXISTS(X2_DIR)
    #define HAS_X2_DIR 1
  #endif
  #if PIN_EXISTS(X2_STEP)
    #define HAS_X2_STEP 1
  #endif
  #if PIN_EXISTS(X2_MS1)
    #define HAS_X2_MS_PINS 1
  #endif
#endif

/**
 * Set defaults for missing (newer) options
 */

#if HAS_Y_AXIS
  #if PIN_EXISTS(Y_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(Y))
    #define HAS_Y_ENABLE 1
  #endif
  #if PIN_EXISTS(Y_DIR)
    #define HAS_Y_DIR 1
  #endif
  #if PIN_EXISTS(Y_STEP)
    #define HAS_Y_STEP 1
  #endif
  #if PIN_EXISTS(Y_MS1)
    #define HAS_Y_MS_PINS 1
  #endif

  #if HAS_Y2_STEPPER
    #if PIN_EXISTS(Y2_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(Y2))
      #define HAS_Y2_ENABLE 1
    #endif
    #if PIN_EXISTS(Y2_DIR)
      #define HAS_Y2_DIR 1
    #endif
    #if PIN_EXISTS(Y2_STEP)
      #define HAS_Y2_STEP 1
    #endif
    #if PIN_EXISTS(Y2_MS1)
      #define HAS_Y2_MS_PINS 1
    #endif
  #endif
#endif

#if HAS_Z_AXIS
  #if PIN_EXISTS(Z_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(Z))
    #define HAS_Z_ENABLE 1
  #endif
  #if PIN_EXISTS(Z_DIR)
    #define HAS_Z_DIR 1
  #endif
  #if PIN_EXISTS(Z_STEP)
    #define HAS_Z_STEP 1
  #endif
  #if PIN_EXISTS(Z_MS1)
    #define HAS_Z_MS_PINS 1
  #endif
#endif

#if NUM_Z_STEPPERS >= 2
  #if PIN_EXISTS(Z2_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(Z2))
    #define HAS_Z2_ENABLE 1
  #endif
  #if PIN_EXISTS(Z2_DIR)
    #define HAS_Z2_DIR 1
  #endif
  #if PIN_EXISTS(Z2_STEP)
    #define HAS_Z2_STEP 1
  #endif
  #if PIN_EXISTS(Z2_MS1)
    #define HAS_Z2_MS_PINS 1
  #endif
#endif

#if NUM_Z_STEPPERS >= 3
  #if PIN_EXISTS(Z3_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(Z3))
    #define HAS_Z3_ENABLE 1
  #endif
  #if PIN_EXISTS(Z3_DIR)
    #define HAS_Z3_DIR 1
  #endif
  #if PIN_EXISTS(Z3_STEP)
    #define HAS_Z3_STEP 1
  #endif
  #if PIN_EXISTS(Z3_MS1)
    #define HAS_Z3_MS_PINS 1
  #endif
#endif

#if NUM_Z_STEPPERS >= 4
  #if PIN_EXISTS(Z4_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(Z4))
    #define HAS_Z4_ENABLE 1
  #endif
  #if PIN_EXISTS(Z4_DIR)
    #define HAS_Z4_DIR 1
  #endif
  #if PIN_EXISTS(Z4_STEP)
    #define HAS_Z4_STEP 1
  #endif
  #if PIN_EXISTS(Z4_MS1)
    #define HAS_Z4_MS_PINS 1
  #endif
#endif

#if HAS_I_AXIS
  #if PIN_EXISTS(I_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(I))
    #define HAS_I_ENABLE 1
  #endif
  #if PIN_EXISTS(I_DIR)
    #define HAS_I_DIR 1
  #endif
  #if PIN_EXISTS(I_STEP)
    #define HAS_I_STEP 1
  #endif
  #if PIN_EXISTS(I_MS1)
    #define HAS_I_MS_PINS 1
  #endif
#endif

#if HAS_J_AXIS
  #if PIN_EXISTS(J_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(J))
    #define HAS_J_ENABLE 1
  #endif
  #if PIN_EXISTS(J_DIR)
    #define HAS_J_DIR 1
  #endif
  #if PIN_EXISTS(J_STEP)
    #define HAS_J_STEP 1
  #endif
  #if PIN_EXISTS(J_MS1)
    #define HAS_J_MS_PINS 1
  #endif
#endif

#if HAS_K_AXIS
  #if PIN_EXISTS(K_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(K))
    #define HAS_K_ENABLE 1
  #endif
  #if PIN_EXISTS(K_DIR)
    #define HAS_K_DIR 1
  #endif
  #if PIN_EXISTS(K_STEP)
    #define HAS_K_STEP 1
  #endif
  #if PIN_EXISTS(K_MS1)
    #define HAS_K_MS_PINS 1
  #endif
#endif

#if HAS_U_AXIS
  #if PIN_EXISTS(U_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(U))
    #define HAS_U_ENABLE 1
  #endif
  #if PIN_EXISTS(U_DIR)
    #define HAS_U_DIR 1
  #endif
  #if PIN_EXISTS(U_STEP)
    #define HAS_U_STEP 1
  #endif
  #if PIN_EXISTS(U_MS1)
    #define HAS_U_MS_PINS 1
  #endif
#endif

#if HAS_V_AXIS
  #if PIN_EXISTS(V_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(V))
    #define HAS_V_ENABLE 1
  #endif
  #if PIN_EXISTS(V_DIR)
    #define HAS_V_DIR 1
  #endif
  #if PIN_EXISTS(V_STEP)
    #define HAS_V_STEP 1
  #endif
  #if PIN_EXISTS(V_MS1)
    #define HAS_V_MS_PINS 1
  #endif
#endif

#if HAS_W_AXIS
  #if PIN_EXISTS(W_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(W))
    #define HAS_W_ENABLE 1
  #endif
  #if PIN_EXISTS(W_DIR)
    #define HAS_W_DIR 1
  #endif
  #if PIN_EXISTS(W_STEP)
    #define HAS_W_STEP 1
  #endif
  #if PIN_EXISTS(W_MS1)
    #define HAS_W_MS_PINS 1
  #endif
#endif

// Extruder steppers and solenoids
#if HAS_EXTRUDERS

  #if PIN_EXISTS(E0_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(E0))
    #define HAS_E0_ENABLE 1
  #endif
  #if PIN_EXISTS(E0_DIR)
    #define HAS_E0_DIR 1
  #endif
  #if PIN_EXISTS(E0_STEP)
    #define HAS_E0_STEP 1
  #endif
  #if PIN_EXISTS(E0_MS1)
    #define HAS_E0_MS_PINS 1
  #endif

  #if E_STEPPERS > 1 || ENABLED(E_DUAL_STEPPER_DRIVERS)
    #if PIN_EXISTS(E1_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(E1))
      #define HAS_E1_ENABLE 1
    #endif
    #if PIN_EXISTS(E1_DIR)
      #define HAS_E1_DIR 1
    #endif
    #if PIN_EXISTS(E1_STEP)
      #define HAS_E1_STEP 1
    #endif
    #if PIN_EXISTS(E1_MS1)
      #define HAS_E1_MS_PINS 1
    #endif
  #endif

  #if E_STEPPERS > 2
    #if PIN_EXISTS(E2_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(E2))
      #define HAS_E2_ENABLE 1
    #endif
    #if PIN_EXISTS(E2_DIR)
      #define HAS_E2_DIR 1
    #endif
    #if PIN_EXISTS(E2_STEP)
      #define HAS_E2_STEP 1
    #endif
    #if PIN_EXISTS(E2_MS1)
      #define HAS_E2_MS_PINS 1
    #endif
  #endif

  #if E_STEPPERS > 3
    #if PIN_EXISTS(E3_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(E3))
      #define HAS_E3_ENABLE 1
    #endif
    #if PIN_EXISTS(E3_DIR)
      #define HAS_E3_DIR 1
    #endif
    #if PIN_EXISTS(E3_STEP)
      #define HAS_E3_STEP 1
    #endif
    #if PIN_EXISTS(E3_MS1)
      #define HAS_E3_MS_PINS 1
    #endif
  #endif

  #if E_STEPPERS > 4
    #if PIN_EXISTS(E4_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(E4))
      #define HAS_E4_ENABLE 1
    #endif
    #if PIN_EXISTS(E4_DIR)
      #define HAS_E4_DIR 1
    #endif
    #if PIN_EXISTS(E4_STEP)
      #define HAS_E4_STEP 1
    #endif
    #if PIN_EXISTS(E4_MS1)
      #define HAS_E4_MS_PINS 1
    #endif
  #endif

  #if E_STEPPERS > 5
    #if PIN_EXISTS(E5_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(E5))
      #define HAS_E5_ENABLE 1
    #endif
    #if PIN_EXISTS(E5_DIR)
      #define HAS_E5_DIR 1
    #endif
    #if PIN_EXISTS(E5_STEP)
      #define HAS_E5_STEP 1
    #endif
    #if PIN_EXISTS(E5_MS1)
      #define HAS_E5_MS_PINS 1
    #endif
  #endif

  #if E_STEPPERS > 6
    #if PIN_EXISTS(E6_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(E6))
      #define HAS_E6_ENABLE 1
    #endif
    #if PIN_EXISTS(E6_DIR)
      #define HAS_E6_DIR 1
    #endif
    #if PIN_EXISTS(E6_STEP)
      #define HAS_E6_STEP 1
    #endif
    #if PIN_EXISTS(E6_MS1)
      #define HAS_E6_MS_PINS 1
    #endif
  #endif

  #if E_STEPPERS > 7
    #if PIN_EXISTS(E7_ENABLE) || (ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_TMC(E7))
      #define HAS_E7_ENABLE 1
    #endif
    #if PIN_EXISTS(E7_DIR)
      #define HAS_E7_DIR 1
    #endif
    #if PIN_EXISTS(E7_STEP)
      #define HAS_E7_STEP 1
    #endif
    #if PIN_EXISTS(E7_MS1)
      #define HAS_E7_MS_PINS 1
    #endif
  #endif

#endif // HAS_EXTRUDERS

/**
 * Set solenoid flags if any features use solenoids
 *   - EXT_SOLENOID (M380, M381) to enable/disable the extruder solenoid
 *   - MANUAL_SOLENOID_CONTROL (M380, M381) to enable/disable solenoids by index
 *   - PARKING_EXTRUDER uses SOL0_PIN and SOL1_PIN
 *   - SOLENOID_PROBE uses SOL1_PIN
 *   - Z_PROBE_SLED uses SOL1_PIN, when defined (unless EXT_SOLENOID is enabled)
 */
#if ANY(EXT_SOLENOID, MANUAL_SOLENOID_CONTROL, PARKING_EXTRUDER, SOLENOID_PROBE, Z_PROBE_SLED)
  #if PIN_EXISTS(SOL0) && (ANY(MANUAL_SOLENOID_CONTROL, PARKING_EXTRUDER) || ALL(EXT_SOLENOID, HAS_EXTRUDERS))
    #define HAS_SOLENOID_0 1
  #endif
  #if PIN_EXISTS(SOL1) && (ANY(MANUAL_SOLENOID_CONTROL, PARKING_EXTRUDER, SOLENOID_PROBE, Z_PROBE_SLED) || TERN0(EXT_SOLENOID, E_STEPPERS > 1))
    #define HAS_SOLENOID_1 1
  #endif
  #if PIN_EXISTS(SOL2) && (ENABLED(MANUAL_SOLENOID_CONTROL) || TERN0(EXT_SOLENOID, E_STEPPERS > 2))
    #define HAS_SOLENOID_2 2
  #endif
  #if PIN_EXISTS(SOL3) && (ENABLED(MANUAL_SOLENOID_CONTROL) || TERN0(EXT_SOLENOID, E_STEPPERS > 3))
    #define HAS_SOLENOID_3 3
  #endif
  #if PIN_EXISTS(SOL4) && (ENABLED(MANUAL_SOLENOID_CONTROL) || TERN0(EXT_SOLENOID, E_STEPPERS > 4))
    #define HAS_SOLENOID_4 4
  #endif
  #if PIN_EXISTS(SOL5) && (ENABLED(MANUAL_SOLENOID_CONTROL) || TERN0(EXT_SOLENOID, E_STEPPERS > 5))
    #define HAS_SOLENOID_5 5
  #endif
  #if PIN_EXISTS(SOL6) && (ENABLED(MANUAL_SOLENOID_CONTROL) || TERN0(EXT_SOLENOID, E_STEPPERS > 6))
    #define HAS_SOLENOID_6 6
  #endif
  #if PIN_EXISTS(SOL7) && (ENABLED(MANUAL_SOLENOID_CONTROL) || TERN0(EXT_SOLENOID, E_STEPPERS > 7))
    #define HAS_SOLENOID_7 7
  #endif
#endif

//
// Trinamic Stepper Drivers
//

#if HAS_TRINAMIC_CONFIG
  #if ANY(STEALTHCHOP_E, STEALTHCHOP_XY, STEALTHCHOP_Z, STEALTHCHOP_I, STEALTHCHOP_J, STEALTHCHOP_K, STEALTHCHOP_U, STEALTHCHOP_V, STEALTHCHOP_W)
    #define STEALTHCHOP_ENABLED 1
  #endif
  #if ANY(SENSORLESS_HOMING, SENSORLESS_PROBING)
    #define USE_SENSORLESS 1
  #endif

  // Disable Z axis sensorless homing if a probe is used to home the Z axis
  #if HOMING_Z_WITH_PROBE
    #undef Z_STALL_SENSITIVITY
    #undef Z2_STALL_SENSITIVITY
    #undef Z3_STALL_SENSITIVITY
    #undef Z4_STALL_SENSITIVITY
  #endif

  #if AXIS_IS_TMC(X)
    #if defined(X_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(X)
      #define X_SENSORLESS 1
      #if ENABLED(SPI_ENDSTOPS) && AXIS_HAS_SPI(X)
        #define X_SPI_SENSORLESS 1
      #endif
    #endif
    #if AXIS_HAS_STEALTHCHOP(X)
      #define X_HAS_STEALTHCHOP 1
    #endif
    #ifndef X_INTERPOLATE
      #define X_INTERPOLATE INTERPOLATE
    #endif
    #ifndef X_HOLD_MULTIPLIER
      #define X_HOLD_MULTIPLIER HOLD_MULTIPLIER
    #endif
    #ifndef X_SLAVE_ADDRESS
      #define X_SLAVE_ADDRESS 0
    #endif
  #endif
  #if AXIS_IS_TMC(X2)
    #if defined(X2_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(X2)
      #define X2_SENSORLESS 1
    #endif
    #if AXIS_HAS_STEALTHCHOP(X2)
      #define X2_HAS_STEALTHCHOP 1
    #endif
    #ifndef X2_INTERPOLATE
      #define X2_INTERPOLATE X_INTERPOLATE
    #endif
    #ifndef X2_HOLD_MULTIPLIER
      #define X2_HOLD_MULTIPLIER X_HOLD_MULTIPLIER
    #endif
    #ifndef X2_SLAVE_ADDRESS
      #define X2_SLAVE_ADDRESS 0
    #endif
  #endif

  #if AXIS_IS_TMC(Y)
    #if defined(Y_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(Y)
      #define Y_SENSORLESS 1
      #if ENABLED(SPI_ENDSTOPS) && AXIS_HAS_SPI(Y)
        #define Y_SPI_SENSORLESS 1
      #endif
    #endif
    #if AXIS_HAS_STEALTHCHOP(Y)
      #define Y_HAS_STEALTHCHOP 1
    #endif
    #ifndef Y_INTERPOLATE
      #define Y_INTERPOLATE INTERPOLATE
    #endif
    #ifndef Y_HOLD_MULTIPLIER
      #define Y_HOLD_MULTIPLIER HOLD_MULTIPLIER
    #endif
    #ifndef Y_SLAVE_ADDRESS
      #define Y_SLAVE_ADDRESS 0
    #endif
  #endif
  #if AXIS_IS_TMC(Y2)
    #if defined(Y2_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(Y2)
      #define Y2_SENSORLESS 1
    #endif
    #if AXIS_HAS_STEALTHCHOP(Y2)
      #define Y2_HAS_STEALTHCHOP 1
    #endif
    #ifndef Y2_INTERPOLATE
      #define Y2_INTERPOLATE Y_INTERPOLATE
    #endif
    #ifndef Y2_HOLD_MULTIPLIER
      #define Y2_HOLD_MULTIPLIER Y_HOLD_MULTIPLIER
    #endif
    #ifndef Y2_SLAVE_ADDRESS
      #define Y2_SLAVE_ADDRESS 0
    #endif
  #endif

  #if AXIS_IS_TMC(Z)
    #if defined(Z_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(Z)
      #define Z_SENSORLESS 1
      #if ENABLED(SPI_ENDSTOPS) && AXIS_HAS_SPI(Z)
        #define Z_SPI_SENSORLESS 1
      #endif
    #endif
    #if AXIS_HAS_STEALTHCHOP(Z)
      #define Z_HAS_STEALTHCHOP 1
    #endif
    #ifndef Z_INTERPOLATE
      #define Z_INTERPOLATE INTERPOLATE
    #endif
    #ifndef Z_HOLD_MULTIPLIER
      #define Z_HOLD_MULTIPLIER HOLD_MULTIPLIER
    #endif
    #ifndef Z_SLAVE_ADDRESS
      #define Z_SLAVE_ADDRESS 0
    #endif
  #endif
  #if NUM_Z_STEPPERS >= 2 && AXIS_IS_TMC(Z2)
    #if defined(Z2_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(Z2)
      #define Z2_SENSORLESS 1
    #endif
    #if AXIS_HAS_STEALTHCHOP(Z2)
      #define Z2_HAS_STEALTHCHOP 1
    #endif
    #ifndef Z2_INTERPOLATE
      #define Z2_INTERPOLATE Z_INTERPOLATE
    #endif
    #ifndef Z2_HOLD_MULTIPLIER
      #define Z2_HOLD_MULTIPLIER Z_HOLD_MULTIPLIER
    #endif
    #ifndef Z2_SLAVE_ADDRESS
      #define Z2_SLAVE_ADDRESS 0
    #endif
  #endif
  #if NUM_Z_STEPPERS >= 3 && AXIS_IS_TMC(Z3)
    #if defined(Z3_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(Z3)
      #define Z3_SENSORLESS 1
    #endif
    #if AXIS_HAS_STEALTHCHOP(Z3)
      #define Z3_HAS_STEALTHCHOP 1
    #endif
    #ifndef Z3_INTERPOLATE
      #define Z3_INTERPOLATE Z_INTERPOLATE
    #endif
    #ifndef Z3_HOLD_MULTIPLIER
      #define Z3_HOLD_MULTIPLIER Z_HOLD_MULTIPLIER
    #endif
    #ifndef Z3_SLAVE_ADDRESS
      #define Z3_SLAVE_ADDRESS 0
    #endif
  #endif
  #if NUM_Z_STEPPERS >= 4 && AXIS_IS_TMC(Z4)
    #if defined(Z4_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(Z4)
      #define Z4_SENSORLESS 1
    #endif
    #if AXIS_HAS_STEALTHCHOP(Z4)
      #define Z4_HAS_STEALTHCHOP 1
    #endif
    #ifndef Z4_INTERPOLATE
      #define Z4_INTERPOLATE Z_INTERPOLATE
    #endif
    #ifndef Z4_HOLD_MULTIPLIER
      #define Z4_HOLD_MULTIPLIER Z_HOLD_MULTIPLIER
    #endif
    #ifndef Z4_SLAVE_ADDRESS
      #define Z4_SLAVE_ADDRESS 0
    #endif
  #endif

  #if AXIS_IS_TMC(I)
    #if defined(I_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(I)
      #define I_SENSORLESS 1
      #if ENABLED(SPI_ENDSTOPS) && AXIS_HAS_SPI(I)
        #define I_SPI_SENSORLESS 1
      #endif
    #endif
    #if AXIS_HAS_STEALTHCHOP(I)
      #define I_HAS_STEALTHCHOP 1
    #endif
    #ifndef I_INTERPOLATE
      #define I_INTERPOLATE INTERPOLATE
    #endif
    #ifndef I_HOLD_MULTIPLIER
      #define I_HOLD_MULTIPLIER HOLD_MULTIPLIER
    #endif
    #ifndef I_SLAVE_ADDRESS
      #define I_SLAVE_ADDRESS 0
    #endif
  #endif

  #if AXIS_IS_TMC(J)
    #if defined(J_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(J)
      #define J_SENSORLESS 1
      #if ENABLED(SPI_ENDSTOPS) && AXIS_HAS_SPI(J)
        #define J_SPI_SENSORLESS 1
      #endif
    #endif
    #if AXIS_HAS_STEALTHCHOP(J)
      #define J_HAS_STEALTHCHOP 1
    #endif
    #ifndef J_INTERPOLATE
      #define J_INTERPOLATE INTERPOLATE
    #endif
    #ifndef J_HOLD_MULTIPLIER
      #define J_HOLD_MULTIPLIER HOLD_MULTIPLIER
    #endif
    #ifndef J_SLAVE_ADDRESS
      #define J_SLAVE_ADDRESS 0
    #endif
  #endif

  #if AXIS_IS_TMC(K)
    #if defined(K_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(K)
      #define K_SENSORLESS 1
      #if ENABLED(SPI_ENDSTOPS) && AXIS_HAS_SPI(K)
        #define K_SPI_SENSORLESS 1
      #endif
    #endif
    #if AXIS_HAS_STEALTHCHOP(K)
      #define K_HAS_STEALTHCHOP 1
    #endif
    #ifndef K_INTERPOLATE
      #define K_INTERPOLATE INTERPOLATE
    #endif
    #ifndef K_HOLD_MULTIPLIER
      #define K_HOLD_MULTIPLIER HOLD_MULTIPLIER
    #endif
    #ifndef K_SLAVE_ADDRESS
      #define K_SLAVE_ADDRESS 0
    #endif
  #endif

  #if AXIS_IS_TMC(U)
    #if defined(U_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(U)
      #define U_SENSORLESS 1
      #if ENABLED(SPI_ENDSTOPS) && AXIS_HAS_SPI(U)
        #define U_SPI_SENSORLESS 1
      #endif
    #endif
    #if AXIS_HAS_STEALTHCHOP(U)
      #define U_HAS_STEALTHCHOP 1
    #endif
    #ifndef U_INTERPOLATE
      #define U_INTERPOLATE INTERPOLATE
    #endif
    #ifndef U_HOLD_MULTIPLIER
      #define U_HOLD_MULTIPLIER HOLD_MULTIPLIER
    #endif
    #ifndef U_SLAVE_ADDRESS
      #define U_SLAVE_ADDRESS 0
    #endif
  #endif

  #if AXIS_IS_TMC(V)
    #if defined(V_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(V)
      #define V_SENSORLESS 1
      #if ENABLED(SPI_ENDSTOPS) && AXIS_HAS_SPI(V)
        #define V_SPI_SENSORLESS 1
      #endif
    #endif
    #if AXIS_HAS_STEALTHCHOP(V)
      #define V_HAS_STEALTHCHOP 1
    #endif
    #ifndef V_INTERPOLATE
      #define V_INTERPOLATE INTERPOLATE
    #endif
    #ifndef V_HOLD_MULTIPLIER
      #define V_HOLD_MULTIPLIER HOLD_MULTIPLIER
    #endif
    #ifndef V_SLAVE_ADDRESS
      #define V_SLAVE_ADDRESS 0
    #endif
  #endif

  #if AXIS_IS_TMC(W)
    #if defined(W_STALL_SENSITIVITY) && AXIS_HAS_STALLGUARD(W)
      #define W_SENSORLESS 1
      #if ENABLED(SPI_ENDSTOPS) && AXIS_HAS_SPI(W)
        #define W_SPI_SENSORLESS 1
      #endif
    #endif
    #if AXIS_HAS_STEALTHCHOP(W)
      #define W_HAS_STEALTHCHOP 1
    #endif
    #ifndef W_INTERPOLATE
      #define W_INTERPOLATE INTERPOLATE
    #endif
    #ifndef W_HOLD_MULTIPLIER
      #define W_HOLD_MULTIPLIER HOLD_MULTIPLIER
    #endif
    #ifndef W_SLAVE_ADDRESS
      #define W_SLAVE_ADDRESS 0
    #endif
  #endif

  #if AXIS_IS_TMC(E0)
    #if AXIS_HAS_STEALTHCHOP(E0)
      #define E0_HAS_STEALTHCHOP 1
    #endif
    #ifndef E0_INTERPOLATE
      #define E0_INTERPOLATE INTERPOLATE
    #endif
    #ifndef E0_HOLD_MULTIPLIER
      #define E0_HOLD_MULTIPLIER HOLD_MULTIPLIER
    #endif
    #ifndef E0_SLAVE_ADDRESS
      #define E0_SLAVE_ADDRESS 0
    #endif
  #endif
  #if AXIS_IS_TMC(E1)
    #if AXIS_HAS_STEALTHCHOP(E1)
      #define E1_HAS_STEALTHCHOP 1
    #endif
    #ifndef E1_INTERPOLATE
      #define E1_INTERPOLATE E0_INTERPOLATE
    #endif
    #ifndef E1_HOLD_MULTIPLIER
      #define E1_HOLD_MULTIPLIER E0_HOLD_MULTIPLIER
    #endif
    #ifndef E1_SLAVE_ADDRESS
      #define E1_SLAVE_ADDRESS 0
    #endif
  #endif
  #if AXIS_IS_TMC(E2)
    #if AXIS_HAS_STEALTHCHOP(E2)
      #define E2_HAS_STEALTHCHOP 1
    #endif
    #ifndef E2_INTERPOLATE
      #define E2_INTERPOLATE E0_INTERPOLATE
    #endif
    #ifndef E2_HOLD_MULTIPLIER
      #define E2_HOLD_MULTIPLIER E0_HOLD_MULTIPLIER
    #endif
    #ifndef E2_SLAVE_ADDRESS
      #define E2_SLAVE_ADDRESS 0
    #endif
  #endif
  #if AXIS_IS_TMC(E3)
    #if AXIS_HAS_STEALTHCHOP(E3)
      #define E3_HAS_STEALTHCHOP 1
    #endif
    #ifndef E3_INTERPOLATE
      #define E3_INTERPOLATE E0_INTERPOLATE
    #endif
    #ifndef E3_HOLD_MULTIPLIER
      #define E3_HOLD_MULTIPLIER E0_HOLD_MULTIPLIER
    #endif
    #ifndef E3_SLAVE_ADDRESS
      #define E3_SLAVE_ADDRESS 0
    #endif
  #endif
  #if AXIS_IS_TMC(E4)
    #if AXIS_HAS_STEALTHCHOP(E4)
      #define E4_HAS_STEALTHCHOP 1
    #endif
    #ifndef E4_INTERPOLATE
      #define E4_INTERPOLATE E0_INTERPOLATE
    #endif
    #ifndef E4_HOLD_MULTIPLIER
      #define E4_HOLD_MULTIPLIER E0_HOLD_MULTIPLIER
    #endif
    #ifndef E4_SLAVE_ADDRESS
      #define E4_SLAVE_ADDRESS 0
    #endif
  #endif
  #if AXIS_IS_TMC(E5)
    #if AXIS_HAS_STEALTHCHOP(E5)
      #define E5_HAS_STEALTHCHOP 1
    #endif
    #ifndef E5_INTERPOLATE
      #define E5_INTERPOLATE E0_INTERPOLATE
    #endif
    #ifndef E5_HOLD_MULTIPLIER
      #define E5_HOLD_MULTIPLIER E0_HOLD_MULTIPLIER
    #endif
    #ifndef E5_SLAVE_ADDRESS
      #define E5_SLAVE_ADDRESS 0
    #endif
  #endif
  #if AXIS_IS_TMC(E6)
    #if AXIS_HAS_STEALTHCHOP(E6)
      #define E6_HAS_STEALTHCHOP 1
    #endif
    #ifndef E6_INTERPOLATE
      #define E6_INTERPOLATE E0_INTERPOLATE
    #endif
    #ifndef E6_HOLD_MULTIPLIER
      #define E6_HOLD_MULTIPLIER E0_HOLD_MULTIPLIER
    #endif
    #ifndef E6_SLAVE_ADDRESS
      #define E6_SLAVE_ADDRESS 0
    #endif
  #endif
  #if AXIS_IS_TMC(E7)
    #if AXIS_HAS_STEALTHCHOP(E7)
      #define E7_HAS_STEALTHCHOP 1
    #endif
    #ifndef E7_INTERPOLATE
      #define E7_INTERPOLATE E0_INTERPOLATE
    #endif
    #ifndef E7_HOLD_MULTIPLIER
      #define E7_HOLD_MULTIPLIER E0_HOLD_MULTIPLIER
    #endif
    #ifndef E7_SLAVE_ADDRESS
      #define E7_SLAVE_ADDRESS 0
    #endif
  #endif
#endif // HAS_TRINAMIC_CONFIG

#if ANY_AXIS_HAS(HW_SERIAL)
  #define HAS_TMC_HW_SERIAL 1
#endif
#if ANY_AXIS_HAS(SW_SERIAL)
  #define HAS_TMC_SW_SERIAL 1
#endif
#ifndef SERIAL_FLOAT_PRECISION
  #define SERIAL_FLOAT_PRECISION 2
#endif

#if DISABLED(SENSORLESS_HOMING)
  #undef SENSORLESS_BACKOFF_MM
#endif

//
// Set USING_HW_SERIALn flags for used Serial Ports
//

// Flag the indexed hardware serial ports in use
#define SERIAL_IN_USE(N) (   (defined(SERIAL_PORT)       && N == SERIAL_PORT) \
                          || (defined(SERIAL_PORT_2)     && N == SERIAL_PORT_2) \
                          || (defined(SERIAL_PORT_3)     && N == SERIAL_PORT_3) \
                          || (defined(MMU_SERIAL_PORT)   && N == MMU_SERIAL_PORT) \
                          || (defined(LCD_SERIAL_PORT)   && N == LCD_SERIAL_PORT) \
                          || (defined(RS485_SERIAL_PORT) && N == RS485_SERIAL_PORT) )

// Flag the named hardware serial ports in use
#define TMC_UART_IS(A,N) (defined(A##_HARDWARE_SERIAL) && (CAT(HW_,A##_HARDWARE_SERIAL) == HW_Serial##N || CAT(HW_,A##_HARDWARE_SERIAL) == HW_MSerial##N))
#define ANY_SERIAL_IS(N) (  SERIAL_IN_USE(N) \
                         || TMC_UART_IS(X,  N) || TMC_UART_IS(Y , N) || TMC_UART_IS(Z , N) \
                         || TMC_UART_IS(I,  N) || TMC_UART_IS(J , N) || TMC_UART_IS(K , N) \
                         || TMC_UART_IS(U,  N) || TMC_UART_IS(V , N) || TMC_UART_IS(W , N) \
                         || TMC_UART_IS(X2, N) || TMC_UART_IS(Y2, N) || TMC_UART_IS(Z2, N) || TMC_UART_IS(Z3, N) || TMC_UART_IS(Z4, N) \
                         || TMC_UART_IS(E0, N) || TMC_UART_IS(E1, N) || TMC_UART_IS(E2, N) || TMC_UART_IS(E3, N) || TMC_UART_IS(E4, N) )

#define HW_Serial    501
#define HW_Serial0   502
#define HW_Serial1   503
#define HW_Serial2   504
#define HW_Serial3   505
#define HW_Serial4   506
#define HW_Serial5   507
#define HW_Serial6   508
#define HW_MSerial0  509
#define HW_MSerial1  510
#define HW_MSerial2  511
#define HW_MSerial3  512
#define HW_MSerial4  513
#define HW_MSerial5  514
#define HW_MSerial6  515
#define HW_MSerial7  516
#define HW_MSerial8  517
#define HW_MSerial9  518
#define HW_MSerial10 519

#if SERIAL_IN_USE(-1)
  #define USING_HW_SERIALUSB 1
#endif
#if ANY_SERIAL_IS(0)
  #define USING_HW_SERIAL0 1
#endif
#if ANY_SERIAL_IS(1)
  #define USING_HW_SERIAL1 1
#endif
#if ANY_SERIAL_IS(2)
  #define USING_HW_SERIAL2 1
#endif
#if ANY_SERIAL_IS(3)
  #define USING_HW_SERIAL3 1
#endif
#if ANY_SERIAL_IS(4)
  #define USING_HW_SERIAL4 1
#endif
#if ANY_SERIAL_IS(5)
  #define USING_HW_SERIAL5 1
#endif
#if ANY_SERIAL_IS(6)
  #define USING_HW_SERIAL6 1
#endif
#if ANY_SERIAL_IS(7)
  #define USING_HW_SERIAL7 1
#endif
#if ANY_SERIAL_IS(8)
  #define USING_HW_SERIAL8 1
#endif
#if ANY_SERIAL_IS(9)
  #define USING_HW_SERIAL9 1
#endif
#if ANY_SERIAL_IS(10)
  #define USING_HW_SERIAL10 1
#endif

#undef HW_Serial
#undef HW_Serial0
#undef HW_Serial1
#undef HW_Serial2
#undef HW_Serial3
#undef HW_Serial4
#undef HW_Serial5
#undef HW_Serial6
#undef HW_MSerial0
#undef HW_MSerial1
#undef HW_MSerial2
#undef HW_MSerial3
#undef HW_MSerial4
#undef HW_MSerial5
#undef HW_MSerial6
#undef HW_MSerial7
#undef HW_MSerial8
#undef HW_MSerial9
#undef HW_MSerial10

#undef _SERIAL_ID
#undef _TMC_UART_IS
#undef TMC_UART_IS
#undef ANY_SERIAL_IS

#if defined(__AVR_ARCH__) && defined(TMC_SPI_MISO) && defined(TMC_SPI_MOSI) && defined(TMC_SPI_SCK)
  // Check that the pins are the solitary supported SPI hardware pins of the (AVR) platform.
  // Otherwise we are forced to enable software SPI.
  #if TMC_SPI_MISO != MISO || TMC_SPI_MOSI != MOSI || TMC_SPI_SCK != SCK
    #define TMC_USE_SW_SPI
  #endif
#endif

// Clean up unused ESP_WIFI pins
#ifdef ESP_WIFI_MODULE_COM
  #if !SERIAL_IN_USE(ESP_WIFI_MODULE_COM)
    #undef ESP_WIFI_MODULE_COM
    #undef ESP_WIFI_MODULE_BAUDRATE
    #undef ESP_WIFI_MODULE_RESET_PIN
    #undef ESP_WIFI_MODULE_ENABLE_PIN
    #undef ESP_WIFI_MODULE_TXD_PIN
    #undef ESP_WIFI_MODULE_RXD_PIN
    #undef ESP_WIFI_MODULE_GPIO0_PIN
    #undef ESP_WIFI_MODULE_GPIO2_PIN
    #undef ESP_WIFI_MODULE_GPIO4_PIN
  #endif
#endif

/**
 * Endstop and probe flags
 * - Set USE_(AXIS)_(MIN|MAX) flags for each used endstop that has a pin, including those for DIAG0 state.
 *   - Note: Dual X Carriage uses "X" and "X2" steppers, but X_MIN and X_MAX endstop states (i.e., not X2_MAX).
 * - Set a HAS_(AXIS)_(MIN|MAX)_STATE flag for each endstop that has a state, including SPI Sensorless which don't use a pin.
 * - Set a HAS_(AXIS)_STATE flag for each axis that has at least one state.
 * - Consider (AXIS)_SAFETY_STOP for the case where the axis has a second endstop.
 *   Currently this must be distinct, but we can add a mechanism to use the same pin for sensorless
 *   or switches wired to the same pin, or for the single SPI stall state on the axis.
 */
#define _ANY_STOP(A,N,M) ANY(A##_HOME_TO_##M, A##N##_SAFETY_STOP)
#define _USE_STOP(A,N,M,C) ((_ANY_STOP(A,N,M) || (C+0)) && PIN_EXISTS(A##N##_##M) && !A##_SPI_SENSORLESS)
#define _HAS_STATE(A,N,M) (USE_##A##N##_##M || (_ANY_STOP(A,N,M) && A##_SPI_SENSORLESS))

#if _USE_STOP(X,,MIN,)
  #define USE_X_MIN 1
#endif
#if _USE_STOP(X,,MAX,ENABLED(DUAL_X_CARRIAGE))
  #define USE_X_MAX 1
#endif
#if _HAS_STATE(X,,MIN)
  #define HAS_X_MIN_STATE 1
#endif
#if _HAS_STATE(X,,MAX)
  #define HAS_X_MAX_STATE 1
#endif
#if HAS_X_MIN_STATE || HAS_X_MAX_STATE
  #define HAS_X_STATE 1
#endif

#if _USE_STOP(Y,,MIN,)
  #define USE_Y_MIN 1
#endif
#if _USE_STOP(Y,,MAX,)
  #define USE_Y_MAX 1
#endif
#if _HAS_STATE(Y,,MIN)
  #define HAS_Y_MIN_STATE 1
#endif
#if _HAS_STATE(Y,,MAX)
  #define HAS_Y_MAX_STATE 1
#endif
#if HAS_Y_MIN_STATE || HAS_Y_MAX_STATE
  #define HAS_Y_STATE 1
#endif

#if _USE_STOP(Z,,MIN,ENABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN)) && (DISABLED(USE_PROBE_FOR_Z_HOMING) || ENABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN))
  #define USE_Z_MIN 1
#endif
#if _USE_STOP(Z,,MAX,)
  #define USE_Z_MAX 1
#endif
#if _HAS_STATE(Z,,MIN)
  #define HAS_Z_MIN_STATE 1
#endif
#if _HAS_STATE(Z,,MAX)
  #define HAS_Z_MAX_STATE 1
#endif
#if HAS_Z_MIN_STATE || HAS_Z_MAX_STATE
  #define HAS_Z_STATE 1
#endif

#if _USE_STOP(I,,MIN,)
  #define USE_I_MIN 1
#endif
#if _USE_STOP(I,,MAX,)
  #define USE_I_MAX 1
#endif
#if _HAS_STATE(I,,MIN)
  #define HAS_I_MIN_STATE 1
#endif
#if _HAS_STATE(I,,MAX)
  #define HAS_I_MAX_STATE 1
#endif
#if HAS_I_MIN_STATE || HAS_I_MAX_STATE
  #define HAS_I_STATE 1
#endif

#if _USE_STOP(J,,MIN,)
  #define USE_J_MIN 1
#endif
#if _USE_STOP(J,,MAX,)
  #define USE_J_MAX 1
#endif
#if _HAS_STATE(J,,MIN)
  #define HAS_J_MIN_STATE 1
#endif
#if _HAS_STATE(J,,MAX)
  #define HAS_J_MAX_STATE 1
#endif
#if HAS_J_MIN_STATE || HAS_J_MAX_STATE
  #define HAS_J_STATE 1
#endif

#if _USE_STOP(K,,MIN,)
  #define USE_K_MIN 1
#endif
#if _USE_STOP(K,,MAX,)
  #define USE_K_MAX 1
#endif
#if _HAS_STATE(K,,MIN)
  #define HAS_K_MIN_STATE 1
#endif
#if _HAS_STATE(K,,MAX)
  #define HAS_K_MAX_STATE 1
#endif
#if HAS_K_MIN_STATE || HAS_K_MAX_STATE
  #define HAS_K_STATE 1
#endif

#if _USE_STOP(U,,MIN,)
  #define USE_U_MIN 1
#endif
#if _USE_STOP(U,,MAX,)
  #define USE_U_MAX 1
#endif
#if _HAS_STATE(U,,MIN)
  #define HAS_U_MIN_STATE 1
#endif
#if _HAS_STATE(U,,MAX)
  #define HAS_U_MAX_STATE 1
#endif
#if HAS_U_MIN_STATE || HAS_U_MAX_STATE
  #define HAS_U_STATE 1
#endif

#if _USE_STOP(V,,MIN,)
  #define USE_V_MIN 1
#endif
#if _USE_STOP(V,,MAX,)
  #define USE_V_MAX 1
#endif
#if _HAS_STATE(V,,MIN)
  #define HAS_V_MIN_STATE 1
#endif
#if _HAS_STATE(V,,MAX)
  #define HAS_V_MAX_STATE 1
#endif
#if HAS_V_MIN_STATE || HAS_V_MAX_STATE
  #define HAS_V_STATE 1
#endif

#if _USE_STOP(W,,MIN,)
  #define USE_W_MIN 1
#endif
#if _USE_STOP(W,,MAX,)
  #define USE_W_MAX 1
#endif
#if _HAS_STATE(W,,MIN)
  #define HAS_W_MIN_STATE 1
#endif
#if _HAS_STATE(W,,MAX)
  #define HAS_W_MAX_STATE 1
#endif
#if HAS_W_MIN_STATE || HAS_W_MAX_STATE
  #define HAS_W_STATE 1
#endif

#if ENABLED(X_DUAL_ENDSTOPS)
  #if _USE_STOP(X,2,MIN,)
    #define USE_X2_MIN 1
  #endif
  #if _USE_STOP(X,2,MAX,)
    #define USE_X2_MAX 1
  #endif
  #if _HAS_STATE(X,2,MIN) || HAS_X_MIN_STATE
    #define HAS_X2_MIN_STATE 1
  #endif
  #if _HAS_STATE(X,2,MAX) || HAS_X_MAX_STATE
    #define HAS_X2_MAX_STATE 1
  #endif
  #if HAS_X2_MIN_STATE || HAS_X2_MAX_STATE
    #define HAS_X2_STATE 1
  #endif
#endif

#if ENABLED(Y_DUAL_ENDSTOPS)
  #if _USE_STOP(Y,2,MIN,)
    #define USE_Y2_MIN 1
  #endif
  #if _USE_STOP(Y,2,MAX,)
    #define USE_Y2_MAX 1
  #endif
  #if _HAS_STATE(Y,2,MIN) || HAS_Y_MIN_STATE
    #define HAS_Y2_MIN_STATE 1
  #endif
  #if _HAS_STATE(Y,2,MAX) || HAS_Y_MAX_STATE
    #define HAS_Y2_MAX_STATE 1
  #endif
  #if HAS_Y2_MIN_STATE || HAS_Y2_MAX_STATE
    #define HAS_Y2_STATE 1
  #endif
#endif

#if ENABLED(Z_MULTI_ENDSTOPS)
  #if _USE_STOP(Z,2,MIN,)
    #define USE_Z2_MIN 1
  #endif
  #if _USE_STOP(Z,2,MAX,)
    #define USE_Z2_MAX 1
  #endif
  #if _HAS_STATE(Z,2,MIN) || HAS_Z_MIN_STATE
    #define HAS_Z2_MIN_STATE 1
  #endif
  #if _HAS_STATE(Z,2,MAX) || HAS_Z_MAX_STATE
    #define HAS_Z2_MAX_STATE 1
  #endif
  #if HAS_Z2_MIN_STATE || HAS_Z2_MAX_STATE
    #define HAS_Z2_STATE 1
  #endif
  #if NUM_Z_STEPPERS >= 3
    #if _USE_STOP(Z,3,MIN,)
      #define USE_Z3_MIN 1
    #endif
    #if _USE_STOP(Z,3,MAX,)
      #define USE_Z3_MAX 1
    #endif
    #if _HAS_STATE(Z,3,MIN) || HAS_Z_MIN_STATE
      #define HAS_Z3_MIN_STATE 1
    #endif
    #if _HAS_STATE(Z,3,MAX) || HAS_Z_MAX_STATE
      #define HAS_Z3_MAX_STATE 1
    #endif
    #if HAS_Z3_MIN_STATE || HAS_Z3_MAX_STATE
      #define HAS_Z3_STATE 1
    #endif
  #endif
  #if NUM_Z_STEPPERS >= 4
    #if _USE_STOP(Z,4,MIN,)
      #define USE_Z4_MIN 1
    #endif
    #if _USE_STOP(Z,4,MAX,)
      #define USE_Z4_MAX 1
    #endif
    #if _HAS_STATE(Z,4,MIN) || HAS_Z_MIN_STATE
      #define HAS_Z4_MIN_STATE 1
    #endif
    #if _HAS_STATE(Z,4,MAX) || HAS_Z_MAX_STATE
      #define HAS_Z4_MAX_STATE 1
    #endif
    #if HAS_Z4_MIN_STATE || HAS_Z4_MAX_STATE
      #define HAS_Z4_STATE 1
    #endif
  #endif
#endif

#if NEED_Z_MIN_PROBE_PIN && PIN_EXISTS(Z_MIN_PROBE)
  #define USE_Z_MIN_PROBE 1
#endif
#if HAS_REAL_BED_PROBE
  #define HAS_Z_PROBE_STATE 1
#endif

#if PIN_EXISTS(CALIBRATION)
  #define USE_CALIBRATION 1
  #define HAS_CALIBRATION_STATE 1
#endif

#undef _ANY_STOP
#undef _USE_STOP
#undef _HAS_STATE

/**
 * Set ENDSTOPPULLUPS for active endstop switches
 */
#if ENABLED(ENDSTOPPULLUPS)
  #if USE_X_MIN
    #define ENDSTOPPULLUP_XMIN
  #endif
  #if USE_X_MAX
    #define ENDSTOPPULLUP_XMAX
  #endif
  #if USE_Y_MIN
    #define ENDSTOPPULLUP_YMIN
  #endif
  #if USE_Y_MAX
    #define ENDSTOPPULLUP_YMAX
  #endif
  #if USE_Z_MIN
    #define ENDSTOPPULLUP_ZMIN
  #endif
  #if USE_Z_MAX
    #define ENDSTOPPULLUP_ZMAX
  #endif
  #if USE_I_MIN
    #define ENDSTOPPULLUP_IMIN
  #endif
  #if USE_I_MAX
    #define ENDSTOPPULLUP_IMAX
  #endif
  #if USE_J_MIN
    #define ENDSTOPPULLUP_JMIN
  #endif
  #if USE_J_MAX
    #define ENDSTOPPULLUP_JMAX
  #endif
  #if USE_K_MIN
    #define ENDSTOPPULLUP_KMIN
  #endif
  #if USE_K_MAX
    #define ENDSTOPPULLUP_KMAX
  #endif
  #if USE_U_MIN
    #define ENDSTOPPULLUP_UMIN
  #endif
  #if USE_U_MAX
    #define ENDSTOPPULLUP_UMAX
  #endif
  #if USE_V_MIN
    #define ENDSTOPPULLUP_VMIN
  #endif
  #if USE_V_MAX
    #define ENDSTOPPULLUP_VMAX
  #endif
  #if USE_W_MIN
    #define ENDSTOPPULLUP_WMIN
  #endif
  #if USE_W_MAX
    #define ENDSTOPPULLUP_WMAX
  #endif
#endif

/**
 * Set ENDSTOPPULLDOWNS for active endstop switches
 */
#if ENABLED(ENDSTOPPULLDOWNS)
  #if USE_X_MIN
    #define ENDSTOPPULLDOWN_XMIN
  #endif
  #if USE_X_MAX
    #define ENDSTOPPULLDOWN_XMAX
  #endif
  #if USE_Y_MIN
    #define ENDSTOPPULLDOWN_YMIN
  #endif
  #if USE_Y_MAX
    #define ENDSTOPPULLDOWN_YMAX
  #endif
  #if USE_Z_MIN
    #define ENDSTOPPULLDOWN_ZMIN
  #endif
  #if USE_Z_MAX
    #define ENDSTOPPULLDOWN_ZMAX
  #endif
  #if USE_I_MIN
    #define ENDSTOPPULLDOWN_IMIN
  #endif
  #if USE_I_MAX
    #define ENDSTOPPULLDOWN_IMAX
  #endif
  #if USE_J_MIN
    #define ENDSTOPPULLDOWN_JMIN
  #endif
  #if USE_J_MAX
    #define ENDSTOPPULLDOWN_JMAX
  #endif
  #if USE_K_MIN
    #define ENDSTOPPULLDOWN_KMIN
  #endif
  #if USE_K_MAX
    #define ENDSTOPPULLDOWN_KMAX
  #endif
  #if USE_U_MIN
    #define ENDSTOPPULLDOWN_UMIN
  #endif
  #if USE_U_MAX
    #define ENDSTOPPULLDOWN_UMAX
  #endif
  #if USE_V_MIN
    #define ENDSTOPPULLDOWN_VMIN
  #endif
  #if USE_V_MAX
    #define ENDSTOPPULLDOWN_VMAX
  #endif
  #if USE_W_MIN
    #define ENDSTOPPULLDOWN_WMIN
  #endif
  #if USE_W_MAX
    #define ENDSTOPPULLDOWN_WMAX
  #endif
#endif

#if !USE_X_MIN
  #undef ENDSTOPPULLUP_XMIN
  #undef ENDSTOPPULLDOWN_XMIN
#endif
#if !USE_X_MAX
  #undef ENDSTOPPULLUP_XMAX
  #undef ENDSTOPPULLDOWN_XMAX
#endif
#if !USE_Y_MIN
  #undef ENDSTOPPULLUP_YMIN
  #undef ENDSTOPPULLDOWN_YMIN
#endif
#if !USE_Y_MAX
  #undef ENDSTOPPULLUP_YMAX
  #undef ENDSTOPPULLDOWN_YMAX
#endif
#if !USE_Z_MIN
  #undef ENDSTOPPULLUP_ZMIN
  #undef ENDSTOPPULLDOWN_ZMIN
#endif
#if !USE_Z_MAX
  #undef ENDSTOPPULLUP_ZMAX
  #undef ENDSTOPPULLDOWN_ZMAX
#endif
#if !USE_I_MIN
  #undef ENDSTOPPULLUP_IMIN
  #undef ENDSTOPPULLDOWN_IMIN
#endif
#if !USE_I_MAX
  #undef ENDSTOPPULLUP_IMAX
  #undef ENDSTOPPULLDOWN_IMAX
#endif
#if !USE_J_MIN
  #undef ENDSTOPPULLUP_JMIN
  #undef ENDSTOPPULLDOWN_JMIN
#endif
#if !USE_J_MAX
  #undef ENDSTOPPULLUP_JMAX
  #undef ENDSTOPPULLDOWN_JMAX
#endif
#if !USE_K_MIN
  #undef ENDSTOPPULLUP_KMIN
  #undef ENDSTOPPULLDOWN_KMIN
#endif
#if !USE_K_MAX
  #undef ENDSTOPPULLUP_KMAX
  #undef ENDSTOPPULLDOWN_KMAX
#endif
#if !USE_U_MIN
  #undef ENDSTOPPULLUP_UMIN
  #undef ENDSTOPPULLDOWN_UMIN
#endif
#if !USE_U_MAX
  #undef ENDSTOPPULLUP_UMAX
  #undef ENDSTOPPULLDOWN_UMAX
#endif
#if !USE_V_MIN
  #undef ENDSTOPPULLUP_VMIN
  #undef ENDSTOPPULLDOWN_VMIN
#endif
#if !USE_V_MAX
  #undef ENDSTOPPULLUP_VMAX
  #undef ENDSTOPPULLDOWN_VMAX
#endif
#if !USE_W_MIN
  #undef ENDSTOPPULLUP_WMIN
  #undef ENDSTOPPULLDOWN_WMIN
#endif
#if !USE_W_MAX
  #undef ENDSTOPPULLUP_WMAX
  #undef ENDSTOPPULLDOWN_WMAX
#endif

#define PCAT(P) P##_PIN
#define NEED_HIT_STATE(P) (USE_##P || (PIN_EXISTS(P) && ((defined(X2_STOP_PIN) && X2_STOP_PIN == PCAT(P)) || (defined(Y2_STOP_PIN) && Y2_STOP_PIN == PCAT(P)) || (defined(Z2_STOP_PIN) && Z2_STOP_PIN == PCAT(P)) || (defined(Z3_STOP_PIN) && Z3_STOP_PIN == PCAT(P)) || (defined(Z4_STOP_PIN) && Z4_STOP_PIN == PCAT(P)))))
#if !NEED_HIT_STATE(X_MIN)
  #undef X_MIN_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(X_MAX)
  #undef X_MAX_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(Y_MIN)
  #undef Y_MIN_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(Y_MAX)
  #undef Y_MAX_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(Z_MIN)
  #undef Z_MIN_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(Z_MAX)
  #undef Z_MAX_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(I_MIN)
  #undef I_MIN_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(I_MAX)
  #undef I_MAX_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(J_MIN)
  #undef J_MIN_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(J_MAX)
  #undef J_MAX_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(K_MIN)
  #undef K_MIN_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(K_MAX)
  #undef K_MAX_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(U_MIN)
  #undef U_MIN_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(U_MAX)
  #undef U_MAX_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(V_MIN)
  #undef V_MIN_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(V_MAX)
  #undef V_MAX_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(W_MIN)
  #undef W_MIN_ENDSTOP_HIT_STATE
#endif
#if !NEED_HIT_STATE(W_MAX)
  #undef W_MAX_ENDSTOP_HIT_STATE
#endif
#undef NEED_HIT_STATE
#undef PCAT

//
// ADC Temp Sensors (Thermistor or Thermocouple with amplifier ADC interface)
//
#define HAS_ADC_TEST(P) (TEMP_SENSOR(P) && PIN_EXISTS(TEMP_##P) && !TEMP_SENSOR_IS_MAX_TC(P) && !TEMP_SENSOR_##P##_IS_DUMMY)
#if HOTENDS > 0 && HAS_ADC_TEST(0)
  #define HAS_TEMP_ADC_0 1
#endif
#if HOTENDS > 1 && HAS_ADC_TEST(1)
  #define HAS_TEMP_ADC_1 1
#endif
#if HOTENDS > 2 && HAS_ADC_TEST(2)
  #define HAS_TEMP_ADC_2 1
#endif
#if HOTENDS > 3 && HAS_ADC_TEST(3)
  #define HAS_TEMP_ADC_3 1
#endif
#if HOTENDS > 4 && HAS_ADC_TEST(4)
  #define HAS_TEMP_ADC_4 1
#endif
#if HOTENDS > 5 && HAS_ADC_TEST(5)
  #define HAS_TEMP_ADC_5 1
#endif
#if HOTENDS > 6 && HAS_ADC_TEST(6)
  #define HAS_TEMP_ADC_6 1
#endif
#if HOTENDS > 7 && HAS_ADC_TEST(7)
  #define HAS_TEMP_ADC_7 1
#endif
#if TEMP_SENSOR_BED
  #define HAS_HEATED_BED 1
  #if HAS_ADC_TEST(BED)
    #define HAS_TEMP_ADC_BED 1
  #endif
#endif
#if HAS_ADC_TEST(PROBE)
  #define HAS_TEMP_ADC_PROBE 1
#endif
#if HAS_ADC_TEST(CHAMBER)
  #define HAS_TEMP_ADC_CHAMBER 1
#endif
#if HAS_ADC_TEST(COOLER)
  #define HAS_TEMP_ADC_COOLER 1
#endif
#if HAS_ADC_TEST(BOARD)
  #define HAS_TEMP_ADC_BOARD 1
#endif
#if HAS_ADC_TEST(SOC)
  #define HAS_TEMP_ADC_SOC 1
#endif
#if HAS_ADC_TEST(REDUNDANT)
  #define HAS_TEMP_ADC_REDUNDANT 1
#endif

#define HAS_TEMP(N) (TEMP_SENSOR_IS_MAX_TC(N) || HAS_TEMP_ADC_##N || TEMP_SENSOR_##N##_IS_DUMMY)
#if HAS_HOTEND && HAS_TEMP(0)
  #define HAS_TEMP_HOTEND 1
#endif
#if HAS_TEMP(BED)
  #define HAS_TEMP_BED 1
#endif
#if HAS_TEMP(CHAMBER)
  #define HAS_TEMP_CHAMBER 1
#endif
#if HAS_TEMP(PROBE)
  #define HAS_TEMP_PROBE 1
#endif
#if HAS_TEMP(COOLER)
  #define HAS_TEMP_COOLER 1
#endif
#if HAS_TEMP(BOARD)
  #define HAS_TEMP_BOARD 1
#endif
#if HAS_TEMP(SOC)
  #define HAS_TEMP_SOC 1
#endif
#if HAS_TEMP(REDUNDANT)
  #define HAS_TEMP_REDUNDANT 1
#endif

#if ENABLED(JOYSTICK)
  #if PIN_EXISTS(JOY_X)
    #define HAS_JOY_ADC_X 1
  #endif
  #if PIN_EXISTS(JOY_Y)
    #define HAS_JOY_ADC_Y 1
  #endif
  #if PIN_EXISTS(JOY_Z)
    #define HAS_JOY_ADC_Z 1
  #endif
  #if PIN_EXISTS(JOY_EN)
    #define HAS_JOY_ADC_EN 1
  #endif
#endif

// Heaters
#if PIN_EXISTS(HEATER_0)
  #define HAS_HEATER_0 1
#endif
#if PIN_EXISTS(HEATER_1)
  #define HAS_HEATER_1 1
#endif
#if PIN_EXISTS(HEATER_2)
  #define HAS_HEATER_2 1
#endif
#if PIN_EXISTS(HEATER_3)
  #define HAS_HEATER_3 1
#endif
#if PIN_EXISTS(HEATER_4)
  #define HAS_HEATER_4 1
#endif
#if PIN_EXISTS(HEATER_5)
  #define HAS_HEATER_5 1
#endif
#if PIN_EXISTS(HEATER_6)
  #define HAS_HEATER_6 1
#endif
#if PIN_EXISTS(HEATER_7)
  #define HAS_HEATER_7 1
#endif
#if PIN_EXISTS(HEATER_BED)
  #define HAS_HEATER_BED 1
#endif
#if PIN_EXISTS(HEATER_CHAMBER)
  #define HAS_HEATER_CHAMBER 1
#endif

// Shorthand for common combinations
#if HAS_HEATED_BED
  #ifndef BED_OVERSHOOT
    #define BED_OVERSHOOT 10
  #endif
  #define BED_MAX_TARGET ((BED_MAXTEMP) - (BED_OVERSHOOT))
#else
  #undef PIDTEMPBED
  #undef PREHEAT_BEFORE_LEVELING
#endif

#if HAS_TEMP_COOLER && PIN_EXISTS(COOLER)
  #define HAS_COOLER 1
  #ifndef COOLER_OVERSHOOT
    #define COOLER_OVERSHOOT 2
  #endif
  #define COOLER_MIN_TARGET ((COOLER_MINTEMP) + (COOLER_OVERSHOOT))
  #define COOLER_MAX_TARGET ((COOLER_MAXTEMP) - (COOLER_OVERSHOOT))
#endif

#if HAS_TEMP_HOTEND || HAS_HEATED_BED || HAS_TEMP_CHAMBER || HAS_TEMP_PROBE || HAS_TEMP_COOLER || HAS_TEMP_BOARD || HAS_TEMP_SOC
  #define HAS_TEMP_SENSOR 1
#endif

#if HAS_TEMP_CHAMBER && HAS_HEATER_CHAMBER
  #define HAS_HEATED_CHAMBER 1
  #ifndef CHAMBER_OVERSHOOT
    #define CHAMBER_OVERSHOOT 10
  #endif
  #define CHAMBER_MAX_TARGET ((CHAMBER_MAXTEMP) - (CHAMBER_OVERSHOOT))
#else
  #undef PIDTEMPCHAMBER
#endif

// PID heating
#if ANY(PIDTEMP, PIDTEMPBED, PIDTEMPCHAMBER)
  #define HAS_PID_HEATING 1
#endif

// Thermal protection
#if ENABLED(THERMAL_PROTECTION_HOTENDS) && WATCH_TEMP_PERIOD > 0
  #define WATCH_HOTENDS 1
#endif
#if ENABLED(THERMAL_PROTECTION_BED) && WATCH_BED_TEMP_PERIOD > 0
  #define WATCH_BED 1
#endif
#if ALL(HAS_HEATED_CHAMBER, THERMAL_PROTECTION_CHAMBER) && WATCH_CHAMBER_TEMP_PERIOD > 0
  #define WATCH_CHAMBER 1
#endif
#if ALL(HAS_COOLER, THERMAL_PROTECTION_COOLER) && WATCH_COOLER_TEMP_PERIOD > 0
  #define WATCH_COOLER 1
#endif
#if NONE(THERMAL_PROTECTION_HOTENDS, THERMAL_PROTECTION_CHAMBER, THERMAL_PROTECTION_BED, THERMAL_PROTECTION_COOLER)
  #undef THERMAL_PROTECTION_VARIANCE_MONITOR
#endif
#if  (ENABLED(THERMAL_PROTECTION_HOTENDS) || !EXTRUDERS) \
  && (ENABLED(THERMAL_PROTECTION_BED)     || !HAS_HEATED_BED) \
  && (ENABLED(THERMAL_PROTECTION_CHAMBER) || !HAS_HEATED_CHAMBER) \
  && (ENABLED(THERMAL_PROTECTION_COOLER)  || !HAS_COOLER)
  #define THERMALLY_SAFE 1
#endif

// Auto fans
#if HAS_HOTEND && PIN_EXISTS(E0_AUTO_FAN)
  #define HAS_AUTO_FAN_0 1
#endif
#if HAS_MULTI_HOTEND && PIN_EXISTS(E1_AUTO_FAN)
  #define HAS_AUTO_FAN_1 1
#endif
#if HOTENDS > 2 && PIN_EXISTS(E2_AUTO_FAN)
  #define HAS_AUTO_FAN_2 1
#endif
#if HOTENDS > 3 && PIN_EXISTS(E3_AUTO_FAN)
  #define HAS_AUTO_FAN_3 1
#endif
#if HOTENDS > 4 && PIN_EXISTS(E4_AUTO_FAN)
  #define HAS_AUTO_FAN_4 1
#endif
#if HOTENDS > 5 && PIN_EXISTS(E5_AUTO_FAN)
  #define HAS_AUTO_FAN_5 1
#endif
#if HOTENDS > 6 && PIN_EXISTS(E6_AUTO_FAN)
  #define HAS_AUTO_FAN_6 1
#endif
#if HOTENDS > 7 && PIN_EXISTS(E7_AUTO_FAN)
  #define HAS_AUTO_FAN_7 1
#endif
#if HAS_TEMP_CHAMBER && PIN_EXISTS(CHAMBER_AUTO_FAN)
  #define HAS_AUTO_CHAMBER_FAN 1
#endif
#if HAS_TEMP_COOLER && PIN_EXISTS(COOLER_AUTO_FAN)
  #define HAS_AUTO_COOLER_FAN 1
#endif
#if ANY(HAS_AUTO_FAN_0, HAS_AUTO_FAN_1, HAS_AUTO_FAN_2, HAS_AUTO_FAN_3, HAS_AUTO_FAN_4, HAS_AUTO_FAN_5, HAS_AUTO_FAN_6, HAS_AUTO_FAN_7)
  #define HAS_E_AUTO_FAN 1
#endif
#if ANY(HAS_E_AUTO_FAN, HAS_AUTO_CHAMBER_FAN, HAS_AUTO_COOLER_FAN)
  #define HAS_AUTO_FAN 1
  #define _FANOVERLAP(I,T) (T##_AUTO_FAN_PIN == E##I##_AUTO_FAN_PIN)
  #if HAS_AUTO_CHAMBER_FAN
    #define _CHFANOVERLAP(I) || _FANOVERLAP(I,CHAMBER)
    #if (0 REPEAT(8, _CHFANOVERLAP))
      #define AUTO_CHAMBER_IS_E 1
    #endif
    #undef _CHFANOVERLAP
  #endif
  #if HAS_AUTO_COOLER_FAN
    #define _COFANOVERLAP(I) || _FANOVERLAP(I,COOLER)
    #if (0 REPEAT(8, _COFANOVERLAP))
      #define AUTO_COOLER_IS_E 1
    #endif
    #undef _COFANOVERLAP
  #endif
#endif

// Fans check
#if HAS_HOTEND && PIN_EXISTS(E0_FAN_TACHO)
  #define HAS_E0_FAN_TACHO 1
#endif
#if HOTENDS > 1 && PIN_EXISTS(E1_FAN_TACHO)
  #define HAS_E1_FAN_TACHO 1
#endif
#if HOTENDS > 2 && PIN_EXISTS(E2_FAN_TACHO)
  #define HAS_E2_FAN_TACHO 1
#endif
#if HOTENDS > 3 && PIN_EXISTS(E3_FAN_TACHO)
  #define HAS_E3_FAN_TACHO 1
#endif
#if HOTENDS > 4 && PIN_EXISTS(E4_FAN_TACHO)
  #define HAS_E4_FAN_TACHO 1
#endif
#if HOTENDS > 5 && PIN_EXISTS(E5_FAN_TACHO)
  #define HAS_E5_FAN_TACHO 1
#endif
#if HOTENDS > 6 && PIN_EXISTS(E6_FAN_TACHO)
  #define HAS_E6_FAN_TACHO 1
#endif
#if HOTENDS > 7 && PIN_EXISTS(E7_FAN_TACHO)
  #define HAS_E7_FAN_TACHO 1
#endif
#if ANY(HAS_E0_FAN_TACHO, HAS_E1_FAN_TACHO, HAS_E2_FAN_TACHO, HAS_E3_FAN_TACHO, HAS_E4_FAN_TACHO, HAS_E5_FAN_TACHO, HAS_E6_FAN_TACHO, HAS_E7_FAN_TACHO)
  #define HAS_FANCHECK 1
  #if HAS_AUTO_FAN && EXTRUDER_AUTO_FAN_SPEED != 255 && DISABLED(FOURWIRES_FANS)
    #define HAS_PWMFANCHECK 1
  #endif
#endif

#if !HAS_TEMP_SENSOR
  #undef AUTO_REPORT_TEMPERATURES
#endif
#if ANY(AUTO_REPORT_TEMPERATURES, AUTO_REPORT_SD_STATUS, AUTO_REPORT_POSITION, AUTO_REPORT_FANS)
  #define HAS_AUTO_REPORTING 1
#endif

#if !HAS_AUTO_CHAMBER_FAN
  #undef CHAMBER_AUTO_FAN_SPEED
  #undef CHAMBER_AUTO_FAN_TEMPERATURE
#endif
#if !HAS_AUTO_COOLER_FAN
  #undef COOLER_AUTO_FAN_SPEED
  #undef COOLER_AUTO_FAN_TEMPERATURE
#endif
#if !HAS_E_AUTO_FAN
  #undef EXTRUDER_AUTO_FAN_SPEED
  #undef EXTRUDER_AUTO_FAN_TEMPERATURE
#endif

#if !HAS_AUTO_CHAMBER_FAN || AUTO_CHAMBER_IS_E
  #undef AUTO_POWER_CHAMBER_FAN
#endif
#if !HAS_AUTO_COOLER_FAN || AUTO_COOLER_IS_E
  #undef AUTO_POWER_COOLER_FAN
#endif
#if !HAS_CUTTER
  #undef AUTO_POWER_SPINDLE_LASER
#endif

/**
 * Controller Fan Settings
 */
#if PIN_EXISTS(CONTROLLER_FAN)
  #define HAS_CONTROLLER_FAN 1
#endif

#if HAS_CONTROLLER_FAN
  #if ENABLED(CONTROLLER_FAN_USE_BOARD_TEMP)
    #define HAS_CONTROLLER_FAN_BOARD_TEMP_TRIGGER 1
    #ifndef CONTROLLER_FAN_TRIGGER_TEMP
      #define CONTROLLER_FAN_TRIGGER_TEMP 30
    #endif
  #else
    #undef CONTROLLER_FAN_TRIGGER_TEMP
  #endif
#endif

/**
 * Up to 12 PWM fans
 */
#ifdef NUM_M106_FANS
  #define MAX_FANS NUM_M106_FANS
#else
  #define MAX_FANS 12  // Max supported fans
#endif

#define _IS_E_AUTO(N,F) (PIN_EXISTS(E##N##_AUTO_FAN) && E##N##_AUTO_FAN_PIN == FAN##F##_PIN)
#define _HAS_FAN(F) (F < MAX_FANS && PIN_EXISTS(FAN##F) \
                     && !(HAS_CONTROLLER_FAN && CONTROLLER_FAN_PIN == FAN##F##_PIN) \
                     && !_IS_E_AUTO(0,F) && !_IS_E_AUTO(1,F) \
                     && !_IS_E_AUTO(2,F) && !_IS_E_AUTO(3,F) \
                     && !_IS_E_AUTO(4,F) && !_IS_E_AUTO(5,F) \
                     && !_IS_E_AUTO(6,F) && !_IS_E_AUTO(7,F))

#if _HAS_FAN(0)
  #define HAS_FAN0 1
#endif
#if _HAS_FAN(1)
  #define HAS_FAN1 1
#endif
#if _HAS_FAN(2)
  #define HAS_FAN2 1
#endif
#if _HAS_FAN(3)
  #define HAS_FAN3 1
#endif
#if _HAS_FAN(4)
  #define HAS_FAN4 1
#endif
#if _HAS_FAN(5)
  #define HAS_FAN5 1
#endif
#if _HAS_FAN(6)
  #define HAS_FAN6 1
#endif
#if _HAS_FAN(7)
  #define HAS_FAN7 1
#endif
#if _HAS_FAN(8)
  #define HAS_FAN8 1
#endif
#if _HAS_FAN(9)
  #define HAS_FAN9 1
#endif
#if _HAS_FAN(10)
  #define HAS_FAN10 1
#endif
#if _HAS_FAN(11)
  #define HAS_FAN11 1
#endif
#undef _HAS_FAN
#undef _IS_E_AUTO

#if HAS_FAN11
  #define FAN_COUNT 12
#elif HAS_FAN10
  #define FAN_COUNT 11
#elif HAS_FAN9
  #define FAN_COUNT 10
#elif HAS_FAN8
  #define FAN_COUNT 9
#elif HAS_FAN7
  #define FAN_COUNT 8
#elif HAS_FAN6
  #define FAN_COUNT 7
#elif HAS_FAN5
  #define FAN_COUNT 6
#elif HAS_FAN4
  #define FAN_COUNT 5
#elif HAS_FAN3
  #define FAN_COUNT 4
#elif HAS_FAN2
  #define FAN_COUNT 3
#elif HAS_FAN1
  #define FAN_COUNT 2
#elif HAS_FAN0
  #define FAN_COUNT 1
#else
  #define FAN_COUNT 0
#endif

#if FAN_COUNT > 0
  #define HAS_FAN 1
#endif

#if PIN_EXISTS(FANMUX0)
  #define HAS_FANMUX 1  // Part Cooling fan multipliexer
#else
  #undef FANMUX0_PIN
  #undef FANMUX1_PIN
  #undef FANMUX2_PIN
#endif

/**
 * MIN/MAX fan PWM scaling
 */
#if ANY(HAS_FAN, USE_CONTROLLER_FAN)
  #ifndef FAN_OFF_PWM
    #define FAN_OFF_PWM 0
  #endif
  #ifndef FAN_MIN_PWM
    #if FAN_OFF_PWM > 0
      #define FAN_MIN_PWM (FAN_OFF_PWM + 1)
    #else
      #define FAN_MIN_PWM 0
    #endif
  #endif
  #ifndef FAN_MAX_PWM
    #define FAN_MAX_PWM 255
  #endif
  #if FAN_MIN_PWM == 0 && FAN_MAX_PWM == 255
    #define CALC_FAN_SPEED(f) (f ?: FAN_OFF_PWM)
  #else
    #define CALC_FAN_SPEED(f) (f ? map(f, 1, 255, FAN_MIN_PWM, FAN_MAX_PWM) : FAN_OFF_PWM)
  #endif
#endif

// Fan Kickstart
#if FAN_KICKSTART_TIME && !defined(FAN_KICKSTART_POWER)
  #define FAN_KICKSTART_POWER TERN(FAN_KICKSTART_LINEAR, 255, 180)
#endif

// Servos
#if NUM_SERVOS > 0
  #define HAS_SERVOS 1
  #if PIN_EXISTS(SERVO0)
    #define HAS_SERVO_0 1
  #endif
  #if PIN_EXISTS(SERVO1) && NUM_SERVOS > 1
    #define HAS_SERVO_1 1
  #endif
  #if PIN_EXISTS(SERVO2) && NUM_SERVOS > 2
    #define HAS_SERVO_2 1
  #endif
  #if PIN_EXISTS(SERVO3) && NUM_SERVOS > 3
    #define HAS_SERVO_3 1
  #endif
  #if PIN_EXISTS(SERVO4) && NUM_SERVOS > 4
    #define HAS_SERVO_4 1
  #endif
  #if PIN_EXISTS(SERVO5) && NUM_SERVOS > 5
    #define HAS_SERVO_5 1
  #endif
  #if defined(PAUSE_SERVO_OUTPUT) && defined(RESUME_SERVO_OUTPUT)
    #define HAS_PAUSE_SERVO_OUTPUT 1
  #endif
#else
  #undef SERVO_DELAY
  #undef DEACTIVATE_SERVOS_AFTER_MOVE
  #undef EDITABLE_SERVO_ANGLES
  #undef SERVO_DETACH_GCODE
#endif

// Sensors
#if PIN_EXISTS(FILWIDTH)
  #define HAS_FILAMENT_WIDTH_SENSOR 1
#endif

// User Interface
#if ENABLED(FREEZE_FEATURE) && !PIN_EXISTS(FREEZE) && PIN_EXISTS(KILL)
  #define FREEZE_PIN KILL_PIN
#elif PIN_EXISTS(KILL) && TERN1(FREEZE_FEATURE, KILL_PIN != FREEZE_PIN)
  #define HAS_KILL 1
#endif
#if PIN_EXISTS(HOME)
  #define HAS_HOME 1
#endif
#if PIN_EXISTS(SUICIDE)
  #define HAS_SUICIDE 1
#endif
#if PIN_EXISTS(PHOTOGRAPH)
  #define HAS_PHOTOGRAPH 1
#endif

// Digital control
#if PIN_EXISTS(STEPPER_RESET)
  #define HAS_STEPPER_RESET 1
#endif
#if PIN_EXISTS(DIGIPOTSS)
  #define HAS_MOTOR_CURRENT_SPI 1
#endif
#if ANY_PIN(MOTOR_CURRENT_PWM_E, MOTOR_CURRENT_PWM_E0, MOTOR_CURRENT_PWM_E1)
  #define HAS_MOTOR_CURRENT_PWM_E 1
#endif
#if HAS_MOTOR_CURRENT_PWM_E || ANY_PIN(MOTOR_CURRENT_PWM_X, MOTOR_CURRENT_PWM_Y, MOTOR_CURRENT_PWM_XY, MOTOR_CURRENT_PWM_Z, MOTOR_CURRENT_PWM_I, MOTOR_CURRENT_PWM_J, MOTOR_CURRENT_PWM_K, MOTOR_CURRENT_PWM_U, MOTOR_CURRENT_PWM_V, MOTOR_CURRENT_PWM_W)
  #define HAS_MOTOR_CURRENT_PWM 1
#endif

#if ANY(HAS_Z_MS_PINS, HAS_Z2_MS_PINS, HAS_Z3_MS_PINS, HAS_Z4_MS_PINS)
  #define HAS_SOME_Z_MS_PINS 1
#endif
#if ANY(HAS_E0_MS_PINS, HAS_E1_MS_PINS, HAS_E2_MS_PINS, HAS_E3_MS_PINS, HAS_E4_MS_PINS, HAS_E5_MS_PINS, HAS_E6_MS_PINS, HAS_E7_MS_PINS)
  #define HAS_SOME_E_MS_PINS 1
#endif
#if ANY(HAS_X_MS_PINS, HAS_X2_MS_PINS, HAS_Y_MS_PINS, HAS_Y2_MS_PINS, HAS_SOME_Z_MS_PINS, HAS_I_MS_PINS, HAS_J_MS_PINS, HAS_K_MS_PINS, HAS_U_MS_PINS, HAS_V_MS_PINS, HAS_W_MS_PINS, HAS_SOME_E_MS_PINS)
  #define HAS_MICROSTEPS 1
#else
  #undef MICROSTEP_MODES
#endif

/**
 * Helper Macros for heaters and extruder fan
 */
#define WRITE_HEATER_0P(v) WRITE(HEATER_0_PIN, (v) ^ ENABLED(HEATER_0_INVERTING))
#if ANY(HAS_MULTI_HOTEND, HEATERS_PARALLEL)
  #define WRITE_HEATER_1(v) WRITE(HEATER_1_PIN, (v) ^ ENABLED(HEATER_1_INVERTING))
  #if HOTENDS > 2
    #define WRITE_HEATER_2(v) WRITE(HEATER_2_PIN, (v) ^ ENABLED(HEATER_2_INVERTING))
    #if HOTENDS > 3
      #define WRITE_HEATER_3(v) WRITE(HEATER_3_PIN, (v) ^ ENABLED(HEATER_3_INVERTING))
      #if HOTENDS > 4
        #define WRITE_HEATER_4(v) WRITE(HEATER_4_PIN, (v) ^ ENABLED(HEATER_4_INVERTING))
        #if HOTENDS > 5
          #define WRITE_HEATER_5(v) WRITE(HEATER_5_PIN, (v) ^ ENABLED(HEATER_5_INVERTING))
          #if HOTENDS > 6
            #define WRITE_HEATER_6(v) WRITE(HEATER_6_PIN, (v) ^ ENABLED(HEATER_6_INVERTING))
            #if HOTENDS > 7
              #define WRITE_HEATER_7(v) WRITE(HEATER_7_PIN, (v) ^ ENABLED(HEATER_7_INVERTING))
            #endif // HOTENDS > 7
          #endif // HOTENDS > 6
        #endif // HOTENDS > 5
      #endif // HOTENDS > 4
    #endif // HOTENDS > 3
  #endif // HOTENDS > 2
#endif // HAS_MULTI_HOTEND || HEATERS_PARALLEL
#if ENABLED(HEATERS_PARALLEL)
  #define WRITE_HEATER_0(v) { WRITE_HEATER_0P(v); WRITE_HEATER_1(v); }
#else
  #define WRITE_HEATER_0(v) WRITE_HEATER_0P(v)
#endif

/**
 * Heated Bed required settings
 */
#if HAS_HEATED_BED
  #ifndef MIN_BED_POWER
    #define MIN_BED_POWER 0
  #endif
  #ifndef MAX_BED_POWER
    #define MAX_BED_POWER 255
  #endif
  #define WRITE_HEATER_BED(v) WRITE(HEATER_BED_PIN, (v) ^ ENABLED(HEATER_BED_INVERTING))
  #if ENABLED(PELTIER_BED)
   /**
    * A "Heated Bed" Peltier device needs a direction (heat/cool) to be
    * implemented by a relay (single pin) or H-bridge (2 or 4 pin).
    * H-Bridge can also perform PWM. (Not recommended for Peltier devices).
    */
    #define WRITE_PELTIER_DIR(v) WRITE(PELTIER_DIR_PIN, (v) ? PELTIER_DIR_HEAT_STATE : !PELTIER_DIR_HEAT_STATE)
  #endif
#endif

/**
 * Heated chamber requires settings
 */
#if HAS_HEATED_CHAMBER
  #ifndef MIN_CHAMBER_POWER
    #define MIN_CHAMBER_POWER 0
  #endif
  #ifndef MAX_CHAMBER_POWER
    #define MAX_CHAMBER_POWER 255
  #endif
  #define WRITE_HEATER_CHAMBER(v) WRITE(HEATER_CHAMBER_PIN, (v) ^ ENABLED(HEATER_CHAMBER_INVERTING))
#endif

/**
 * Laser Cooling requires settings
 */
#if HAS_COOLER
  #ifndef MAX_COOLER_POWER
    #define MAX_COOLER_POWER 255
  #endif
  #define WRITE_HEATER_COOLER(v) WRITE(COOLER_PIN, (v) ^ ENABLED(COOLER_INVERTING))
#endif

#if HAS_HOTEND || HAS_HEATED_BED || HAS_HEATED_CHAMBER || HAS_COOLER
  #define HAS_TEMPERATURE 1
#endif

#if HAS_TEMPERATURE && ANY(HAS_MARLINUI_MENU, HAS_DWIN_E3V2, HAS_DGUS_LCD_CLASSIC)
  #ifdef PREHEAT_10_LABEL
    #define PREHEAT_COUNT 10
  #elif defined(PREHEAT_9_LABEL)
    #define PREHEAT_COUNT 9
  #elif defined(PREHEAT_8_LABEL)
    #define PREHEAT_COUNT 8
  #elif defined(PREHEAT_7_LABEL)
    #define PREHEAT_COUNT 7
  #elif defined(PREHEAT_6_LABEL)
    #define PREHEAT_COUNT 6
  #elif defined(PREHEAT_5_LABEL)
    #define PREHEAT_COUNT 5
  #elif defined(PREHEAT_4_LABEL)
    #define PREHEAT_COUNT 4
  #elif defined(PREHEAT_3_LABEL)
    #define PREHEAT_COUNT 3
  #elif defined(PREHEAT_2_LABEL)
    #define PREHEAT_COUNT 2
  #elif defined(PREHEAT_1_LABEL)
    #define PREHEAT_COUNT 1
  #endif
  #if PREHEAT_COUNT && ANY(HAS_HOTEND, HAS_HEATED_BED, HAS_FAN)
    #define HAS_PREHEAT 1
  #endif
#endif

#if !HAS_PREHEAT
  #undef PREHEAT_SHORTCUT_MENU_ITEM
  #undef DGUS_PREHEAT_UI
#endif

/**
 * MIN/MAX case light PWM scaling
 */
#if ENABLED(CASE_LIGHT_ENABLE)
  #ifndef CASE_LIGHT_MAX_PWM
    #define CASE_LIGHT_MAX_PWM 255
  #elif !WITHIN(CASE_LIGHT_MAX_PWM, 1, 255)
    #error "CASE_LIGHT_MAX_PWM must be a value from 1 to 255."
  #endif
#endif

/**
 * Bed Probe dependencies
 */
#if ANY(MESH_BED_LEVELING, HAS_BED_PROBE)
  #ifndef PROBE_OFFSET_ZMIN
    #define PROBE_OFFSET_ZMIN -20
  #endif
  #ifndef PROBE_OFFSET_ZMAX
    #define PROBE_OFFSET_ZMAX  20
  #endif
#endif
#if HAS_BED_PROBE
  #ifndef PROBE_OFFSET_XMIN
    #define PROBE_OFFSET_XMIN -(X_BED_SIZE)
  #endif
  #ifndef PROBE_OFFSET_XMAX
    #define PROBE_OFFSET_XMAX  X_BED_SIZE
  #endif
  #ifndef PROBE_OFFSET_YMIN
    #define PROBE_OFFSET_YMIN -(Y_BED_SIZE)
  #endif
  #ifndef PROBE_OFFSET_YMAX
    #define PROBE_OFFSET_YMAX Y_BED_SIZE
  #endif
  #if ALL(ENDSTOPPULLUPS, USE_Z_MIN_PROBE)
    #define ENDSTOPPULLUP_ZMIN_PROBE
  #endif
  #ifndef XY_PROBE_FEEDRATE
    #define XY_PROBE_FEEDRATE ((homing_feedrate_mm_m.x + homing_feedrate_mm_m.y) / 2)
  #endif
  #ifndef NOZZLE_TO_PROBE_OFFSET
    #define NOZZLE_TO_PROBE_OFFSET { 0, 0, 0 }
  #endif
#endif

/**
 * XYZ Bed Skew Correction
 */
#if ENABLED(SKEW_CORRECTION)
  #define SKEW_FACTOR_MIN -1
  #define SKEW_FACTOR_MAX 1

  #define _GET_SIDE(a,b,c) (SQRT(2*sq(a)+2*sq(b)-4*sq(c))*0.5)
  #define _SKEW_SIDE(a,b,c) tan(M_PI*0.5-acos((sq(a)-sq(b)-sq(c))/(2*c*b)))
  #define _SKEW_FACTOR(a,b,c) _SKEW_SIDE(float(a),_GET_SIDE(float(a),float(b),float(c)),float(c))

  #ifndef XY_SKEW_FACTOR
    #if defined(XY_DIAG_AC) && defined(XY_DIAG_BD) && defined(XY_SIDE_AD)
      #define XY_SKEW_FACTOR _SKEW_FACTOR(XY_DIAG_AC, XY_DIAG_BD, XY_SIDE_AD)
    #else
      #define XY_SKEW_FACTOR 0.0
    #endif
  #endif
  #ifndef XZ_SKEW_FACTOR
    #if defined(XY_SIDE_AD) && !defined(XZ_SIDE_AD)
      #define XZ_SIDE_AD XY_SIDE_AD
    #endif
    #if defined(XZ_DIAG_AC) && defined(XZ_DIAG_BD) && defined(XZ_SIDE_AD)
      #define XZ_SKEW_FACTOR _SKEW_FACTOR(XZ_DIAG_AC, XZ_DIAG_BD, XZ_SIDE_AD)
    #else
      #define XZ_SKEW_FACTOR 0.0
    #endif
  #endif
  #ifndef YZ_SKEW_FACTOR
    #if defined(YZ_DIAG_AC) && defined(YZ_DIAG_BD) && defined(YZ_SIDE_AD)
      #define YZ_SKEW_FACTOR _SKEW_FACTOR(YZ_DIAG_AC, YZ_DIAG_BD, YZ_SIDE_AD)
    #else
      #define YZ_SKEW_FACTOR 0.0
    #endif
  #endif
#endif // SKEW_CORRECTION

/**
 * Heater, Fan, and Probe interactions
 */
#if !ALL(HAS_HOTEND, HAS_FAN)
  #undef ADAPTIVE_FAN_SLOWING
#endif
#if DISABLED(ADAPTIVE_FAN_SLOWING)
  #undef REPORT_ADAPTIVE_FAN_SLOWING
#endif
#if DISABLED(ADAPTIVE_FAN_SLOWING) || NONE(MPCTEMP, PIDTEMP)
  #undef TEMP_TUNING_MAINTAIN_FAN
#endif
#if !ALL(HAS_BED_PROBE, HAS_FAN)
  #undef PROBING_FANS_OFF
#endif
#if !ALL(HAS_BED_PROBE, HAS_EXTRUDERS)
  #undef PROBING_ESTEPPERS_OFF
#elif ENABLED(PROBING_STEPPERS_OFF)
  // PROBING_STEPPERS_OFF implies PROBING_ESTEPPERS_OFF, make sure it is defined
  #define PROBING_ESTEPPERS_OFF
#endif
#if ANY(ADVANCED_PAUSE_FEATURE, PROBING_HEATERS_OFF)
  #define HEATER_IDLE_HANDLER 1
#endif
#if HAS_BED_PROBE && (ANY(PROBING_HEATERS_OFF, PROBING_STEPPERS_OFF, PROBING_ESTEPPERS_OFF, PROBING_FANS_OFF) || DELAY_BEFORE_PROBING > 0)
  #define HAS_QUIET_PROBING 1
#endif

/**
 * Advanced Pause - Filament Change
 */
#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #if ANY(HAS_MARLINUI_MENU, EXTENSIBLE_UI, DWIN_CREALITY_LCD_JYERSUI, SOVOL_SV06_RTS) || ALL(EMERGENCY_PARSER, HOST_PROMPT_SUPPORT)
    #define M600_PURGE_MORE_RESUMABLE 1  // UI provides some way to Purge More / Resume
  #endif
  #ifndef FILAMENT_CHANGE_SLOW_LOAD_LENGTH
    #define FILAMENT_CHANGE_SLOW_LOAD_LENGTH 0
  #endif
#endif

#if HAS_MULTI_EXTRUDER && !defined(TOOLCHANGE_FS_EXTRA_PRIME)
  #define TOOLCHANGE_FS_EXTRA_PRIME 0
#endif

/**
 * Only constrain Z on DELTA / SCARA machines
 */
#if ENABLED(POLAR)
  #undef MIN_SOFTWARE_ENDSTOP_Y
  #undef MAX_SOFTWARE_ENDSTOP_Y
#elif IS_KINEMATIC
  #undef MIN_SOFTWARE_ENDSTOP_X
  #undef MIN_SOFTWARE_ENDSTOP_Y
  #undef MAX_SOFTWARE_ENDSTOP_X
  #undef MAX_SOFTWARE_ENDSTOP_Y
#endif

/**
 * Bed Probing bounds
 */

#if IS_KINEMATIC
  #undef PROBING_MARGIN_LEFT
  #undef PROBING_MARGIN_RIGHT
  #undef PROBING_MARGIN_FRONT
  #undef PROBING_MARGIN_BACK
#elif PROBING_MARGIN
  #ifndef PROBING_MARGIN_LEFT
    #define PROBING_MARGIN_LEFT PROBING_MARGIN
  #endif
  #ifndef PROBING_MARGIN_RIGHT
    #define PROBING_MARGIN_RIGHT PROBING_MARGIN
  #endif
  #ifndef PROBING_MARGIN_FRONT
    #define PROBING_MARGIN_FRONT PROBING_MARGIN
  #endif
  #ifndef PROBING_MARGIN_BACK
    #define PROBING_MARGIN_BACK PROBING_MARGIN
  #endif
#endif
#ifndef PROBING_MARGIN_LEFT
  #define PROBING_MARGIN_LEFT 0
#endif
#ifndef PROBING_MARGIN_RIGHT
  #define PROBING_MARGIN_RIGHT 0
#endif
#ifndef PROBING_MARGIN_FRONT
  #define PROBING_MARGIN_FRONT 0
#endif
#ifndef PROBING_MARGIN_BACK
  #define PROBING_MARGIN_BACK 0
#endif

#if ENABLED(DELTA)
  /**
   * Delta radius/rod trimmers/angle trimmers
   */
  #ifndef DELTA_ENDSTOP_ADJ
    #define DELTA_ENDSTOP_ADJ { 0, 0, 0 }
  #endif
  #ifndef DELTA_TOWER_ANGLE_TRIM
    #define DELTA_TOWER_ANGLE_TRIM { 0, 0, 0 }
  #endif
  #ifndef DELTA_RADIUS_TRIM_TOWER
    #define DELTA_RADIUS_TRIM_TOWER { 0, 0, 0 }
  #endif
  #ifndef DELTA_DIAGONAL_ROD_TRIM_TOWER
    #define DELTA_DIAGONAL_ROD_TRIM_TOWER { 0, 0, 0 }
  #endif
#endif

#if ENABLED(SEGMENT_LEVELED_MOVES) && !defined(LEVELED_SEGMENT_LENGTH)
  #define LEVELED_SEGMENT_LENGTH 5
#endif

/**
 * Default mesh area is an area with an inset margin on the print area.
 */
#if ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_UBL)
  #if IS_KINEMATIC
    // Probing points may be verified at compile time within the radius
    // using static_assert(HYPOT2(X2-X1,Y2-Y1)<=sq(PRINTABLE_RADIUS),"bad probe point!")
    // so that may be added to SanityCheck.h in the future.
    #define _MESH_MIN_X (X_MIN_BED + _MESH_INSET)
    #define _MESH_MIN_Y (Y_MIN_BED + _MESH_INSET)
    #define _MESH_MAX_X (X_MAX_BED - _MESH_INSET)
    #define _MESH_MAX_Y (Y_MAX_BED - _MESH_INSET)
  #else
    // Boundaries for Cartesian probing based on set limits
    #define _MESH_MIN_X (_MAX(X_MIN_BED + _MESH_INSET, X_MIN_POS)) // UBL is careful not to probe off the bed. It doesn't
    #define _MESH_MIN_Y (_MAX(Y_MIN_BED + _MESH_INSET, Y_MIN_POS)) // need NOZZLE_TO_PROBE_OFFSET in the mesh dimensions.
    #define _MESH_MAX_X (_MIN(X_MAX_BED - _MESH_INSET, X_MAX_POS))
    #define _MESH_MAX_Y (_MIN(Y_MAX_BED - _MESH_INSET, Y_MAX_POS))
  #endif

  // These may be overridden in Configuration.h if a smaller area is desired
  #ifndef MESH_MIN_X
    #define MESH_MIN_X _MESH_MIN_X
  #endif
  #ifndef MESH_MIN_Y
    #define MESH_MIN_Y _MESH_MIN_Y
  #endif
  #ifndef MESH_MAX_X
    #define MESH_MAX_X _MESH_MAX_X
  #endif
  #ifndef MESH_MAX_Y
    #define MESH_MAX_Y _MESH_MAX_Y
  #endif
#else
  #undef MESH_MIN_X
  #undef MESH_MIN_Y
  #undef MESH_MAX_X
  #undef MESH_MAX_Y
#endif

#if NEEDS_THREE_PROBE_POINTS && defined(PROBE_PT_1)
  #define HAS_FIXED_3POINT 1  // Points are defined for ABL/UBL. Else calculated in probe.get_three_points.
#endif

/**
 * Buzzer/Speaker
 */
#if PIN_EXISTS(BEEPER)
  #define HAS_BEEPER 1
#endif
#if ANY(IS_TFTGLCD_PANEL, PCA9632_BUZZER, LCD_USE_I2C_BUZZER)
  #define USE_MARLINUI_BUZZER 1
#endif
#if ANY(HAS_BEEPER, USE_MARLINUI_BUZZER)
  #define HAS_SOUND 1
#endif

#if ENABLED(LCD_USE_I2C_BUZZER)
  #ifndef LCD_FEEDBACK_FREQUENCY_HZ
    #define LCD_FEEDBACK_FREQUENCY_HZ 1000
  #endif
  #ifndef LCD_FEEDBACK_FREQUENCY_DURATION_MS
    #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
  #endif
#elif HAS_SOUND
  #ifndef LCD_FEEDBACK_FREQUENCY_HZ
    #define LCD_FEEDBACK_FREQUENCY_HZ 5000
  #endif
  #ifndef LCD_FEEDBACK_FREQUENCY_DURATION_MS
    #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2
  #endif
#endif

#if HAS_SOUND
  #if LCD_FEEDBACK_FREQUENCY_DURATION_MS && LCD_FEEDBACK_FREQUENCY_HZ
    #define HAS_CHIRP 1
  #endif
#else
  #undef SOUND_MENU_ITEM   // No buzzer menu item without a buzzer
  #undef SOUND_ON_DEFAULT
#endif

// Define a starting height for measuring manual probe points
#ifndef MANUAL_PROBE_START_Z
  #if ANY(MESH_BED_LEVELING, PROBE_MANUALLY)
    // Leave MANUAL_PROBE_START_Z undefined so the prior Z height will be used.
    // Note: If Z_CLEARANCE_BETWEEN_MANUAL_PROBES is 0 there will be no raise between points
  #elif ENABLED(AUTO_BED_LEVELING_UBL) && defined(Z_CLEARANCE_BETWEEN_PROBES)
    #define MANUAL_PROBE_START_Z Z_CLEARANCE_BETWEEN_PROBES
  #endif
#endif

#ifndef __SAM3X8E__ //todo: hal: broken hal encapsulation
  #undef UI_VOLTAGE_LEVEL
  #undef RADDS_DISPLAY
  #undef MOTOR_CURRENT
#endif

// G92 shifts the workspace
#if DISABLED(NO_WORKSPACE_OFFSETS)
  #define HAS_WORKSPACE_OFFSET 1
#endif
#if DISABLED(NO_HOME_OFFSETS)
  #if IS_CARTESIAN
    #define HAS_HOME_OFFSET 1     // M206 affects the Native Machine Space on G28
  #elif IS_SCARA
    #define HAS_SCARA_OFFSET 1    // The SCARA home offset applies only on G28
  #endif
#endif

#if ANY(HAS_MARLINUI_MENU, TOUCH_UI_FTDI_EVE)
  // LCD timeout to status screen default is 15s
  #ifndef LCD_TIMEOUT_TO_STATUS
    #define LCD_TIMEOUT_TO_STATUS 15000
  #endif
  #if LCD_TIMEOUT_TO_STATUS
    #define HAS_SCREEN_TIMEOUT 1
  #endif
#endif

// Add commands that need sub-codes to this list
#if ANY(G38_PROBE_TARGET, CNC_COORDINATE_SYSTEMS, POWER_LOSS_RECOVERY, HAS_ROTATIONAL_AXES)
  #define USE_GCODE_SUBCODES 1
#endif

// Parking Extruder
#if ENABLED(PARKING_EXTRUDER)
  #ifndef PARKING_EXTRUDER_GRAB_DISTANCE
    #define PARKING_EXTRUDER_GRAB_DISTANCE 0
  #endif
  #ifndef PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE
    #define PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE HIGH
  #endif
#endif

// Touch Calibration
#if ANY(HAS_SPI_TFT, HAS_FSMC_TFT, HAS_LTDC_TFT)
  #ifndef TOUCH_CALIBRATION_X
    #define TOUCH_CALIBRATION_X 0
  #endif
  #ifndef TOUCH_CALIBRATION_Y
    #define TOUCH_CALIBRATION_Y 0
  #endif
  #ifndef TOUCH_OFFSET_X
    #define TOUCH_OFFSET_X 0
  #endif
  #ifndef TOUCH_OFFSET_Y
    #define TOUCH_OFFSET_Y 0
  #endif
  #ifndef TOUCH_ORIENTATION
    #define TOUCH_ORIENTATION TOUCH_LANDSCAPE
  #endif
#endif

// Number of VFAT entries used. Each entry has 13 UTF-16 characters
#if ANY(SCROLL_LONG_FILENAMES, HAS_DWIN_E3V2, TFT_COLOR_UI)
  #define VFAT_ENTRIES_LIMIT 5
#else
  #define VFAT_ENTRIES_LIMIT 2
#endif
#define MAX_VFAT_ENTRIES 20 // by VFAT specs to fit LFN of length 255

// Nozzle park for Delta
#if ALL(NOZZLE_PARK_FEATURE, DELTA)
  #undef NOZZLE_PARK_Z_FEEDRATE
  #define NOZZLE_PARK_Z_FEEDRATE NOZZLE_PARK_XY_FEEDRATE
#endif

// Force SDCARD_SORT_ALPHA to be enabled for Graphical LCD on LPC1768
// on boards where SD card and LCD display share the same SPI bus
// because of a bug in the shared SPI implementation. (See #8122)
#if defined(TARGET_LPC1768) && IS_RRD_FG_SC && (SD_SCK_PIN == LCD_PINS_D4)
  #define SDCARD_SORT_ALPHA         // Keep one directory level in RAM. Changing directory levels
                                    // may still glitch the screen, but LCD updates clean it up.
  #if SDSORT_LIMIT > 64 || !SDSORT_USES_RAM || SDSORT_USES_STACK || !SDSORT_CACHE_NAMES
    #undef SDSORT_LIMIT
    #undef SDSORT_USES_RAM
    #undef SDSORT_USES_STACK
    #undef SDSORT_CACHE_NAMES
    #define SDSORT_LIMIT       64
    #define SDSORT_USES_RAM    true
    #define SDSORT_USES_STACK  false
    #define SDSORT_CACHE_NAMES true
    #define SDSORT_CACHE_LPC1768_WARNING 1
  #endif
  #ifndef SDSORT_FOLDERS
    #define SDSORT_FOLDERS     -1
  #endif
  #ifndef SDSORT_GCODE
    #define SDSORT_GCODE       false
  #endif
  #ifndef SDSORT_DYNAMIC_RAM
    #define SDSORT_DYNAMIC_RAM false
  #endif
  #ifndef SDSORT_CACHE_VFATS
    #define SDSORT_CACHE_VFATS 2
  #endif
#endif

// Fallback SPI Speed for SD
#if HAS_MEDIA && !defined(SD_SPI_SPEED)
  #define SD_SPI_SPEED SPI_FULL_SPEED
#endif

#if HAS_WIRED_LCD
  // Get LCD character width/height, which may be overridden by pins, configs, etc.
  #ifndef LCD_WIDTH
    #if HAS_MARLINUI_U8GLIB
      #define LCD_WIDTH 21
    #elif IS_DWIN_MARLINUI
      // Defined by header
    #elif HAS_GRAPHICAL_TFT
      #define LCD_WIDTH ((TFT_WIDTH) / 16)
    #else
      #define LCD_WIDTH TERN(IS_ULTIPANEL, 20, 16)
    #endif
  #endif
  #ifndef LCD_HEIGHT
    #if HAS_MARLINUI_U8GLIB
      #define LCD_HEIGHT 5
    #elif IS_DWIN_MARLINUI
      // Defined by header
    #else
      #define LCD_HEIGHT TERN(IS_ULTIPANEL, 4, 2)
    #endif
  #endif
  // Prepare the LCD to show the bootscreen early in setup
  #if ENABLED(SHOW_BOOTSCREEN) && ANY(HAS_LCD_CONTRAST, HAS_LCD_BRIGHTNESS)
    #define HAS_EARLY_LCD_SETTINGS 1
  #endif
#endif

#if BUTTONS_EXIST(EN1, EN2, ENC)
  #define HAS_ROTARY_ENCODER 1
#endif

#if PIN_EXISTS(SAFE_POWER) && DISABLED(DISABLE_DRIVER_SAFE_POWER_PROTECT)
  #define HAS_DRIVER_SAFE_POWER_PROTECT 1
#endif

#if ANY(ENDSTOPPULLDOWNS, ENDSTOPPULLDOWN_ZMIN_PROBE, \
    ENDSTOPPULLDOWN_XMIN, ENDSTOPPULLDOWN_YMIN, ENDSTOPPULLDOWN_ZMIN, \
    ENDSTOPPULLDOWN_IMIN, ENDSTOPPULLDOWN_JMIN, ENDSTOPPULLDOWN_KMIN, \
    ENDSTOPPULLDOWN_XMAX, ENDSTOPPULLDOWN_YMAX, ENDSTOPPULLDOWN_ZMAX, \
    ENDSTOPPULLDOWN_IMAX, ENDSTOPPULLDOWN_JMAX, ENDSTOPPULLDOWN_KMAX, \
    POWER_LOSS_PULLDOWN, CALIBRATION_PIN_PULLDOWN, FIL_RUNOUT_PULLDOWN, \
    FIL_RUNOUT1_PULLDOWN, FIL_RUNOUT2_PULLDOWN, FIL_RUNOUT3_PULLDOWN, FIL_RUNOUT4_PULLDOWN, \
    FIL_RUNOUT5_PULLDOWN, FIL_RUNOUT6_PULLDOWN, FIL_RUNOUT7_PULLDOWN, FIL_RUNOUT8_PULLDOWN)
  #define USING_PULLDOWNS 1
#endif

// Machine UUID can come from STM32 CPU Serial Number
#ifdef MACHINE_UUID
  #undef HAS_STM32_UID
#elif !HAS_STM32_UID && defined(DEFAULT_MACHINE_UUID)
  #define MACHINE_UUID DEFAULT_MACHINE_UUID
#endif

// Flag whether hex_print.cpp is needed
#if ANY(AUTO_BED_LEVELING_UBL, M100_FREE_MEMORY_WATCHER, DEBUG_GCODE_PARSER, TMC_DEBUG, MARLIN_DEV_MODE, DEBUG_CARDREADER, M20_TIMESTAMP_SUPPORT, HAS_STM32_UID)
  #define NEED_HEX_PRINT 1
#endif
