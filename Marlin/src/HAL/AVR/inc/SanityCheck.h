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
 * Test AVR-specific configuration values for errors at compile-time.
 */

#if HAS_SPI_TFT || HAS_FSMC_TFT
  #error "Sorry! TFT displays are not available for HAL/AVR."
#endif

/**
 * Check for common serial pin conflicts
 */
#define CHECK_SERIAL_PIN(N) ( \
     X_STOP_PIN == N || Y_STOP_PIN == N || Z_STOP_PIN  == N \
  || X_MIN_PIN  == N || Y_MIN_PIN  == N || Z_MIN_PIN   == N \
  || X_MAX_PIN  == N || Y_MAX_PIN  == N || Z_MAX_PIN   == N \
  || X_STEP_PIN == N || Y_STEP_PIN == N || Z_STEP_PIN  == N \
  || X_DIR_PIN  == N || Y_DIR_PIN  == N || Z_DIR_PIN   == N \
  || X_ENA_PIN  == N || Y_ENA_PIN  == N || Z_ENA_PIN   == N \
  || BTN_EN1    == N || BTN_EN2    == N || LCD_PINS_EN == N \
)
#if SERIAL_IN_USE(0)
  // D0-D1. No known conflicts.
#endif
#if SERIAL_IN_USE(1)
  #if NOT_TARGET(__AVR_ATmega644P__, __AVR_ATmega1284P__)
    #if CHECK_SERIAL_PIN(18) || CHECK_SERIAL_PIN(19)
      #error "Serial Port 1 pin D18 and/or D19 conflicts with another pin on the board."
    #endif
  #else
    #if CHECK_SERIAL_PIN(10) || CHECK_SERIAL_PIN(11)
      #error "Serial Port 1 pin D10 and/or D11 conflicts with another pin on the board."
    #endif
  #endif
#endif
#if SERIAL_IN_USE(2) && (CHECK_SERIAL_PIN(16) || CHECK_SERIAL_PIN(17))
  #error "Serial Port 2 pin D16 and/or D17 conflicts with another pin on the board."
#endif
#if SERIAL_IN_USE(3) && (CHECK_SERIAL_PIN(14) || CHECK_SERIAL_PIN(15))
  #error "Serial Port 3 pin D14 and/or D15 conflicts with another pin on the board."
#endif
#undef CHECK_SERIAL_PIN

/**
 * Checks for FAST PWM
 */
#if ALL(FAST_PWM_FAN, USE_OCR2A_AS_TOP, HAS_TCCR2)
  #error "USE_OCR2A_AS_TOP does not apply to devices with a single output TIMER2."
#endif

/**
 * Checks for SOFT PWM
 */
#if HAS_FAN0 && FAN0_PIN == 9 && DISABLED(FAN_SOFT_PWM) && ENABLED(SPEAKER)
  #error "FAN0_PIN 9 Hardware PWM uses Timer 2 which conflicts with Arduino AVR Tone Timer (for SPEAKER)."
  #error "Disable SPEAKER or enable FAN_SOFT_PWM."
#endif

/**
 * Sanity checks for Spindle / Laser PWM
 */
#if ENABLED(SPINDLE_LASER_USE_PWM)
  #include "../ServoTimers.h"   // Needed to check timer availability (_useTimer3)
  #if SPINDLE_LASER_PWM_PIN == 4 || WITHIN(SPINDLE_LASER_PWM_PIN, 11, 13)
    #error "Counter/Timer for SPINDLE_LASER_PWM_PIN is used by a system interrupt."
  #elif LASER_PWM_PIN == 4 || WITHIN(LASER_PWM_PIN, 11, 13)
    #error "Counter/Timer for LASER_PWM_PIN is used by a system interrupt."
  #elif NUM_SERVOS > 0 && defined(_useTimer3) && (WITHIN(SPINDLE_LASER_PWM_PIN, 2, 3) || SPINDLE_LASER_PWM_PIN == 5)
    #error "Counter/Timer for SPINDLE_LASER_PWM_PIN is used by the servo system."
  #endif
#elif SPINDLE_LASER_FREQUENCY
  #error "SPINDLE_LASER_FREQUENCY requires SPINDLE_LASER_USE_PWM."
#endif

/**
 * The Trinamic library includes SoftwareSerial.h, leading to a compile error.
 */
#if ALL(HAS_TRINAMIC_CONFIG, ENDSTOP_INTERRUPTS_FEATURE)
  #error "TMCStepper includes SoftwareSerial.h which is incompatible with ENDSTOP_INTERRUPTS_FEATURE. Disable ENDSTOP_INTERRUPTS_FEATURE to continue."
#endif

#if ALL(HAS_TMC_SW_SERIAL, MONITOR_DRIVER_STATUS)
  #error "MONITOR_DRIVER_STATUS causes performance issues when used with SoftwareSerial-connected drivers. Disable MONITOR_DRIVER_STATUS or use hardware serial to continue."
#endif

/**
 * Postmortem debugging
 */
#if ENABLED(POSTMORTEM_DEBUGGING)
  #error "POSTMORTEM_DEBUGGING is not supported on AVR boards."
#endif

#if USING_PULLDOWNS
  #error "PULLDOWN pin mode is not available on AVR boards."
#endif
