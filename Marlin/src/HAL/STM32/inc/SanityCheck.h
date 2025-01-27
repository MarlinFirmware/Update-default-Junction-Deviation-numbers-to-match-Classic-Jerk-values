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
 * Test STM32-specific configuration values for errors at compile-time.
 */
//#if ENABLED(SPINDLE_LASER_USE_PWM) && !(SPINDLE_LASER_PWM_PIN == 4 || SPINDLE_LASER_PWM_PIN == 6 || SPINDLE_LASER_PWM_PIN == 11)
//  #error "SPINDLE_LASER_PWM_PIN must use SERVO0, SERVO1 or SERVO3 connector"
//#endif

#if ENABLED(SDCARD_EEPROM_EMULATION) && !HAS_MEDIA
  #undef SDCARD_EEPROM_EMULATION // Avoid additional error noise
  #if USE_FALLBACK_EEPROM
    #warning "EEPROM type not specified. Fallback is SDCARD_EEPROM_EMULATION."
  #endif
  #error "SDCARD_EEPROM_EMULATION requires SDSUPPORT. Enable SDSUPPORT or choose another EEPROM emulation."
#endif

#if NONE(STM32F4xx, STM32H7xx) && ENABLED(FLASH_EEPROM_LEVELING)
  #error "FLASH_EEPROM_LEVELING is currently only supported on STM32F4/H7 hardware." // IRON
#endif

#if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
  #error "SERIAL_STATS_MAX_RX_QUEUED is not supported on STM32."
#elif ENABLED(SERIAL_STATS_DROPPED_RX)
  #error "SERIAL_STATS_DROPPED_RX is not supported on STM32."
#endif

#if ANY(TFT_COLOR_UI, TFT_LVGL_UI, TFT_CLASSIC_UI) && NOT_TARGET(STM32H7xx, STM32F4xx, STM32F1xx)
  #error "TFT_COLOR_UI, TFT_LVGL_UI and TFT_CLASSIC_UI are currently only supported on STM32H7, STM32F4 and STM32F1 hardware."
#endif

#if TEMP_SENSOR_SOC && defined(ATEMP) && TEMP_SOC_PIN != ATEMP
  #error "TEMP_SENSOR_SOC requires 'TEMP_SOC_PIN ATEMP' on STM32."
#endif

/**
 * Check for common serial pin conflicts
 */
#define _CHECK_SERIAL_PIN(N) (( \
    BTN_EN1 == N || BTN_EN2 == N || DOGLCD_CS == N || HEATER_BED_PIN == N || FAN0_PIN == N || \
    SDIO_D2_PIN == N || SDIO_D3_PIN == N || SDIO_CK_PIN == N || SDIO_CMD_PIN == N || \
    Y_STEP_PIN == N || Y_ENABLE_PIN == N || E0_ENABLE_PIN == N || POWER_LOSS_PIN == N \
  ))
#define CHECK_SERIAL_PIN(T,N) defined(UART##N##_##T##_PIN) && _CHECK_SERIAL_PIN(UART##N##_##T##_PIN)
#if SERIAL_IN_USE(1)
  #if CHECK_SERIAL_PIN(TX,1)
    #error "Serial Port 1 TX IO pins conflict with another pin on the board."
  #endif
  #if CHECK_SERIAL_PIN(RX,1)
    #error "Serial Port 1 RX IO pins conflict with another pin on the board."
  #endif
#endif
#if SERIAL_IN_USE(2)
  #if CHECK_SERIAL_PIN(TX,2)
    #error "Serial Port 2 TX IO pins conflict with another pin on the board."
  #endif
  #if CHECK_SERIAL_PIN(RX,2)
    #error "Serial Port 2 RX IO pins conflict with another pin on the board."
  #endif
#endif
#if SERIAL_IN_USE(3)
  #if CHECK_SERIAL_PIN(TX,3)
    #error "Serial Port 3 TX IO pins conflict with another pin on the board."
  #endif
  #if CHECK_SERIAL_PIN(RX,3)
    #error "Serial Port 3 RX IO pins conflict with another pin on the board."
  #endif
#endif
#if SERIAL_IN_USE(4)
  #if CHECK_SERIAL_PIN(TX,4)
    #error "Serial Port 4 TX IO pins conflict with another pin on the board."
  #endif
  #if CHECK_SERIAL_PIN(RX,4)
    #error "Serial Port 4 RX IO pins conflict with another pin on the board."
  #endif
#endif
#if SERIAL_IN_USE(5)
  #if CHECK_SERIAL_PIN(TX,5)
    #error "Serial Port 5 TX IO pins conflict with another pin on the board."
  #endif
  #if CHECK_SERIAL_PIN(RX,5)
    #error "Serial Port 5 RX IO pins conflict with another pin on the board."
  #endif
#endif
#if SERIAL_IN_USE(6)
  #if CHECK_SERIAL_PIN(TX,6)
    #error "Serial Port 6 TX IO pins conflict with another pin on the board."
  #endif
  #if CHECK_SERIAL_PIN(RX,6)
    #error "Serial Port 6 RX IO pins conflict with another pin on the board."
  #endif
#endif
#undef CHECK_SERIAL_PIN
#undef _CHECK_SERIAL_PIN
