/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#include <core_util.h>

#if !defined(ARDUINO_CORE_VERSION_INT) || !defined(GET_VERSION_INT)
  // version macros were introduced in arduino core version 1.1.0
  // below that version, we polyfill them
  #define GET_VERSION_INT(major, minor, patch) ((major * 100000) + (minor * 1000) + patch)
  #define ARDUINO_CORE_VERSION_INT GET_VERSION_INT(1, 0, 0)
#endif

#if ARDUINO_CORE_VERSION_INT < GET_VERSION_INT(1, 1, 0)
  // because we use app_config.h introduced in arduino core version 1.1.0, the
  // HAL is not compatible with older versions
  #error "The HC32 HAL is not compatible with Arduino Core versions < 1.1.0. Consider updating the Arduino Core."
#endif

#ifndef BOARD_XTAL_FREQUENCY
  #error "BOARD_XTAL_FREQUENCY is required for HC32F460."
#endif

#if ENABLED(FAST_PWM_FAN)
  #error "FAST_PWM_FAN is not yet implemented for this platform."
#endif

#if !defined(HAVE_SW_SERIAL) && HAS_TMC_SW_SERIAL
  #error "Missing SoftwareSerial implementation."
#endif

#if ENABLED(SDCARD_EEPROM_EMULATION) && !HAS_MEDIA
  #undef SDCARD_EEPROM_EMULATION // Avoid additional error noise
  #if USE_FALLBACK_EEPROM
    #warning "EEPROM type not specified. Fallback is SDCARD_EEPROM_EMULATION."
  #endif

  #error "SDCARD_EEPROM_EMULATION requires SDSUPPORT. Enable SDSUPPORT or choose another EEPROM emulation."
#endif

#if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
  #error "SERIAL_STATS_MAX_RX_QUEUED is not supported on this platform."
#elif ENABLED(SERIAL_STATS_DROPPED_RX)
  #error "SERIAL_STATS_DROPPED_RX is not supported on this platform."
#endif

#if ENABLED(NEOPIXEL_LED) && DISABLED(MKS_MINI_12864_V3)
  #error "NEOPIXEL_LED (Adafruit NeoPixel) is not supported for HC32F460. Comment out this line to proceed at your own risk!"
#endif

// Emergency Parser needs at least one serial with HardwareSerial.
#if ENABLED(EMERGENCY_PARSER) && ((SERIAL_PORT == -1 && !defined(SERIAL_PORT_2)) || (SERIAL_PORT_2 == -1 && !defined(SERIAL_PORT)))
  #error "EMERGENCY_PARSER is only supported by HardwareSerial on HC32F460."
#endif

#if TEMP_SENSOR_SOC
  #ifndef TEMP_SOC_PIN
    #error "TEMP_SOC_PIN must be defined to use TEMP_SENSOR_SOC."
  #elif IS_GPIO_PIN(TEMP_SOC_PIN)
    #error "TEMP_SOC_PIN must not be a valid GPIO pin to avoid conflicts."
  #endif
#endif

#if ENABLED(POSTMORTEM_DEBUGGING) && !defined(CORE_DISABLE_FAULT_HANDLER)
  #error "POSTMORTEM_DEBUGGING requires CORE_DISABLE_FAULT_HANDLER to be set."
#endif

#if defined(PANIC_ENABLE)
  #if defined(PANIC_USART1_TX_PIN) || defined(PANIC_USART2_TX_PIN) || defined(PANIC_USART3_TX_PIN) || defined(PANIC_USART3_TX_PIN)
    #error "HC32 HAL uses a custom panic handler. Do not define PANIC_USARTx_TX_PIN."
  #endif
#endif

#if ENABLED(SERIAL_DMA)
  #if !defined(USART_RX_DMA_SUPPORT)
    #error "SERIAL_DMA requires USART_RX_DMA_SUPPORT to be enabled in the arduino core."
  #endif

  // Before arduino core version 1.2.0, USART_RX_DMA_SUPPORT did not implement
  // core_hook_usart_rx_irq, which is required for the emergency parser.
  // With 1.2.0, this was fixed (see https://github.com/shadow578/framework-arduino-hc32f46x/pull/25).
  #if ENABLED(EMERGENCY_PARSER) && ARDUINO_CORE_VERSION_INT < GET_VERSION_INT(1, 2, 0)
    #error "EMERGENCY_PARSER is not supported with SERIAL_DMA. Please disable either SERIAL_DMA or EMERGENCY_PARSER."
  #endif

  #if ARDUINO_CORE_VERSION_INT < GET_VERSION_INT(1, 1, 0)
    #error "SERIAL_DMA is not supported with arduino core version < 1.1.0."
  #endif
#endif
