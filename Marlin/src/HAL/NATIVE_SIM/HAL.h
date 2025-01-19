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

#include <stdint.h>
#include <stdarg.h>
#undef min
#undef max
#include <algorithm>
#include "pinmapping.h"

void _printf (const  char *format, ...);
void _putc(uint8_t c);
uint8_t _getc();

//arduino: Print.h
#define DEC 10
#define HEX 16
#define OCT  8
#define BIN  2
//arduino: binary.h (weird defines)
#define B01 1
#define B10 2

#include "../shared/Marduino.h"
#include "../shared/math_32bit.h"
#include "../shared/HAL_SPI.h"
#include "fastio.h"
#include "serial.h"

// ------------------------
// Defines
// ------------------------

#define CPU_32_BIT
#define SHARED_SERVOS HAS_SERVOS  // Use shared/servos.cpp

#define F_CPU 100000000
#define SystemCoreClock F_CPU

#define CPU_ST7920_DELAY_1 600
#define CPU_ST7920_DELAY_2 750
#define CPU_ST7920_DELAY_3 750

// ------------------------
// Serial ports
// ------------------------

extern MSerialT serial_stream_0;
extern MSerialT serial_stream_1;
extern MSerialT serial_stream_2;
extern MSerialT serial_stream_3;

#define _MSERIAL(X) serial_stream_##X

#define SERIAL_INDEX_MIN 0
#define SERIAL_INDEX_MAX 3
#include "../shared/serial_ports.h"

// ------------------------
// Interrupts
// ------------------------

#define CRITICAL_SECTION_START()
#define CRITICAL_SECTION_END()

// ------------------------
// ADC
// ------------------------

#define HAL_ADC_VREF_MV   5000
#define HAL_ADC_RESOLUTION  10

/* ---------------- Delay in cycles */

#define DELAY_CYCLES(x) Kernel::delayCycles(x)
#define SYSTEM_YIELD() Kernel::yield()

// Maple Compatibility
typedef void (*systickCallback_t)(void);
void systick_attach_callback(systickCallback_t cb);
extern volatile uint32_t systick_uptime_millis;

// Marlin uses strstr in constexpr context, this is not supported, workaround by defining constexpr versions of the required functions.
#define strstr(a, b) strstr_constexpr((a), (b))

constexpr inline std::size_t strlen_constexpr(const char* str) {
  // https://github.com/gcc-mirror/gcc/blob/5c7634a0e5f202935aa6c11b6ea953b8bf80a00a/libstdc%2B%2B-v3/include/bits/char_traits.h#L329
  if (str != nullptr) {
    std::size_t i = 0;
    while (str[i] != '\0') ++i;
    return i;
  }
  return 0;
}

constexpr inline int strncmp_constexpr(const char* lhs, const char* rhs, std::size_t count) {
  // https://github.com/gcc-mirror/gcc/blob/13b9cbfc32fe3ac4c81c4dd9c42d141c8fb95db4/libstdc%2B%2B-v3/include/bits/char_traits.h#L655
  if (lhs == nullptr || rhs == nullptr)
    return rhs != nullptr ? -1 : 1;

  for (std::size_t i = 0; i < count; ++i)
    if (lhs[i] != rhs[i])
      return lhs[i] < rhs[i] ? -1 : 1;
    else if (lhs[i] == '\0')
      return 0;

  return 0;
}

constexpr inline const char* strstr_constexpr(const char* str, const char* target) {
  // https://github.com/freebsd/freebsd/blob/master/sys/libkern/strstr.c
  if (char c = target != nullptr ? *target++ : '\0'; c != '\0' && str != nullptr) {
    std::size_t len = strlen_constexpr(target);
    do {
      char sc = {};
      do {
        if ((sc = *str++) == '\0') return nullptr;
      } while (sc != c);
    } while (strncmp_constexpr(str, target, len) != 0);
    --str;
  }
  return str;
}

constexpr inline char* strstr_constexpr(char* str, const char* target) {
  // https://github.com/freebsd/freebsd/blob/master/sys/libkern/strstr.c
  if (char c = target != nullptr ? *target++ : '\0'; c != '\0' && str != nullptr) {
    std::size_t len = strlen_constexpr(target);
    do {
      char sc = {};
      do {
        if ((sc = *str++) == '\0') return nullptr;
      } while (sc != c);
    } while (strncmp_constexpr(str, target, len) != 0);
    --str;
  }
  return str;
}

// ------------------------
// Free Memory Accessor
// ------------------------

#pragma GCC diagnostic push
#if GCC_VERSION <= 50000
  #pragma GCC diagnostic ignored "-Wunused-function"
#endif

int freeMemory();

#pragma GCC diagnostic pop

// ------------------------
// MarlinHAL Class
// ------------------------

class MarlinHAL {
public:

  // Earliest possible init, before setup()
  MarlinHAL() {}

  // Watchdog
  static void watchdog_init();
  static void watchdog_refresh();

  static void init() {}        // Called early in setup()
  static void init_board() {}  // Called less early in setup()
  static void reboot();        // Restart the firmware from 0x0

  // Interrupts
  static bool isr_state() { return true; }
  static void isr_on()  {}
  static void isr_off() {}

  static void delay_ms(const int ms) { _delay_ms(ms); }

  // Tasks, called from idle()
  static void idletask();

  // Reset
  static constexpr uint8_t reset_reason = RST_POWER_ON;
  static uint8_t get_reset_source() { return reset_reason; }
  static void clear_reset_source() {}

  // Free SRAM
  static int freeMemory() { return ::freeMemory(); }

  //
  // ADC Methods
  //

  static uint8_t active_ch;

  // Called by Temperature::init once at startup
  static void adc_init();

  // Called by Temperature::init for each sensor at startup
  static void adc_enable(const uint8_t ch);

  // Begin ADC sampling on the given channel. Called from Temperature::isr!
  static void adc_start(const uint8_t ch);

  // Is the ADC ready for reading?
  static bool adc_ready();

  // The current value of the ADC register
  static uint16_t adc_value();

  /**
   * Set the PWM duty cycle for the pin to the given value.
   * No option to invert the duty cycle [default = false]
   * No option to change the scale of the provided value to enable finer PWM duty control [default = 255]
   */
  static void set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t=255, const bool=false) {
    analogWrite(pin, v);
  }

  static void set_pwm_frequency(const pin_t, int) {}

  #ifndef HAS_LIBBSD
    /**
     * Redirect missing strlcpy here
     */
    static size_t _strlcpy(char *dst, const char *src, size_t dsize);
    #define strlcpy hal._strlcpy
  #endif

};
