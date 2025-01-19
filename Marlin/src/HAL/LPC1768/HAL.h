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
 * HAL_LPC1768/HAL.h
 * Hardware Abstraction Layer for NXP LPC1768
 */

#define CPU_32_BIT

#include <stdint.h>
#include <stdarg.h>
#include <algorithm>

extern "C" volatile uint32_t _millis;

#include "../shared/Marduino.h"
#include "../shared/math_32bit.h"
#include "../shared/HAL_SPI.h"
#include "fastio.h"

#include <adc.h>
#include <pinmapping.h>

//
// Serial Ports
//

#include "MarlinSerial.h"

//
// Interrupts
//

#define CRITICAL_SECTION_START()  const bool irqon = !__get_PRIMASK(); __disable_irq()
#define CRITICAL_SECTION_END()    if (irqon) __enable_irq()

//
// ADC
//

#define ADC_MEDIAN_FILTER_SIZE (23) // Higher values increase step delay (phase shift),
                                    // (ADC_MEDIAN_FILTER_SIZE + 1) / 2 sample step delay (12 samples @ 500Hz: 24ms phase shift)
                                    // Memory usage per ADC channel (bytes): (6 * ADC_MEDIAN_FILTER_SIZE) + 16
                                    // 8 * ((6 * 23) + 16 ) = 1232 Bytes for 8 channels

#define ADC_LOWPASS_K_VALUE    (2)  // Higher values increase rise time
                                    // Rise time sample delays for 100% signal convergence on full range step
                                    // (1 : 13, 2 : 32, 3 : 67, 4 : 139, 5 : 281, 6 : 565, 7 : 1135, 8 : 2273)
                                    // K = 6, 565 samples, 500Hz sample rate, 1.13s convergence on full range step
                                    // Memory usage per ADC channel (bytes): 4 (32 Bytes for 8 channels)

#define HAL_ADC_VREF_MV      3300   // ADC voltage reference

#define HAL_ADC_RESOLUTION     12   // 15 bit maximum, raw temperature is stored as int16_t
#define HAL_ADC_FILTERED            // Disable oversampling done in Marlin as ADC values already filtered in HAL

//
// Pin Mapping for M42, M43, M226
//

// Test whether the pin is valid
constexpr bool isValidPin(const pin_t pin) {
  return LPC176x::pin_is_valid(pin);
}

// Get the analog index for a digital pin
constexpr int8_t digitalPinToAnalogIndex(const pin_t pin) {
  return (LPC176x::pin_is_valid(pin) && LPC176x::pin_has_adc(pin)) ? pin : -1;
}

// Return the index of a pin number
constexpr int16_t GET_PIN_MAP_INDEX(const pin_t pin) {
  return LPC176x::pin_index(pin);
}

// Get the pin number at the given index
constexpr pin_t GET_PIN_MAP_PIN(const int16_t index) {
  return LPC176x::pin_index(index);
}

// Parse a G-code word into a pin index
int16_t PARSED_PIN_INDEX(const char code, const int16_t dval);
// P0.6 thru P0.9 are for the onboard SD card
#define HAL_SENSITIVE_PINS P0_06, P0_07, P0_08, P0_09

// ------------------------
// Defines
// ------------------------

#ifndef PLATFORM_M997_SUPPORT
  #define PLATFORM_M997_SUPPORT
#endif
void flashFirmware(const int16_t);

#define HAL_CAN_SET_PWM_FREQ   // This HAL supports PWM Frequency adjustment

// Default graphical display delays
#define CPU_ST7920_DELAY_1 600
#define CPU_ST7920_DELAY_2 750
#define CPU_ST7920_DELAY_3 750

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

  static void init();          // Called early in setup()
  static void init_board() {}  // Called less early in setup()
  static void reboot();        // Restart the firmware from 0x0

  // Interrupts
  static bool isr_state() { return !__get_PRIMASK(); }
  static void isr_on()  { __enable_irq(); }
  static void isr_off() { __disable_irq(); }

  static void delay_ms(const int ms) { _delay_ms(ms); }

  // Watchdog
  static void watchdog_init() IF_DISABLED(USE_WATCHDOG, {});
  static void watchdog_refresh() IF_DISABLED(USE_WATCHDOG, {});
  static bool watchdog_timed_out() IF_DISABLED(USE_WATCHDOG, { return false; });
  static void watchdog_clear_timeout_flag() IF_DISABLED(USE_WATCHDOG, {});

  // Tasks, called from idle()
  static void idletask();

  // Reset
  static uint8_t get_reset_source();
  static void clear_reset_source();

  // Free SRAM
  static int freeMemory() { return ::freeMemory(); }

  //
  // ADC Methods
  //

  using FilteredADC = LPC176x::ADC<ADC_LOWPASS_K_VALUE, ADC_MEDIAN_FILTER_SIZE>;

  // Called by Temperature::init once at startup
  static void adc_init() {}

  // Called by Temperature::init for each sensor at startup
  static void adc_enable(const pin_t pin) {
    FilteredADC::enable_channel(pin);
  }

  // Begin ADC sampling on the given pin. Called from Temperature::isr!
  static uint32_t adc_result;
  static pin_t adc_pin;

  static void adc_start(const pin_t pin) { adc_pin = pin; }

  // Is the ADC ready for reading?
  static bool adc_ready() { return LPC176x::adc_hardware.done(LPC176x::pin_get_adc_channel(adc_pin)); }

  // The current value of the ADC register
  static uint16_t adc_value() {
    adc_result = FilteredADC::read(adc_pin) >> (16 - HAL_ADC_RESOLUTION); // returns 16bit value, reduce to required bits
    return uint16_t(adc_result);
  }

  /**
   * Set the PWM duty cycle for the pin to the given value.
   * Optionally invert the duty cycle [default = false]
   * Optionally change the scale of the provided value to enable finer PWM duty control [default = 255]
   */
  static void set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t v_size=255, const bool invert=false);

  /**
   * Set the frequency of the timer corresponding to the provided pin
   * All Hardware PWM pins will run at the same frequency and
   * All Software PWM pins will run at the same frequency
   */
  static void set_pwm_frequency(const pin_t pin, const uint16_t f_desired);
};
