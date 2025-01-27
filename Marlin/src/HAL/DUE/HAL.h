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
 * HAL for Arduino Due and compatible (SAM3X8E)
 */

#define CPU_32_BIT

#include "../shared/Marduino.h"
#include "../shared/eeprom_if.h"
#include "../shared/math_32bit.h"
#include "../shared/HAL_SPI.h"
#include "fastio.h"

#include <stdint.h>

//
// Serial Ports
//

#include "MarlinSerial.h"
#include "MarlinSerialUSB.h"

// ------------------------
// Types
// ------------------------

typedef int8_t pin_t;

#define SHARED_SERVOS HAS_SERVOS  // Use shared/servos.cpp

class Servo;
typedef Servo hal_servo_t;

//
// Interrupts
//
#define sei() interrupts()
#define cli() noInterrupts()

#define CRITICAL_SECTION_START()  const bool _irqon = hal.isr_state(); hal.isr_off()
#define CRITICAL_SECTION_END()    if (_irqon) hal.isr_on()

//
// ADC
//
#define HAL_ADC_VREF_MV   3300
#define HAL_ADC_RESOLUTION  10

#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) pin_t((p < 12U) ? (p) + 54U : -1)
#endif

//
// Pin Mapping for M42, M43, M226
//
#define GET_PIN_MAP_PIN(index) index
#define GET_PIN_MAP_INDEX(pin) pin
#define PARSED_PIN_INDEX(code, dval) parser.intval(code, dval)

//
// Tone
//
void tone(const pin_t _pin, const unsigned int frequency, const unsigned long duration=0);
void noTone(const pin_t _pin);

// ------------------------
// Class Utilities
// ------------------------

#pragma GCC diagnostic push
#if GCC_VERSION <= 50000
  #pragma GCC diagnostic ignored "-Wunused-function"
#endif

#pragma GCC diagnostic pop

#ifdef __cplusplus
  extern "C" {
#endif
char *dtostrf(double __val, signed char __width, unsigned char __prec, char *__s);
#ifdef __cplusplus
  }
#endif

// Return free RAM between end of heap (or end bss) and whatever is current
int freeMemory();

// ------------------------
// MarlinHAL Class
// ------------------------

class MarlinHAL {
public:

  // Earliest possible init, before setup()
  MarlinHAL() {}

  // Watchdog
  static void watchdog_init()    IF_DISABLED(USE_WATCHDOG, {});
  static void watchdog_refresh() IF_DISABLED(USE_WATCHDOG, {});

  static void init();          // Called early in setup()
  static void init_board();    // Called less early in setup()
  static void reboot();        // Restart the firmware

  // Interrupts
  static bool isr_state() { return !__get_PRIMASK(); }
  static void isr_on()  { __enable_irq(); }
  static void isr_off() { __disable_irq(); }

  static void delay_ms(const int ms) { delay(ms); }

  // Tasks, called from idle()
  static void idletask();

  // Reset
  static uint8_t get_reset_source();
  static void clear_reset_source() {}

  // Free SRAM
  static int freeMemory() { return ::freeMemory(); }

  //
  // ADC Methods
  //

  static uint16_t adc_result;

  // Called by Temperature::init once at startup
  static void adc_init() {}

  // Called by Temperature::init for each sensor at startup
  static void adc_enable(const uint8_t /*ch*/) {}

  // Begin ADC sampling on the given channel. Called from Temperature::isr!
  static void adc_start(const uint8_t ch) { adc_result = analogRead(ch); }

  // Is the ADC ready for reading?
  static bool adc_ready() { return true; }

  // The current value of the ADC register
  static uint16_t adc_value() { return adc_result; }

  /**
   * Set the PWM duty cycle for the pin to the given value.
   * No inverting the duty cycle in this HAL.
   * No changing the maximum size of the provided value to enable finer PWM duty control in this HAL.
   */
  static void set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t=255, const bool=false) {
    analogWrite(pin, v);
  }

};
