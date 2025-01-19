/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * SAMD21 HAL developed by Bart Meijer (brupje)
 * Based on SAMD51 HAL by Giuliano Zaro (AKA GMagician)
 */

#define CPU_32_BIT

#include "../shared/Marduino.h"
#include "../shared/math_32bit.h"
#include "../shared/HAL_SPI.h"
#include "fastio.h"

// ------------------------
// Serial ports
// ------------------------

#include "../../core/serial_hook.h"
typedef ForwardSerial1Class< decltype(SerialUSB) > DefaultSerial1;
extern DefaultSerial1 MSerialUSB;

typedef ForwardSerial1Class< decltype(Serial1) > DefaultSerial2;
typedef ForwardSerial1Class< decltype(Serial2) > DefaultSerial3;

extern DefaultSerial2 MSerial0;
extern DefaultSerial3 MSerial1;

#define __MSERIAL(X) MSerial##X
#define _MSERIAL(X) __MSERIAL(X)
#define MSERIAL(X) _MSERIAL(INCREMENT(X))

#define SERIAL_INDEX_MIN 0
#define SERIAL_INDEX_MAX 1
#define USB_SERIAL_PORT(...) MSerialUSB
#include "../shared/serial_ports.h"

typedef int8_t pin_t;

#define SHARED_SERVOS HAS_SERVOS  // Use shared/servos.cpp

class Servo;
typedef Servo hal_servo_t;

//
// Interrupts
//
#define CRITICAL_SECTION_START()  const bool irqon = !__get_PRIMASK(); __disable_irq()
#define CRITICAL_SECTION_END()    if (irqon) __enable_irq()

#define cli() __disable_irq() // Disable interrupts
#define sei() __enable_irq()  // Enable interrupts

//
// ADC
//

#define HAL_ADC_FILTERED     1     // Disable Marlin's oversampling. The HAL filters ADC values.
#define HAL_ADC_VREF_MV   3300
#define HAL_ADC_RESOLUTION  12
#define HAL_ADC_AIN_START ADC_INPUTCTRL_MUXPOS_PIN3
#define HAL_ADC_AIN_NUM_SENSORS 3
#define HAL_ADC_AIN_LEN HAL_ADC_AIN_NUM_SENSORS-1

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

#ifdef __cplusplus
  extern "C" {
#endif

char *dtostrf(double __val, signed char __width, unsigned char __prec, char *__s);

extern "C" int freeMemory();

#ifdef __cplusplus
  }
#endif

#pragma GCC diagnostic pop

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
  static void init_board() {}  // Called less early in setup()
  static void reboot();        // Restart the firmware from 0x0

  // Interrupts
  static bool isr_state() { return !__get_PRIMASK(); }
  static void isr_on()  { sei(); }
  static void isr_off() { cli(); }

  static void delay_ms(const int ms) { delay(ms); }

  // Tasks, called from idle()
  static void idletask() {}

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
  static void adc_init();

  // Called by Temperature::init for each sensor at startup
  static void adc_enable(const uint8_t ch) {}

  // Begin ADC sampling on the given pin. Called from Temperature::isr!
  static void adc_start(const pin_t pin);

  // Is the ADC ready for reading?
  static bool adc_ready() { return true; }

  // The current value of the ADC register
  static uint16_t adc_value() { return adc_result; }

  /**
   * Set the PWM duty cycle for the pin to the given value.
   * No option to invert the duty cycle [default = false]
   * No option to change the scale of the provided value to enable finer PWM duty control [default = 255]
   */
  static void set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t=255, const bool=false) {
    analogWrite(pin, v);
  }

private:
  static void dma_init();
};
