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

#define CPU_32_BIT

#include "../../core/macros.h"
#include "../shared/Marduino.h"
#include "../shared/math_32bit.h"
#include "../shared/HAL_SPI.h"
#include "temp_soc.h"
#include "fastio.h"
#include "Servo.h"

#include "../../inc/MarlinConfigPre.h"

#include <stdint.h>

//
// Default graphical display delays
//
#define CPU_ST7920_DELAY_1 300
#define CPU_ST7920_DELAY_2  40
#define CPU_ST7920_DELAY_3 340

//
// Serial Ports
//

#include "MarlinSerial.h"

/**
 * TODO: review this to return 1 for pins that are not analog input
 */
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) pin_t(p)
#endif

//
// Interrupts
//
#define CRITICAL_SECTION_START()  const bool irqon = !__get_PRIMASK(); __disable_irq()
#define CRITICAL_SECTION_END()    if (irqon) __enable_irq()
#define cli() __disable_irq()
#define sei() __enable_irq()

// ------------------------
// Types
// ------------------------

typedef double isr_float_t;   // FPU ops are used for single-precision, so use double for ISRs.

typedef int32_t pin_t;        // Parity with platform/ststm32

class libServo;
typedef libServo hal_servo_t;
#define PAUSE_SERVO_OUTPUT() libServo::pause_all_servos()
#define RESUME_SERVO_OUTPUT() libServo::resume_all_servos()

// ------------------------
// ADC
// ------------------------

#ifdef ADC_RESOLUTION
  #define HAL_ADC_RESOLUTION ADC_RESOLUTION
#else
  #define HAL_ADC_RESOLUTION 12
#endif

#define HAL_ADC_VREF_MV   3300

//
// Pin Mapping for M42, M43, M226
//
#define GET_PIN_MAP_PIN(index) index
#define GET_PIN_MAP_INDEX(pin) pin
#define PARSED_PIN_INDEX(code, dval) parser.intval(code, dval)

#ifdef STM32F1xx
  #define JTAG_DISABLE() AFIO_DBGAFR_CONFIG(AFIO_MAPR_SWJ_CFG_JTAGDISABLE)
  #define JTAGSWD_DISABLE() AFIO_DBGAFR_CONFIG(AFIO_MAPR_SWJ_CFG_DISABLE)
  #define JTAGSWD_RESET() AFIO_DBGAFR_CONFIG(AFIO_MAPR_SWJ_CFG_RESET); // Reset: FULL SWD+JTAG
#endif

#ifndef PLATFORM_M997_SUPPORT
  #define PLATFORM_M997_SUPPORT
#endif
void flashFirmware(const int16_t);

// Maple Compatibility
typedef void (*systickCallback_t)(void);
void systick_attach_callback(systickCallback_t cb);
void HAL_SYSTICK_Callback();

extern volatile uint32_t systick_uptime_millis;

#define HAL_CAN_SET_PWM_FREQ   // This HAL supports PWM Frequency adjustment

// ------------------------
// Class Utilities
// ------------------------

// Memory related
#define __bss_end __bss_end__

extern "C" char* _sbrk(int incr);

#pragma GCC diagnostic push
#if GCC_VERSION <= 50000
  #pragma GCC diagnostic ignored "-Wunused-function"
#endif

static inline int freeMemory() {
  volatile char top;
  return &top - reinterpret_cast<char*>(_sbrk(0));
}

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
  static void idletask();

  // Reset
  static uint8_t get_reset_source();
  static void clear_reset_source();

  // Free SRAM
  static int freeMemory() { return ::freeMemory(); }

  //
  // ADC Methods
  //

  static uint16_t adc_result;

  // Called by Temperature::init once at startup
  static void adc_init() {
    analogReadResolution(HAL_ADC_RESOLUTION);
  }

  // Called by Temperature::init for each sensor at startup
  static void adc_enable(const pin_t pin) { pinMode(pin, INPUT); }

  // Begin ADC sampling on the given pin. Called from Temperature::isr!
  static void adc_start(const pin_t pin) { adc_result = analogRead(pin); }

  // Is the ADC ready for reading?
  static bool adc_ready() { return true; }

  // The current value of the ADC register
  static uint16_t adc_value() { return adc_result; }

  /**
   * Set the PWM duty cycle for the pin to the given value.
   * Optionally invert the duty cycle [default = false]
   * Optionally change the maximum size of the provided value to enable finer PWM duty control [default = 255]
   */
  static void set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t v_size=255, const bool invert=false);

  /**
   * Set the frequency of the timer for the given pin.
   * All Timer PWM pins run at the same frequency.
   */
  static void set_pwm_frequency(const pin_t pin, const uint16_t f_desired);

};
