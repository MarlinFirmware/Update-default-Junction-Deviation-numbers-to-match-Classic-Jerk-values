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
#ifdef __STM32F1__

#include "../../inc/MarlinConfig.h"

#include <pwm.h>

#define NR_TIMERS TERN(STM32_XL_DENSITY, 14, 8) // Maple timers, 14 for STM32_XL_DENSITY (F/G chips), 8 for HIGH density (C D E)

static uint16_t timer_freq[NR_TIMERS];

inline uint8_t timer_and_index_for_pin(const pin_t pin, timer_dev **timer_ptr) {
  *timer_ptr = PIN_MAP[pin].timer_device;
  for (uint_fast8_t i = 0; i < NR_TIMERS; i++) if (*timer_ptr == HAL_get_timer_dev(i))
    return i;
  return 0;
}

void MarlinHAL::set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t v_size/*=255*/, const bool invert/*=false*/) {
  const uint16_t duty = invert ? v_size - v : v;
  if (PWM_PIN(pin)) {
    timer_dev *timer;
    if (timer_freq[timer_and_index_for_pin(pin, &timer)] == 0)
      set_pwm_frequency(pin, PWM_FREQUENCY);
    const uint8_t channel = PIN_MAP[pin].timer_channel;
    timer_set_compare(timer, channel, duty);
    timer_set_mode(timer, channel, TIMER_PWM); // PWM Output Mode
  }
  else {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, duty < v_size / 2 ? LOW : HIGH);
  }
}

void MarlinHAL::set_pwm_frequency(const pin_t pin, const uint16_t f_desired) {
  if (!PWM_PIN(pin)) return;                    // Don't proceed if no hardware timer

  timer_dev *timer;
  timer_freq[timer_and_index_for_pin(pin, &timer)] = f_desired;

  // Protect used timers
  if (timer == HAL_get_timer_dev(MF_TIMER_TEMP)) return;
  if (timer == HAL_get_timer_dev(MF_TIMER_STEP)) return;
  #if MF_TIMER_PULSE != MF_TIMER_STEP
    if (timer == HAL_get_timer_dev(MF_TIMER_PULSE)) return;
  #endif

  if (!(timer->regs.bas->SR & TIMER_CR1_CEN))   // Ensure the timer is enabled
    timer_init(timer);

  const uint8_t channel = PIN_MAP[pin].timer_channel;
  timer_set_mode(timer, channel, TIMER_PWM);
  // Preload (resolution) cannot be equal to duty of 255 otherwise it may not result in digital off or on.
  uint16_t preload = 254;
  int32_t prescaler = (HAL_TIMER_RATE) / (preload + 1) / f_desired - 1;
  if (prescaler > 65535) {                      // For low frequencies increase prescaler
    prescaler = 65535;
    preload = (HAL_TIMER_RATE) / (prescaler + 1) / f_desired - 1;
  }
  if (prescaler < 0) return;                    // Too high frequency
  timer_set_reload(timer, preload);
  timer_set_prescaler(timer, prescaler);
}

#endif // __STM32F1__
