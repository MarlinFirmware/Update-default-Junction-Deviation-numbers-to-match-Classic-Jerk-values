/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#include "../platforms.h"

#ifdef __PLAT_RP2040__

#include "../../inc/MarlinConfig.h"

alarm_pool_t* HAL_timer_pool_0;
alarm_pool_t* HAL_timer_pool_1;
alarm_pool_t* HAL_timer_pool_2;
alarm_pool_t* HAL_timer_pool_3;

struct repeating_timer HAL_timer_0;
struct repeating_timer HAL_timer_1;
struct repeating_timer HAL_timer_2;
struct repeating_timer HAL_timer_3;

volatile bool HAL_timer_irq_en[4] = { false, false, false, false };

void HAL_timer_init() {
  //reserve all the available alarm pools to use as "pseudo" hardware timers
  //HAL_timer_pool_0 = alarm_pool_create(0,2);
  HAL_timer_pool_1 = alarm_pool_create(1, 6);
  HAL_timer_pool_0 = HAL_timer_pool_1;
  HAL_timer_pool_2 = alarm_pool_create(2, 6);
  HAL_timer_pool_3 = HAL_timer_pool_2;
  //HAL_timer_pool_3 = alarm_pool_create(3, 6);

  irq_set_priority(TIMER_IRQ_0, 0xC0);
  irq_set_priority(TIMER_IRQ_1, 0x80);
  irq_set_priority(TIMER_IRQ_2, 0x40);
  irq_set_priority(TIMER_IRQ_3, 0x00);

  //alarm_pool_init_default();
}

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  const int64_t freq = (int64_t)frequency,
                  us = (1000000ll / freq) * -1ll;
  bool result;
  switch (timer_num) {
    case 0:
      result = alarm_pool_add_repeating_timer_us(HAL_timer_pool_0, us, HAL_timer_repeating_0_callback, NULL, &HAL_timer_0);
      break;
    case 1:
      result = alarm_pool_add_repeating_timer_us(HAL_timer_pool_1, us, HAL_timer_repeating_1_callback, NULL, &HAL_timer_1);
      break;
    case 2:
      result = alarm_pool_add_repeating_timer_us(HAL_timer_pool_2, us, HAL_timer_repeating_2_callback, NULL, &HAL_timer_2);
      break;
    case 3:
      result = alarm_pool_add_repeating_timer_us(HAL_timer_pool_3, us, HAL_timer_repeating_3_callback, NULL, &HAL_timer_3);
      break;
  }
  UNUSED(result);
}

void HAL_timer_stop(const uint8_t timer_num) {
   switch (timer_num) {
    case 0: cancel_repeating_timer(&HAL_timer_0); break;
    case 1: cancel_repeating_timer(&HAL_timer_1); break;
    case 2: cancel_repeating_timer(&HAL_timer_2); break;
    case 3: cancel_repeating_timer(&HAL_timer_3); break;
  }
}

int64_t HAL_timer_alarm_pool_0_callback(long int, void*) {
  if (HAL_timer_irq_en[0]) HAL_timer_0_callback();
  return 0;
}
int64_t HAL_timer_alarm_pool_1_callback(long int, void*) {
  if (HAL_timer_irq_en[1]) HAL_timer_1_callback();
  return 0;
}
int64_t HAL_timer_alarm_pool_2_callback(long int, void*) {
  if (HAL_timer_irq_en[2]) HAL_timer_2_callback();
  return 0;
}
int64_t HAL_timer_alarm_pool_3_callback(long int, void*) {
  if (HAL_timer_irq_en[3]) HAL_timer_3_callback();
  return 0;
}

bool HAL_timer_repeating_0_callback(repeating_timer* timer) {
  if (HAL_timer_irq_en[0]) HAL_timer_0_callback();
  return true;
}
bool HAL_timer_repeating_1_callback(repeating_timer* timer) {
  if (HAL_timer_irq_en[1]) HAL_timer_1_callback();
  return true;
}
bool HAL_timer_repeating_2_callback(repeating_timer* timer) {
  if (HAL_timer_irq_en[2]) HAL_timer_2_callback();
  return true;
}
bool HAL_timer_repeating_3_callback(repeating_timer* timer) {
  if (HAL_timer_irq_en[3]) HAL_timer_3_callback();
  return true;
}

void __attribute__((weak)) HAL_timer_0_callback() {}
void __attribute__((weak)) HAL_timer_1_callback() {}
void __attribute__((weak)) HAL_timer_2_callback() {}
void __attribute__((weak)) HAL_timer_3_callback() {}

#endif // __PLAT_RP2040__
