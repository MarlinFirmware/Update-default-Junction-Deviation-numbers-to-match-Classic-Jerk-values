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

#if HAS_SERVOS

#include "Servo.h"

static uint_fast8_t servoCount = 0;
static libServo *servos[NUM_SERVOS] = {0};
constexpr millis_t servoDelay[] = SERVO_DELAY;
static_assert(COUNT(servoDelay) == NUM_SERVOS, "SERVO_DELAY must be an array NUM_SERVOS long.");

libServo::libServo()
: delay(servoDelay[servoCount]),
  was_attached_before_pause(false),
  value_before_pause(0)
{
  servos[servoCount++] = this;
}

int8_t libServo::attach(const int pin) {
  if (servoCount >= MAX_SERVOS) return -1;
  if (pin > 0) servo_pin = pin;
  auto result = pico_servo.attach(servo_pin);
  return result;
}

int8_t libServo::attach(const int pin, const int min, const int max) {
  if (servoCount >= MAX_SERVOS) return -1;
  if (pin > 0) servo_pin = pin;
  auto result = pico_servo.attach(servo_pin, min, max);
  return result;
}

void libServo::move(const int value) {
  if (attach(0) >= 0) {
    pico_servo.write(value);
    safe_delay(delay);
    TERN_(DEACTIVATE_SERVOS_AFTER_MOVE, detach());
  }
}

void libServo::pause() {
  was_attached_before_pause = pico_servo.attached();
  if (was_attached_before_pause) {
    value_before_pause = pico_servo.read();
    pico_servo.detach();
  }
}

void libServo::resume() {
  if (was_attached_before_pause) {
    attach();
    move(value_before_pause);
  }
}

void libServo::pause_all_servos() {
  for (auto& servo : servos)
    if (servo) servo->pause();
}

void libServo::resume_all_servos() {
  for (auto& servo : servos)
    if (servo) servo->resume();
}

#endif // HAS_SERVOS
#endif // __PLAT_RP2040__
