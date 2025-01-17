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

#include "../core/serial.h"

/**
 * bresenham_t.h - Bresenham algorithm template
 *
 * An array of values / counters that tick together
 */

#define FORCE_INLINE  __attribute__((always_inline)) inline
#define __O3          __attribute__((optimize("O3")))

template <uint8_t uid, uint8_t size>
struct BresenhamCfg { static constexpr uint8_t UID = uid, SIZE = size; };

template<typename T, typename Cfg>
class Bresenham {
private:

  static constexpr T signtest = -1;
  static_assert(signtest < 0, "Bresenham type must be signed!");

public:

  static T divisor, value[Cfg::SIZE], dir[Cfg::SIZE], dividend[Cfg::SIZE], counter[Cfg::SIZE];

  // Default: Instantiate all items with the identical parameters
  Bresenham(const T &inDivisor=1, const int8_t &inDir=1, const T &inDividend=1, const T &inValue=0) {
    for (uint_fast8_t i = 0; i < Cfg::SIZE; i++) init(i, inDivisor, inDir, inDividend, inValue);
  }

  // Instantiate all items with the same divisor
  Bresenham(const T &inDivisor, const int8_t (&inDir)[Cfg::SIZE], const T (&inDividend)[Cfg::SIZE], const T (&inValue)[Cfg::SIZE]={0}) {
    init(inDivisor, inDir, inDividend, inValue);
  }

  // Instantiate all items with the same divisor and direction
  Bresenham(const T &inDivisor, const int8_t &inDir, const T (&inDividend)[Cfg::SIZE], const T (&inValue)[Cfg::SIZE]={0}) {
    init(inDivisor, inDir, inDividend, inValue);
  }

  // Init all items with the same parameters
  FORCE_INLINE static void init(const uint8_t index, const T &inDivisor=1, const int8_t &inDir=1, const T &inDividend=1, const T &inValue=0) {
    divisor         = inDivisor;
    dir[index]      = inDir;
    dividend[index] = inDividend;
    value[index]    = inValue;
    prime(index);
  }

  // Init all items with the same divisor
  FORCE_INLINE static void init(const T &inDivisor, const int8_t (&inDir)[Cfg::SIZE], const T (&inDividend)[Cfg::SIZE], const T (&inValue)[Cfg::SIZE]={0}) {
    divisor = inDivisor;
    for (uint_fast8_t i = 0; i < Cfg::SIZE; i++) {
      dir[i]      = inDir[i];
      dividend[i] = inDividend[i];
      value[i]    = inValue[i];
    }
    prime();
  }

  // Init all items with the same divisor and direction
  FORCE_INLINE static void init(const T &inDivisor, const int8_t &inDir, const T (&inDividend)[Cfg::SIZE], const T (&inValue)[Cfg::SIZE]={0}) {
    divisor = inDivisor;
    for (uint_fast8_t i = 0; i < Cfg::SIZE; i++) {
      dir[i]      = inDir;
      dividend[i] = inDividend[i];
      value[i]    = inValue[i];
    }
    prime();
  }

  // Reinit item with new dir, dividend, value keeping the same divisor
  FORCE_INLINE static void reinit(const uint8_t index, const int8_t &inDir=1, const T &inDividend=1, const T &inValue=0) {
    dir[index]      = inDir;
    dividend[index] = inDividend;
    value[index]    = inValue;
    prime();
  }

  FORCE_INLINE static void prime(const uint8_t index) { counter[index] = -(divisor / 2); }
  FORCE_INLINE static void prime() { for (uint_fast8_t i = 0; i < Cfg::SIZE; i++) prime(i); }

  FORCE_INLINE static void back(const uint8_t index) { counter[index] -= divisor; }

  FORCE_INLINE static bool tick1(const uint8_t index) {
    counter[index] += dividend[index];
    return counter[index] > 0;
  }

  FORCE_INLINE static void tick(const uint8_t index) {
    if (tick1(index)) { value[index] += dir[index]; back(index); }
  }

  FORCE_INLINE static void tick1() __O3 { for (uint_fast8_t i = 0; i < Cfg::SIZE; i++) (void)tick1(i); }

  FORCE_INLINE static void tick() __O3 { for (uint_fast8_t i = 0; i < Cfg::SIZE; i++) (void)tick(i); }

  static void report(const uint8_t index) {
    if (index < Cfg::SIZE) {
      SERIAL_ECHOPGM("bresenham ", index, " : (", dividend[index], "/", divisor, ") ");
      if (counter[index] >= 0) SERIAL_CHAR(' ');
      if (labs(counter[index]) < 100) { SERIAL_CHAR(' '); if (labs(counter[index]) < 10) SERIAL_CHAR(' '); }
      SERIAL_ECHO(counter[index]);
      SERIAL_ECHOLNPGM(" ... ", value[index]);
    }
  }

  static void report() { for (uint_fast8_t i = 0; i < Cfg::SIZE; i++) report(i); }
};
