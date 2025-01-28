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

#include "../core/types.h"
#include "../core/macros.h"

#ifdef __cplusplus

namespace Endianness {
  static constexpr uint32_t _dword = 0x01020304;
  static constexpr uint8_t _lsb = (const uint8_t&)_dword;

  static constexpr bool cpuIsLittleEndian = _lsb == 0x04;
  static constexpr bool cpuIsBigEndian = _lsb == 0x01;
  static_assert(cpuIsLittleEndian ^ cpuIsBigEndian, "Unknown CPU endianness");

  // constexpr byte swapping for integral types
  template<typename T> static constexpr typename Private::enable_if<Private::is_integral<T>::value, T>::type swap(T V, T swappedV=(T)0, size_t byteIndex=0) {
    return byteIndex == sizeof(T)
        ? swappedV
        : swap<T>((T)(V >> 8), (swappedV << 8) | (V & (T)0xFF), byteIndex + 1);
  }

  // constexpr byte swapping for types derived from integral types (e.g. enums)
  template<typename T> static constexpr typename Private::enable_if<
      Private::is_same<uint16_t, typename Private::underlying_type<T>::type>::value, T>::type swap(T V) { return (T)swap<uint16_t>((uint16_t)V); }
  template<typename T> static constexpr typename Private::enable_if<
      Private::is_same<uint32_t, typename Private::underlying_type<T>::type>::value, T>::type swap(T V) { return (T)swap<uint32_t>((uint32_t)V); }
  template<typename T> static constexpr typename Private::enable_if<
      Private::is_same<uint64_t, typename Private::underlying_type<T>::type>::value, T>::type swap(T V) { return (T)swap<uint64_t>((uint64_t)V); }

  // Generic byte swapping
  // CANNOT be used to initialize constexpr declarations
  template<typename T> static constexpr typename Private::enable_if<!Private::is_integral<T>::value && !Private::is_enum<T>::value, T>::type swap(T V) {
    union {
      T val;
      char byte[sizeof(T)];
    } src{}, dst{};

    src.val = V;
    for (uint_fast8_t i = 0; i < sizeof(T); ++i) dst.byte[i] = src.byte[sizeof(T) - i - 1];
    return dst.val;
  }

  // Convert to / from known endianness, depending on the host endianness
  template<typename T> static constexpr T toBE(T V) { return cpuIsLittleEndian ? swap(V) : V; }
  template<typename T> static constexpr T toLE(T V) { return cpuIsLittleEndian ? V : swap(V); }
  template<typename T> static constexpr T fromBE(T V) { return cpuIsLittleEndian ? swap(V) : V; }
  template<typename T> static constexpr T fromLE(T V) { return cpuIsLittleEndian ? V : swap(V); }

  // Reads a big/little endian from a pointer and converts it to the host endianness
  template<typename T> static constexpr T fromBE_P(void* V) { return fromBE(*(T*)V); }
  template<typename T> static constexpr T fromLE_P(void* V) { return fromLE(*(T*)V); }
};

#endif // __cplusplus
