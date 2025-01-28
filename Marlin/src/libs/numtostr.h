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

#include "../inc/MarlinConfigPre.h"
#include "../core/types.h"

// Format uint8_t (0-100) as rj string with 123% / _12% / __1% format
const char* pcttostrpctrj(const uint8_t i);

// Convert uint8_t (0-255) to a percentage, format as above
const char* ui8tostr4pctrj(const uint8_t i);

// Convert uint8_t to string with 12 format
const char* ui8tostr2(const uint8_t x);

// Convert uint8_t to string with 123 format
const char* ui8tostr3rj(const uint8_t i);

// Convert int8_t to string with 123 format
const char* i8tostr3rj(const int8_t x);

#if HAS_PRINT_PROGRESS_PERMYRIAD
  // Convert 16-bit unsigned permyriad value to percent: _100 / __23 / 23.4 / 3.45
  const char* permyriadtostr4(const uint16_t xx);
#endif

// Convert uint16_t to string with 12345 format
const char* ui16tostr5rj(const uint16_t x);

// Convert uint16_t to string with 1234 format
const char* ui16tostr4rj(const uint16_t x);

// Convert uint16_t to string with 123 format
const char* ui16tostr3rj(const uint16_t x);

// Convert int16_t to string with 123 format
const char* i16tostr3rj(const int16_t x);

// Convert signed int to lj string with 123 format
const char* i16tostr3left(const int16_t xx);

// Convert unsigned int to string 1, 12, 123 format, capped at 999
const char* utostr3(const uint16_t x);

// Convert signed int to rj string with _123, -123, _-12, or __-1 format
const char* i16tostr4signrj(const int16_t x);

// Convert unsigned float to string with 1.2 format
const char* ftostr11ns(const_float_t x);

// Convert unsigned float to string with 1.23 format
const char* ftostr12ns(const_float_t x);

// Convert unsigned float to string with 12.3 format
const char* ftostr31ns(const_float_t x);

// Convert unsigned float to string with 123.4 format
const char* ftostr41ns(const_float_t x);

// Convert signed float to fixed-length string with 12.34 / _2.34 / -2.34 or -23.45 / 123.45 format
const char* ftostr42_52(const_float_t x);

// Convert signed float to fixed-length string with 023.45 / -23.45 format
const char* ftostr52(const_float_t x);

// Convert signed float to fixed-length string with 12.345 / -2.345 or 023.456 / -23.456 format
const char* ftostr53_63(const_float_t x);

// Convert signed float to fixed-length string with 023.456 / -23.456 format
const char* ftostr63(const_float_t x);

// Convert signed float to fixed-length string with +12.3 / -12.3 format
const char* ftostr31sign(const_float_t x);

// Convert signed float to fixed-length string with +123.4 / -123.4 format
const char* ftostr41sign(const_float_t x);

// Convert signed float to fixed-length string with +1234.5 / +1234.5 format
const char* ftostr51sign(const_float_t x);

// Convert signed float to string (6 digit) with -1.234 / _0.000 / +1.234 format
const char* ftostr43sign(const_float_t x, char plus=' ');

// Convert signed float to string (7 chars) with -12.345 / _00.000 / +12.345 format
const char* ftostr53sign(const_float_t x, char plus=' ');

// Convert signed float to string (5 digit) with -1.2345 / _0.0000 / +1.2345 format
const char* ftostr54sign(const_float_t x, char plus=' ');

// Convert unsigned float to rj string with 12345 format
const char* ftostr5rj(const_float_t x);

// Convert signed float to fixed-length string with +12.3 / -12.3 format
const char* ftostr31sign(const_float_t x);

// Convert signed float to fixed-length string with +123.4 / -123.4 format
const char* ftostr41sign(const_float_t x);

// Convert signed float to fixed-length string with +1234.5 format
const char* ftostr51sign(const_float_t x);

// Convert signed float to space-padded string with -_23.4_ format
const char* ftostr52sp(const_float_t x);

const char* ftostr52custom(const_float_t f);

// Convert signed float to string with +123.45 format
const char* ftostr52sign(const_float_t x);

// Convert unsigned float to string with _2.3 / 12.3 format
const char* ftostr31rj(const_float_t x);

// Convert unsigned float to string with __3.4 / _23.4 / 123.4 format
const char* ftostr41rj(const_float_t x);

// Convert unsigned float to string with ___4.5 / __34.5 / _234.5 / 1234.5 format
const char* ftostr51rj(const_float_t x);

// Convert unsigned float to string with ____5.6 / ___45.6 / __345.6 / _2345.6 / 12345.6 format
const char* ftostr61rj(const_float_t x);

// Convert usigned float to string with ____5.67, ___45.67, __345.67, _2345.67, 12345.67 format
const char* ftostr7xrj(int32_t intVal, int32_t decimal);

// Convert unsigned float to string with 1.23 format
const char* ftostr32rj(const_float_t f);

// Convert unsigned float to string with _2.34, 12.34 format
const char* ftostr42rj(const_float_t f);

// Convert unsigned float to string with __3.45, _23.45, 123.45 format
const char* ftostr52rj(const_float_t f);

// Convert unsigned float to string with ___4.56, __34.56, _234.56, 1234.56 format
const char* ftostr62rj(const_float_t f);

// Convert unsigned float to string with ____5.67, ___45.67, __345.67, _2345.67, 12345.67 format
const char* ftostr72rj(const_float_t x);

// Convert signed float to rj string with 123 or -12 format
FORCE_INLINE const char* ftostr3rj(const_float_t x) { return i16tostr3rj(int16_t(x + (x < 0 ? -0.5f : 0.5f))); }

#if ENABLED(LCD_DECIMAL_SMALL_XY)
  // Convert signed float to rj string with 1234, _123, 12.3, _1.2, -123, _-12, or -1.2 format
  const char* ftostr4sign(const_float_t fx);
#else
  // Convert signed float to rj string with 1234, _123, -123, __12, _-12, ___1, or __-1 format
  FORCE_INLINE const char* ftostr4sign(const_float_t x) { return i16tostr4signrj(int16_t(x + (x < 0 ? -0.5f : 0.5f))); }
#endif

// Convert signed float to space-padded string with 1.23, 12.34, 123.45 format
const char* ftostr52sprj(const_float_t f);

const char* shortenNum(const char * convptr, bool removeWhole0 = true, bool removeUnit = true);
