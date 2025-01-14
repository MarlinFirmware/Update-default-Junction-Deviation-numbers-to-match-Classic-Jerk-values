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

#include "numtostr.h"

#include "../inc/MarlinConfigPre.h"
#include "../core/utility.h"

#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"

constexpr char DIGIT(const uint8_t n) { return '0' + n; }

template <typename T1, typename T2>
constexpr char DIGIMOD(const T1 n, const T2 f) { return DIGIT((n / f) % 10); }

template <typename T1, typename T2>
constexpr char RJDIGIT(const T1 n, const T2 f) { return (n >= (T1)f ? DIGIMOD(n, f) : ' '); }

template <typename T>
constexpr char MINUSOR(T &n, const char alt) { return (n >= 0) ? alt : (n = -n) ? '-' : '-'; }

constexpr long INTFLOAT(const float V, const int N) {
  return long((V * 10.0f * pow(10.0f, N) + (V < 0.0f ? -5.0f : 5.0f)) / 10.0f);
}
constexpr long UINTFLOAT(const float V, const int N) {
  return INTFLOAT(V < 0.0f ? -V : V, N);
}

char conv[9] = { 0 };

// Format uint8_t (0-100) as rj string with __3% / _23% / 123% format
const char* pcttostrpctrj(const uint8_t i) {
  conv[4] = RJDIGIT(i, 100);
  conv[5] = RJDIGIT(i, 10);
  conv[6] = DIGIMOD(i, 1);
  conv[7] = '%';
  return &conv[4];
}

// Convert uint8_t (0-255) to a percentage, format as above
const char* ui8tostr4pctrj(const uint8_t i) {
  return pcttostrpctrj(ui8_to_percent(i));
}

// Convert unsigned 8bit int to string with __3 / _23 / 123 format
const char* ui8tostr3rj(const uint8_t i) {
  conv[5] = RJDIGIT(i, 100);
  conv[6] = RJDIGIT(i, 10);
  conv[7] = DIGIMOD(i, 1);
  return &conv[5];
}

// Convert uint8_t to string with 12 format
const char* ui8tostr2(const uint8_t i) {
  conv[6] = DIGIMOD(i, 10);
  conv[7] = DIGIMOD(i, 1);
  return &conv[6];
}

// Convert signed 8bit int to rj string with __3 / _23 / 123 / -_3 / -23 format
const char* i8tostr3rj(const int8_t x) {
  int xx = x;
  conv[5] = MINUSOR(xx, RJDIGIT(xx, 100));
  conv[6] = RJDIGIT(xx, 10);
  conv[7] = DIGIMOD(xx, 1);
  return &conv[5];
}

#if HAS_PRINT_PROGRESS_PERMYRIAD
  // Convert unsigned 16-bit permyriad to percent with 100 / 23.4 / 3.45 format
  const char* permyriadtostr4(const uint16_t xx) {
    if (xx >= 10000)
      return " 100"; // space to keep 4-width alignment
    else if (xx >= 1000) {
      conv[4] = DIGIMOD(xx, 1000);
      conv[5] = DIGIMOD(xx, 100);
      conv[6] = '.';
      conv[7] = DIGIMOD(xx, 10);
      return &conv[4];
    }
    else {
      conv[4] = DIGIMOD(xx, 100);
      conv[5] = '.';
      conv[6] = DIGIMOD(xx, 10);
      conv[7] = RJDIGIT(xx, 1);
      return &conv[4];
    }
  }
#endif

// Convert unsigned 16bit int to right-justified string
inline const char* ui16tostrXrj(const uint16_t xx, const int index) {
  switch (index) {
    case 0 ... 3: conv[3] = RJDIGIT(xx, 10000);
    case 4:       conv[4] = RJDIGIT(xx, 1000);
    case 5:       conv[5] = RJDIGIT(xx, 100);
    case 6:       conv[6] = RJDIGIT(xx, 10);
  }
  conv[7] = DIGIMOD(xx, 1);
  return &conv[index];
}

// Convert unsigned 16bit int to string with 12345 format
const char* ui16tostr5rj(const uint16_t xx) { return ui16tostrXrj(xx, 8 - 5); }

// Convert unsigned 16bit int to string with 1234 format
const char* ui16tostr4rj(const uint16_t xx) { return ui16tostrXrj(xx, 8 - 4); }

// Convert unsigned 16bit int to string with 123 format
const char* ui16tostr3rj(const uint16_t xx) { return ui16tostrXrj(xx, 8 - 3); }

// Convert signed 16bit int to rj string with 123 or -12 format
const char* i16tostr3rj(const int16_t x) {
  int xx = x;
  conv[5] = MINUSOR(xx, RJDIGIT(xx, 100));
  conv[6] = RJDIGIT(xx, 10);
  conv[7] = DIGIMOD(xx, 1);
  return &conv[5];
}

// Convert unsigned 16bit int to lj string with 123 format
const char* i16tostr3left(const int16_t i) {
  char *str = &conv[7];
  *str = DIGIMOD(i, 1);
  if (i >= 10) {
    *(--str) = DIGIMOD(i, 10);
    if (i >= 100)
      *(--str) = DIGIMOD(i, 100);
  }
  return str;
}

// Convert signed 16bit int to rj string with 1234, _123, -123, _-12, or __-1 format
const char* i16tostr4signrj(const int16_t i) {
  const bool neg = i < 0;
  const int ii = neg ? -i : i;
  if (i >= 1000) {
    conv[4] = DIGIMOD(ii, 1000);
    conv[5] = DIGIMOD(ii, 100);
    conv[6] = DIGIMOD(ii, 10);
  }
  else if (ii >= 100) {
    conv[4] = neg ? '-' : ' ';
    conv[5] = DIGIMOD(ii, 100);
    conv[6] = DIGIMOD(ii, 10);
  }
  else {
    conv[4] = ' ';
    conv[5] = ' ';
    if (ii >= 10) {
      conv[5] = neg ? '-' : ' ';
      conv[6] = DIGIMOD(ii, 10);
    }
    else {
      conv[6] = neg ? '-' : ' ';
    }
  }
  conv[7] = DIGIMOD(ii, 1);
  return &conv[4];
}

// Convert unsigned float to string with 1.1 format
const char* ftostr11ns(const_float_t f) {
  const long i = UINTFLOAT(f, 1);
  conv[5] = DIGIMOD(i, 10);
  conv[6] = '.';
  conv[7] = DIGIMOD(i, 1);
  return &conv[5];
}

// Convert unsigned float to string with 1.23 format
const char* ftostr12ns(const_float_t f) {
  const long i = UINTFLOAT(f, 2);
  conv[4] = DIGIMOD(i, 100);
  conv[5] = '.';
  conv[6] = DIGIMOD(i, 10);
  conv[7] = DIGIMOD(i, 1);
  return &conv[4];
}

// Convert unsigned float to string with 12.3 format
const char* ftostr31ns(const_float_t f) {
  const long i = UINTFLOAT(f, 1);
  conv[4] = DIGIMOD(i, 100);
  conv[5] = DIGIMOD(i, 10);
  conv[6] = '.';
  conv[7] = DIGIMOD(i, 1);
  return &conv[4];
}

// Convert unsigned float to string with 123.4 format
const char* ftostr41ns(const_float_t f) {
  const long i = UINTFLOAT(f, 1);
  conv[3] = DIGIMOD(i, 1000);
  conv[4] = DIGIMOD(i, 100);
  conv[5] = DIGIMOD(i, 10);
  conv[6] = '.';
  conv[7] = DIGIMOD(i, 1);
  return &conv[3];
}

// Convert float to fixed-length string with 12.34 / _2.34 / -2.34 or -23.45 / 123.45 format
const char* ftostr42_52(const_float_t f) {
  if (f <= -10 || f >= 100) return ftostr52(f); // -23.45 / 123.45
  long i = INTFLOAT(f, 2);
  conv[3] = (f >= 0 && f < 10) ? ' ' : MINUSOR(i, DIGIMOD(i, 1000));
  conv[4] = DIGIMOD(i, 100);
  conv[5] = '.';
  conv[6] = DIGIMOD(i, 10);
  conv[7] = DIGIMOD(i, 1);
  return &conv[3];
}

// Convert float to fixed-length string with 023.45 / -23.45 format
const char* ftostr52(const_float_t f) {
  long i = INTFLOAT(f, 2);
  conv[2] = MINUSOR(i, DIGIMOD(i, 10000));
  conv[3] = DIGIMOD(i, 1000);
  conv[4] = DIGIMOD(i, 100);
  conv[5] = '.';
  conv[6] = DIGIMOD(i, 10);
  conv[7] = DIGIMOD(i, 1);
  return &conv[2];
}

// Convert float to fixed-length string with 12.345 / _2.345 / -2.345 or -23.45 / 123.45 format
const char* ftostr53_63(const_float_t f) {
  if (f <= -10 || f >= 100) return ftostr63(f); // -23.456 / 123.456
  long i = INTFLOAT(f, 3);
  conv[2] = (f >= 0 && f < 10) ? ' ' : MINUSOR(i, DIGIMOD(i, 10000));
  conv[3] = DIGIMOD(i, 1000);
  conv[4] = '.';
  conv[5] = DIGIMOD(i, 100);
  conv[6] = DIGIMOD(i, 10);
  conv[7] = DIGIMOD(i, 1);
  return &conv[2];
}

// Convert float to fixed-length string with 023.456 / -23.456 format
const char* ftostr63(const_float_t f) {
  long i = INTFLOAT(f, 3);
  conv[1] = MINUSOR(i, DIGIMOD(i, 100000));
  conv[2] = DIGIMOD(i, 10000);
  conv[3] = DIGIMOD(i, 1000);
  conv[4] = '.';
  conv[5] = DIGIMOD(i, 100);
  conv[6] = DIGIMOD(i, 10);
  conv[7] = DIGIMOD(i, 1);
  return &conv[1];
}

#if ENABLED(LCD_DECIMAL_SMALL_XY)

  // Convert float to rj string with 1234, _123, -123, _-12, 12.3, _1.2, or -1.2 format
  const char* ftostr4sign(const_float_t f) {
    const int i = INTFLOAT(f, 1);
    if (!WITHIN(i, -99, 999)) return i16tostr4signrj((int)f);
    const bool neg = i < 0;
    const int ii = neg ? -i : i;
    conv[4] = neg ? '-' : (ii >= 100 ? DIGIMOD(ii, 100) : ' ');
    conv[5] = DIGIMOD(ii, 10);
    conv[6] = '.';
    conv[7] = DIGIMOD(ii, 1);
    return &conv[4];
  }

#endif

//
// Convert float to fixed-length string with +/- and a single decimal place
//
inline const char* ftostrX1sign(const_float_t f, const int index) {
  long i = INTFLOAT(f, 1);
  conv[index] = MINUSOR(i, '+');
  switch (index + 1) {
    case 1: conv[1] = DIGIMOD(i, 100000);
    case 2: conv[2] = DIGIMOD(i, 10000);
    case 3: conv[3] = DIGIMOD(i, 1000);
    case 4: conv[4] = DIGIMOD(i, 100);
  }
  conv[5] = DIGIMOD(i, 10);
  conv[6] = '.';
  conv[7] = DIGIMOD(i, 1);
  return &conv[index];
}

// Convert float to fixed-length string with +12.3 / -12.3 format
const char* ftostr31sign(const_float_t f) { return ftostrX1sign(f, 3); }

// Convert float to fixed-length string with +123.4 / -123.4 format
const char* ftostr41sign(const_float_t f) { return ftostrX1sign(f, 2); }

// Convert float to fixed-length string with +1234.5 / +1234.5 format
const char* ftostr51sign(const_float_t f) { return ftostrX1sign(f, 1); }

//
// Convert float to string with +/ /- and 3 decimal places
//
inline const char* ftostrX3sign(const_float_t f, const int index, char plus/*=' '*/) {
  long i = INTFLOAT(f, 3);
  conv[index] = i ? MINUSOR(i, plus) : ' ';
  switch (index + 1) {
    case 1: conv[1] = DIGIMOD(i, 100000);
    case 2: conv[2] = DIGIMOD(i, 10000);
  }
  conv[3] = DIGIMOD(i, 1000);
  conv[4] = '.';
  conv[5] = DIGIMOD(i, 100);
  conv[6] = DIGIMOD(i, 10);
  conv[7] = DIGIMOD(i, 1);
  return &conv[index];
}

// Convert float to string (6 chars) with -1.234 / _0.000 / +1.234 format
const char* ftostr43sign(const_float_t f, char plus/*=' '*/) { return ftostrX3sign(f, 2, plus); }

// Convert float to string (7 chars) with -12.345 / _00.000 / +12.345 format
const char* ftostr53sign(const_float_t f, char plus/*=' '*/) { return ftostrX3sign(f, 1, plus); }

// Convert float to string (7 chars) with -1.2345 / _0.0000 / +1.2345 format
const char* ftostr54sign(const_float_t f, char plus/*=' '*/) {
  long i = INTFLOAT(f, 4);
  conv[1] = i ? MINUSOR(i, plus) : ' ';
  conv[2] = DIGIMOD(i, 10000);
  conv[3] = '.';
  conv[4] = DIGIMOD(i, 1000);
  conv[5] = DIGIMOD(i, 100);
  conv[6] = DIGIMOD(i, 10);
  conv[7] = DIGIMOD(i, 1);
  return &conv[1];
}

// Convert unsigned float to rj string with 12345 format
const char* ftostr5rj(const_float_t f) {
  const long i = UINTFLOAT(f, 0);
  return ui16tostr5rj(i);
}

// Convert signed float to string with +123.45 format
const char* ftostr52sign(const_float_t f) {
  long i = INTFLOAT(f, 2);
  conv[1] = MINUSOR(i, '+');
  conv[2] = DIGIMOD(i, 10000);
  conv[3] = DIGIMOD(i, 1000);
  conv[4] = DIGIMOD(i, 100);
  conv[5] = '.';
  conv[6] = DIGIMOD(i, 10);
  conv[7] = DIGIMOD(i, 1);
  return &conv[1];
}

// Convert unsigned float to string with a single digit precision
inline const char* ftostrX1rj(const_float_t f, const int index=1) {
  const long i = UINTFLOAT(f, 1);
  switch (index) {
    case 0: conv[0] = RJDIGIT(i, 1000000);
    case 1: conv[1] = RJDIGIT(i, 100000);
    case 2: conv[2] = RJDIGIT(i, 10000);
    case 3: conv[3] = RJDIGIT(i, 1000);
    case 4: conv[4] = RJDIGIT(i, 100);
  }
  conv[5] = DIGIMOD(i, 10);
  conv[6] = '.';
  conv[7] = DIGIMOD(i, 1);
  return &conv[index];
}

// Convert unsigned float to string with _2.3 / 12.3 format
const char* ftostr31rj(const_float_t f) { return ftostrX1rj(f, 7 - 3); }

// Convert unsigned float to string with __3.4 / _23.4 / 123.4 format
const char* ftostr41rj(const_float_t f) { return ftostrX1rj(f, 7 - 4); }

// Convert unsigned float to string with ___4.5 / __34.5 / _234.5 / 1234.5 format
const char* ftostr51rj(const_float_t f) { return ftostrX1rj(f, 7 - 5); }

// Convert unsigned float to string with ____5.6 / ___45.6 / __345.6 / _2345.6 / 12345.6 format
const char* ftostr61rj(const_float_t f) { return ftostrX1rj(f, 7 - 6); }

// Convert unsigned float to string with two digit precision
inline const char* ftostrX2rj(const_float_t f, const int index=1) {
  const long i = UINTFLOAT(f, 2);
  switch (index) {
    case 0: conv[0] = RJDIGIT(i, 1000000);
    case 1: conv[1] = RJDIGIT(i, 100000);
    case 2: conv[2] = RJDIGIT(i, 10000);
    case 3: conv[3] = RJDIGIT(i, 1000);
    case 4: conv[4] = RJDIGIT(i, 100);
  }
  conv[5] = '.';
  conv[6] = DIGIMOD(i, 10);
  conv[7] = DIGIMOD(i, 1);
  return &conv[index];
}

// Convert unsigned float to string with 1.23 format
const char* ftostr32rj(const_float_t f) { return ftostrX2rj(f, 4); }

// Convert unsigned float to string with _2.34, 12.34 format
const char* ftostr42rj(const_float_t f) { return ftostrX2rj(f, 3); }

// Convert unsigned float to string with __3.45, _23.45, 123.45 format
const char* ftostr52rj(const_float_t f) { return ftostrX2rj(f, 2); }

// Convert unsigned float to string with ___4.56, __34.56, _234.56, 1234.56 format
const char* ftostr62rj(const_float_t f) { return ftostrX2rj(f, 1); }

// Convert unsigned float to string with ____5.67, ___45.67, __345.67, _2345.67, 12345.67 format
const char* ftostr72rj(const_float_t f) { return ftostrX2rj(f, 0); }

// Convert float to space-padded string with -_23.4_ format
const char* ftostr52sp(const_float_t f) {
  long i = INTFLOAT(f, 2);
  uint8_t dig;
  conv[1] = MINUSOR(i, ' ');
  conv[2] = RJDIGIT(i, 10000);
  conv[3] = RJDIGIT(i, 1000);
  conv[4] = DIGIMOD(i, 100);

  if ((dig = i % 10)) {           // Second digit after decimal point?
    conv[5] = '.';
    conv[6] = DIGIMOD(i, 10);
    conv[7] = DIGIT(dig);
  }
  else {
    if ((dig = (i / 10) % 10)) {  // First digit after decimal point?
      conv[5] = '.';
      conv[6] = DIGIT(dig);
    }
    else                          // Nothing after decimal point
      conv[5] = conv[6] = ' ';
    conv[7] = ' ';
  }
  return &conv[1];
}

// Convert unsigned 16bit int to string 1, 12, 123 format, capped at 999
const char* utostr3(const uint16_t x) {
  return i16tostr3left(_MIN(x, 999U));
}

// Convert float to space-padded string with 1.23, 12.34, 123.45 format
const char* ftostr52sprj(const_float_t f) {
  long i = INTFLOAT(f, 2);
  LIMIT(i, -99999, 99999);            // cap to -999.99 - 999.99 range
  if (WITHIN(i, -999, 999)) {         // -9.99 - 9.99 range
    conv[1] = conv[2] = ' ';          // default to ' ' for smaller numbers
    conv[3] = MINUSOR(i, ' ');
  }
  else if (WITHIN(i, -9999, 9999)) {  // -99.99 - 99.99 range
    conv[1] = ' ';
    conv[2] = MINUSOR(i, ' ');
    conv[3] = DIGIMOD(i, 1000);
  }
  else {                              // -999.99 - 999.99 range
    conv[1] = MINUSOR(i, ' ');
    conv[2] = DIGIMOD(i, 10000);
    conv[3] = DIGIMOD(i, 1000);
  }
  conv[4] = DIGIMOD(i, 100);          // always convert last 3 digits
  conv[5] = '.';
  conv[6] = DIGIMOD(i, 10);
  conv[7] = DIGIMOD(i, 1);

  return &conv[1];
}
