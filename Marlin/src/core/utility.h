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
#include "../core/millis_t.h"

void safe_delay(millis_t ms);           // Delay ensuring that temperatures are updated and the watchdog is kept alive.

#if ENABLED(SERIAL_OVERRUN_PROTECTION)
  void serial_delay(const millis_t ms);
#else
  inline void serial_delay(const millis_t) {}
#endif

#if GRID_MAX_POINTS

  // 16x16 bit arrays
  template <int W, int H>
  struct FlagBits {
    bits_t(W) flags[H];
    void fill()                                   { memset(flags, 0xFF, sizeof(flags)); }
    void reset()                                  { memset(flags, 0x00, sizeof(flags)); }
    void unmark(const uint8_t x, const uint8_t y) { CBI(flags[y], x); }
    void mark(const uint8_t x, const uint8_t y)   { SBI(flags[y], x); }
    bool marked(const uint8_t x, const uint8_t y) { return TEST(flags[y], x); }
    inline void unmark(const xy_int8_t &xy)       { unmark(xy.x, xy.y); }
    inline void mark(const xy_int8_t &xy)         { mark(xy.x, xy.y); }
    inline bool marked(const xy_int8_t &xy)       { return marked(xy.x, xy.y); }
  };

  typedef FlagBits<GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y> MeshFlags;

#endif

#if ENABLED(DEBUG_LEVELING_FEATURE)
  void log_machine_info();
#else
  #define log_machine_info() NOOP
#endif

/**
 * A restorer instance remembers a variable's value before setting a
 * new value, then restores the old value when it goes out of scope.
 * Put operator= on your type to get extended behavior on value change.
 */
template<typename T>
class restorer {
  T& ref_;
  T  val_;
public:
  restorer(T& perm) : ref_(perm), val_(perm) {}
  restorer(T& perm, T temp_val) : ref_(perm), val_(perm) { perm = temp_val; }
  ~restorer() { restore(); }
  inline void restore() { ref_ = val_; }
};

#define REMEMBER(N,X,V...) restorer<__typeof__(X)> restorer_##N(X, ##V)
#define RESTORE(N) restorer_##N.restore()

// Converts from an uint8_t in the range of 0-255 to an uint8_t
// in the range 0-100 while avoiding rounding artifacts
constexpr uint8_t ui8_to_percent(const uint8_t i) { return (int(i) * 100 + 127) / 255; }

// Axis names for G-code parsing, reports, etc.
const xyze_char_t axis_codes LOGICAL_AXIS_ARRAY('E', 'X', 'Y', 'Z', AXIS4_NAME, AXIS5_NAME, AXIS6_NAME, AXIS7_NAME, AXIS8_NAME, AXIS9_NAME);
#if NUM_AXES <= XYZ && !HAS_EXTRUDERS
  #define AXIS_CHAR(A) ((char)('X' + A))
  #if ANY(HAS_MOTOR_CURRENT_SPI, HAS_MOTOR_CURRENT_I2C, HAS_MOTOR_CURRENT_DAC)
    #define IAXIS_CHAR AXIS_CHAR
  #endif
#else
  #if ANY(HAS_MOTOR_CURRENT_SPI, HAS_MOTOR_CURRENT_I2C, HAS_MOTOR_CURRENT_DAC)
    const xyze_char_t iaxis_codes LOGICAL_AXIS_ARRAY('E', 'X', 'Y', 'Z', 'I', 'J', 'K', 'U', 'V', 'W');
    #define IAXIS_CHAR(A) iaxis_codes[A]
  #endif
  #define AXIS_CHAR(A) axis_codes[A]
#endif

#if ENABLED(MARLIN_DEV_MODE)
  enum MarlinError : uint8_t {
    ERR_NONE,
    ERR_STRING_RANGE, // A string buffer was too small to set the whole blob
    ERR_ASSERTION,    // An assertion was triggered
    ERR_MALFUNCTION,
    ERR_MEMORY_LEAK,
    ERR_COMMS_SERIAL,
    ERR_COMMS_SPI,
    ERR_PLANNER_STARVED,
    ERR_TMC_SHUTDOWN,
    ERR_PROCEDURE_FAILED,
    ERR_TOO_WACK,
    ERR_PLAID_IN_SUMMER
  };
  extern MarlinError marlin_error_number;   // Error Number - Marlin can beep, display, and emit...
  inline void error(const MarlinError err) { marlin_error_number = err; }
#endif
