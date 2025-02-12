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

/**
 * This module is off by default, but can be enabled to facilitate the display of
 * extra debug information during code development.
 *
 * Just connect up 5V and GND to give it power, then connect up the pins assigned
 * in Configuration_adv.h. For example, on the Re-ARM you could use:
 *
 *   #define MAX7219_CLK_PIN   77
 *   #define MAX7219_DIN_PIN   78
 *   #define MAX7219_LOAD_PIN  79
 *
 * send() is called automatically at startup, and then there are a number of
 * support functions available to control the LEDs in the 8x8 grid.
 */

#include "../inc/MarlinConfigPre.h"

#if ENABLED(MAX7219_DEBUG)

#define MAX7219_ERRORS // Requires ~400 bytes of flash

#include "max7219.h"

#include "../module/planner.h"
#include "../module/stepper.h"
#include "../MarlinCore.h"
#include "../HAL/shared/Delay.h"

#if ENABLED(MAX7219_SIDE_BY_SIDE) && MAX7219_NUMBER_UNITS > 1
  #define HAS_SIDE_BY_SIDE 1
#endif

#define _ROT ((MAX7219_ROTATE + 360) % 360)
#if _ROT == 0 || _ROT == 180
  #define MAX7219_X_LEDS TERN(HAS_SIDE_BY_SIDE, 8, MAX7219_LINES)
  #define MAX7219_Y_LEDS TERN(HAS_SIDE_BY_SIDE, MAX7219_LINES, 8)
#elif _ROT == 90 || _ROT == 270
  #define MAX7219_X_LEDS TERN(HAS_SIDE_BY_SIDE, MAX7219_LINES, 8)
  #define MAX7219_Y_LEDS TERN(HAS_SIDE_BY_SIDE, 8, MAX7219_LINES)
#else
  #error "MAX7219_ROTATE must be a multiple of +/- 90°."
#endif

#ifdef MAX7219_DEBUG_PROFILE
  CodeProfiler::Mode CodeProfiler::mode = ACCUMULATE_AVERAGE;
  uint8_t CodeProfiler::instance_count = 0;
  uint32_t CodeProfiler::last_calc_time = micros();
  uint8_t CodeProfiler::time_fraction = 0;
  uint32_t CodeProfiler::total_time = 0;
  uint16_t CodeProfiler::call_count = 0;
#endif

#if defined(MAX7219_DEBUG_PLANNER_HEAD) && defined(MAX7219_DEBUG_PLANNER_TAIL) && MAX7219_DEBUG_PLANNER_HEAD == MAX7219_DEBUG_PLANNER_TAIL
  static int16_t last_head_cnt = 0xF, last_tail_cnt = 0xF;
#else
  #ifdef MAX7219_DEBUG_PLANNER_HEAD
    static int16_t last_head_cnt = 0x1;
  #endif
  #ifdef MAX7219_DEBUG_PLANNER_TAIL
    static int16_t last_tail_cnt = 0x1;
  #endif
#endif
#ifdef MAX7219_DEBUG_PLANNER_QUEUE
  static int16_t last_depth = 0;
#endif
#ifdef MAX7219_DEBUG_PROFILE
  static uint8_t last_time_fraction = 0;
#endif
#ifdef MAX7219_DEBUG_MULTISTEPPING
  static uint8_t last_multistepping = 0;
#endif

Max7219 max7219;

uint8_t Max7219::led_line[MAX7219_LINES]; // = { 0 };
uint8_t Max7219::suspended; // = 0;

#define LINE_REG(Q)     (max7219_reg_digit0 + ((Q) & 0x7))

#if (_ROT == 0 || _ROT == 270) == DISABLED(MAX7219_REVERSE_EACH)
  #define _LED_BIT(Q)   (7 - ((Q) & 0x7))
#else
  #define _LED_BIT(Q)   ((Q) & 0x7)
#endif
#if _ROT == 0 || _ROT == 180
  #define LED_BIT(X,Y)  _LED_BIT(X)
#else
  #define LED_BIT(X,Y)  _LED_BIT(Y)
#endif
#if _ROT == 0 || _ROT == 90
  #define _LED_IND(P,Q) (_LED_TOP(P) + ((Q) & 0x7))
#else
  #define _LED_IND(P,Q) (_LED_TOP(P) + (7 - ((Q) & 0x7)))
#endif

#if HAS_SIDE_BY_SIDE
  #if (_ROT == 0 || _ROT == 90) == DISABLED(MAX7219_REVERSE_ORDER)
    #define _LED_TOP(Q)  ((MAX7219_NUMBER_UNITS - 1 - ((Q) >> 3)) << 3)
  #else
    #define _LED_TOP(Q)  ((Q) & ~0x7)
  #endif
  #if _ROT == 0 || _ROT == 180
    #define LED_IND(X,Y)  _LED_IND(Y,Y)
  #elif _ROT == 90 || _ROT == 270
    #define LED_IND(X,Y)  _LED_IND(X,X)
  #endif
#else
  #if (_ROT == 0 || _ROT == 270) == DISABLED(MAX7219_REVERSE_ORDER)
    #define _LED_TOP(Q)  ((Q) & ~0x7)
  #else
    #define _LED_TOP(Q)  ((MAX7219_NUMBER_UNITS - 1 - ((Q) >> 3)) << 3)
  #endif
  #if _ROT == 0 || _ROT == 180
    #define LED_IND(X,Y)  _LED_IND(X,Y)
  #elif _ROT == 90 || _ROT == 270
    #define LED_IND(X,Y)  _LED_IND(Y,X)
  #endif
#endif

#define XOR_7219(X,Y) do{ led_line[LED_IND(X,Y)] ^=  _BV(LED_BIT(X,Y)); }while(0)
#define SET_7219(X,Y) do{ led_line[LED_IND(X,Y)] |=  _BV(LED_BIT(X,Y)); }while(0)
#define CLR_7219(X,Y) do{ led_line[LED_IND(X,Y)] &= ~_BV(LED_BIT(X,Y)); }while(0)
#define BIT_7219(X,Y) TEST(led_line[LED_IND(X,Y)], LED_BIT(X,Y))

#ifdef CPU_32_BIT
  #define SIG_DELAY() DELAY_US(1)   // Approximate a 1µs delay on 32-bit ARM
  #undef CRITICAL_SECTION_START
  #undef CRITICAL_SECTION_END
  #define CRITICAL_SECTION_START() NOOP
  #define CRITICAL_SECTION_END()   NOOP
#else
  #define SIG_DELAY() DELAY_NS(250)
#endif

void Max7219::error(FSTR_P const func, const int32_t v1, const int32_t v2/*=-1*/) {
  #if ENABLED(MAX7219_ERRORS)
    SERIAL_ECHO(F("??? Max7219::"), func, C('('), v1);
    if (v2 > 0) SERIAL_ECHOPGM(", ", v2);
    SERIAL_CHAR(')');
    SERIAL_EOL();
  #else
    UNUSED(func); UNUSED(v1); UNUSED(v2);
  #endif
}

/**
 * Flip the lowest n_bytes of the supplied bits:
 *  flipped(x, 1) flips the low 8  bits of x.
 *  flipped(x, 2) flips the low 16 bits of x.
 *  flipped(x, 3) flips the low 24 bits of x.
 *  flipped(x, 4) flips the low 32 bits of x.
 */
inline uint32_t flipped(const uint32_t bits, const uint8_t n_bytes) {
  uint32_t mask = 1, outbits = 0;
  for (uint8_t b = 0; b < n_bytes * 8; ++b) {
    outbits <<= 1;
    if (bits & mask) outbits |= 1;
    mask <<= 1;
  }
  return outbits;
}

void Max7219::noop() {
  CRITICAL_SECTION_START();
  SIG_DELAY();
  WRITE(MAX7219_DIN_PIN, LOW);
  for (uint8_t i = 16; i--;) {
    SIG_DELAY();
    WRITE(MAX7219_CLK_PIN, LOW);
    SIG_DELAY();
    SIG_DELAY();
    WRITE(MAX7219_CLK_PIN, HIGH);
    SIG_DELAY();
  }
  CRITICAL_SECTION_END();
}

void Max7219::putbyte(uint8_t data) {
  CRITICAL_SECTION_START();
  for (uint8_t i = 8; i--;) {
    SIG_DELAY();
    WRITE(MAX7219_CLK_PIN, LOW);       // tick
    SIG_DELAY();
    WRITE(MAX7219_DIN_PIN, (data & 0x80) ? HIGH : LOW);  // send 1 or 0 based on data bit
    SIG_DELAY();
    WRITE(MAX7219_CLK_PIN, HIGH);      // tock
    SIG_DELAY();
    data <<= 1;
  }
  CRITICAL_SECTION_END();
}

void Max7219::pulse_load() {
  SIG_DELAY();
  WRITE(MAX7219_LOAD_PIN, LOW);  // tell the chip to load the data
  SIG_DELAY();
  WRITE(MAX7219_LOAD_PIN, HIGH);
  SIG_DELAY();
}

void Max7219::send(const uint8_t reg, const uint8_t data) {
  SIG_DELAY();
  CRITICAL_SECTION_START();
  SIG_DELAY();
  putbyte(reg);          // specify register
  SIG_DELAY();
  putbyte(data);         // put data
  CRITICAL_SECTION_END();
}

// Send out a single native row of bits to just one unit
void Max7219::refresh_unit_line(const uint8_t line) {
  if (suspended) return;
  #if MAX7219_NUMBER_UNITS == 1
    send(LINE_REG(line), led_line[line]);
  #else
    for (uint8_t u = MAX7219_NUMBER_UNITS; u--;)
      if (u == (line >> 3)) send(LINE_REG(line), led_line[line]); else noop();
  #endif
  pulse_load();
}

// Send out a single native row of bits to all units
void Max7219::refresh_line(const uint8_t line) {
  if (suspended) return;
  #if MAX7219_NUMBER_UNITS == 1
    refresh_unit_line(line);
  #else
    for (uint8_t u = MAX7219_NUMBER_UNITS; u--;)
      send(LINE_REG(line), led_line[(u << 3) | (line & 0x7)]);
  #endif
  pulse_load();
}

void Max7219::set(const uint8_t line, const uint8_t bits) {
  led_line[line] = bits;
  refresh_unit_line(line);
}

#if ENABLED(MAX7219_NUMERIC)

  // Draw an integer with optional leading zeros and optional decimal point
  void Max7219::print(const uint8_t start, int16_t value, uint8_t size, const bool leadzero=false, bool dec=false) {
    if (suspended) return;
    constexpr uint8_t led_numeral[10] = { 0x7E, 0x60, 0x6D, 0x79, 0x63, 0x5B, 0x5F, 0x70, 0x7F, 0x7A },
                      led_decimal = 0x80, led_minus = 0x01;
    bool blank = false, neg = value < 0;
    if (neg) value *= -1;
    while (size--) {
      const bool minus = neg && blank;
      if (minus) neg = false;
      send(
        max7219_reg_digit0 + start + size,
        minus ? led_minus : blank ? 0x00 : led_numeral[value % 10] | (dec ? led_decimal : 0x00)
      );
      pulse_load();  // tell the chips to load the clocked out data
      value /= 10;
      if (!value && !leadzero) blank = true;
      dec = false;
    }
  }

  // Draw a float with a decimal point and optional digits
  void Max7219::print(const uint8_t start, const_float_t value, const uint8_t pre_size, const uint8_t post_size, const bool leadzero=false) {
    if (pre_size) print(start, value, pre_size, leadzero, !!post_size);
    if (post_size) {
      const int16_t after = ABS(value) * (10 ^ post_size);
      print(start + pre_size, after, post_size, true);
    }
  }

#endif // MAX7219_NUMERIC

// Modify a single LED bit and send the changed line
void Max7219::led_set(const uint8_t x, const uint8_t y, const bool on, uint8_t * const rcm/*=nullptr*/) {
  if (x >= MAX7219_X_LEDS || y >= MAX7219_Y_LEDS) return error(F("led_set"), x, y);
  if (BIT_7219(x, y) == on) return;
  XOR_7219(x, y);
  refresh_unit_line(LED_IND(x, y));
  if (rcm != nullptr) *rcm |= _BV(LED_IND(x, y) & 0x07);
}

void Max7219::led_on(const uint8_t x, const uint8_t y, uint8_t * const rcm/*=nullptr*/) {
  if (x >= MAX7219_X_LEDS || y >= MAX7219_Y_LEDS) return error(F("led_on"), x, y);
  led_set(x, y, true, rcm);
}

void Max7219::led_off(const uint8_t x, const uint8_t y, uint8_t * const rcm/*=nullptr*/) {
  if (x >= MAX7219_X_LEDS || y >= MAX7219_Y_LEDS) return error(F("led_off"), x, y);
  led_set(x, y, false, rcm);
}

void Max7219::led_toggle(const uint8_t x, const uint8_t y, uint8_t * const rcm/*=nullptr*/) {
  if (x >= MAX7219_X_LEDS || y >= MAX7219_Y_LEDS) return error(F("led_toggle"), x, y);
  led_set(x, y, !BIT_7219(x, y), rcm);
}

void Max7219::send_row(const uint8_t row) {
  if (suspended) return;
  #if _ROT == 0 || _ROT == 180            // Native Lines are horizontal too
    #if MAX7219_X_LEDS <= 8
      refresh_unit_line(LED_IND(0, row)); // A single unit line
    #else
      refresh_line(LED_IND(0, row));      // Same line, all units
    #endif
  #else                                   // Native lines are vertical
    UNUSED(row);
    refresh();                            // Actually a column
  #endif
}

void Max7219::send_column(const uint8_t col) {
  if (suspended) return;
  #if _ROT == 90 || _ROT == 270           // Native Lines are vertical too
    #if MAX7219_Y_LEDS <= 8
      refresh_unit_line(LED_IND(col, 0)); // A single unit line
    #else
      refresh_line(LED_IND(col, 0));      // Same line, all units
    #endif
  #else                                   // Native lines are horizontal
    UNUSED(col);
    refresh();                            // Actually a row
  #endif
}

void Max7219::clear() {
  ZERO(led_line);
  refresh();
}

void Max7219::fill() {
  memset(led_line, 0xFF, sizeof(led_line));
  refresh();
}

void Max7219::clear_row(const uint8_t row) {
  if (row >= MAX7219_Y_LEDS) return error(F("clear_row"), row);
  for (uint8_t x = 0; x < MAX7219_X_LEDS; ++x) CLR_7219(x, row);
  send_row(row);
}

void Max7219::clear_column(const uint8_t col) {
  if (col >= MAX7219_X_LEDS) return error(F("set_column"), col);
  for (uint8_t y = 0; y < MAX7219_Y_LEDS; ++y) CLR_7219(col, y);
  send_column(col);
}

/**
 * Plot the low order bits of val to the specified row of the matrix.
 * With 4 Max7219 units in the chain, it's possible to set 32 bits at
 * once with a single call to the function (if rotated 90° or 270°).
 */
void Max7219::set_row(const uint8_t row, const uint32_t val) {
  if (row >= MAX7219_Y_LEDS) return error(F("set_row"), row);
  uint32_t mask = _BV32(MAX7219_X_LEDS - 1);
  for (uint8_t x = 0; x < MAX7219_X_LEDS; ++x) {
    if (val & mask) SET_7219(x, row); else CLR_7219(x, row);
    mask >>= 1;
  }
  send_row(row);
}

/**
 * Plot the low order bits of val to the specified column of the matrix.
 * With 4 Max7219 units in the chain, it's possible to set 32 bits at
 * once with a single call to the function (if rotated 0° or 180°).
 */
void Max7219::set_column(const uint8_t col, const uint32_t val) {
  if (col >= MAX7219_X_LEDS) return error(F("set_column"), col);
  uint32_t mask = _BV32(MAX7219_Y_LEDS - 1);
  for (uint8_t y = 0; y < MAX7219_Y_LEDS; ++y) {
    if (val & mask) SET_7219(col, y); else CLR_7219(col, y);
    mask >>= 1;
  }
  send_column(col);
}

void Max7219::set_rows_16bits(const uint8_t y, uint32_t val) {
  #if MAX7219_X_LEDS == 8
    if (y > MAX7219_Y_LEDS - 2) return error(F("set_rows_16bits"), y, val);
    set_row(y + 1, val); val >>= 8;
    set_row(y + 0, val);
  #else // at least 16 bits on each row
    if (y > MAX7219_Y_LEDS - 1) return error(F("set_rows_16bits"), y, val);
    set_row(y, val);
  #endif
}

void Max7219::set_rows_32bits(const uint8_t y, uint32_t val) {
  #if MAX7219_X_LEDS == 8
    if (y > MAX7219_Y_LEDS - 4) return error(F("set_rows_32bits"), y, val);
    set_row(y + 3, val); val >>= 8;
    set_row(y + 2, val); val >>= 8;
    set_row(y + 1, val); val >>= 8;
    set_row(y + 0, val);
  #elif MAX7219_X_LEDS == 16
    if (y > MAX7219_Y_LEDS - 2) return error(F("set_rows_32bits"), y, val);
    set_row(y + 1, val); val >>= 16;
    set_row(y + 0, val);
  #else // at least 24 bits on each row.  In the 3 matrix case, just display the low 24 bits
    if (y > MAX7219_Y_LEDS - 1) return error(F("set_rows_32bits"), y, val);
    set_row(y, val);
  #endif
}

void Max7219::set_columns_16bits(const uint8_t x, uint32_t val) {
  #if MAX7219_Y_LEDS == 8
    if (x > MAX7219_X_LEDS - 2) return error(F("set_columns_16bits"), x, val);
    set_column(x + 0, val); val >>= 8;
    set_column(x + 1, val);
  #else // at least 16 bits in each column
    if (x > MAX7219_X_LEDS - 1) return error(F("set_columns_16bits"), x, val);
    set_column(x, val);
  #endif
}

void Max7219::set_columns_32bits(const uint8_t x, uint32_t val) {
  #if MAX7219_Y_LEDS == 8
    if (x > MAX7219_X_LEDS - 4) return error(F("set_rows_32bits"), x, val);
    set_column(x + 3, val); val >>= 8;
    set_column(x + 2, val); val >>= 8;
    set_column(x + 1, val); val >>= 8;
    set_column(x + 0, val);
  #elif MAX7219_Y_LEDS == 16
    if (x > MAX7219_X_LEDS - 2) return error(F("set_rows_32bits"), x, val);
    set_column(x + 1, val); val >>= 16;
    set_column(x + 0, val);
  #else // at least 24 bits on each row.  In the 3 matrix case, just display the low 24 bits
    if (x > MAX7219_X_LEDS - 1) return error(F("set_rows_32bits"), x, val);
    set_column(x, val);
  #endif
}

// Initialize the Max7219
void Max7219::register_setup() {
  for (uint8_t i = 0; i < MAX7219_NUMBER_UNITS; ++i)
    send(max7219_reg_scanLimit, 0x07);
  pulse_load();                               // Tell the chips to load the clocked out data

  for (uint8_t i = 0; i < MAX7219_NUMBER_UNITS; ++i)
    send(max7219_reg_decodeMode, 0x00);       // Using an led matrix (not digits)
  pulse_load();                               // Tell the chips to load the clocked out data

  for (uint8_t i = 0; i < MAX7219_NUMBER_UNITS; ++i)
    send(max7219_reg_shutdown, 0x01);         // Not in shutdown mode
  pulse_load();                               // Tell the chips to load the clocked out data

  for (uint8_t i = 0; i < MAX7219_NUMBER_UNITS; ++i)
    send(max7219_reg_displayTest, 0x00);      // No display test
  pulse_load();                               // Tell the chips to load the clocked out data

  for (uint8_t i = 0; i < MAX7219_NUMBER_UNITS; ++i)
    send(max7219_reg_intensity, 0x01 & 0x0F); // The first 0x0F is the value you can set
                                              // Range: 0x00 to 0x0F
  pulse_load();                               // Tell the chips to load the clocked out data
}

#if MAX7219_INIT_TEST

  uint8_t test_mode = 0;
  millis_t next_patt_ms;
  bool patt_on;

  #if MAX7219_INIT_TEST == 2

    #define MAX7219_LEDS (MAX7219_X_LEDS * MAX7219_Y_LEDS)

    constexpr millis_t pattern_delay = 4;

    int8_t spiralx, spiraly, spiral_dir;
    uvalue_t(MAX7219_LEDS) spiral_count;

    void Max7219::test_pattern() {
      constexpr int8_t way[][2] = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } };
      led_set(spiralx, spiraly, patt_on);
      const int8_t x = spiralx + way[spiral_dir][0], y = spiraly + way[spiral_dir][1];
      if (!WITHIN(x, 0, MAX7219_X_LEDS - 1) || !WITHIN(y, 0, MAX7219_Y_LEDS - 1) || BIT_7219(x, y) == patt_on)
        spiral_dir = (spiral_dir + 1) & 0x3;
      spiralx += way[spiral_dir][0];
      spiraly += way[spiral_dir][1];
      if (!spiral_count--) {
        if (!patt_on)
          test_mode = 0;
        else {
          spiral_count = MAX7219_LEDS;
          spiralx = spiraly = spiral_dir = 0;
          patt_on = false;
        }
      }
    }

  #else

    constexpr millis_t pattern_delay = 20;
    int8_t sweep_count, sweepx, sweep_dir;

    void Max7219::test_pattern() {
      set_column(sweepx, patt_on ? 0xFFFFFFFF : 0x00000000);
      sweepx += sweep_dir;
      if (!WITHIN(sweepx, 0, MAX7219_X_LEDS - 1)) {
        if (!patt_on) {
          sweep_dir *= -1;
          sweepx += sweep_dir;
        }
        else
          sweepx -= MAX7219_X_LEDS * sweep_dir;
        FLIP(patt_on);
        next_patt_ms += 100;
        if (++test_mode > 4) test_mode = 0;
      }
    }

  #endif

  void Max7219::run_test_pattern() {
    const millis_t ms = millis();
    if (PENDING(ms, next_patt_ms)) return;
    next_patt_ms = ms + pattern_delay;
    test_pattern();
  }

  void Max7219::start_test_pattern() {
    clear();
    test_mode = 1;
    patt_on = true;
    #if MAX7219_INIT_TEST == 2
      spiralx = spiraly = spiral_dir = 0;
      spiral_count = MAX7219_LEDS;
    #else
      sweep_dir = 1;
      sweepx = 0;
      sweep_count = MAX7219_X_LEDS;
    #endif
  }

#endif // MAX7219_INIT_TEST

void Max7219::init() {
  SET_OUTPUT(MAX7219_DIN_PIN);
  SET_OUTPUT(MAX7219_CLK_PIN);
  OUT_WRITE(MAX7219_LOAD_PIN, HIGH);
  delay(1);

  register_setup();

  clear();

  #if MAX7219_INIT_TEST
    start_test_pattern();
  #endif

  #ifdef MAX7219_REINIT_ON_POWERUP
    #if defined(MAX7219_DEBUG_PLANNER_HEAD) && defined(MAX7219_DEBUG_PLANNER_TAIL) && MAX7219_DEBUG_PLANNER_HEAD == MAX7219_DEBUG_PLANNER_TAIL
      last_head_cnt = 0xF;
      last_tail_cnt = 0xF;
    #else
      #ifdef MAX7219_DEBUG_PLANNER_HEAD
        last_head_cnt = 0x1;
      #endif
      #ifdef MAX7219_DEBUG_PLANNER_TAIL
        last_tail_cnt = 0x1;
      #endif
    #endif
    #ifdef MAX7219_DEBUG_PLANNER_QUEUE
      last_depth = 0;
    #endif
    #ifdef MAX7219_DEBUG_PROFILE
      last_time_fraction = 0;
    #endif
    #ifdef MAX7219_DEBUG_MULTISTEPPING
      last_multistepping = 0;
    #endif
  #endif
}

/**
 * This code demonstrates some simple debugging using a single 8x8 LED Matrix. If your feature could
 * benefit from matrix display, add its code here. Very little processing is required, so the 7219 is
 * ideal for debugging when realtime feedback is important but serial output can't be used.
 */

// Apply changes to update a marker
void Max7219::mark16(const uint8_t pos, const uint8_t v1, const uint8_t v2, uint8_t * const rcm/*=nullptr*/) {
  #if MAX7219_X_LEDS > 8    // At least 16 LEDs on the X-Axis. Use single line.
    led_off(v1 & 0xF, pos, rcm);
     led_on(v2 & 0xF, pos, rcm);
  #elif MAX7219_Y_LEDS > 8  // At least 16 LEDs on the Y-Axis. Use a single column.
    led_off(pos, v1 & 0xF, rcm);
     led_on(pos, v2 & 0xF, rcm);
  #else                     // Single 8x8 LED matrix. Use two lines to get 16 LEDs.
    led_off(v1 & 0x7, pos + (v1 >= 8), rcm);
     led_on(v2 & 0x7, pos + (v2 >= 8), rcm);
  #endif
}

// Apply changes to update a tail-to-head range
void Max7219::range16(const uint8_t y, const uint8_t ot, const uint8_t nt, const uint8_t oh,
  const uint8_t nh, uint8_t * const rcm/*=nullptr*/) {
  #if MAX7219_X_LEDS > 8    // At least 16 LEDs on the X-Axis. Use single line.
    if (ot != nt) for (uint8_t n = ot & 0xF; n != (nt & 0xF) && n != (nh & 0xF); n = (n + 1) & 0xF)
      led_off(n & 0xF, y, rcm);
    if (oh != nh) for (uint8_t n = (oh + 1) & 0xF; n != ((nh + 1) & 0xF); n = (n + 1) & 0xF)
       led_on(n & 0xF, y, rcm);
  #elif MAX7219_Y_LEDS > 8  // At least 16 LEDs on the Y-Axis. Use a single column.
    if (ot != nt) for (uint8_t n = ot & 0xF; n != (nt & 0xF) && n != (nh & 0xF); n = (n + 1) & 0xF)
      led_off(y, n & 0xF, rcm);
    if (oh != nh) for (uint8_t n = (oh + 1) & 0xF; n != ((nh + 1) & 0xF); n = (n + 1) & 0xF)
       led_on(y, n & 0xF, rcm);
  #else                     // Single 8x8 LED matrix. Use two lines to get 16 LEDs.
    if (ot != nt) for (uint8_t n = ot & 0xF; n != (nt & 0xF) && n != (nh & 0xF); n = (n + 1) & 0xF)
      led_off(n & 0x7, y + (n >= 8), rcm);
    if (oh != nh) for (uint8_t n = (oh + 1) & 0xF; n != ((nh + 1) & 0xF); n = (n + 1) & 0xF)
       led_on(n & 0x7, y + (n >= 8), rcm);
  #endif
}

// Apply changes to update a quantity
void Max7219::quantity(const uint8_t pos, const uint8_t ov, const uint8_t nv, uint8_t * const rcm/*=nullptr*/) {
  for (uint8_t i = _MIN(nv, ov); i < _MAX(nv, ov); i++)
    led_set(
      #if MAX7219_X_LEDS >= MAX7219_Y_LEDS
        i, pos  // Single matrix or multiple matrices in Landscape
      #else
        pos, i  // Multiple matrices in Portrait
      #endif
      , nv >= ov
      , rcm
    );
}

void Max7219::quantity16(const uint8_t pos, const uint8_t ov, const uint8_t nv, uint8_t * const rcm/*=nullptr*/) {
  for (uint8_t i = _MIN(nv, ov); i < _MAX(nv, ov); i++)
    led_set(
      #if MAX7219_X_LEDS > 8    // At least 16 LEDs on the X-Axis. Use single line.
        i, pos
      #elif MAX7219_Y_LEDS > 8  // At least 16 LEDs on the Y-Axis. Use a single column.
        pos, i
      #else                     // Single 8x8 LED matrix. Use two lines to get 16 LEDs.
        i >> 1, pos + (i & 1)
      #endif
      , nv >= ov
      , rcm
    );
}

void Max7219::idle_tasks() {
  #define MAX7219_USE_HEAD (defined(MAX7219_DEBUG_PLANNER_HEAD) || defined(MAX7219_DEBUG_PLANNER_QUEUE))
  #define MAX7219_USE_TAIL (defined(MAX7219_DEBUG_PLANNER_TAIL) || defined(MAX7219_DEBUG_PLANNER_QUEUE))
  #if MAX7219_USE_HEAD || MAX7219_USE_TAIL
    CRITICAL_SECTION_START();
    #if MAX7219_USE_HEAD
      const uint8_t head = planner.block_buffer_head;
    #endif
    #if MAX7219_USE_TAIL
      const uint8_t tail = planner.block_buffer_tail;
    #endif
    CRITICAL_SECTION_END();
  #endif

  #if ENABLED(MAX7219_DEBUG_PRINTER_ALIVE)
    static uint8_t refresh_cnt; // = 0
    constexpr uint16_t refresh_limit = 5;
    static millis_t next_blink = 0;
    const millis_t ms = millis();
    const bool do_blink = ELAPSED(ms, next_blink);
  #else
    static uint16_t refresh_cnt; // = 0
    constexpr bool do_blink = true;
    constexpr uint16_t refresh_limit = 50000;
  #endif

  // Some Max7219 units are vulnerable to electrical noise, especially
  // with long wires next to high current wires. If the display becomes
  // corrupted, this will fix it within a couple seconds.
  if (do_blink && ++refresh_cnt >= refresh_limit) {
    refresh_cnt = 0;
    register_setup();
  }

  #if MAX7219_INIT_TEST
    if (test_mode) {
      run_test_pattern();
      return;
    }
  #endif

  // suspend updates and record which lines have changed for batching later
  suspended++;
  uint8_t row_change_mask = 0x00;

  #if ENABLED(MAX7219_DEBUG_PRINTER_ALIVE)
    if (do_blink) {
      led_toggle(MAX7219_X_LEDS - 1, MAX7219_Y_LEDS - 1, &row_change_mask);
      next_blink = ms + 1000;
    }
  #endif

  #if defined(MAX7219_DEBUG_PLANNER_HEAD) && defined(MAX7219_DEBUG_PLANNER_TAIL) && MAX7219_DEBUG_PLANNER_HEAD == MAX7219_DEBUG_PLANNER_TAIL

    if (last_head_cnt != head || last_tail_cnt != tail) {
      range16(MAX7219_DEBUG_PLANNER_HEAD, last_tail_cnt, tail, last_head_cnt, head, &row_change_mask);
      last_head_cnt = head;
      last_tail_cnt = tail;
    }

  #else

    #ifdef MAX7219_DEBUG_PLANNER_HEAD
      if (last_head_cnt != head) {
        mark16(MAX7219_DEBUG_PLANNER_HEAD, last_head_cnt, head, &row_change_mask);
        last_head_cnt = head;
      }
    #endif

    #ifdef MAX7219_DEBUG_PLANNER_TAIL
      if (last_tail_cnt != tail) {
        mark16(MAX7219_DEBUG_PLANNER_TAIL, last_tail_cnt, tail, &row_change_mask);
        last_tail_cnt = tail;
      }
    #endif

  #endif

  #ifdef MAX7219_DEBUG_PLANNER_QUEUE
    const int16_t current_depth = BLOCK_MOD(head - tail + (BLOCK_BUFFER_SIZE)) & 0xF;
    if (current_depth != last_depth) {
      quantity16(MAX7219_DEBUG_PLANNER_QUEUE, last_depth, current_depth, &row_change_mask);
      last_depth = current_depth;
    }
  #endif

  #ifdef MAX7219_DEBUG_PROFILE
    const uint8_t current_time_fraction = (uint16_t(CodeProfiler::get_time_fraction()) * MAX7219_NUMBER_UNITS + 8) / 16;
    if (current_time_fraction != last_time_fraction) {
      quantity(MAX7219_DEBUG_PROFILE, last_time_fraction, current_time_fraction, &row_change_mask);
      last_time_fraction = current_time_fraction;
    }
  #endif

  #ifdef MAX7219_DEBUG_MULTISTEPPING
    static uint8_t last_multistepping = 0;
    const uint8_t multistepping = Stepper::steps_per_isr;
    if (multistepping != last_multistepping) {
      static uint8_t log2_old = 0;
      uint8_t log2_new = 0;
      for (uint8_t val = multistepping; val > 1; val >>= 1) log2_new++;
      mark16(MAX7219_DEBUG_MULTISTEPPING, log2_old, log2_new, &row_change_mask);
      last_multistepping = multistepping;
      log2_old = log2_new;
    }
  #endif

  #ifdef MAX7219_DEBUG_SLOWDOWN
    static uint8_t last_slowdown_count = 0;
    const uint8_t slowdown_count = Planner::slowdown_count;
    if (slowdown_count != last_slowdown_count) {
      mark16(MAX7219_DEBUG_SLOWDOWN, last_slowdown_count, slowdown_count, &row_change_mask);
      last_slowdown_count = slowdown_count;
    }
  #endif

  // batch line updates
  suspended--;
  if (!suspended)
    for (uint8_t i = 0; i < 8; ++i) if (row_change_mask & _BV(i))
      refresh_line(i);

  // After resume() automatically do a refresh()
  if (suspended == 0x80) {
    suspended = 0;
    refresh();
  }
}

#endif // MAX7219_DEBUG
