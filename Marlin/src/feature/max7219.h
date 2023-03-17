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
 * max7219.init() is called automatically at startup, and then there are a number of
 * support functions available to control the LEDs in the 8x8 grid.
 *
 * If you are using the Max7219 matrix for firmware debug purposes in time sensitive
 * areas of the code, please be aware that the orientation (rotation) of the display can
 * affect the speed.   The Max7219 can update a single column fairly fast.  It is much
 * faster to do a Max7219_Set_Column() with a rotation of 90 or 270 degrees than to do
 * a Max7219_Set_Row().    The opposite is true for rotations of 0 or 180 degrees.
 */

#include "../inc/MarlinConfig.h"

#ifndef MAX7219_ROTATE
  #define MAX7219_ROTATE 0
#endif

#ifndef MAX7219_NUMBER_UNITS
  #define MAX7219_NUMBER_UNITS 1
#endif
#define MAX7219_LINES (8 * (MAX7219_NUMBER_UNITS))

//
// MAX7219 registers
//
#define max7219_reg_noop        0x00
#define max7219_reg_digit0      0x01
#define max7219_reg_digit1      0x02
#define max7219_reg_digit2      0x03
#define max7219_reg_digit3      0x04
#define max7219_reg_digit4      0x05
#define max7219_reg_digit5      0x06
#define max7219_reg_digit6      0x07
#define max7219_reg_digit7      0x08

#define max7219_reg_decodeMode  0x09
#define max7219_reg_intensity   0x0A
#define max7219_reg_scanLimit   0x0B
#define max7219_reg_shutdown    0x0C
#define max7219_reg_displayTest 0x0F

#ifdef MAX7219_DEBUG_PROFILE
  // This class sums up the amount of time for which its instances exist.
  // By default there is one instantiated for the duration of the idle()
  // function. But an instance can be created in any code block to measure
  // the time spent from the point of instantiation until the CPU leaves
  // block. Be careful about having multiple instances of CodeProfiler as
  // it does not guard against double counting. In general mixing ISR and
  // non-ISR use will require critical sections but note that mode setting
  // is atomic so the total or average times can safely be read if you set
  // mode to FREEZE first.
  class CodeProfiler {
  public:
    enum Mode : uint8_t { ACCUMULATE_AVERAGE, ACCUMULATE_TOTAL, FREEZE };

  private:
    static Mode mode;
    static uint8_t instance_count;
    static uint32_t last_calc_time;
    static uint32_t total_time;
    static uint8_t time_fraction;
    static uint16_t call_count;

    uint32_t start_time;

  public:
    CodeProfiler() : start_time(micros()) { instance_count++; }
    ~CodeProfiler() {
      instance_count--;
      if (mode == FREEZE) return;

      call_count++;

      const uint32_t now = micros();
      total_time += now - start_time;

      if (mode == ACCUMULATE_TOTAL) return;

      // update time_fraction every hundred milliseconds
      if (instance_count == 0 && ELAPSED(now, last_calc_time + 100000)) {
        time_fraction = total_time * 128 / (now - last_calc_time);
        last_calc_time = now;
        total_time = 0;
      }
    }

    static void set_mode(Mode _mode) { mode = _mode; }
    static void reset() {
      time_fraction = 0;
      last_calc_time = micros();
      total_time = 0;
      call_count = 0;
    }
    // returns fraction of total time which was measured, scaled from 0 to 128
    static uint8_t get_time_fraction() { return time_fraction; }
    // returns total time in microseconds
    static uint32_t get_total_time() { return total_time; }

    static uint16_t get_call_count() { return call_count; }
  };
#endif

class Max7219 {
public:
  static uint8_t led_line[MAX7219_LINES];

  Max7219() {}

  static void init();
  static void register_setup();
  static void putbyte(uint8_t data);
  static void pulse_load();

  // Set a single register (e.g., a whole native row)
  static void send(const uint8_t reg, const uint8_t data);

  // Refresh all units
  static void refresh() { for (uint_fast8_t i = 0; i < 8; i++) refresh_line(i); }

  // Suspend / resume updates to the LED unit
  // Use these methods to speed up multiple changes
  // or to apply updates from interrupt context.
  static void suspend() { suspended++; }
  static void resume() { suspended--; suspended |= 0x80; }

  // Update a single native line on all units
  static void refresh_line(const uint8_t line);

  // Update a single native line on just one unit
  static void refresh_unit_line(const uint8_t line);

  #if ENABLED(MAX7219_NUMERIC)
    // Draw an integer with optional leading zeros and optional decimal point
    void print(const uint8_t start, int16_t value, uint8_t size, const bool leadzero=false, bool dec=false);
    // Draw a float with a decimal point and optional digits
    void print(const uint8_t start, const_float_t value, const uint8_t pre_size, const uint8_t post_size, const bool leadzero=false);
  #endif

  // Set a single LED by XY coordinate
  static void led_set(const uint8_t x, const uint8_t y, const bool on, uint8_t * const rcm=nullptr);
  static void led_on(const uint8_t x, const uint8_t y, uint8_t * const rcm=nullptr);
  static void led_off(const uint8_t x, const uint8_t y, uint8_t * const rcm=nullptr);
  static void led_toggle(const uint8_t x, const uint8_t y, uint8_t * const rcm=nullptr);

  // Set all LEDs in a single column
  static void set_column(const uint8_t col, const uint32_t val);
  static void clear_column(const uint8_t col);

  // Set all LEDs in a single row
  static void set_row(const uint8_t row, const uint32_t val);
  static void clear_row(const uint8_t row);

  // 16 and 32 bit versions of Row and Column functions
  // Multiple rows and columns will be used to display the value if
  // the array of matrix LED's is too narrow to accomplish the goal
  static void set_rows_16bits(const uint8_t y, uint32_t val);
  static void set_rows_32bits(const uint8_t y, uint32_t val);
  static void set_columns_16bits(const uint8_t x, uint32_t val);
  static void set_columns_32bits(const uint8_t x, uint32_t val);

  // Quickly clear the whole matrix
  static void clear();

  // Quickly fill the whole matrix
  static void fill();

  // Apply custom code to update the matrix
  static void idle_tasks();

private:
  static uint8_t suspended;
  static void error(FSTR_P const func, const int32_t v1, const int32_t v2=-1);
  static void noop();
  static void set(const uint8_t line, const uint8_t bits);
  static void send_row(const uint8_t row);
  static void send_column(const uint8_t col);
  static void mark16(const uint8_t y, const uint8_t v1, const uint8_t v2, uint8_t * const rcm=nullptr);
  static void range16(const uint8_t y, const uint8_t ot, const uint8_t nt, const uint8_t oh, const uint8_t nh, uint8_t * const rcm=nullptr);
  static void quantity(const uint8_t y, const uint8_t ov, const uint8_t nv, uint8_t * const rcm=nullptr);
  static void quantity16(const uint8_t y, const uint8_t ov, const uint8_t nv, uint8_t * const rcm=nullptr);

  #if MAX7219_INIT_TEST
    static void test_pattern();
    static void run_test_pattern();
    static void start_test_pattern();
  #endif
};

extern Max7219 max7219;
