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
 * planner.cpp
 *
 * Buffer movement commands and manage the acceleration profile plan
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Ring buffer gleaned from wiring_serial library by David A. Mellis.
 *
 * Fast inverse function needed for Bézier interpolation for AVR
 * was designed, written and tested by Eduardo José Tagle, April 2018.
 *
 * Planner mathematics (Mathematica-style):
 *
 * Where: s == speed, a == acceleration, t == time, d == distance
 *
 * Basic definitions:
 *   Speed[s_, a_, t_] := s + (a*t)
 *   Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 *
 * Distance to reach a specific speed with a constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 *   d -> (m^2 - s^2) / (2 a)
 *
 * Speed after a given distance of travel with constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 *   m -> Sqrt[2 a d + s^2]
 *
 * DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 *
 * When to start braking (di) to reach a specified destination speed (s2) after
 * acceleration from initial speed s1 without ever reaching a plateau:
 *   Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 *   di -> (2 a d - s1^2 + s2^2)/(4 a)
 *
 * We note, as an optimization, that if we have already calculated an
 * acceleration distance d1 from s1 to m and a deceration distance d2
 * from m to s2 then
 *
 *   d1 -> (m^2 - s1^2) / (2 a)
 *   d2 -> (m^2 - s2^2) / (2 a)
 *   di -> (d + d1 - d2) / 2
 */

#include "planner.h"
#include "stepper.h"
#include "motion.h"
#include "temperature.h"
#if ENABLED(FT_MOTION)
  #include "ft_motion.h"
#endif
#include "../lcd/marlinui.h"
#include "../gcode/parser.h"

#include "../MarlinCore.h"

#if HAS_LEVELING
  #include "../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  #include "../feature/filwidth.h"
#endif

#if ENABLED(BARICUDA)
  #include "../feature/baricuda.h"
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "../feature/mixing.h"
#endif

#if ENABLED(AUTO_POWER_CONTROL)
  #include "../feature/power.h"
#endif

#if ENABLED(BACKLASH_COMPENSATION)
  #include "../feature/backlash.h"
#endif

#if ENABLED(CANCEL_OBJECTS)
  #include "../feature/cancel_object.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../feature/powerloss.h"
#endif

#if HAS_CUTTER
  #include "../feature/spindle_laser.h"
#endif

// Delay for delivery of first block to the stepper ISR, if the queue contains 2 or
// fewer movements. The delay is measured in milliseconds, and must be less than 250ms
#define BLOCK_DELAY_NONE         0U
#define BLOCK_DELAY_FOR_1ST_MOVE 100U

Planner planner;

// public:

/**
 * A ring buffer of moves described in steps
 */
block_t Planner::block_buffer[BLOCK_BUFFER_SIZE];
volatile uint8_t Planner::block_buffer_head,    // Index of the next block to be pushed
                 Planner::block_buffer_nonbusy, // Index of the first non-busy block
                 Planner::block_buffer_tail;    // Index of the busy block, if any
uint16_t Planner::cleaning_buffer_counter;      // A counter to disable queuing of blocks
uint8_t Planner::delay_before_delivering;       // Delay block delivery so initial blocks in an empty queue may merge

#if ENABLED(EDITABLE_STEPS_PER_UNIT)
  float Planner::mm_per_step[DISTINCT_AXES];    // (mm) Millimeters per step
#else
  constexpr float PlannerSettings::axis_steps_per_mm[DISTINCT_AXES],
                  Planner::mm_per_step[DISTINCT_AXES];
#endif
planner_settings_t Planner::settings;           // Initialized by settings.load

/**
 * Set up inline block variables
 * Set laser_power_floor based on SPEED_POWER_MIN to pevent a zero power output state with LASER_POWER_TRAP
 */
#if ENABLED(LASER_FEATURE)
  laser_state_t Planner::laser_inline;          // Current state for blocks
  const uint8_t laser_power_floor = cutter.pct_to_ocr(SPEED_POWER_MIN);
#endif

uint32_t Planner::max_acceleration_steps_per_s2[DISTINCT_AXES]; // (steps/s^2) Derived from mm_per_s2

#if HAS_JUNCTION_DEVIATION
  float Planner::junction_deviation_mm;         // (mm) M205 J
  #if HAS_LINEAR_E_JERK
    float Planner::max_e_jerk[DISTINCT_E];      // Calculated from junction_deviation_mm
  #endif
#else // CLASSIC_JERK
  TERN(HAS_LINEAR_E_JERK, xyz_pos_t, xyze_pos_t) Planner::max_jerk;
#endif

#if ENABLED(SD_ABORT_ON_ENDSTOP_HIT)
  bool Planner::abort_on_endstop_hit = false;
#endif

#if ENABLED(DISTINCT_E_FACTORS)
  uint8_t Planner::last_extruder = 0;     // Respond to extruder change
#endif

#if ENABLED(DIRECT_STEPPING)
  uint32_t Planner::last_page_step_rate = 0;
  AxisBits Planner::last_page_dir; // = 0
#endif

#if HAS_EXTRUDERS
  int16_t Planner::flow_percentage[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(100); // Extrusion factor for each extruder
  float Planner::e_factor[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(1.0f); // The flow percentage and volumetric multiplier combine to scale E movement
#endif

#if DISABLED(NO_VOLUMETRICS)
  float Planner::volumetric_area_nominal = CIRCLE_AREA(float(DEFAULT_NOMINAL_FILAMENT_DIA) * 0.5f); // Nominal cross-sectional area
  float Planner::filament_size[EXTRUDERS],          // diameter of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder
        Planner::volumetric_multiplier[EXTRUDERS];  // Reciprocal of cross-sectional area of filament (in mm^2). Pre-calculated to reduce computation in the planner
#endif

#if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
  float Planner::volumetric_extruder_limit[EXTRUDERS],          // max mm^3/sec the extruder is able to handle
        Planner::volumetric_extruder_feedrate_limit[EXTRUDERS]; // pre calculated extruder feedrate limit based on volumetric_extruder_limit; pre-calculated to reduce computation in the planner
#endif

#ifdef MAX7219_DEBUG_SLOWDOWN
  uint8_t Planner::slowdown_count = 0;
#endif

#if HAS_LEVELING
  bool Planner::leveling_active = false; // Flag that auto bed leveling is enabled
  #if ABL_PLANAR
    matrix_3x3 Planner::bed_level_matrix; // Transform to compensate for bed level
  #endif
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    float Planner::z_fade_height,      // Initialized by settings.load
          Planner::inverse_z_fade_height,
          Planner::last_fade_z;
  #endif
#else
  constexpr bool Planner::leveling_active;
#endif

#if ENABLED(SKEW_CORRECTION)
  skew_factor_t Planner::skew_factor; // Initialized by settings.load
#endif

#if ENABLED(AUTOTEMP)
  autotemp_t Planner::autotemp = { AUTOTEMP_MIN, AUTOTEMP_MAX, AUTOTEMP_FACTOR, false };
#endif

// private:

xyze_long_t Planner::position{0};

uint32_t Planner::acceleration_long_cutoff;

xyze_float_t Planner::previous_speed;
float Planner::previous_nominal_speed;

#if ENABLED(DISABLE_OTHER_EXTRUDERS)
  last_move_t Planner::extruder_last_move[E_STEPPERS] = { 0 };
#endif

#ifdef XY_FREQUENCY_LIMIT
  int8_t Planner::xy_freq_limit_hz = XY_FREQUENCY_LIMIT;
  float Planner::xy_freq_min_speed_factor = (XY_FREQUENCY_MIN_PERCENT) * 0.01f;
  int32_t Planner::xy_freq_min_interval_us = LROUND(1000000.0f / (XY_FREQUENCY_LIMIT));
#endif

#if ENABLED(LIN_ADVANCE)
  float Planner::extruder_advance_K[DISTINCT_E]; // Initialized by settings.load
#endif

#if HAS_POSITION_FLOAT
  xyze_pos_t Planner::position_float; // Needed for accurate maths. Steps cannot be used!
#endif

#if IS_KINEMATIC
  xyze_pos_t Planner::position_cart;
#endif

#if HAS_WIRED_LCD
  volatile uint32_t Planner::block_buffer_runtime_us = 0;
#endif

/**
 * Class and Instance Methods
 */

Planner::Planner() { init(); }

void Planner::init() {
  position.reset();
  TERN_(HAS_POSITION_FLOAT, position_float.reset());
  TERN_(IS_KINEMATIC, position_cart.reset());

  previous_speed.reset();
  previous_nominal_speed = 0;

  TERN_(ABL_PLANAR, bed_level_matrix.set_to_identity());

  clear_block_buffer();
  delay_before_delivering = 0;

  #if ENABLED(DIRECT_STEPPING)
    last_page_step_rate = 0;
    last_page_dir.reset();
  #endif
}

#if ENABLED(S_CURVE_ACCELERATION)
  #ifdef __AVR__
    /**
     * This routine returns 0x1000000 / d, getting the inverse as fast as possible.
     * A fast-converging iterative Newton-Raphson method can reach full precision in
     * just 1 iteration, and takes 211 cycles (worst case; the mean case is less, up
     * to 30 cycles for small divisors), instead of the 500 cycles a normal division
     * would take.
     *
     * Inspired by the following page:
     *  https://stackoverflow.com/questions/27801397/newton-raphson-division-with-big-integers
     *
     * Suppose we want to calculate  floor(2 ^ k / B)  where B is a positive integer
     * Then, B must be <= 2^k, otherwise, the quotient is 0.
     *
     * The Newton - Raphson iteration for x = B / 2 ^ k yields:
     *  q[n + 1] = q[n] * (2 - q[n] * B / 2 ^ k)
     *
     * This can be rearranged to:
     *  q[n + 1] = q[n] * (2 ^ (k + 1) - q[n] * B) >> k
     *
     * Each iteration requires only integer multiplications and bit shifts.
     * It doesn't necessarily converge to floor(2 ^ k / B) but in the worst case
     * it eventually alternates between floor(2 ^ k / B) and ceil(2 ^ k / B).
     * So it checks for this case and extracts floor(2 ^ k / B).
     *
     * A simple but important optimization for this approach is to truncate
     * multiplications (i.e., calculate only the higher bits of the product) in the
     * early iterations of the Newton - Raphson method. This is done so the results
     * of the early iterations are far from the quotient. Then it doesn't matter if
     * they are done inaccurately.
     * It's important to pick a good starting value for x. Knowing how many
     * digits the divisor has, it can be estimated:
     *
     *   2^k / x = 2 ^ log2(2^k / x)
     *   2^k / x = 2 ^(log2(2^k)-log2(x))
     *   2^k / x = 2 ^(k*log2(2)-log2(x))
     *   2^k / x = 2 ^ (k-log2(x))
     *   2^k / x >= 2 ^ (k-floor(log2(x)))
     *   floor(log2(x)) is simply the index of the most significant bit set.
     *
     * If this estimation can be improved even further the number of iterations can be
     * reduced a lot, saving valuable execution time.
     * The paper "Software Integer Division" by Thomas L.Rodeheffer, Microsoft
     * Research, Silicon Valley,August 26, 2008, available at
     * https://www.microsoft.com/en-us/research/wp-content/uploads/2008/08/tr-2008-141.pdf
     * suggests, for its integer division algorithm, using a table to supply the first
     * 8 bits of precision, then, due to the quadratic convergence nature of the
     * Newton-Raphon iteration, just 2 iterations should be enough to get maximum
     * precision of the division.
     * By precomputing values of inverses for small denominator values, just one
     * Newton-Raphson iteration is enough to reach full precision.
     * This code uses the top 9 bits of the denominator as index.
     *
     * The AVR assembly function implements this C code using the data below:
     *
     *  // For small divisors, it is best to directly retrieve the results
     *  if (d <= 110) return pgm_read_dword(&small_inv_tab[d]);
     *
     *  // Compute initial estimation of 0x1000000/x -
     *  // Get most significant bit set on divider
     *  uint8_t idx = 0;
     *  uint32_t nr = d;
     *  if (!(nr & 0xFF0000)) {
     *    nr <<= 8; idx += 8;
     *    if (!(nr & 0xFF0000)) { nr <<= 8; idx += 8; }
     *  }
     *  if (!(nr & 0xF00000)) { nr <<= 4; idx += 4; }
     *  if (!(nr & 0xC00000)) { nr <<= 2; idx += 2; }
     *  if (!(nr & 0x800000)) { nr <<= 1; idx += 1; }
     *
     *  // Isolate top 9 bits of the denominator, to be used as index into the initial estimation table
     *  uint32_t tidx = nr >> 15,                                       // top 9 bits. bit8 is always set
     *           ie = inv_tab[tidx & 0xFF] + 256,                       // Get the table value. bit9 is always set
     *           x = idx <= 8 ? (ie >> (8 - idx)) : (ie << (idx - 8));  // Position the estimation at the proper place
     *
     *  x = uint32_t((x * uint64_t(_BV(25) - x * d)) >> 24);            // Refine estimation by newton-raphson. 1 iteration is enough
     *  const uint32_t r = _BV(24) - x * d;                             // Estimate remainder
     *  if (r >= d) x++;                                                // Check whether to adjust result
     *  return uint32_t(x);                                             // x holds the proper estimation
     */
    static uint32_t get_period_inverse(uint32_t d) {

      static const uint8_t inv_tab[256] PROGMEM = {
        255,253,252,250,248,246,244,242,240,238,236,234,233,231,229,227,
        225,224,222,220,218,217,215,213,212,210,208,207,205,203,202,200,
        199,197,195,194,192,191,189,188,186,185,183,182,180,179,178,176,
        175,173,172,170,169,168,166,165,164,162,161,160,158,157,156,154,
        153,152,151,149,148,147,146,144,143,142,141,139,138,137,136,135,
        134,132,131,130,129,128,127,126,125,123,122,121,120,119,118,117,
        116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,
        100,99,98,97,96,95,94,93,92,91,90,89,88,88,87,86,
        85,84,83,82,81,80,80,79,78,77,76,75,74,74,73,72,
        71,70,70,69,68,67,66,66,65,64,63,62,62,61,60,59,
        59,58,57,56,56,55,54,53,53,52,51,50,50,49,48,48,
        47,46,46,45,44,43,43,42,41,41,40,39,39,38,37,37,
        36,35,35,34,33,33,32,32,31,30,30,29,28,28,27,27,
        26,25,25,24,24,23,22,22,21,21,20,19,19,18,18,17,
        17,16,15,15,14,14,13,13,12,12,11,10,10,9,9,8,
        8,7,7,6,6,5,5,4,4,3,3,2,2,1,0,0
      };

      // For small denominators, it is cheaper to directly store the result.
      //  For bigger ones, just ONE Newton-Raphson iteration is enough to get
      //  maximum precision we need
      static const uint32_t small_inv_tab[111] PROGMEM = {
        16777216,16777216,8388608,5592405,4194304,3355443,2796202,2396745,2097152,1864135,1677721,1525201,1398101,1290555,1198372,1118481,
        1048576,986895,932067,883011,838860,798915,762600,729444,699050,671088,645277,621378,599186,578524,559240,541200,
        524288,508400,493447,479349,466033,453438,441505,430185,419430,409200,399457,390167,381300,372827,364722,356962,
        349525,342392,335544,328965,322638,316551,310689,305040,299593,294337,289262,284359,279620,275036,270600,266305,
        262144,258111,254200,250406,246723,243148,239674,236298,233016,229824,226719,223696,220752,217885,215092,212369,
        209715,207126,204600,202135,199728,197379,195083,192841,190650,188508,186413,184365,182361,180400,178481,176602,
        174762,172960,171196,169466,167772,166111,164482,162885,161319,159783,158275,156796,155344,153919,152520
      };

      // For small divisors, it is best to directly retrieve the results
      if (d <= 110) return pgm_read_dword(&small_inv_tab[d]);

      uint8_t r8 = d & 0xFF,
              r9 = (d >> 8) & 0xFF,
              r10 = (d >> 16) & 0xFF,
              r2,r3,r4,r5,r6,r7,r11,r12,r13,r14,r15,r16,r17,r18;
      const uint8_t *ptab = inv_tab;

      __asm__ __volatile__(
        // %8:%7:%6 = interval
        // r31:r30: MUST be those registers, and they must point to the inv_tab

        A("clr %13")                       // %13 = 0

        // Now we must compute
        // result = 0xFFFFFF / d
        // %8:%7:%6 = interval
        // %16:%15:%14 = nr
        // %13 = 0

        // A plain division of 24x24 bits should take 388 cycles to complete. We will
        // use Newton-Raphson for the calculation, and will strive to get way less cycles
        // for the same result - Using C division, it takes 500cycles to complete .

        A("clr %3")                       // idx = 0
        A("mov %14,%6")
        A("mov %15,%7")
        A("mov %16,%8")                   // nr = interval
        A("tst %16")                      // nr & 0xFF0000 == 0 ?
        A("brne 2f")                      // No, skip this
        A("mov %16,%15")
        A("mov %15,%14")                  // nr <<= 8, %14 not needed
        A("subi %3,-8")                   // idx += 8
        A("tst %16")                      // nr & 0xFF0000 == 0 ?
        A("brne 2f")                      // No, skip this
        A("mov %16,%15")                  // nr <<= 8, %14 not needed
        A("clr %15")                      // We clear %14
        A("subi %3,-8")                   // idx += 8

        // here %16 != 0 and %16:%15 contains at least 9 MSBits, or both %16:%15 are 0
        L("2")
        A("cpi %16,0x10")                 // (nr & 0xF00000) == 0 ?
        A("brcc 3f")                      // No, skip this
        A("swap %15")                     // Swap nybbles
        A("swap %16")                     // Swap nybbles. Low nybble is 0
        A("mov %14, %15")
        A("andi %14,0x0F")                // Isolate low nybble
        A("andi %15,0xF0")                // Keep proper nybble in %15
        A("or %16, %14")                  // %16:%15 <<= 4
        A("subi %3,-4")                   // idx += 4

        L("3")
        A("cpi %16,0x40")                 // (nr & 0xC00000) == 0 ?
        A("brcc 4f")                      // No, skip this
        A("add %15,%15")
        A("adc %16,%16")
        A("add %15,%15")
        A("adc %16,%16")                  // %16:%15 <<= 2
        A("subi %3,-2")                   // idx += 2

        L("4")
        A("cpi %16,0x80")                 // (nr & 0x800000) == 0 ?
        A("brcc 5f")                      // No, skip this
        A("add %15,%15")
        A("adc %16,%16")                  // %16:%15 <<= 1
        A("inc %3")                       // idx += 1

        // Now %16:%15 contains its MSBit set to 1, or %16:%15 is == 0. We are now absolutely sure
        // we have at least 9 MSBits available to enter the initial estimation table
        L("5")
        A("add %15,%15")
        A("adc %16,%16")                  // %16:%15 = tidx = (nr <<= 1), we lose the top MSBit (always set to 1, %16 is the index into the inverse table)
        A("add r30,%16")                  // Only use top 8 bits
        A("adc r31,%13")                  // r31:r30 = inv_tab + (tidx)
        A("lpm %14, Z")                   // %14 = inv_tab[tidx]
        A("ldi %15, 1")                   // %15 = 1  %15:%14 = inv_tab[tidx] + 256

        // We must scale the approximation to the proper place
        A("clr %16")                      // %16 will always be 0 here
        A("subi %3,8")                    // idx == 8 ?
        A("breq 6f")                      // yes, no need to scale
        A("brcs 7f")                      // If C=1, means idx < 8, result was negative!

        // idx > 8, now %3 = idx - 8. We must perform a left shift. idx range:[1-8]
        A("sbrs %3,0")                    // shift by 1bit position?
        A("rjmp 8f")                      // No
        A("add %14,%14")
        A("adc %15,%15")                  // %15:16 <<= 1
        L("8")
        A("sbrs %3,1")                    // shift by 2bit position?
        A("rjmp 9f")                      // No
        A("add %14,%14")
        A("adc %15,%15")
        A("add %14,%14")
        A("adc %15,%15")                  // %15:16 <<= 1
        L("9")
        A("sbrs %3,2")                    // shift by 4bits position?
        A("rjmp 16f")                     // No
        A("swap %15")                     // Swap nybbles. lo nybble of %15 will always be 0
        A("swap %14")                     // Swap nybbles
        A("mov %12,%14")
        A("andi %12,0x0F")                // isolate low nybble
        A("andi %14,0xF0")                // and clear it
        A("or %15,%12")                   // %15:%16 <<= 4
        L("16")
        A("sbrs %3,3")                    // shift by 8bits position?
        A("rjmp 6f")                      // No, we are done
        A("mov %16,%15")
        A("mov %15,%14")
        A("clr %14")
        A("jmp 6f")

        // idx < 8, now %3 = idx - 8. Get the count of bits
        L("7")
        A("neg %3")                       // %3 = -idx = count of bits to move right. idx range:[1...8]
        A("sbrs %3,0")                    // shift by 1 bit position ?
        A("rjmp 10f")                     // No, skip it
        A("asr %15")                      // (bit7 is always 0 here)
        A("ror %14")
        L("10")
        A("sbrs %3,1")                    // shift by 2 bit position ?
        A("rjmp 11f")                     // No, skip it
        A("asr %15")                      // (bit7 is always 0 here)
        A("ror %14")
        A("asr %15")                      // (bit7 is always 0 here)
        A("ror %14")
        L("11")
        A("sbrs %3,2")                    // shift by 4 bit position ?
        A("rjmp 12f")                     // No, skip it
        A("swap %15")                     // Swap nybbles
        A("andi %14, 0xF0")               // Lose the lowest nybble
        A("swap %14")                     // Swap nybbles. Upper nybble is 0
        A("or %14,%15")                   // Pass nybble from upper byte
        A("andi %15, 0x0F")               // And get rid of that nybble
        L("12")
        A("sbrs %3,3")                    // shift by 8 bit position ?
        A("rjmp 6f")                      // No, skip it
        A("mov %14,%15")
        A("clr %15")
        L("6")                            // %16:%15:%14 = initial estimation of 0x1000000 / d

        // Now, we must refine the estimation present on %16:%15:%14 using 1 iteration
        // of Newton-Raphson. As it has a quadratic convergence, 1 iteration is enough
        // to get more than 18bits of precision (the initial table lookup gives 9 bits of
        // precision to start from). 18bits of precision is all what is needed here for result

        // %8:%7:%6 = d = interval
        // %16:%15:%14 = x = initial estimation of 0x1000000 / d
        // %13 = 0
        // %3:%2:%1:%0 = working accumulator

        // Compute 1<<25 - x*d. Result should never exceed 25 bits and should always be positive
        A("clr %0")
        A("clr %1")
        A("clr %2")
        A("ldi %3,2")                     // %3:%2:%1:%0 = 0x2000000
        A("mul %6,%14")                   // r1:r0 = LO(d) * LO(x)
        A("sub %0,r0")
        A("sbc %1,r1")
        A("sbc %2,%13")
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= LO(d) * LO(x)
        A("mul %7,%14")                   // r1:r0 = MI(d) * LO(x)
        A("sub %1,r0")
        A("sbc %2,r1" )
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= MI(d) * LO(x) << 8
        A("mul %8,%14")                   // r1:r0 = HI(d) * LO(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= MIL(d) * LO(x) << 16
        A("mul %6,%15")                   // r1:r0 = LO(d) * MI(x)
        A("sub %1,r0")
        A("sbc %2,r1")
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= LO(d) * MI(x) << 8
        A("mul %7,%15")                   // r1:r0 = MI(d) * MI(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= MI(d) * MI(x) << 16
        A("mul %8,%15")                   // r1:r0 = HI(d) * MI(x)
        A("sub %3,r0")                    // %3:%2:%1:%0 -= MIL(d) * MI(x) << 24
        A("mul %6,%16")                   // r1:r0 = LO(d) * HI(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= LO(d) * HI(x) << 16
        A("mul %7,%16")                   // r1:r0 = MI(d) * HI(x)
        A("sub %3,r0")                    // %3:%2:%1:%0 -= MI(d) * HI(x) << 24
        // %3:%2:%1:%0 = (1<<25) - x*d     [169]

        // We need to multiply that result by x, and we are only interested in the top 24bits of that multiply

        // %16:%15:%14 = x = initial estimation of 0x1000000 / d
        // %3:%2:%1:%0 = (1<<25) - x*d = acc
        // %13 = 0

        // result = %11:%10:%9:%5:%4
        A("mul %14,%0")                   // r1:r0 = LO(x) * LO(acc)
        A("mov %4,r1")
        A("clr %5")
        A("clr %9")
        A("clr %10")
        A("clr %11")                      // %11:%10:%9:%5:%4 = LO(x) * LO(acc) >> 8
        A("mul %15,%0")                   // r1:r0 = MI(x) * LO(acc)
        A("add %4,r0")
        A("adc %5,r1")
        A("adc %9,%13")
        A("adc %10,%13")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 += MI(x) * LO(acc)
        A("mul %16,%0")                   // r1:r0 = HI(x) * LO(acc)
        A("add %5,r0")
        A("adc %9,r1")
        A("adc %10,%13")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 += MI(x) * LO(acc) << 8

        A("mul %14,%1")                   // r1:r0 = LO(x) * MIL(acc)
        A("add %4,r0")
        A("adc %5,r1")
        A("adc %9,%13")
        A("adc %10,%13")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 = LO(x) * MIL(acc)
        A("mul %15,%1")                   // r1:r0 = MI(x) * MIL(acc)
        A("add %5,r0")
        A("adc %9,r1")
        A("adc %10,%13")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 += MI(x) * MIL(acc) << 8
        A("mul %16,%1")                   // r1:r0 = HI(x) * MIL(acc)
        A("add %9,r0")
        A("adc %10,r1")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 += MI(x) * MIL(acc) << 16

        A("mul %14,%2")                   // r1:r0 = LO(x) * MIH(acc)
        A("add %5,r0")
        A("adc %9,r1")
        A("adc %10,%13")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 = LO(x) * MIH(acc) << 8
        A("mul %15,%2")                   // r1:r0 = MI(x) * MIH(acc)
        A("add %9,r0")
        A("adc %10,r1")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 += MI(x) * MIH(acc) << 16
        A("mul %16,%2")                   // r1:r0 = HI(x) * MIH(acc)
        A("add %10,r0")
        A("adc %11,r1")                   // %11:%10:%9:%5:%4 += MI(x) * MIH(acc) << 24

        A("mul %14,%3")                   // r1:r0 = LO(x) * HI(acc)
        A("add %9,r0")
        A("adc %10,r1")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 = LO(x) * HI(acc) << 16
        A("mul %15,%3")                   // r1:r0 = MI(x) * HI(acc)
        A("add %10,r0")
        A("adc %11,r1")                   // %11:%10:%9:%5:%4 += MI(x) * HI(acc) << 24
        A("mul %16,%3")                   // r1:r0 = HI(x) * HI(acc)
        A("add %11,r0")                   // %11:%10:%9:%5:%4 += MI(x) * HI(acc) << 32

        // At this point, %11:%10:%9 contains the new estimation of x.

        // Finally, we must correct the result. Estimate remainder as
        // (1<<24) - x*d
        // %11:%10:%9 = x
        // %8:%7:%6 = d = interval" "\n\t"
        A("ldi %3,1")
        A("clr %2")
        A("clr %1")
        A("clr %0")                       // %3:%2:%1:%0 = 0x1000000
        A("mul %6,%9")                    // r1:r0 = LO(d) * LO(x)
        A("sub %0,r0")
        A("sbc %1,r1")
        A("sbc %2,%13")
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= LO(d) * LO(x)
        A("mul %7,%9")                    // r1:r0 = MI(d) * LO(x)
        A("sub %1,r0")
        A("sbc %2,r1")
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= MI(d) * LO(x) << 8
        A("mul %8,%9")                    // r1:r0 = HI(d) * LO(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= MIL(d) * LO(x) << 16
        A("mul %6,%10")                   // r1:r0 = LO(d) * MI(x)
        A("sub %1,r0")
        A("sbc %2,r1")
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= LO(d) * MI(x) << 8
        A("mul %7,%10")                   // r1:r0 = MI(d) * MI(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= MI(d) * MI(x) << 16
        A("mul %8,%10")                   // r1:r0 = HI(d) * MI(x)
        A("sub %3,r0")                    // %3:%2:%1:%0 -= MIL(d) * MI(x) << 24
        A("mul %6,%11")                   // r1:r0 = LO(d) * HI(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= LO(d) * HI(x) << 16
        A("mul %7,%11")                   // r1:r0 = MI(d) * HI(x)
        A("sub %3,r0")                    // %3:%2:%1:%0 -= MI(d) * HI(x) << 24
        // %3:%2:%1:%0 = r = (1<<24) - x*d
        // %8:%7:%6 = d = interval

        // Perform the final correction
        A("sub %0,%6")
        A("sbc %1,%7")
        A("sbc %2,%8")                    // r -= d
        A("brcs 14f")                     // if ( r >= d)

        // %11:%10:%9 = x
        A("ldi %3,1")
        A("add %9,%3")
        A("adc %10,%13")
        A("adc %11,%13")                  // x++
        L("14")

        // Estimation is done. %11:%10:%9 = x
        A("clr __zero_reg__")              // Make C runtime happy
        // [211 cycles total]
        : "=r" (r2),
          "=r" (r3),
          "=r" (r4),
          "=d" (r5),
          "=r" (r6),
          "=r" (r7),
          "+r" (r8),
          "+r" (r9),
          "+r" (r10),
          "=d" (r11),
          "=r" (r12),
          "=r" (r13),
          "=d" (r14),
          "=d" (r15),
          "=d" (r16),
          "=d" (r17),
          "=d" (r18),
          "+z" (ptab)
        :
        : "r0", "r1", "cc"
      );

      // Return the result
      return r11 | (uint16_t(r12) << 8) | (uint32_t(r13) << 16);
    }
  #else
    // All other 32-bit MPUs can easily do inverse using hardware division,
    // so we don't need to reduce precision or to use assembly language at all.
    // This routine, for all other archs, returns 0x100000000 / d ~= 0xFFFFFFFF / d
    FORCE_INLINE static uint32_t get_period_inverse(const uint32_t d) {
      return d ? 0xFFFFFFFF / d : 0xFFFFFFFF;
    }
  #endif
#endif

/**
 * Get the current block for processing
 * and mark the block as busy.
 * Return nullptr if the buffer is empty
 * or if there is a first-block delay.
 *
 * WARNING: Called from Stepper ISR context!
 */
block_t* Planner::get_current_block() {
  // Get the number of moves in the planner queue so far
  const uint8_t nr_moves = movesplanned();

  // If there are any moves queued ...
  if (nr_moves) {

    // If there is still delay of delivery of blocks running, decrement it
    if (delay_before_delivering) {
      --delay_before_delivering;
      // If the number of movements queued is less than 3, and there is still time
      //  to wait, do not deliver anything
      if (nr_moves < 3 && delay_before_delivering) return nullptr;
      delay_before_delivering = 0;
    }

    // If we are here, there is no excuse to deliver the block
    block_t * const block = &block_buffer[block_buffer_tail];

    // No trapezoid calculated? Don't execute yet.
    if (block->flag.recalculate) return nullptr;

    // We can't be sure how long an active block will take, so don't count it.
    TERN_(HAS_WIRED_LCD, block_buffer_runtime_us -= block->segment_time_us);

    // As this block is busy, advance the nonbusy block pointer
    block_buffer_nonbusy = next_block_index(block_buffer_tail);

    // Return the block
    return block;
  }

  // The queue became empty
  TERN_(HAS_WIRED_LCD, clear_block_buffer_runtime()); // paranoia. Buffer is empty now - so reset accumulated time to zero.

  return nullptr;
}

/**
 * Calculate trapezoid parameters, multiplying the entry- and exit-speeds
 * by the provided factors. If entry_factor is 0 don't change the initial_rate.
 * Assumes that the implied initial_rate and final_rate are no less than
 * sqrt(block->acceleration_steps_per_s2 / 2). This is ensured through
 * minimum_planner_speed_sqr / min_entry_speed_sqr though note there's one
 * exception in recalculate_trapezoids().
 *
 * ############ VERY IMPORTANT ############
 * NOTE: The PRECONDITION to call this function is that the block is
 * NOT BUSY and it is marked as RECALCULATE. That WARRANTIES the Stepper ISR
 * is not and will not use the block while we modify it.
 */
void Planner::calculate_trapezoid_for_block(block_t * const block, const_float_t entry_speed, const_float_t exit_speed) {

  const float spmm = block->steps_per_mm;
  uint32_t initial_rate = entry_speed ? LROUND(entry_speed * spmm) : block->initial_rate,
           final_rate = LROUND(exit_speed * spmm);

  NOLESS(initial_rate,        stepper.minimal_step_rate);
  NOLESS(final_rate,          stepper.minimal_step_rate);
  NOLESS(block->nominal_rate, stepper.minimal_step_rate);

  #if ANY(S_CURVE_ACCELERATION, LIN_ADVANCE)
    // If we have some plateau time, the cruise rate will be the nominal rate
    uint32_t cruise_rate = block->nominal_rate;
  #endif

  // Steps for acceleration, plateau and deceleration
  int32_t plateau_steps = block->step_event_count,
          accelerate_steps = 0,
          decelerate_steps = 0;

  const int32_t accel = block->acceleration_steps_per_s2;
  float inverse_accel = 0.0f;
  if (accel != 0) {
    inverse_accel = 1.0f / accel;
    const float half_inverse_accel = 0.5f * inverse_accel,
                nominal_rate_sq = FLOAT_SQ(block->nominal_rate),
                // Steps required for acceleration, deceleration to/from nominal rate
                decelerate_steps_float = half_inverse_accel * (nominal_rate_sq - FLOAT_SQ(final_rate)),
                accelerate_steps_float = half_inverse_accel * (nominal_rate_sq - FLOAT_SQ(initial_rate));
    // Aims to fully reach nominal and final rates
    accelerate_steps = CEIL(accelerate_steps_float);
    decelerate_steps = CEIL(decelerate_steps_float);

    // Steps between acceleration and deceleration, if any
    plateau_steps -= accelerate_steps + decelerate_steps;

    // Does accelerate_steps + decelerate_steps exceed step_event_count?
    // Then we can't possibly reach the nominal rate, there will be no cruising.
    // Calculate accel / braking time in order to reach the final_rate exactly
    // at the end of this block.
    if (plateau_steps < 0) {
      accelerate_steps = LROUND((block->step_event_count + accelerate_steps_float - decelerate_steps_float) * 0.5f);
      LIMIT(accelerate_steps, 0, int32_t(block->step_event_count));
      decelerate_steps = block->step_event_count - accelerate_steps;

      #if ANY(S_CURVE_ACCELERATION, LIN_ADVANCE)
        // We won't reach the cruising rate. Let's calculate the speed we will reach
        NOMORE(cruise_rate, final_speed(initial_rate, accel, accelerate_steps));
      #endif
    }
  }

  #if ENABLED(S_CURVE_ACCELERATION)
    const float rate_factor = inverse_accel * (STEPPER_TIMER_RATE);
    // Jerk controlled speed requires to express speed versus time, NOT steps
    uint32_t acceleration_time = rate_factor * float(cruise_rate - initial_rate),
             deceleration_time = rate_factor * float(cruise_rate - final_rate),
    // And to offload calculations from the ISR, we also calculate the inverse of those times here
             acceleration_time_inverse = get_period_inverse(acceleration_time),
             deceleration_time_inverse = get_period_inverse(deceleration_time);
  #endif

  // Store new block parameters
  block->accelerate_before = accelerate_steps;
  block->decelerate_start = block->step_event_count - decelerate_steps;
  block->initial_rate = initial_rate;
  #if ENABLED(S_CURVE_ACCELERATION)
    block->acceleration_time = acceleration_time;
    block->deceleration_time = deceleration_time;
    block->acceleration_time_inverse = acceleration_time_inverse;
    block->deceleration_time_inverse = deceleration_time_inverse;
    block->cruise_rate = cruise_rate;
  #endif
  block->final_rate = final_rate;

  #if ENABLED(LIN_ADVANCE)
    if (block->la_advance_rate) {
      const float comp = extruder_advance_K[E_INDEX_N(block->extruder)] * block->steps.e / block->step_event_count;
      block->max_adv_steps = cruise_rate * comp;
      block->final_adv_steps = final_rate * comp;
    }
  #endif

  #if ENABLED(LASER_POWER_TRAP)
    /**
     * Laser Trapezoid Calculations
     *
     * Approximate the trapezoid with the laser, incrementing the power every `trap_ramp_entry_incr`
     * steps while accelerating, and decrementing the power every `trap_ramp_exit_decr` while decelerating,
     * to keep power proportional to feedrate. Laser power trap will reduce the initial power to no less
     * than the laser_power_floor value. Based on the number of calculated accel/decel steps the power is
     * distributed over the trapezoid entry- and exit-ramp steps.
     *
     * trap_ramp_active_pwr - The active power is initially set at a reduced level factor of initial
     * power / accel steps and will be additively incremented using a trap_ramp_entry_incr value for each
     * accel step processed later in the stepper code. The trap_ramp_exit_decr value is calculated as
     * power / decel steps and is also adjusted to no less than the power floor.
     *
     * If the power == 0 the inline mode variables need to be set to zero to prevent stepper processing.
     * The method allows for simpler non-powered moves like G0 or G28.
     *
     * Laser Trap Power works for all Jerk and Curve modes; however Arc-based moves will have issues since
     * the segments are usually too small.
     */
    if (cutter.cutter_mode == CUTTER_MODE_CONTINUOUS) {
      if (planner.laser_inline.status.isPowered && planner.laser_inline.status.isEnabled) {
        if (block->laser.power > 0) {
          NOLESS(block->laser.power, laser_power_floor);
          block->laser.trap_ramp_active_pwr = (block->laser.power - laser_power_floor) * (initial_rate / float(block->nominal_rate)) + laser_power_floor;
          block->laser.trap_ramp_entry_incr = (block->laser.power - block->laser.trap_ramp_active_pwr) / accelerate_steps;
          float laser_pwr = block->laser.power * (final_rate / float(block->nominal_rate));
          NOLESS(laser_pwr, laser_power_floor);
          block->laser.trap_ramp_exit_decr = (block->laser.power - laser_pwr) / decelerate_steps;
          #if ENABLED(DEBUG_LASER_TRAP)
            SERIAL_ECHO_MSG("lp:",block->laser.power);
            SERIAL_ECHO_MSG("as:",accelerate_steps);
            SERIAL_ECHO_MSG("ds:",decelerate_steps);
            SERIAL_ECHO_MSG("p.trap:",block->laser.trap_ramp_active_pwr);
            SERIAL_ECHO_MSG("p.incr:",block->laser.trap_ramp_entry_incr);
            SERIAL_ECHO_MSG("p.decr:",block->laser.trap_ramp_exit_decr);
          #endif
        }
        else {
          block->laser.trap_ramp_active_pwr = 0;
          block->laser.trap_ramp_entry_incr = 0;
          block->laser.trap_ramp_exit_decr = 0;
        }

      }
    }
  #endif // LASER_POWER_TRAP
}

/**
 *                              PLANNER SPEED DEFINITION
 *                                     +--------+   <- current->nominal_speed
 *                                    /          \
 *         current->entry_speed ->   +            \
 *                                   |             + <- next->entry_speed (aka exit speed)
 *                                   +-------------+
 *                                       time -->
 *
 *  Recalculates the motion plan according to the following basic guidelines:
 *
 *    1. Go over blocks sequentially in reverse order and maximize the entry junction speed:
 *      a. Entry speed should stay below/at the pre-computed maximum junction speed limit
 *      b. Aim for the maximum entry speed which is the one reverse-computed from its exit speed
 *         (next->entry_speed) if assuming maximum deceleration over the full block travel distance
 *      c. The last (newest appended) block uses safe_exit_speed exit speed (there's no 'next')
 *    2. Go over blocks in chronological (forward) order and fix the exit junction speed:
 *      a. Exit speed (next->entry_speed) must be below/at the maximum exit speed forward-computed
 *         from its entry speed if assuming maximum acceleration over the full block travel distance
 *      b. Exit speed should stay above/at the pre-computed minimum junction speed limit
 *    3. Convert entry / exit speeds (mm/s) into final/initial steps/s
 *
 *  When these stages are complete, the planner will have maximized the velocity profiles throughout the all
 *  of the planner blocks, where every block is operating at its maximum allowable acceleration limits. In
 *  other words, for all of the blocks in the planner, the plan is optimal and no further speed improvements
 *  are possible. If a new block is added to the buffer, the plan is recomputed according to the said
 *  guidelines for a new optimal plan.
 *
 *  To increase computational efficiency of these guidelines:
 *    1. We keep track of which blocks need calculation (block->flag.recalculate)
 *    2. We stop the reverse pass on the first block whose entry_speed == max_entry_speed. As soon
 *       as that happens, there can be no further increases (ensured by the previous recalculate)
 *    3. On the forward pass we skip through to the first block with a modified exit speed
 *       (next->entry_speed)
 *    4. On the forward pass if we encounter a full acceleration block that limits its exit speed
 *       (next->entry_speed) we also update the maximum for that junction (next->max_entry_speed)
 *       so it's never updated again
 *    5. We use speed squared (ex: entry_speed_sqr in mm^2/s^2) in acceleration limit computations
 *    6. We don't recompute sqrt(entry_speed_sqr) if the block's entry speed didn't change
 *
 *  Planner buffer index mapping:
 *  - block_buffer_tail: Points to the beginning of the planner buffer. First to be executed or being executed.
 *  - block_buffer_head: Points to the buffer block after the last block in the buffer. Used to indicate whether
 *      the buffer is full or empty. As described for standard ring buffers, this block is always empty.
 *
 *  NOTE: Since the planner only computes on what's in the planner buffer, some motions with many short
 *        segments (e.g., complex curves) may seem to move slowly. This is because there simply isn't
 *        enough combined distance traveled in the entire buffer to accelerate up to the nominal speed and
 *        then decelerate to a complete stop at the end of the buffer, as stated by the guidelines. If this
 *        happens and becomes an annoyance, there are a few simple solutions:
 *
 *    - Maximize the machine acceleration. The planner will be able to compute higher velocity profiles
 *      within the same combined distance.
 *
 *    - Maximize line motion(s) distance per block to a desired tolerance. The more combined distance the
 *      planner has to use, the faster it can go.
 *
 *    - Maximize the planner buffer size. This also will increase the combined distance for the planner to
 *      compute over. It also increases the number of computations the planner has to perform to compute an
 *      optimal plan, so select carefully.
 *
 *    - Use G2/G3 arcs instead of many short segments. Arcs inform the planner of a safe exit speed at the
 *      end of the last segment, which alleviates this problem.
 */

// The kernel called by recalculate() when scanning the plan from last to first entry.
// Returns true if it could increase the current block's entry speed.
bool Planner::reverse_pass_kernel(block_t * const current, const block_t * const next, const_float_t safe_exit_speed_sqr) {
  // We need to recalculate only for the last block added or if next->entry_speed_sqr changed.
  if (!next || next->flag.recalculate) {
    // And only if we're not already at max entry speed.
    if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
      const float next_entry_speed_sqr = next ? next->entry_speed_sqr : safe_exit_speed_sqr;
      float new_entry_speed_sqr = max_allowable_speed_sqr(-current->acceleration, next_entry_speed_sqr, current->millimeters);
      NOMORE(new_entry_speed_sqr, current->max_entry_speed_sqr);
      if (current->entry_speed_sqr != new_entry_speed_sqr) {

        // Need to recalculate the block speed - Mark it now, so the stepper
        // ISR does not consume the block before being recalculated
        current->flag.recalculate = true;

        // But there is an inherent race condition here, as the block may have
        // become BUSY just before being marked RECALCULATE, so check for that!
        if (stepper.is_block_busy(current)) {
          // Block became busy. Clear the RECALCULATE flag (no point in
          // recalculating BUSY blocks).
          current->flag.recalculate = false;
        }
        else {
          // Block is not BUSY so this is ahead of the Stepper ISR:

          current->entry_speed_sqr = new_entry_speed_sqr;
          return true;
        }
      }
    }
  }
  return false;
}

/**
 * recalculate() needs to go over the current plan twice.
 * Once in reverse and once forward. This implements the reverse pass that
 * coarsely maximizes the entry speeds starting from last block.
 * Requires there's at least one block with flag.recalculate in the buffer.
 */
void Planner::reverse_pass(const_float_t safe_exit_speed_sqr) {
  // Initialize block index to the last block in the planner buffer.
  // This last block will have flag.recalculate set.
  uint8_t block_index = prev_block_index(block_buffer_head);

  // The ISR may change block_buffer_nonbusy so get a stable local copy.
  uint8_t nonbusy_block_index = block_buffer_nonbusy;

  const block_t *next = nullptr;
  // Don't try to change the entry speed of the first non-busy block.
  while (block_index != nonbusy_block_index) {
    block_t *current = &block_buffer[block_index];

    // Only process movement blocks
    if (current->is_move()) {
      // If no entry speed increase was possible we end the reverse pass.
      if (!reverse_pass_kernel(current, next, safe_exit_speed_sqr)) return;
      next = current;
    }

    block_index = prev_block_index(block_index);

    // The ISR could advance block_buffer_nonbusy while we were doing the reverse pass.
    // We must try to avoid using an already consumed block as the last one - So follow
    // changes to the pointer and make sure to limit the loop to the currently busy block
    while (nonbusy_block_index != block_buffer_nonbusy) {

      // If we reached the busy block or an already processed block, break the loop now
      if (block_index == nonbusy_block_index) return;

      // Advance the pointer, following the busy block
      nonbusy_block_index = next_block_index(nonbusy_block_index);
    }
  }
}

// The kernel called during the forward pass. Assumes current->flag.recalculate.
void Planner::forward_pass_kernel(const block_t * const previous, block_t * const current) {
  // Check if the previous block is accelerating.
  if (previous->entry_speed_sqr < current->entry_speed_sqr) {
    // Compute the maximum achievable speed if the previous block was fully accelerating.
    float new_exit_speed_sqr = max_allowable_speed_sqr(-previous->acceleration, previous->entry_speed_sqr, previous->millimeters);

    if (new_exit_speed_sqr < current->entry_speed_sqr) {
      // Current entry speed limited by full acceleration from previous entry speed.

      // Make sure entry speed not lower than minimum_planner_speed_sqr.
      NOLESS(new_exit_speed_sqr, current->min_entry_speed_sqr);
      current->entry_speed_sqr = new_exit_speed_sqr;
      // Ensure we don't try updating entry_speed_sqr again.
      current->max_entry_speed_sqr = new_exit_speed_sqr;
    }
  }

  // The fully optimized entry speed is our new minimum speed.
  current->min_entry_speed_sqr = current->entry_speed_sqr;
}

/**
 * Do the forward pass and recalculate the trapezoid speed profiles for all blocks in the plan
 * according to entry/exit speeds.
 */
void Planner::recalculate_trapezoids(const_float_t safe_exit_speed_sqr) {
  // Start with the block that's about to execute or is executing.
  uint8_t block_index = block_buffer_tail,
          head_block_index = block_buffer_head;

  block_t *block = nullptr, *next = nullptr;
  float next_entry_speed = 0.0f;
  while (block_index != head_block_index) {

    next = &block_buffer[block_index];

    if (next->is_move()) {
      // Check if the next block's entry speed changed
      if (next->flag.recalculate) {
        if (!block) {
          // 'next' is the first move due to either being the first added move or due to the planner
          // having completely fallen behind. Revert any reverse pass change.
          next->entry_speed_sqr = next->min_entry_speed_sqr;
          next_entry_speed = SQRT(next->min_entry_speed_sqr);
        }
        else {
          // Try to fix exit speed which requires trapezoid recalculation
          block->flag.recalculate = true;

          // But there is an inherent race condition here, as the block may have
          // become BUSY just before being marked RECALCULATE, so check for that!
          if (stepper.is_block_busy(block)) {
            // Block is BUSY so we can't change the exit speed. Revert any reverse pass change.
            next->entry_speed_sqr = next->min_entry_speed_sqr;
            if (!next->initial_rate) {
              // 'next' was never calculated. Planner is falling behind so for maximum efficiency
              // set next's stepping speed directly and forgo checking against min_entry_speed_sqr.
              // calculate_trapezoid_for_block() can handle it, albeit sub-optimally.
              next->initial_rate = block->final_rate;
            }
            // Note that at this point next_entry_speed is (still) 0.
          }
          else {
            // Block is not BUSY: we won the race against the ISR or recalculate was already set

            if (next->entry_speed_sqr != next->min_entry_speed_sqr)
              forward_pass_kernel(block, next);

            const float current_entry_speed = next_entry_speed;
            next_entry_speed = SQRT(next->entry_speed_sqr);

            calculate_trapezoid_for_block(block, current_entry_speed, next_entry_speed);
          }

          // Reset current only to ensure next trapezoid is computed - The
          // stepper is free to use the block from now on.
          block->flag.recalculate = false;
        }
      }

      block = next;
    }

    block_index = next_block_index(block_index);
  }

  // Last/newest block in buffer. The above guarantees it's a move block.
  if (block && block->flag.recalculate) {
    const float current_entry_speed = next_entry_speed;
    next_entry_speed = SQRT(safe_exit_speed_sqr);

    calculate_trapezoid_for_block(block, current_entry_speed, next_entry_speed);

    // Reset block to ensure its trapezoid is computed - The stepper is free to use
    // the block from now on.
    block->flag.recalculate = false;
  }
}

// Requires there's at least one block with flag.recalculate in the buffer
void Planner::recalculate(const_float_t safe_exit_speed_sqr) {
  reverse_pass(safe_exit_speed_sqr);
  // The forward pass is done as part of recalculate_trapezoids()
  recalculate_trapezoids(safe_exit_speed_sqr);
}

/**
 * Apply fan speeds
 */
#if HAS_FAN

  void Planner::sync_fan_speeds(uint8_t (&fan_speed)[FAN_COUNT]) {

    #if ENABLED(FAN_SOFT_PWM)
      #define _FAN_SET(F) thermalManager.soft_pwm_amount_fan[F] = CALC_FAN_SPEED(fan_speed[F]);
    #else
      #define _FAN_SET(F) hal.set_pwm_duty(pin_t(FAN##F##_PIN), CALC_FAN_SPEED(fan_speed[F]));
    #endif
    #define FAN_SET(F) do{ kickstart_fan(fan_speed, ms, F); _FAN_SET(F); }while(0)

    const millis_t ms = millis();
    TERN_(HAS_FAN0, FAN_SET(0)); TERN_(HAS_FAN1, FAN_SET(1));
    TERN_(HAS_FAN2, FAN_SET(2)); TERN_(HAS_FAN3, FAN_SET(3));
    TERN_(HAS_FAN4, FAN_SET(4)); TERN_(HAS_FAN5, FAN_SET(5));
    TERN_(HAS_FAN6, FAN_SET(6)); TERN_(HAS_FAN7, FAN_SET(7));
  }

  #if FAN_KICKSTART_TIME

    void Planner::kickstart_fan(uint8_t (&fan_speed)[FAN_COUNT], const millis_t &ms, const uint8_t f) {
      static millis_t fan_kick_end[FAN_COUNT] = { 0 };
      #if ENABLED(FAN_KICKSTART_LINEAR)
        static uint8_t set_fan_speed[FAN_COUNT] = { 0 };
      #endif
      if (fan_speed[f] > FAN_OFF_PWM) {
        const bool first_kick = fan_kick_end[f] == 0 && TERN1(FAN_KICKSTART_LINEAR, fan_speed[f] > set_fan_speed[f]);
        if (first_kick)
          fan_kick_end[f] = ms + (FAN_KICKSTART_TIME) TERN_(FAN_KICKSTART_LINEAR, * (fan_speed[f] - set_fan_speed[f]) / 255);
        if (first_kick || PENDING(ms, fan_kick_end[f])) {
          fan_speed[f] = FAN_KICKSTART_POWER;
          return;
        }
      }
      fan_kick_end[f] = 0;
      TERN_(FAN_KICKSTART_LINEAR, set_fan_speed[f] = fan_speed[f]);
    }

  #endif

#endif // HAS_FAN

/**
 * Maintain fans, paste extruder pressure, spindle/laser power
 */
void Planner::check_axes_activity() {

  #if HAS_DISABLE_AXES
    xyze_bool_t axis_active = { false };
  #endif

  #if HAS_FAN && DISABLED(LASER_SYNCHRONOUS_M106_M107)
    #define HAS_TAIL_FAN_SPEED 1
    static uint8_t tail_fan_speed[FAN_COUNT] = ARRAY_N_1(FAN_COUNT, 13);
    bool fans_need_update = false;
  #endif

  #if ENABLED(BARICUDA)
    #if HAS_HEATER_1
      uint8_t tail_valve_pressure;
    #endif
    #if HAS_HEATER_2
      uint8_t tail_e_to_p_pressure;
    #endif
  #endif

  if (has_blocks_queued()) {

    #if ANY(HAS_TAIL_FAN_SPEED, BARICUDA)
      block_t *block = &block_buffer[block_buffer_tail];
    #endif

    #if HAS_TAIL_FAN_SPEED
      FANS_LOOP(i) {
        const uint8_t spd = thermalManager.scaledFanSpeed(i, block->fan_speed[i]);
        if (tail_fan_speed[i] != spd) {
          fans_need_update = true;
          tail_fan_speed[i] = spd;
        }
      }
    #endif

    #if ENABLED(BARICUDA)
      TERN_(HAS_HEATER_1, tail_valve_pressure = block->valve_pressure);
      TERN_(HAS_HEATER_2, tail_e_to_p_pressure = block->e_to_p_pressure);
    #endif

    #if HAS_DISABLE_AXES
      for (uint8_t b = block_buffer_tail; b != block_buffer_head; b = next_block_index(b)) {
        block_t * const bnext = &block_buffer[b];
        LOGICAL_AXIS_CODE(
          if (TERN0(DISABLE_E, bnext->steps.e)) axis_active.e = true,
          if (TERN0(DISABLE_X, bnext->steps.x)) axis_active.x = true,
          if (TERN0(DISABLE_Y, bnext->steps.y)) axis_active.y = true,
          if (TERN0(DISABLE_Z, bnext->steps.z)) axis_active.z = true,
          if (TERN0(DISABLE_I, bnext->steps.i)) axis_active.i = true,
          if (TERN0(DISABLE_J, bnext->steps.j)) axis_active.j = true,
          if (TERN0(DISABLE_K, bnext->steps.k)) axis_active.k = true,
          if (TERN0(DISABLE_U, bnext->steps.u)) axis_active.u = true,
          if (TERN0(DISABLE_V, bnext->steps.v)) axis_active.v = true,
          if (TERN0(DISABLE_W, bnext->steps.w)) axis_active.w = true
        );
      }
    #endif
  }
  else {

    TERN_(HAS_CUTTER, if (cutter.cutter_mode == CUTTER_MODE_STANDARD) cutter.refresh());

    #if HAS_TAIL_FAN_SPEED
      FANS_LOOP(i) {
        const uint8_t spd = thermalManager.scaledFanSpeed(i);
        if (tail_fan_speed[i] != spd) {
          fans_need_update = true;
          tail_fan_speed[i] = spd;
        }
      }
    #endif

    #if ENABLED(BARICUDA)
      TERN_(HAS_HEATER_1, tail_valve_pressure = baricuda_valve_pressure);
      TERN_(HAS_HEATER_2, tail_e_to_p_pressure = baricuda_e_to_p_pressure);
    #endif
  }

  //
  // Disable inactive axes
  //
  #if HAS_DISABLE_AXES
    LOGICAL_AXIS_CODE(
      if (TERN0(DISABLE_E, !axis_active.e)) stepper.disable_e_steppers(),
      if (TERN0(DISABLE_X, !axis_active.x)) stepper.disable_axis(X_AXIS),
      if (TERN0(DISABLE_Y, !axis_active.y)) stepper.disable_axis(Y_AXIS),
      if (TERN0(DISABLE_Z, !axis_active.z)) stepper.disable_axis(Z_AXIS),
      if (TERN0(DISABLE_I, !axis_active.i)) stepper.disable_axis(I_AXIS),
      if (TERN0(DISABLE_J, !axis_active.j)) stepper.disable_axis(J_AXIS),
      if (TERN0(DISABLE_K, !axis_active.k)) stepper.disable_axis(K_AXIS),
      if (TERN0(DISABLE_U, !axis_active.u)) stepper.disable_axis(U_AXIS),
      if (TERN0(DISABLE_V, !axis_active.v)) stepper.disable_axis(V_AXIS),
      if (TERN0(DISABLE_W, !axis_active.w)) stepper.disable_axis(W_AXIS)
    );
  #endif

  //
  // Update Fan speeds
  // Only if synchronous M106/M107 is disabled
  //
  TERN_(HAS_TAIL_FAN_SPEED, if (fans_need_update) sync_fan_speeds(tail_fan_speed));

  TERN_(AUTOTEMP, autotemp_task());

  #if ENABLED(BARICUDA)
    TERN_(HAS_HEATER_1, hal.set_pwm_duty(pin_t(HEATER_1_PIN), tail_valve_pressure));
    TERN_(HAS_HEATER_2, hal.set_pwm_duty(pin_t(HEATER_2_PIN), tail_e_to_p_pressure));
  #endif
}

#if ENABLED(AUTOTEMP)

  #if ENABLED(AUTOTEMP_PROPORTIONAL)
    void Planner::_autotemp_update_from_hotend() {
      const celsius_t target = thermalManager.degTargetHotend(active_extruder);
      autotemp.min = target + AUTOTEMP_MIN_P;
      autotemp.max = target + AUTOTEMP_MAX_P;
    }
  #endif

  /**
   * Called after changing tools to:
   *  - Reset or re-apply the default proportional autotemp factor.
   *  - Enable autotemp if the factor is non-zero.
   */
  void Planner::autotemp_update() {
    _autotemp_update_from_hotend();
    autotemp.factor = TERN0(AUTOTEMP_PROPORTIONAL, AUTOTEMP_FACTOR_P);
    autotemp.enabled = autotemp.factor != 0;
  }

  /**
   * Called by the M104/M109 commands after setting Hotend Temperature
   *
   */
  void Planner::autotemp_M104_M109() {
    _autotemp_update_from_hotend();

    if (parser.seenval('S')) autotemp.min = parser.value_celsius();
    if (parser.seenval('B')) autotemp.max = parser.value_celsius();

    // When AUTOTEMP_PROPORTIONAL is enabled, F0 disables autotemp.
    // Normally, leaving off F also disables autotemp.
    autotemp.factor = parser.seen('F') ? parser.value_float() : TERN0(AUTOTEMP_PROPORTIONAL, AUTOTEMP_FACTOR_P);
    autotemp.enabled = autotemp.factor != 0;
  }

  /**
   * Called every so often to adjust the hotend target temperature
   * based on the extrusion speed, which is calculated from the blocks
   * currently in the planner.
   */
  void Planner::autotemp_task() {
    static float oldt = 0.0f;

    if (!autotemp.enabled) return;
    if (thermalManager.degTargetHotend(active_extruder) < autotemp.min - 2) return; // Below the min?

    float high = 0.0f;
    for (uint8_t b = block_buffer_tail; b != block_buffer_head; b = next_block_index(b)) {
      const block_t * const block = &block_buffer[b];
      if (NUM_AXIS_GANG(block->steps.x, || block->steps.y, || block->steps.z, || block->steps.i, || block->steps.j, || block->steps.k, || block->steps.u, || block->steps.v, || block->steps.w)) {
        const float se = float(block->steps.e) / block->step_event_count * block->nominal_speed; // mm/sec
        NOLESS(high, se);
      }
    }

    float t = autotemp.min + high * autotemp.factor;
    LIMIT(t, autotemp.min, autotemp.max);
    if (t < oldt) t = t * (1.0f - (AUTOTEMP_OLDWEIGHT)) + oldt * (AUTOTEMP_OLDWEIGHT);
    oldt = t;
    thermalManager.setTargetHotend(t, active_extruder);
  }

#endif // AUTOTEMP

#if DISABLED(NO_VOLUMETRICS)

  /**
   * Get a volumetric multiplier from a filament diameter.
   * This is the reciprocal of the circular cross-section area.
   * Return 1.0 with volumetric off or a diameter of 0.0.
   */
  inline float calculate_volumetric_multiplier(const_float_t diameter) {
    return (parser.volumetric_enabled && diameter) ? 1.0f / CIRCLE_AREA(diameter * 0.5f) : 1;
  }

  /**
   * Convert the filament sizes into volumetric multipliers.
   * The multiplier converts a given E value into a length.
   */
  void Planner::calculate_volumetric_multipliers() {
    for (uint8_t i = 0; i < COUNT(filament_size); ++i) {
      volumetric_multiplier[i] = calculate_volumetric_multiplier(filament_size[i]);
      refresh_e_factor(i);
    }
    #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
      calculate_volumetric_extruder_limits(); // update volumetric_extruder_limits as well.
    #endif
  }

#endif // !NO_VOLUMETRICS

#if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)

  /**
   * Convert volumetric based limits into pre calculated extruder feedrate limits.
   */
  void Planner::calculate_volumetric_extruder_limit(const uint8_t e) {
    const float &lim = volumetric_extruder_limit[e], &siz = filament_size[e];
    volumetric_extruder_feedrate_limit[e] = (lim && siz) ? lim / CIRCLE_AREA(siz * 0.5f) : 0;
  }
  void Planner::calculate_volumetric_extruder_limits() {
    EXTRUDER_LOOP() calculate_volumetric_extruder_limit(e);
  }

#endif

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  /**
   * Convert the ratio value given by the filament width sensor
   * into a volumetric multiplier. Conversion differs when using
   * linear extrusion vs volumetric extrusion.
   */
  void Planner::apply_filament_width_sensor(const int8_t encoded_ratio) {
    // Reconstitute the nominal/measured ratio
    const float nom_meas_ratio = 1 + 0.01f * encoded_ratio,
                ratio_2 = sq(nom_meas_ratio);

    volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM] = parser.volumetric_enabled
      ? ratio_2 / CIRCLE_AREA(filwidth.nominal_mm * 0.5f) // Volumetric uses a true volumetric multiplier
      : ratio_2;                                          // Linear squares the ratio, which scales the volume

    refresh_e_factor(FILAMENT_SENSOR_EXTRUDER_NUM);
  }
#endif

#if ENABLED(IMPROVE_HOMING_RELIABILITY)

  void Planner::enable_stall_prevention(const bool onoff) {
    static motion_state_t saved_motion_state;
    if (onoff) {
      saved_motion_state.acceleration.x = settings.max_acceleration_mm_per_s2[X_AXIS];
      saved_motion_state.acceleration.y = settings.max_acceleration_mm_per_s2[Y_AXIS];
      settings.max_acceleration_mm_per_s2[X_AXIS] = settings.max_acceleration_mm_per_s2[Y_AXIS] = 100;
      #if ENABLED(DELTA)
        saved_motion_state.acceleration.z = settings.max_acceleration_mm_per_s2[Z_AXIS];
        settings.max_acceleration_mm_per_s2[Z_AXIS] = 100;
      #endif
      #if ENABLED(CLASSIC_JERK)
        saved_motion_state.jerk_state = max_jerk;
        max_jerk.set(0, 0 OPTARG(DELTA, 0));
      #endif
    }
    else {
      settings.max_acceleration_mm_per_s2[X_AXIS] = saved_motion_state.acceleration.x;
      settings.max_acceleration_mm_per_s2[Y_AXIS] = saved_motion_state.acceleration.y;
      TERN_(DELTA, settings.max_acceleration_mm_per_s2[Z_AXIS] = saved_motion_state.acceleration.z);
      TERN_(CLASSIC_JERK, max_jerk = saved_motion_state.jerk_state);
    }
    refresh_acceleration_rates();
  }

#endif

#if HAS_LEVELING

  constexpr xy_pos_t level_fulcrum = {
    TERN(Z_SAFE_HOMING, Z_SAFE_HOMING_X_POINT, X_HOME_POS),
    TERN(Z_SAFE_HOMING, Z_SAFE_HOMING_Y_POINT, Y_HOME_POS)
  };

  /**
   * rx, ry, rz - Cartesian positions in mm
   *              Leveled XYZ on completion
   */
  void Planner::apply_leveling(xyz_pos_t &raw) {
    if (!leveling_active) return;

    #if ABL_PLANAR

      xy_pos_t d = raw - level_fulcrum;
      bed_level_matrix.apply_rotation_xyz(d.x, d.y, raw.z);
      raw = d + level_fulcrum;

    #elif HAS_MESH

      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        const float fade_scaling_factor = fade_scaling_factor_for_z(raw.z);
        if (fade_scaling_factor) raw.z += fade_scaling_factor * bedlevel.get_z_correction(raw);
      #else
        raw.z += bedlevel.get_z_correction(raw);
      #endif

      TERN_(MESH_BED_LEVELING, raw.z += bedlevel.get_z_offset());

    #endif
  }

  void Planner::unapply_leveling(xyz_pos_t &raw) {
    if (!leveling_active) return;

    #if ABL_PLANAR

      matrix_3x3 inverse = matrix_3x3::transpose(bed_level_matrix);

      xy_pos_t d = raw - level_fulcrum;
      inverse.apply_rotation_xyz(d.x, d.y, raw.z);
      raw = d + level_fulcrum;

    #elif HAS_MESH

      const float z_correction = bedlevel.get_z_correction(raw),
                  z_full_fade = DIFF_TERN(MESH_BED_LEVELING, raw.z, bedlevel.get_z_offset()),
                  z_no_fade = z_full_fade - z_correction;

      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        if (!z_fade_height || z_no_fade <= 0.0f)                              // Not fading or at bed level?
          raw.z = z_no_fade;                                                  //  Unapply full mesh Z.
        else if (z_full_fade >= z_fade_height)                                // Above the fade height?
          raw.z = z_full_fade;                                                //  Nothing more to unapply.
        else                                                                  // Within the fade zone?
          raw.z = z_no_fade / (1.0f - z_correction * inverse_z_fade_height);  //  Unapply the faded Z offset
      #else
        raw.z = z_no_fade;
      #endif

    #endif
  }

#endif // HAS_LEVELING

#if ENABLED(FWRETRACT)
  /**
   * rz, e - Cartesian positions in mm
   */
  void Planner::apply_retract(float &rz, float &e) {
    rz += fwretract.current_hop;
    e -= fwretract.current_retract[active_extruder];
  }

  void Planner::unapply_retract(float &rz, float &e) {
    rz -= fwretract.current_hop;
    e += fwretract.current_retract[active_extruder];
  }

#endif

void Planner::quick_stop() {

  /**
   * Remove all the queued blocks.
   * NOTE: This function is NOT called from the Stepper ISR,
   * so we must consider tail as readonly!
   * That is why we set head to tail - But there is a race condition that
   * must be handled: The tail could change between the read and the assignment
   * so this must be enclosed in a critical section
   */

  const bool was_enabled = stepper.suspend();

  // Drop all queue entries
  block_buffer_nonbusy = block_buffer_head = block_buffer_tail;

  // Restart the block delay for the first movement - As the queue was
  // forced to empty, there's no risk the ISR will touch this.

  delay_before_delivering = TERN_(FT_MOTION, ftMotion.cfg.active ? BLOCK_DELAY_NONE :) BLOCK_DELAY_FOR_1ST_MOVE;

  TERN_(HAS_WIRED_LCD, clear_block_buffer_runtime()); // Clear the accumulated runtime

  // Make sure to drop any attempt of queuing moves for 1 second
  cleaning_buffer_counter = TEMP_TIMER_FREQUENCY;

  // Reenable Stepper ISR
  if (was_enabled) stepper.wake_up();

  // And stop the stepper ISR
  stepper.quick_stop();
}

#if ENABLED(REALTIME_REPORTING_COMMANDS)

  void Planner::quick_pause() {
    // Suspend until quick_resume is called
    // Don't empty buffers or queues
    const bool did_suspend = stepper.suspend();
    if (did_suspend)
      TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_HOLD));
  }

  // Resume if suspended
  void Planner::quick_resume() {
    TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(grbl_state_for_marlin_state()));
    stepper.wake_up();
  }

#endif

void Planner::endstop_triggered(const AxisEnum axis) {
  // Record stepper position and discard the current block
  stepper.endstop_triggered(axis);
}

float Planner::triggered_position_mm(const AxisEnum axis) {
  const float result = DIFF_TERN(BACKLASH_COMPENSATION, stepper.triggered_position(axis), backlash.get_applied_steps(axis));
  return result * mm_per_step[axis];
}

bool Planner::busy() {
  return (has_blocks_queued() || cleaning_buffer_counter
      || TERN0(EXTERNAL_CLOSED_LOOP_CONTROLLER, CLOSED_LOOP_WAITING())
      || TERN0(HAS_ZV_SHAPING, stepper.input_shaping_busy())
      || TERN0(FT_MOTION, ftMotion.busy)
  );
}

void Planner::finish_and_disable() {
  while (has_blocks_queued() || cleaning_buffer_counter) idle();
  stepper.disable_all_steppers();
}

/**
 * Get an axis position according to stepper position(s)
 * For CORE machines apply translation from ABC to XYZ.
 */
float Planner::get_axis_position_mm(const AxisEnum axis) {
  float axis_steps;
  #if IS_CORE

    // Requesting one of the "core" axes?
    if (axis == CORE_AXIS_1 || axis == CORE_AXIS_2) {

      // Protect the access to the position.
      const bool was_enabled = stepper.suspend();

      const int32_t p1 = DIFF_TERN(BACKLASH_COMPENSATION, stepper.position(CORE_AXIS_1), backlash.get_applied_steps(CORE_AXIS_1)),
                    p2 = DIFF_TERN(BACKLASH_COMPENSATION, stepper.position(CORE_AXIS_2), backlash.get_applied_steps(CORE_AXIS_2));

      if (was_enabled) stepper.wake_up();

      // ((a1+a2)+(a1-a2))/2 -> (a1+a2+a1-a2)/2 -> (a1+a1)/2 -> a1
      // ((a1+a2)-(a1-a2))/2 -> (a1+a2-a1+a2)/2 -> (a2+a2)/2 -> a2
      axis_steps = (axis == CORE_AXIS_2 ? CORESIGN(p1 - p2) : p1 + p2) * 0.5f;
    }
    else
      axis_steps = DIFF_TERN(BACKLASH_COMPENSATION, stepper.position(axis), backlash.get_applied_steps(axis));

  #elif ANY(MARKFORGED_XY, MARKFORGED_YX)

    // Requesting one of the joined axes?
    if (axis == CORE_AXIS_1 || axis == CORE_AXIS_2) {
      // Protect the access to the position.
      const bool was_enabled = stepper.suspend();

      const int32_t p1 = stepper.position(CORE_AXIS_1),
                    p2 = stepper.position(CORE_AXIS_2);

      if (was_enabled) stepper.wake_up();

      axis_steps = ((axis == CORE_AXIS_1) ? p1 - p2 : p2);
    }
    else
      axis_steps = DIFF_TERN(BACKLASH_COMPENSATION, stepper.position(axis), backlash.get_applied_steps(axis));

  #else

    axis_steps = stepper.position(axis);
    TERN_(BACKLASH_COMPENSATION, axis_steps -= backlash.get_applied_steps(axis));

  #endif

  return axis_steps * mm_per_step[axis];
}

/**
 * Block until the planner is finished processing
 */
void Planner::synchronize() { while (busy()) idle(); }

/**
 * @brief Add a new linear movement to the planner queue (in terms of steps).
 *
 * @param target        Target position in steps units
 * @param target_float  Target position in direct (mm, degrees) units.
 * @param cart_dist_mm  The pre-calculated move lengths for all axes, in mm
 * @param fr_mm_s       (Target) speed of the move
 * @param extruder      Target extruder
 * @param hints         Parameters to aid planner calculations
 *
 * @return  true if movement was properly queued, false otherwise (if cleaning)
 */
bool Planner::_buffer_steps(const xyze_long_t &target
  OPTARG(HAS_POSITION_FLOAT, const xyze_pos_t &target_float)
  OPTARG(HAS_DIST_MM_ARG, const xyze_float_t &cart_dist_mm)
  , feedRate_t fr_mm_s, const uint8_t extruder, const PlannerHints &hints
) {

  // Wait for the next available block
  uint8_t next_buffer_head;
  block_t * const block = get_next_free_block(next_buffer_head);

  // If we are cleaning, do not accept queuing of movements
  // This must be after get_next_free_block() because it calls idle()
  // where cleaning_buffer_counter can be changed
  if (cleaning_buffer_counter) return false;

  // Fill the block with the specified movement
  float minimum_planner_speed_sqr;
  if (!_populate_block(block, target
        OPTARG(HAS_POSITION_FLOAT, target_float)
        OPTARG(HAS_DIST_MM_ARG, cart_dist_mm)
        , fr_mm_s, extruder, hints
        , minimum_planner_speed_sqr
      )
  ) {
    // Movement was not queued, probably because it was too short.
    //  Simply accept that as movement queued and done
    return true;
  }

  // If this is the first added movement, reload the delay, otherwise, cancel it.
  if (block_buffer_head == block_buffer_tail) {
    // If it was the first queued block, restart the 1st block delivery delay, to
    // give the planner an opportunity to queue more movements and plan them
    // As there are no queued movements, the Stepper ISR will not touch this
    // variable, so there is no risk setting this here (but it MUST be done
    // before the following line!!)
    delay_before_delivering = TERN_(FT_MOTION, ftMotion.cfg.active ? BLOCK_DELAY_NONE :) BLOCK_DELAY_FOR_1ST_MOVE;
  }

  // Move buffer head
  block_buffer_head = next_buffer_head;

  // find a speed from which the new block can stop safely
  const float safe_exit_speed_sqr = _MAX(
    TERN0(HINTS_SAFE_EXIT_SPEED, hints.safe_exit_speed_sqr),
    minimum_planner_speed_sqr
  );

  // Recalculate and optimize trapezoidal speed profiles
  recalculate(safe_exit_speed_sqr);

  // Movement successfully queued!
  return true;
}

/**
 * @brief Populate a block in preparation for insertion.
 * @details Populate the fields of a new linear movement block
 *          that will be added to the queue and processed soon
 *          by the Stepper ISR.
 *
 * @param block         A block to populate
 * @param target        Target position in steps units
 * @param target_float  Target position in native mm
 * @param cart_dist_mm  The pre-calculated move lengths for all axes, in mm
 * @param fr_mm_s       (Target) speed of the move
 * @param extruder      Target extruder
 * @param hints         Parameters to aid planner calculations
 *
 * @return  true if movement is acceptable, false otherwise
 */
bool Planner::_populate_block(
  block_t * const block,
  const abce_long_t &target
  OPTARG(HAS_POSITION_FLOAT, const xyze_pos_t &target_float)
  OPTARG(HAS_DIST_MM_ARG, const xyze_float_t &cart_dist_mm)
  , feedRate_t fr_mm_s, const uint8_t extruder, const PlannerHints &hints
  , float &minimum_planner_speed_sqr
) {
  xyze_long_t dist = target - position;

  /* <-- add a slash to enable
    SERIAL_ECHOLNPGM(
      "  _populate_block FR:", fr_mm_s,
      #if HAS_X_AXIS
        " " STR_A ":", target.a, " (", dist.a, " steps)"
      #endif
      #if HAS_Y_AXIS
        " " STR_B ":", target.b, " (", dist.b, " steps)"
      #endif
      #if HAS_Z_AXIS
        " " STR_C ":", target.c, " (", dist.c, " steps)"
      #endif
      #if HAS_I_AXIS
        " " STR_I ":", target.i, " (", dist.i, " steps)"
      #endif
      #if HAS_J_AXIS
        " " STR_J ":", target.j, " (", dist.j, " steps)"
      #endif
      #if HAS_K_AXIS
        " " STR_K ":", target.k, " (", dist.k, " steps)"
      #endif
      #if HAS_U_AXIS
        " " STR_U ":", target.u, " (", dist.u, " steps)"
      #endif
      #if HAS_V_AXIS
        " " STR_V ":", target.v, " (", dist.v, " steps)"
      #endif
      #if HAS_W_AXIS
        " " STR_W ":", target.w, " (", dist.w, " steps)"
      #endif
      #if HAS_EXTRUDERS
        " E:", target.e, " (", dist.e, " steps)"
      #endif
    );
  //*/

  #if ANY(PREVENT_COLD_EXTRUSION, PREVENT_LENGTHY_EXTRUDE)
    if (dist.e) {
      #if ENABLED(PREVENT_COLD_EXTRUSION)
        if (thermalManager.tooColdToExtrude(extruder)) {
          position.e = target.e; // Behave as if the move really took place, but ignore E part
          TERN_(HAS_POSITION_FLOAT, position_float.e = target_float.e);
          dist.e = 0; // no difference
          SERIAL_ECHO_MSG(STR_ERR_COLD_EXTRUDE_STOP);
        }
      #endif // PREVENT_COLD_EXTRUSION
      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        const float e_steps = ABS(dist.e * e_factor[extruder]);
        const float max_e_steps = settings.axis_steps_per_mm[E_AXIS_N(extruder)] * (EXTRUDE_MAXLENGTH);
        if (e_steps > max_e_steps) {
          #if ENABLED(MIXING_EXTRUDER)
            bool ignore_e = false;
            float collector[MIXING_STEPPERS];
            mixer.refresh_collector(1.0f, mixer.get_current_vtool(), collector);
            MIXER_STEPPER_LOOP(e)
              if (e_steps * collector[e] > max_e_steps) { ignore_e = true; break; }
          #else
            constexpr bool ignore_e = true;
          #endif
          if (ignore_e) {
            position.e = target.e; // Behave as if the move really took place, but ignore E part
            TERN_(HAS_POSITION_FLOAT, position_float.e = target_float.e);
            dist.e = 0; // no difference
            SERIAL_ECHO_MSG(STR_ERR_LONG_EXTRUDE_STOP);
          }
        }
      #endif // PREVENT_LENGTHY_EXTRUDE
    }
  #endif // PREVENT_COLD_EXTRUSION || PREVENT_LENGTHY_EXTRUDE

  // Compute direction bit-mask for this block
  AxisBits dm;
  #if ANY(CORE_IS_XY, CORE_IS_XZ, MARKFORGED_XY, MARKFORGED_YX)
    dm.hx = (dist.a > 0);                       // True direction in X
  #endif
  #if ANY(CORE_IS_XY, CORE_IS_YZ, MARKFORGED_XY, MARKFORGED_YX)
    dm.hy = (dist.b > 0);                       // True direction in Y
  #endif
  #if ANY(CORE_IS_XZ, CORE_IS_YZ)
    dm.hz = (dist.c > 0);                       // True direction in Z
  #endif
  #if CORE_IS_XY
    dm.a  = (dist.a + dist.b > 0);              // Motor A direction
    dm.b  = (CORESIGN(dist.a - dist.b) > 0);    // Motor B direction
    TERN_(HAS_Z_AXIS, dm.z = (dist.c > 0));     // Axis  Z direction
  #elif CORE_IS_XZ
    dm.a  = (dist.a + dist.c > 0);              // Motor A direction
    dm.y  = (dist.b > 0);                       // Axis  Y direction
    dm.c  = (CORESIGN(dist.a - dist.c) > 0);    // Motor C direction
  #elif CORE_IS_YZ
    dm.x  = (dist.a > 0);                       // Axis  X direction
    dm.b  = (dist.b + dist.c > 0);              // Motor B direction
    dm.c  = (CORESIGN(dist.b - dist.c) > 0);    // Motor C direction
  #elif ENABLED(MARKFORGED_XY)
    dm.a = (dist.a TERN(MARKFORGED_INVERSE, -, +) dist.b > 0); // Motor A direction
    dm.b = (dist.b > 0);                        // Motor B direction
    TERN_(HAS_Z_AXIS, dm.z = (dist.c > 0));     // Axis  Z direction
  #elif ENABLED(MARKFORGED_YX)
    dm.a = (dist.a > 0);                        // Motor A direction
    dm.b = (dist.b TERN(MARKFORGED_INVERSE, -, +) dist.a > 0); // Motor B direction
    TERN_(HAS_Z_AXIS, dm.z = (dist.c > 0));     // Axis  Z direction
  #else
    XYZ_CODE(
      dm.x = (dist.a > 0),
      dm.y = (dist.b > 0),
      dm.z = (dist.c > 0)
    );
  #endif

  SECONDARY_AXIS_CODE(
    dm.i = (dist.i > 0), dm.j = (dist.j > 0), dm.k = (dist.k > 0),
    dm.u = (dist.u > 0), dm.v = (dist.v > 0), dm.w = (dist.w > 0)
  );

  #if HAS_EXTRUDERS
    dm.e = (dist.e > 0);
    const float esteps_float = dist.e * e_factor[extruder];
    const uint32_t esteps = ABS(esteps_float);
  #else
    constexpr uint32_t esteps = 0;
  #endif

  // Clear all flags, including the "busy" bit
  block->flag.clear();

  // Set direction bits
  block->direction_bits = dm;

  /**
   * Update block laser power
   * For standard mode get the cutter.power value for processing, since it's
   * only set by apply_power().
   */
  #if HAS_CUTTER
    switch (cutter.cutter_mode) {
      default: break;

      case CUTTER_MODE_STANDARD: block->cutter_power = cutter.power; break;

      #if ENABLED(LASER_FEATURE)
        /**
         * For inline mode get the laser_inline variables, including power and status.
         * Dynamic mode only needs to update if the feedrate has changed, since it's
         * calculated from the current feedrate and power level.
         */
        case CUTTER_MODE_CONTINUOUS:
          block->laser.power = laser_inline.power;
          block->laser.status = laser_inline.status;
          break;

        case CUTTER_MODE_DYNAMIC:
          if (cutter.laser_feedrate_changed())  // Only process changes in rate
            block->laser.power = laser_inline.power = cutter.calc_dynamic_power();
          break;
      #endif
    }
  #endif

  // Number of steps for each axis
  // See https://www.corexy.com/theory.html
  block->steps.set(NUM_AXIS_LIST(
    #if CORE_IS_XY
      ABS(dist.a + dist.b), ABS(dist.a - dist.b), ABS(dist.c)
    #elif CORE_IS_XZ
      ABS(dist.a + dist.c), ABS(dist.b), ABS(dist.a - dist.c)
    #elif CORE_IS_YZ
      ABS(dist.a), ABS(dist.b + dist.c), ABS(dist.b - dist.c)
    #elif ENABLED(MARKFORGED_XY)
      ABS(dist.a TERN(MARKFORGED_INVERSE, -, +) dist.b), ABS(dist.b), ABS(dist.c)
    #elif ENABLED(MARKFORGED_YX)
      ABS(dist.a), ABS(dist.b TERN(MARKFORGED_INVERSE, -, +) dist.a), ABS(dist.c)
    #elif IS_SCARA
      ABS(dist.a), ABS(dist.b), ABS(dist.c)
    #else // default non-h-bot planning
      ABS(dist.a), ABS(dist.b), ABS(dist.c)
    #endif
    , ABS(dist.i), ABS(dist.j), ABS(dist.k), ABS(dist.u), ABS(dist.v), ABS(dist.w)
  ));

  /**
   * This part of the code calculates the total length of the movement.
   * For cartesian bots, the distance along the X axis equals the X_AXIS joint displacement and same holds true for Y_AXIS.
   * But for geometries like CORE_XY that is not true. For these machines we need to create 2 additional variables, named X_HEAD and Y_HEAD, to store the displacent of the head along the X and Y axes in a cartesian coordinate system.
   * The displacement of the head along the axes of the cartesian coordinate system has to be calculated from "X_AXIS" and "Y_AXIS" (should be renamed to A_JOINT and B_JOINT)
   * displacements in joints space using forward kinematics (A=X+Y and B=X-Y in the case of CORE_XY).
   * Next we can calculate the total movement length and apply the desired speed.
   */
  struct DistanceMM : abce_float_t {
    #if ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX)
      struct { float x, y, z; } head;
    #endif
  } dist_mm;

  #if ANY(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX)
    dist_mm.head.x = dist.a * mm_per_step[A_AXIS];
    dist_mm.head.y = dist.b * mm_per_step[B_AXIS];
    TERN_(HAS_Z_AXIS, dist_mm.z = dist.c * mm_per_step[Z_AXIS]);
  #endif
  #if CORE_IS_XY
    dist_mm.a      = (dist.a + dist.b) * mm_per_step[A_AXIS];
    dist_mm.b      = CORESIGN(dist.a - dist.b) * mm_per_step[B_AXIS];
  #elif CORE_IS_XZ
    dist_mm.head.x = dist.a * mm_per_step[A_AXIS];
    dist_mm.y      = dist.b * mm_per_step[Y_AXIS];
    dist_mm.head.z = dist.c * mm_per_step[C_AXIS];
    dist_mm.a      = (dist.a + dist.c) * mm_per_step[A_AXIS];
    dist_mm.c      = CORESIGN(dist.a - dist.c) * mm_per_step[C_AXIS];
  #elif CORE_IS_YZ
    dist_mm.x      = dist.a * mm_per_step[X_AXIS];
    dist_mm.head.y = dist.b * mm_per_step[B_AXIS];
    dist_mm.head.z = dist.c * mm_per_step[C_AXIS];
    dist_mm.b      = (dist.b + dist.c) * mm_per_step[B_AXIS];
    dist_mm.c      = CORESIGN(dist.b - dist.c) * mm_per_step[C_AXIS];
  #elif ENABLED(MARKFORGED_XY)
    dist_mm.a = (dist.a TERN(MARKFORGED_INVERSE, +, -) dist.b) * mm_per_step[A_AXIS];
    dist_mm.b = dist.b * mm_per_step[B_AXIS];
  #elif ENABLED(MARKFORGED_YX)
    dist_mm.a = dist.a * mm_per_step[A_AXIS];
    dist_mm.b = (dist.b TERN(MARKFORGED_INVERSE, +, -) dist.a) * mm_per_step[B_AXIS];
  #else
    XYZ_CODE(
      dist_mm.a = dist.a * mm_per_step[A_AXIS],
      dist_mm.b = dist.b * mm_per_step[B_AXIS],
      dist_mm.c = dist.c * mm_per_step[C_AXIS]
    );
  #endif

  SECONDARY_AXIS_CODE(
    dist_mm.i = dist.i * mm_per_step[I_AXIS], dist_mm.j = dist.j * mm_per_step[J_AXIS], dist_mm.k = dist.k * mm_per_step[K_AXIS],
    dist_mm.u = dist.u * mm_per_step[U_AXIS], dist_mm.v = dist.v * mm_per_step[V_AXIS], dist_mm.w = dist.w * mm_per_step[W_AXIS]
  );

  TERN_(HAS_EXTRUDERS, dist_mm.e = esteps_float * mm_per_step[E_AXIS_N(extruder)]);

  TERN_(LCD_SHOW_E_TOTAL, e_move_accumulator += dist_mm.e);

  #if HAS_ROTATIONAL_AXES
    bool cartesian_move = hints.cartesian_move;
  #endif

  if (true NUM_AXIS_GANG(
      && block->steps.a < MIN_STEPS_PER_SEGMENT, && block->steps.b < MIN_STEPS_PER_SEGMENT, && block->steps.c < MIN_STEPS_PER_SEGMENT,
      && block->steps.i < MIN_STEPS_PER_SEGMENT, && block->steps.j < MIN_STEPS_PER_SEGMENT, && block->steps.k < MIN_STEPS_PER_SEGMENT,
      && block->steps.u < MIN_STEPS_PER_SEGMENT, && block->steps.v < MIN_STEPS_PER_SEGMENT, && block->steps.w < MIN_STEPS_PER_SEGMENT
    )
  ) {
    block->millimeters = TERN0(HAS_EXTRUDERS, ABS(dist_mm.e));
  }
  else {
    if (hints.millimeters)
      block->millimeters = hints.millimeters;
    else {
      const xyze_pos_t displacement = LOGICAL_AXIS_ARRAY(
        dist_mm.e,
        #if ANY(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX)
          dist_mm.head.x, dist_mm.head.y, dist_mm.z,
        #elif CORE_IS_XZ
          dist_mm.head.x, dist_mm.y, dist_mm.head.z,
        #elif CORE_IS_YZ
          dist_mm.x, dist_mm.head.y, dist_mm.head.z,
        #else
          dist_mm.x, dist_mm.y, dist_mm.z,
        #endif
        dist_mm.i, dist_mm.j, dist_mm.k,
        dist_mm.u, dist_mm.v, dist_mm.w
      );

      block->millimeters = get_move_distance(displacement OPTARG(HAS_ROTATIONAL_AXES, cartesian_move));
    }

    /**
     * At this point at least one of the axes has more steps than
     * MIN_STEPS_PER_SEGMENT, ensuring the segment won't get dropped as
     * zero-length. It's important to not apply corrections
     * to blocks that would get dropped!
     *
     * A correction function is permitted to add steps to an axis, it
     * should *never* remove steps!
     */
    TERN_(BACKLASH_COMPENSATION, backlash.add_correction_steps(dist, dm, block));
  }

  TERN_(HAS_EXTRUDERS, block->steps.e = esteps);

  block->step_event_count = (
    #if NUM_AXES
      _MAX(LOGICAL_AXIS_LIST(esteps,
        block->steps.a, block->steps.b, block->steps.c,
        block->steps.i, block->steps.j, block->steps.k,
        block->steps.u, block->steps.v, block->steps.w
      ))
    #elif HAS_EXTRUDERS
      esteps
    #endif
  );

  // Bail if this is a zero-length block
  if (block->step_event_count < MIN_STEPS_PER_SEGMENT) return false;

  TERN_(MIXING_EXTRUDER, mixer.populate_block(block->b_color));

  #if HAS_FAN
    FANS_LOOP(i) block->fan_speed[i] = thermalManager.fan_speed[i];
  #endif

  #if ENABLED(BARICUDA)
    block->valve_pressure = baricuda_valve_pressure;
    block->e_to_p_pressure = baricuda_e_to_p_pressure;
  #endif

  E_TERN_(block->extruder = extruder);

  #if ENABLED(AUTO_POWER_CONTROL)
    if (NUM_AXIS_GANG(
         block->steps.x, || block->steps.y, || block->steps.z,
      || block->steps.i, || block->steps.j, || block->steps.k,
      || block->steps.u, || block->steps.v, || block->steps.w
    )) powerManager.power_on();
  #endif

  // Enable active axes
  #if ANY(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX)
    if (block->steps.a || block->steps.b) {
      stepper.enable_axis(X_AXIS);
      stepper.enable_axis(Y_AXIS);
    }
    #if HAS_Z_AXIS && DISABLED(Z_LATE_ENABLE)
      if (block->steps.z) stepper.enable_axis(Z_AXIS);
    #endif
  #elif CORE_IS_XZ
    if (block->steps.a || block->steps.c) {
      stepper.enable_axis(X_AXIS);
      stepper.enable_axis(Z_AXIS);
    }
    if (block->steps.y) stepper.enable_axis(Y_AXIS);
  #elif CORE_IS_YZ
    if (block->steps.b || block->steps.c) {
      stepper.enable_axis(Y_AXIS);
      stepper.enable_axis(Z_AXIS);
    }
    if (block->steps.x) stepper.enable_axis(X_AXIS);
  #else
    NUM_AXIS_CODE(
      if (block->steps.x) stepper.enable_axis(X_AXIS),
      if (block->steps.y) stepper.enable_axis(Y_AXIS),
      if (TERN(Z_LATE_ENABLE, 0, block->steps.z)) stepper.enable_axis(Z_AXIS),
      if (block->steps.i) stepper.enable_axis(I_AXIS),
      if (block->steps.j) stepper.enable_axis(J_AXIS),
      if (block->steps.k) stepper.enable_axis(K_AXIS),
      if (block->steps.u) stepper.enable_axis(U_AXIS),
      if (block->steps.v) stepper.enable_axis(V_AXIS),
      if (block->steps.w) stepper.enable_axis(W_AXIS)
    );
  #endif
  #if ANY(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX)
    SECONDARY_AXIS_CODE(
      if (block->steps.i) stepper.enable_axis(I_AXIS), if (block->steps.j) stepper.enable_axis(J_AXIS),
      if (block->steps.k) stepper.enable_axis(K_AXIS), if (block->steps.u) stepper.enable_axis(U_AXIS),
      if (block->steps.v) stepper.enable_axis(V_AXIS), if (block->steps.w) stepper.enable_axis(W_AXIS)
    );
  #endif

  // Enable extruder(s)
  #if HAS_EXTRUDERS
    if (esteps) {
      TERN_(AUTO_POWER_CONTROL, powerManager.power_on());

      #if ENABLED(DISABLE_OTHER_EXTRUDERS) // Enable only the selected extruder

        // Count down all steppers that were recently moved
        for (uint8_t i = 0; i < E_STEPPERS; ++i)
          if (extruder_last_move[i]) extruder_last_move[i]--;

        // Switching Extruder uses one E stepper motor per two nozzles
        #define E_STEPPER_INDEX(E) TERN(HAS_SWITCHING_EXTRUDER, (E) / 2, E)

        // Enable all (i.e., both) E steppers for IDEX-style duplication, but only active E steppers for multi-nozzle (i.e., single wide X carriage) duplication
        #define _IS_DUPE(N) TERN0(HAS_DUPLICATION_MODE, (extruder_duplication_enabled && TERN1(MULTI_NOZZLE_DUPLICATION, TEST(duplication_e_mask, N))))

        #define ENABLE_ONE_E(N) do{ \
          if (N == E_STEPPER_INDEX(extruder) || _IS_DUPE(N)) {  /* N is 'extruder', or N is duplicating */ \
            stepper.ENABLE_EXTRUDER(N);                         /* Enable the relevant E stepper... */ \
            extruder_last_move[N] = (BLOCK_BUFFER_SIZE) * 2;    /* ...and reset its counter */ \
          } \
          else if (!extruder_last_move[N])                      /* Counter expired since last E stepper enable */ \
            stepper.DISABLE_EXTRUDER(N);                        /* Disable the E stepper */ \
        }while(0);

      #else

        #define ENABLE_ONE_E(N) stepper.ENABLE_EXTRUDER(N);

      #endif

      REPEAT(E_STEPPERS, ENABLE_ONE_E); // (ENABLE_ONE_E must end with semicolon)
    }
  #endif // HAS_EXTRUDERS

  if (esteps)
    NOLESS(fr_mm_s, settings.min_feedrate_mm_s);
  else
    NOLESS(fr_mm_s, settings.min_travel_feedrate_mm_s);

  const float inverse_millimeters = 1.0f / block->millimeters;  // Inverse millimeters to remove multiple divides

  /**
   * Calculate inverse time for this move. No divide by zero due to previous checks.
   * EXAMPLE: At 120mm/s a 60mm move involving XYZ axes takes 0.5s. So this will give 2.0.
   * EXAMPLE: At 120°/s a 60° move involving only rotational axes takes 0.5s. So this will give 2.0.
   */
  float inverse_secs = inverse_millimeters * (
    #if ALL(HAS_ROTATIONAL_AXES, INCH_MODE_SUPPORT)
      /**
       * Workaround for premature feedrate conversion
       * from in/s to mm/s by get_distance_from_command.
       */
      cartesian_move ? fr_mm_s : LINEAR_UNIT(fr_mm_s)
    #else
      fr_mm_s
    #endif
  );

  // Get the number of non busy movements in queue (non busy means that they can be altered)
  const uint8_t moves_queued = nonbusy_movesplanned();

  // Slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
  #if ANY(SLOWDOWN, HAS_WIRED_LCD) || defined(XY_FREQUENCY_LIMIT)
    // Segment time in microseconds
    int32_t segment_time_us = LROUND(1000000.0f / inverse_secs);
  #endif

  #if ENABLED(SLOWDOWN)
    #ifndef SLOWDOWN_DIVISOR
      #define SLOWDOWN_DIVISOR 2
    #endif
    if (WITHIN(moves_queued, 2, (BLOCK_BUFFER_SIZE) / (SLOWDOWN_DIVISOR) - 1)) {
      #ifdef MAX7219_DEBUG_SLOWDOWN
        slowdown_count = (slowdown_count + 1) & 0x0F;
      #endif
      const int32_t time_diff = settings.min_segment_time_us - segment_time_us;
      if (time_diff > 0) {
        // Buffer is draining so add extra time. The amount of time added increases if the buffer is still emptied more.
        const int32_t nst = segment_time_us + LROUND(2 * time_diff / moves_queued);
        inverse_secs = 1000000.0f / nst;
        #if defined(XY_FREQUENCY_LIMIT) || HAS_WIRED_LCD
          segment_time_us = nst;
        #endif
      }
    }
  #endif

  #if HAS_WIRED_LCD
    // Protect the access to the position.
    const bool was_enabled = stepper.suspend();

    block_buffer_runtime_us += segment_time_us;
    block->segment_time_us = segment_time_us;

    if (was_enabled) stepper.wake_up();
  #endif

  block->nominal_speed = block->millimeters * inverse_secs;           // (mm/sec) Always > 0
  block->nominal_rate = CEIL(block->step_event_count * inverse_secs); // (step/sec) Always > 0

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    if (extruder == FILAMENT_SENSOR_EXTRUDER_NUM)   // Only for extruder with filament sensor
      filwidth.advance_e(dist_mm.e);
  #endif

  // Calculate and limit speed in mm/sec (linear) or degrees/sec (rotational)

  xyze_float_t current_speed;
  float speed_factor = 1.0f; // factor <1 decreases speed

  // Linear axes first with less logic
  LOOP_NUM_AXES(i) {
    current_speed[i] = dist_mm[i] * inverse_secs;
    const feedRate_t cs = ABS(current_speed[i]),
                 max_fr = settings.max_feedrate_mm_s[i];
    if (cs > max_fr) NOMORE(speed_factor, max_fr / cs);
  }

  // Limit speed on extruders, if any
  #if HAS_EXTRUDERS
  {
    current_speed.e = dist_mm.e * inverse_secs;
    #if HAS_MIXER_SYNC_CHANNEL
      // Move all mixing extruders at the specified rate
      if (mixer.get_current_vtool() == MIXER_AUTORETRACT_TOOL)
        current_speed.e *= MIXING_STEPPERS;
    #endif

    const feedRate_t cs = ABS(current_speed.e),
                 max_fr = MUL_TERN(HAS_MIXER_SYNC_CHANNEL, settings.max_feedrate_mm_s[E_AXIS_N(extruder)], MIXING_STEPPERS);

    if (cs > max_fr) NOMORE(speed_factor, max_fr / cs); // Respect max feedrate on any move (travel and print)

    #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
      const feedRate_t max_vfr = MUL_TERN(HAS_MIXER_SYNC_CHANNEL, volumetric_extruder_feedrate_limit[extruder], MIXING_STEPPERS);

      // TODO: Doesn't work properly for joined segments. Set MIN_STEPS_PER_SEGMENT 1 as workaround.

      if (block->steps.a || block->steps.b || block->steps.c) {

        if (max_vfr > 0 && cs > max_vfr) {
          NOMORE(speed_factor, max_vfr / cs); // respect volumetric extruder limit (if any)
          /* <-- add a slash to enable
          SERIAL_ECHOPGM("volumetric extruder limit enforced: ", (cs * CIRCLE_AREA(filament_size[extruder] * 0.5f)));
          SERIAL_ECHOPGM(" mm^3/s (", cs);
          SERIAL_ECHOPGM(" mm/s) limited to ", (max_vfr * CIRCLE_AREA(filament_size[extruder] * 0.5f)));
          SERIAL_ECHOPGM(" mm^3/s (", max_vfr);
          SERIAL_ECHOLNPGM(" mm/s)");
          //*/
        }
      }
    #endif
  }
  #endif // HAS_EXTRUDERS

  #ifdef XY_FREQUENCY_LIMIT

    static AxisBits old_direction_bits; // = 0

    if (xy_freq_limit_hz) {
      // Check and limit the xy direction change frequency
      const AxisBits direction_change = block->direction_bits ^ old_direction_bits;
      old_direction_bits = block->direction_bits;
      segment_time_us = LROUND(float(segment_time_us) / speed_factor);

      static int32_t xs0, xs1, xs2, ys0, ys1, ys2;
      if (segment_time_us > xy_freq_min_interval_us)
        xs2 = xs1 = ys2 = ys1 = xy_freq_min_interval_us;
      else {
        xs2 = xs1; xs1 = xs0;
        ys2 = ys1; ys1 = ys0;
      }
      xs0 = direction_change.x ? segment_time_us : xy_freq_min_interval_us;
      ys0 = direction_change.y ? segment_time_us : xy_freq_min_interval_us;

      if (segment_time_us < xy_freq_min_interval_us) {
        const int32_t least_xy_segment_time = _MIN(_MAX(xs0, xs1, xs2), _MAX(ys0, ys1, ys2));
        if (least_xy_segment_time < xy_freq_min_interval_us) {
          float freq_xy_feedrate = (speed_factor * least_xy_segment_time) / xy_freq_min_interval_us;
          NOLESS(freq_xy_feedrate, xy_freq_min_speed_factor);
          NOMORE(speed_factor, freq_xy_feedrate);
        }
      }
    }

  #endif // XY_FREQUENCY_LIMIT

  // Correct the speed
  if (speed_factor < 1.0f) {
    current_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
    block->nominal_speed *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  const float steps_per_mm = block->step_event_count * inverse_millimeters;
  block->steps_per_mm = steps_per_mm;
  uint32_t accel;
  #if ENABLED(LIN_ADVANCE)
    bool use_advance_lead = false;
  #endif
  if (!ANY_AXIS_MOVES(block)) {                                   // Is this a retract / recover move?
    accel = CEIL(settings.retract_acceleration * steps_per_mm);   // Convert to: acceleration steps/sec^2
  }
  else {
    #define LIMIT_ACCEL_LONG(AXIS,INDX) do{ \
      if (block->steps[AXIS] && max_acceleration_steps_per_s2[AXIS+INDX] < accel) { \
        const uint32_t max_possible = max_acceleration_steps_per_s2[AXIS+INDX] * block->step_event_count / block->steps[AXIS]; \
        NOMORE(accel, max_possible); \
      } \
    }while(0)

    #define LIMIT_ACCEL_FLOAT(AXIS,INDX) do{ \
      if (block->steps[AXIS] && max_acceleration_steps_per_s2[AXIS+INDX] < accel) { \
        const float max_possible = float(max_acceleration_steps_per_s2[AXIS+INDX]) * float(block->step_event_count) / float(block->steps[AXIS]); \
        NOMORE(accel, max_possible); \
      } \
    }while(0)

    // Start with print or travel acceleration
    accel = CEIL((esteps ? settings.acceleration : settings.travel_acceleration) * steps_per_mm);

    #if ENABLED(LIN_ADVANCE)
      // Linear advance is currently not ready for HAS_I_AXIS
      #define MAX_E_JERK(N) TERN(HAS_LINEAR_E_JERK, max_e_jerk[E_INDEX_N(N)], max_jerk.e)

      /**
       * Use LIN_ADVANCE for blocks if all these are true:
       *
       * esteps                       : This is a print move, because we checked for A, B, C steps before.
       *
       * extruder_advance_K[extruder] : There is an advance factor set for this extruder.
       *
       * dm.e                         : Extruder is running forward (e.g., for "Wipe while retracting" (Slic3r) or "Combing" (Cura) moves)
       */
      use_advance_lead = esteps && extruder_advance_K[E_INDEX_N(extruder)] && dm.e;

      if (use_advance_lead) {
        float e_D_ratio = (target_float.e - position_float.e) /
          TERN(IS_KINEMATIC, block->millimeters,
            SQRT(sq(target_float.x - position_float.x)
               + sq(target_float.y - position_float.y)
               + sq(target_float.z - position_float.z))
          );

        // Check for unusual high e_D ratio to detect if a retract move was combined with the last print move due to min. steps per segment. Never execute this with advance!
        // This assumes no one will use a retract length of 0mm < retr_length < ~0.2mm and no one will print 100mm wide lines using 3mm filament or 35mm wide lines using 1.75mm filament.
        if (e_D_ratio > 3.0f)
          use_advance_lead = false;
        else {
          // Scale E acceleration so that it will be possible to jump to the advance speed.
          const uint32_t max_accel_steps_per_s2 = MAX_E_JERK(extruder) / (extruder_advance_K[E_INDEX_N(extruder)] * e_D_ratio) * steps_per_mm;
          if (accel > max_accel_steps_per_s2) {
            accel = max_accel_steps_per_s2;
            if (ENABLED(LA_DEBUG)) SERIAL_ECHOLNPGM("Acceleration limited.");
          }
        }
      }
    #endif

    // Limit acceleration per axis
    if (block->step_event_count <= acceleration_long_cutoff) {
      LOGICAL_AXIS_CODE(
        LIMIT_ACCEL_LONG(E_AXIS, E_INDEX_N(extruder)),
        LIMIT_ACCEL_LONG(A_AXIS, 0), LIMIT_ACCEL_LONG(B_AXIS, 0), LIMIT_ACCEL_LONG(C_AXIS, 0),
        LIMIT_ACCEL_LONG(I_AXIS, 0), LIMIT_ACCEL_LONG(J_AXIS, 0), LIMIT_ACCEL_LONG(K_AXIS, 0),
        LIMIT_ACCEL_LONG(U_AXIS, 0), LIMIT_ACCEL_LONG(V_AXIS, 0), LIMIT_ACCEL_LONG(W_AXIS, 0)
      );
    }
    else {
      LOGICAL_AXIS_CODE(
        LIMIT_ACCEL_FLOAT(E_AXIS, E_INDEX_N(extruder)),
        LIMIT_ACCEL_FLOAT(A_AXIS, 0), LIMIT_ACCEL_FLOAT(B_AXIS, 0), LIMIT_ACCEL_FLOAT(C_AXIS, 0),
        LIMIT_ACCEL_FLOAT(I_AXIS, 0), LIMIT_ACCEL_FLOAT(J_AXIS, 0), LIMIT_ACCEL_FLOAT(K_AXIS, 0),
        LIMIT_ACCEL_FLOAT(U_AXIS, 0), LIMIT_ACCEL_FLOAT(V_AXIS, 0), LIMIT_ACCEL_FLOAT(W_AXIS, 0)
      );
    }
  }
  block->acceleration_steps_per_s2 = accel;
  block->acceleration = accel / steps_per_mm;
  #if DISABLED(S_CURVE_ACCELERATION)
    block->acceleration_rate = (uint32_t)(accel * (float(1UL << 24) / (STEPPER_TIMER_RATE)));
  #endif

  #if ENABLED(LIN_ADVANCE)
    block->la_advance_rate = 0;
    block->la_scaling = 0;

    if (use_advance_lead) {
      // the Bresenham algorithm will convert this step rate into extruder steps
      block->la_advance_rate = extruder_advance_K[E_INDEX_N(extruder)] * block->acceleration_steps_per_s2;

      // reduce LA ISR frequency by calling it only often enough to ensure that there will
      // never be more than four extruder steps per call
      for (uint32_t dividend = block->steps.e << 1; dividend <= (block->step_event_count >> 2); dividend <<= 1)
        block->la_scaling++;

      #if ENABLED(LA_DEBUG)
        if (block->la_advance_rate >> block->la_scaling > 10000)
          SERIAL_ECHOLNPGM("eISR running at > 10kHz: ", block->la_advance_rate);
      #endif
    }
  #endif

  // Formula for the average speed over a 1 step worth of distance if starting from zero and
  // accelerating at the current limit. Since we can only change the speed every step this is a
  // good lower limit for the entry and exit speeds. Note that for calculate_trapezoid_for_block()
  // to work correctly, this must be accurately set and propagated.
  minimum_planner_speed_sqr = 0.5f * block->acceleration / steps_per_mm;
  // Go straight to/from nominal speed if block->acceleration is too high for it.
  NOMORE(minimum_planner_speed_sqr, sq(block->nominal_speed));

  float vmax_junction_sqr; // Initial limit on the segment entry velocity (mm/s)^2

  #if HAS_JUNCTION_DEVIATION
    /**
     * Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
     * Let a circle be tangent to both previous and current path line segments, where the junction
     * deviation is defined as the distance from the junction to the closest edge of the circle,
     * colinear with the circle center. The circular segment joining the two paths represents the
     * path of centripetal acceleration. Solve for max velocity based on max acceleration about the
     * radius of the circle, defined indirectly by junction deviation. This may be also viewed as
     * path width or max_jerk in the previous Grbl version. This approach does not actually deviate
     * from path, but used as a robust way to compute cornering speeds, as it takes into account the
     * nonlinearities of both the junction angle and junction velocity.
     *
     * NOTE: If the junction deviation value is finite, Grbl executes the motions in an exact path
     * mode (G61). If the junction deviation value is zero, Grbl will execute the motion in an exact
     * stop mode (G61.1) manner. In the future, if continuous mode (G64) is desired, the math here
     * is exactly the same. Instead of motioning all the way to junction point, the machine will
     * just follow the arc circle defined here. The Arduino doesn't have the CPU cycles to perform
     * a continuous mode path, but ARM-based microcontrollers most certainly do.
     *
     * NOTE: The max junction speed is a fixed value, since machine acceleration limits cannot be
     * changed dynamically during operation nor can the line move geometry. This must be kept in
     * memory in the event of a feedrate override changing the nominal speeds of blocks, which can
     * change the overall maximum entry speed conditions of all blocks.
     *
     * #######
     * https://github.com/MarlinFirmware/Marlin/issues/10341#issuecomment-388191754
     *
     * hoffbaked: on May 10 2018 tuned and improved the GRBL algorithm for Marlin:
          Okay! It seems to be working good. I somewhat arbitrarily cut it off at 1mm
          on then on anything with less sides than an octagon. With this, and the
          reverse pass actually recalculating things, a corner acceleration value
          of 1000 junction deviation of .05 are pretty reasonable. If the cycles
          can be spared, a better acos could be used. For all I know, it may be
          already calculated in a different place. */

    // Unit vector of previous path line segment
    static xyze_float_t prev_unit_vec;

    xyze_float_t unit_vec =
      #if HAS_DIST_MM_ARG
        cart_dist_mm
      #else
        LOGICAL_AXIS_ARRAY(dist_mm.e,
          dist_mm.x, dist_mm.y, dist_mm.z,
          dist_mm.i, dist_mm.j, dist_mm.k,
          dist_mm.u, dist_mm.v, dist_mm.w)
      #endif
    ;

    /**
     * On CoreXY the length of the vector [A,B] is SQRT(2) times the length of the head movement vector [X,Y].
     * So taking Z and E into account, we cannot scale to a unit vector with "inverse_millimeters".
     * => normalize the complete junction vector.
     * Elsewise, when needed JD will factor-in the E component
     */
    if (ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX) || esteps > 0)
      normalize_junction_vector(unit_vec);  // Normalize with XYZE components
    else
      unit_vec *= inverse_millimeters;      // Use pre-calculated (1 / SQRT(x^2 + y^2 + z^2))

    // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
    if (moves_queued && !UNEAR_ZERO(previous_nominal_speed)) {
      // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
      // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
      float junction_cos_theta = LOGICAL_AXIS_GANG(
                                 + (-prev_unit_vec.e * unit_vec.e),
                                 + (-prev_unit_vec.x * unit_vec.x),
                                 + (-prev_unit_vec.y * unit_vec.y),
                                 + (-prev_unit_vec.z * unit_vec.z),
                                 + (-prev_unit_vec.i * unit_vec.i),
                                 + (-prev_unit_vec.j * unit_vec.j),
                                 + (-prev_unit_vec.k * unit_vec.k),
                                 + (-prev_unit_vec.u * unit_vec.u),
                                 + (-prev_unit_vec.v * unit_vec.v),
                                 + (-prev_unit_vec.w * unit_vec.w)
                               );

      // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
      if (junction_cos_theta > 0.999999f) {
        // For a 0 degree acute junction, just set minimum junction speed.
        vmax_junction_sqr = minimum_planner_speed_sqr;
      }
      else {
        // Convert delta vector to unit vector
        xyze_float_t junction_unit_vec = unit_vec - prev_unit_vec;
        normalize_junction_vector(junction_unit_vec);

        const float junction_acceleration = limit_value_by_axis_maximum(block->acceleration, junction_unit_vec);

        if (TERN0(HINTS_CURVE_RADIUS, hints.curve_radius)) {
          TERN_(HINTS_CURVE_RADIUS, vmax_junction_sqr = junction_acceleration * hints.curve_radius);
        }
        else {
          NOLESS(junction_cos_theta, -0.999999f); // Check for numerical round-off to avoid divide by zero.

          const float sin_theta_d2 = SQRT(0.5f * (1.0f - junction_cos_theta)); // Trig half angle identity. Always positive.

          vmax_junction_sqr = junction_acceleration * junction_deviation_mm * sin_theta_d2 / (1.0f - sin_theta_d2);

          #if ENABLED(JD_HANDLE_SMALL_SEGMENTS)

            // For small moves with >135° junction (octagon) find speed for approximate arc
            if (block->millimeters < 1 && junction_cos_theta < -0.7071067812f) {

              #if ENABLED(JD_USE_MATH_ACOS)

                #error "TODO: Inline maths with the MCU / FPU."

              #elif ENABLED(JD_USE_LOOKUP_TABLE)

                // Fast acos approximation (max. error +-0.01 rads)
                // Based on LUT table and linear interpolation

                /**
                 *  // Generate the JD Lookup Table
                 *  constexpr float c = 1.00751495f; // Correction factor to center error around 0
                 *  for (int i = 0; i < jd_lut_count - 1; ++i) {
                 *    const float x0 = (sq(i) - 1) / sq(i),
                 *                y0 = acos(x0) * (i == 0 ? 1 : c),
                 *                x1 = i < jd_lut_count - 1 ?  0.5 * x0 + 0.5 : 0.999999f,
                 *                y1 = acos(x1) * (i < jd_lut_count - 1 ? c : 1);
                 *    jd_lut_k[i] = (y0 - y1) / (x0 - x1);
                 *    jd_lut_b[i] = (y1 * x0 - y0 * x1) / (x0 - x1);
                 *  }
                 *
                 *  // Compute correction factor (Set c to 1.0f first!)
                 *  float min = INFINITY, max = -min;
                 *  for (float t = 0; t <= 1; t += 0.0003f) {
                 *    const float e = acos(t) / approx(t);
                 *    if (isfinite(e)) {
                 *      if (e < min) min = e;
                 *      if (e > max) max = e;
                 *    }
                 *  }
                 *  fprintf(stderr, "%.9gf, ", (min + max) / 2);
                 */
                static constexpr int16_t  jd_lut_count = 16;
                static constexpr uint16_t jd_lut_tll   = _BV(jd_lut_count - 1);
                static constexpr int16_t  jd_lut_tll0  = __builtin_clz(jd_lut_tll) + 1; // i.e., 16 - jd_lut_count + 1
                static constexpr float jd_lut_k[jd_lut_count] PROGMEM = {
                  -1.03145837f, -1.30760646f, -1.75205851f, -2.41705704f,
                  -3.37769222f, -4.74888992f, -6.69649887f, -9.45661736f,
                  -13.3640480f, -18.8928222f, -26.7136841f, -37.7754593f,
                  -53.4201813f, -75.5458374f, -106.836761f, -218.532821f };
                static constexpr float jd_lut_b[jd_lut_count] PROGMEM = {
                   1.57079637f,  1.70887053f,  2.04220939f,  2.62408352f,
                   3.52467871f,  4.85302639f,  6.77020454f,  9.50875854f,
                   13.4009285f,  18.9188995f,  26.7321243f,  37.7885055f,
                   53.4293975f,  75.5523529f,  106.841369f,  218.534011f };

                const float neg = junction_cos_theta < 0 ? -1 : 1,
                            t = neg * junction_cos_theta;

                const int16_t idx = (t < 0.00000003f) ? 0 : __builtin_clz(uint16_t((1.0f - t) * jd_lut_tll)) - jd_lut_tll0;

                float junction_theta = t * pgm_read_float(&jd_lut_k[idx]) + pgm_read_float(&jd_lut_b[idx]);
                if (neg > 0) junction_theta = RADIANS(180) - junction_theta; // acos(-t)

              #else

                // Fast acos(-t) approximation (max. error +-0.033rad = 1.89°)
                // Based on MinMax polynomial published by W. Randolph Franklin, see
                // https://wrf.ecse.rpi.edu/Research/Short_Notes/arcsin/onlyelem.html
                //  acos( t) = pi / 2 - asin(x)
                //  acos(-t) = pi - acos(t) ... pi / 2 + asin(x)

                const float neg = junction_cos_theta < 0 ? -1 : 1,
                            t = neg * junction_cos_theta,
                            asinx =       0.032843707f
                                  + t * (-1.451838349f
                                  + t * ( 29.66153956f
                                  + t * (-131.1123477f
                                  + t * ( 262.8130562f
                                  + t * (-242.7199627f
                                  + t * ( 84.31466202f ) ))))),
                            junction_theta = RADIANS(90) + neg * asinx; // acos(-t)

                // NOTE: junction_theta bottoms out at 0.033 which avoids divide by 0.

              #endif

              const float limit_sqr = (block->millimeters * junction_acceleration) / junction_theta;
              NOMORE(vmax_junction_sqr, limit_sqr);
            }

          #endif // JD_HANDLE_SMALL_SEGMENTS
        }
      }

      // Get the lowest speed
      vmax_junction_sqr = _MIN(vmax_junction_sqr, sq(block->nominal_speed), sq(previous_nominal_speed));
    }
    else vmax_junction_sqr = minimum_planner_speed_sqr;

    prev_unit_vec = unit_vec;

  #else // CLASSIC_JERK

    /**
     * Heavily modified. Originally adapted from Průša firmware.
     * https://github.com/prusa3d/Prusa-Firmware
     */
    #if defined(TRAVEL_EXTRA_XYJERK) || ENABLED(LIN_ADVANCE)
      xyze_float_t max_j = max_jerk;
    #else
      const xyze_float_t &max_j = max_jerk;
    #endif

    #ifdef TRAVEL_EXTRA_XYJERK
      if (dist.e <= 0) {
        max_j.x += TRAVEL_EXTRA_XYJERK;
        max_j.y += TRAVEL_EXTRA_XYJERK;
      }
    #endif

    #if ENABLED(LIN_ADVANCE)
      // Advance affects E_AXIS speed and therefore jerk. Add a speed correction whenever
      // LA is turned OFF. No correction is applied when LA is turned ON (because it didn't
      // perform well; it takes more time/effort to push/melt filament than the reverse).
      static uint32_t previous_advance_rate;
      static float previous_e_mm_per_step;
      if (dist.e < 0 && previous_advance_rate) {
        // Retract move after a segment with LA that ended with an E speed decrease.
        // Correct for this to allow a faster junction speed. Since the decrease always helps to
        // get E to nominal retract speed, the equation simplifies to an increase in max jerk.
        max_j.e += previous_advance_rate * previous_e_mm_per_step;
      }
      // Prepare for next segment.
      previous_advance_rate = block->la_advance_rate;
      previous_e_mm_per_step = mm_per_step[E_AXIS_N(extruder)];
    #endif

    xyze_float_t speed_diff = current_speed;
    float vmax_junction;
    if (!moves_queued || UNEAR_ZERO(previous_nominal_speed)) {
      // Limited by a jerk to/from full halt.
      vmax_junction = block->nominal_speed;
    }
    else {
      // Compute the maximum velocity allowed at a joint of two successive segments.

      // The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
      // Scale per-axis velocities for the same vmax_junction.
      if (block->nominal_speed < previous_nominal_speed) {
        vmax_junction = block->nominal_speed;
        const float previous_scale = vmax_junction / previous_nominal_speed;
        LOOP_LOGICAL_AXES(i) speed_diff[i] -= previous_speed[i] * previous_scale;
      }
      else {
        vmax_junction = previous_nominal_speed;
        const float current_scale = vmax_junction / block->nominal_speed;
        LOOP_LOGICAL_AXES(i) speed_diff[i] = speed_diff[i] * current_scale - previous_speed[i];
      }
    }

    // Now limit the jerk in all axes.
    float v_factor = 1.0f;
    LOOP_LOGICAL_AXES(i) {
      // Jerk is the per-axis velocity difference.
      const float jerk = ABS(speed_diff[i]), maxj = max_j[i];
      if (jerk * v_factor > maxj) v_factor = maxj / jerk;
    }
    vmax_junction_sqr = sq(vmax_junction * v_factor);

  #endif // CLASSIC_JERK

  // High acceleration limits override low jerk/junction deviation limits (as fixing trapezoids
  // or reducing acceleration introduces too much complexity and/or too much compute)
  NOLESS(vmax_junction_sqr, minimum_planner_speed_sqr);

  // Max entry speed of this block equals the max exit speed of the previous block.
  block->max_entry_speed_sqr = vmax_junction_sqr;
  // Set entry speed. The reverse and forward passes will optimize it later.
  block->entry_speed_sqr = minimum_planner_speed_sqr;
  // Set min entry speed. Rarely it could be higher than the previous nominal speed but that's ok.
  block->min_entry_speed_sqr = minimum_planner_speed_sqr;
  // Zero the initial_rate to indicate that calculate_trapezoid_for_block() hasn't been called yet.
  block->initial_rate = 0;

  block->flag.recalculate = true;

  // Update previous path unit_vector and nominal speed
  previous_speed = current_speed;
  previous_nominal_speed = block->nominal_speed;

  position = target;  // Update the position

  #if ENABLED(POWER_LOSS_RECOVERY)
    block->sdpos = recovery.command_sdpos();
    block->start_position = position_float.asLogical();
  #endif

  TERN_(HAS_POSITION_FLOAT, position_float = target_float);
  TERN_(GRADIENT_MIX, mixer.gradient_control(target_float.z));

  return true;        // Movement was accepted

} // _populate_block()

/**
 * @brief Add a block to the buffer that just updates the position.
 * @details Supports LASER_SYNCHRONOUS_M106_M107 and LASER_POWER_SYNC power sync block buffer queueing.
 *
 * @param sync_flag  The sync flag to set, determining the type of sync the block will do
 */
void Planner::buffer_sync_block(const BlockFlagBit sync_flag/*=BLOCK_BIT_SYNC_POSITION*/) {

  // Wait for the next available block
  uint8_t next_buffer_head;
  block_t * const block = get_next_free_block(next_buffer_head);

  // Clear block
  block->reset();
  block->flag.apply(sync_flag);

  block->position = position;

  #if ENABLED(BACKLASH_COMPENSATION)
    LOOP_NUM_AXES(axis) block->position[axis] += backlash.get_applied_steps((AxisEnum)axis);
  #endif

  #if ENABLED(LASER_SYNCHRONOUS_M106_M107)
    FANS_LOOP(i) block->fan_speed[i] = thermalManager.fan_speed[i];
  #endif

  /**
   * M3-based power setting can be processed inline with a laser power sync block.
   * During active moves cutter.power is processed immediately, otherwise on the next move.
   */
  TERN_(LASER_POWER_SYNC, block->laser.power = cutter.power);

  // If this is the first added movement, reload the delay, otherwise, cancel it.
  if (block_buffer_head == block_buffer_tail) {
    // If it was the first queued block, restart the 1st block delivery delay, to
    // give the planner an opportunity to queue more movements and plan them
    // As there are no queued movements, the Stepper ISR will not touch this
    // variable, so there is no risk setting this here (but it MUST be done
    // before the following line!!)
    delay_before_delivering = TERN_(FT_MOTION, ftMotion.cfg.active ? BLOCK_DELAY_NONE :) BLOCK_DELAY_FOR_1ST_MOVE;
  }

  block_buffer_head = next_buffer_head;

  stepper.wake_up();
} // buffer_sync_block()

/**
 * @brief Add a single linear movement.
 * @details Add a new linear movement to the buffer in axis units.
 *          Leveling and kinematics should be applied before calling this.
 *
 * @param abce          Target position in mm and/or degrees
 * @param cart_dist_mm  The pre-calculated move lengths for all axes, in mm
 * @param fr_mm_s       (Target) speed of the move
 * @param extruder      Optional target extruder (otherwise active_extruder)
 * @param hints         Optional parameters to aid planner calculations
 *
 * @return  false if no segment was queued due to cleaning, cold extrusion, full queue, etc.
 */
bool Planner::buffer_segment(const abce_pos_t &abce
  OPTARG(HAS_DIST_MM_ARG, const xyze_float_t &cart_dist_mm)
  , const_feedRate_t fr_mm_s
  , const uint8_t extruder/*=active_extruder*/
  , const PlannerHints &hints/*=PlannerHints()*/
) {

  // If we are cleaning, do not accept queuing of movements
  if (cleaning_buffer_counter) return false;

  // When changing extruders recalculate steps corresponding to the E position
  #if ENABLED(DISTINCT_E_FACTORS)
    if (last_extruder != extruder && settings.axis_steps_per_mm[E_AXIS_N(extruder)] != settings.axis_steps_per_mm[E_AXIS_N(last_extruder)]) {
      position.e = LROUND(position.e * settings.axis_steps_per_mm[E_AXIS_N(extruder)] * mm_per_step[E_AXIS_N(last_extruder)]);
      last_extruder = extruder;
    }
  #endif

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  const abce_long_t target = {
     LOGICAL_AXIS_LIST(
      int32_t(LROUND(abce.e * settings.axis_steps_per_mm[E_AXIS_N(extruder)])),
      int32_t(LROUND(abce.a * settings.axis_steps_per_mm[A_AXIS])),
      int32_t(LROUND(abce.b * settings.axis_steps_per_mm[B_AXIS])),
      int32_t(LROUND(abce.c * settings.axis_steps_per_mm[C_AXIS])),
      int32_t(LROUND(abce.i * settings.axis_steps_per_mm[I_AXIS])),
      int32_t(LROUND(abce.j * settings.axis_steps_per_mm[J_AXIS])),
      int32_t(LROUND(abce.k * settings.axis_steps_per_mm[K_AXIS])),
      int32_t(LROUND(abce.u * settings.axis_steps_per_mm[U_AXIS])),
      int32_t(LROUND(abce.v * settings.axis_steps_per_mm[V_AXIS])),
      int32_t(LROUND(abce.w * settings.axis_steps_per_mm[W_AXIS]))
    )
  };

  #if HAS_POSITION_FLOAT
    const xyze_pos_t target_float = abce;
  #endif

  #if HAS_EXTRUDERS
    // DRYRUN prevents E moves from taking place
    if (DEBUGGING(DRYRUN) || TERN0(CANCEL_OBJECTS, cancelable.state.skipping)) {
      position.e = target.e;
      TERN_(HAS_POSITION_FLOAT, position_float.e = abce.e);
    }
  #endif

  /* <-- add a slash to enable
    SERIAL_ECHOPGM("  buffer_segment FR:", fr_mm_s);
    #if IS_KINEMATIC
      SERIAL_ECHOPGM(" A:", abce.a, " (", position.a, "->", target.a, ") B:", abce.b);
    #else
      SERIAL_ECHOPGM_P(SP_X_LBL, abce.a);
      SERIAL_ECHOPGM(" (", position.x, "->", target.x);
      SERIAL_CHAR(')');
      SERIAL_ECHOPGM_P(SP_Y_LBL, abce.b);
    #endif
    SERIAL_ECHOPGM(" (", position.y, "->", target.y);
    #if HAS_Z_AXIS
      #if ENABLED(DELTA)
        SERIAL_ECHOPGM(") C:", abce.c);
      #else
        SERIAL_CHAR(')');
        SERIAL_ECHOPGM_P(SP_Z_LBL, abce.c);
      #endif
      SERIAL_ECHOPGM(" (", position.z, "->", target.z);
      SERIAL_CHAR(')');
    #endif
    #if HAS_I_AXIS
      SERIAL_ECHOPGM_P(SP_I_LBL, abce.i);
      SERIAL_ECHOPGM(" (", position.i, "->", target.i);
      SERIAL_CHAR(')');
    #endif
    #if HAS_J_AXIS
      SERIAL_ECHOPGM_P(SP_J_LBL, abce.j);
      SERIAL_ECHOPGM(" (", position.j, "->", target.j);
      SERIAL_CHAR(')');
    #endif
    #if HAS_K_AXIS
      SERIAL_ECHOPGM_P(SP_K_LBL, abce.k);
      SERIAL_ECHOPGM(" (", position.k, "->", target.k);
      SERIAL_CHAR(')');
    #endif
    #if HAS_U_AXIS
      SERIAL_ECHOPGM_P(SP_U_LBL, abce.u);
      SERIAL_ECHOPGM(" (", position.u, "->", target.u);
      SERIAL_CHAR(')');
    #endif
    #if HAS_V_AXIS
      SERIAL_ECHOPGM_P(SP_V_LBL, abce.v);
      SERIAL_ECHOPGM(" (", position.v, "->", target.v);
      SERIAL_CHAR(')');
    #endif
    #if HAS_W_AXIS
      SERIAL_ECHOPGM_P(SP_W_LBL, abce.w);
      SERIAL_ECHOPGM(" (", position.w, "->", target.w);
      SERIAL_CHAR(')');
    #endif
    #if HAS_EXTRUDERS
      SERIAL_ECHOPGM_P(SP_E_LBL, abce.e);
      SERIAL_ECHOLNPGM(" (", position.e, "->", target.e, ")");
    #else
      SERIAL_EOL();
    #endif
  //*/

  // Queue the movement. Return 'false' if the move was not queued.
  if (!_buffer_steps(target
      OPTARG(HAS_POSITION_FLOAT, target_float)
      OPTARG(HAS_DIST_MM_ARG, cart_dist_mm)
      , fr_mm_s, extruder, hints
  )) return false;

  stepper.wake_up();
  return true;
} // buffer_segment()

/**
 * @brief Add a new linear movement to the buffer.
 * @details The target is cartesian. It's translated to
 *          delta/scara if needed.
 *
 * @param cart      Target position in mm or degrees
 * @param fr_mm_s   (Target) speed of the move (mm/s)
 * @param extruder  Optional target extruder (otherwise active_extruder)
 * @param hints     Optional parameters to aid planner calculations
 */
bool Planner::buffer_line(const xyze_pos_t &cart, const_feedRate_t fr_mm_s
  , const uint8_t extruder/*=active_extruder*/
  , const PlannerHints &hints/*=PlannerHints()*/
) {
  xyze_pos_t machine = cart;
  TERN_(HAS_POSITION_MODIFIERS, apply_modifiers(machine));

  #if IS_KINEMATIC

    #if HAS_JUNCTION_DEVIATION
      const xyze_pos_t cart_dist_mm = LOGICAL_AXIS_ARRAY(
        cart.e - position_cart.e,
        cart.x - position_cart.x, cart.y - position_cart.y, cart.z - position_cart.z,
        cart.i - position_cart.i, cart.j - position_cart.j, cart.k - position_cart.k,
        cart.u - position_cart.u, cart.v - position_cart.v, cart.w - position_cart.w
      );
    #else
      const xyz_pos_t cart_dist_mm = NUM_AXIS_ARRAY(
        cart.x - position_cart.x, cart.y - position_cart.y, cart.z - position_cart.z,
        cart.i - position_cart.i, cart.j - position_cart.j, cart.k - position_cart.k,
        cart.u - position_cart.u, cart.v - position_cart.v, cart.w - position_cart.w
      );
    #endif

    // Cartesian XYZ to kinematic ABC, stored in global 'delta'
    inverse_kinematics(machine);

    PlannerHints ph = hints;
    if (!hints.millimeters)
      ph.millimeters = get_move_distance(xyze_pos_t(cart_dist_mm) OPTARG(HAS_ROTATIONAL_AXES, ph.cartesian_move));

    #if DISABLED(FEEDRATE_SCALING)

      const feedRate_t feedrate = fr_mm_s;

    #elif IS_SCARA

      // For SCARA scale the feedrate from mm/s to degrees/s
      // i.e., Complete the angular vector in the given time.
      const float duration_recip = hints.inv_duration ?: fr_mm_s / ph.millimeters;
      const xyz_pos_t diff = delta - position_float;
      const feedRate_t feedrate = diff.magnitude() * duration_recip;

    #elif ENABLED(POLAR)

      /**
       * Motion problem for Polar axis near center / origin:
       *
       * 3D printing:
       * Movements very close to the center of the polar axis take more time than others.
       * This brief delay results in more material deposition due to the pressure in the nozzle.
       *
       * Current Kinematics and feedrate scaling deals with this by making the movement as fast
       * as possible. It works for slow movements but doesn't work well with fast ones. A more
       * complicated extrusion compensation must be implemented.
       *
       * Ideally, it should estimate that a long rotation near the center is ahead and will cause
       * unwanted deposition. Therefore it can compensate the extrusion beforehand.
       *
       * Laser cutting:
       * Same thing would be a problem for laser engraving too. As it spends time rotating at the
       * center point, more likely it will burn more material than it should. Therefore similar
       * compensation would be implemented for laser-cutting operations.
       *
       * Milling:
       * This shouldn't be a problem for cutting/milling operations.
       */
      feedRate_t calculated_feedrate = fr_mm_s;
      const xyz_pos_t diff = delta - position_float;
      if (!NEAR_ZERO(diff.b)) {
        if (delta.a <= POLAR_FAST_RADIUS )
          calculated_feedrate = settings.max_feedrate_mm_s[Y_AXIS];
        else {
          // Normalized vector of movement
          const float diffBLength = ABS((2.0f * M_PI * diff.a) * (diff.b / 360.0f)),
                      diffTheta = DEGREES(ATAN2(diff.a, diffBLength)),
                      normalizedTheta = 1.0f - (ABS(diffTheta > 90.0f ? 180.0f - diffTheta : diffTheta) / 90.0f);

          // Normalized position along the radius
          const float radiusRatio = (PRINTABLE_RADIUS) / delta.a;
          calculated_feedrate += (fr_mm_s * radiusRatio * normalizedTheta);
        }
      }
      const feedRate_t feedrate = calculated_feedrate;

    #endif // POLAR && FEEDRATE_SCALING

    TERN_(HAS_EXTRUDERS, delta.e = machine.e);
    if (buffer_segment(delta OPTARG(HAS_DIST_MM_ARG, cart_dist_mm), feedrate, extruder, ph)) {
      position_cart = cart;
      return true;
    }
    return false;

  #else // !IS_KINEMATIC

    return buffer_segment(machine, fr_mm_s, extruder, hints);

  #endif

} // buffer_line()

#if ENABLED(DIRECT_STEPPING)

  void Planner::buffer_page(const page_idx_t page_idx, const uint8_t extruder, const uint16_t num_steps) {
    if (!last_page_step_rate) {
      kill(GET_TEXT_F(MSG_BAD_PAGE_SPEED));
      return;
    }

    uint8_t next_buffer_head;
    block_t * const block = get_next_free_block(next_buffer_head);

    block->flag.reset(BLOCK_BIT_PAGE);

    #if HAS_FAN
      FANS_LOOP(i) block->fan_speed[i] = thermalManager.fan_speed[i];
    #endif

    E_TERN_(block->extruder = extruder);

    block->page_idx = page_idx;

    block->step_event_count = num_steps;
    block->initial_rate = block->final_rate = block->nominal_rate = last_page_step_rate; // steps/s

    block->accelerate_before = 0;
    block->decelerate_start = block->step_event_count;

    // Will be set to last direction later if directional format.
    block->direction_bits.reset();

    if (!DirectStepping::Config::DIRECTIONAL) {
      #define PAGE_UPDATE_DIR(AXIS) do{ if (last_page_dir.AXIS) block->direction_bits.AXIS = true; }while(0);
      LOGICAL_AXIS_MAP(PAGE_UPDATE_DIR);
    }

    // If this is the first added movement, reload the delay, otherwise, cancel it.
    if (block_buffer_head == block_buffer_tail) {
      // If it was the first queued block, restart the 1st block delivery delay, to
      // give the planner an opportunity to queue more movements and plan them
      // As there are no queued movements, the Stepper ISR will not touch this
      // variable, so there is no risk setting this here (but it MUST be done
      // before the following line!!)
      delay_before_delivering = TERN_(FT_MOTION, ftMotion.cfg.active ? BLOCK_DELAY_NONE :) BLOCK_DELAY_FOR_1ST_MOVE;
    }

    // Move buffer head
    block_buffer_head = next_buffer_head;

    stepper.enable_all_steppers();
    stepper.wake_up();
  }

#endif // DIRECT_STEPPING

/**
 * Directly set the planner ABCE position (and stepper positions)
 * converting mm (or angles for SCARA) into steps.
 *
 * The provided ABCE position is in machine units.
 */
void Planner::set_machine_position_mm(const abce_pos_t &abce) {

  // When FT Motion is enabled, call synchronize() here instead of generating a sync block
  if (TERN0(FT_MOTION, ftMotion.cfg.active)) synchronize();

  TERN_(DISTINCT_E_FACTORS, last_extruder = active_extruder);
  TERN_(HAS_POSITION_FLOAT, position_float = abce);
  position.set(
    LOGICAL_AXIS_LIST(
      LROUND(abce.e * settings.axis_steps_per_mm[E_AXIS_N(active_extruder)]),
      LROUND(abce.a * settings.axis_steps_per_mm[A_AXIS]),
      LROUND(abce.b * settings.axis_steps_per_mm[B_AXIS]),
      LROUND(abce.c * settings.axis_steps_per_mm[C_AXIS]),
      LROUND(abce.i * settings.axis_steps_per_mm[I_AXIS]),
      LROUND(abce.j * settings.axis_steps_per_mm[J_AXIS]),
      LROUND(abce.k * settings.axis_steps_per_mm[K_AXIS]),
      LROUND(abce.u * settings.axis_steps_per_mm[U_AXIS]),
      LROUND(abce.v * settings.axis_steps_per_mm[V_AXIS]),
      LROUND(abce.w * settings.axis_steps_per_mm[W_AXIS])
    )
  );

  if (has_blocks_queued()) {
    //previous_nominal_speed = 0.0f; // Reset planner junction speeds. Assume start from rest.
    //previous_speed.reset();
    buffer_sync_block(BLOCK_BIT_SYNC_POSITION);
  }
  else {
    #if ENABLED(BACKLASH_COMPENSATION)
      abce_long_t stepper_pos = position;
      LOOP_NUM_AXES(axis) stepper_pos[axis] += backlash.get_applied_steps((AxisEnum)axis);
      stepper.set_position(stepper_pos);
    #else
      stepper.set_position(position);
    #endif
  }
}

/**
 * Set the machine positions in millimeters (soon) given the native position.
 * Synchonized with the planner queue.
 *
 *   - Modifiers are applied for skew, leveling, retract, etc.
 *   - Kinematics are applied to remap cartesian positions to stepper positions.
 *   - The resulting stepper positions are synchronized at the end of the planner queue.
 */
void Planner::set_position_mm(const xyze_pos_t &xyze) {
  xyze_pos_t machine = xyze;
  TERN_(HAS_POSITION_MODIFIERS, apply_modifiers(machine, true));
  #if IS_KINEMATIC
    position_cart = xyze;
    inverse_kinematics(machine);
    TERN_(HAS_EXTRUDERS, delta.e = machine.e);
    set_machine_position_mm(delta);
  #else
    set_machine_position_mm(machine);
  #endif
}

#if HAS_EXTRUDERS

  /**
   * Special setter for planner E position (also setting E stepper position).
   */
  void Planner::set_e_position_mm(const_float_t e) {
    const uint8_t axis_index = E_AXIS_N(active_extruder);
    TERN_(DISTINCT_E_FACTORS, last_extruder = active_extruder);

    // Unapply the current retraction before (immediately) setting the planner position
    const float e_new = DIFF_TERN(FWRETRACT, e, fwretract.current_retract[active_extruder]);
    position.e = LROUND(settings.axis_steps_per_mm[axis_index] * e_new);
    TERN_(HAS_POSITION_FLOAT, position_float.e = e_new);
    TERN_(IS_KINEMATIC, TERN_(HAS_EXTRUDERS, position_cart.e = e));

    if (has_blocks_queued())
      buffer_sync_block(BLOCK_BIT_SYNC_POSITION);
    else
      stepper.set_e_position(position.e);
  }

#endif // HAS_EXTRUDERS

/**
 * Recalculate steps/s^2 acceleration rates based on mm/s^2 acceleration rates
 */
void Planner::refresh_acceleration_rates() {
  uint32_t highest_rate = 1;
  LOOP_DISTINCT_AXES(i) {
    max_acceleration_steps_per_s2[i] = settings.max_acceleration_mm_per_s2[i] * settings.axis_steps_per_mm[i];
    if (TERN1(DISTINCT_E_FACTORS, i < E_AXIS || i == E_AXIS_N(active_extruder)))
      NOLESS(highest_rate, max_acceleration_steps_per_s2[i]);
  }
  acceleration_long_cutoff = 4294967295UL / highest_rate; // 0xFFFFFFFFUL
  TERN_(HAS_LINEAR_E_JERK, recalculate_max_e_jerk());
}

/**
 * Recalculate 'position' and 'mm_per_step'.
 * Must be called whenever settings.axis_steps_per_mm changes!
 */
void Planner::refresh_positioning() {
  #if ENABLED(EDITABLE_STEPS_PER_UNIT)
    LOOP_DISTINCT_AXES(i) mm_per_step[i] = 1.0f / settings.axis_steps_per_mm[i];
  #endif
  set_position_mm(current_position);
  refresh_acceleration_rates();
}

// Apply limits to a variable and give a warning if the value was out of range
inline void limit_and_warn(float &val, const AxisEnum axis, FSTR_P const setting_name, const xyze_float_t &max_limit) {
  const uint8_t lim_axis = TERN_(HAS_EXTRUDERS, axis > E_AXIS ? E_AXIS :) axis;
  const float before = val;
  LIMIT(val, 0.1f, max_limit[lim_axis]);
  if (before != val)
    SERIAL_ECHOLN(C(AXIS_CHAR(lim_axis)), F(" Max "), setting_name, F(" limited to "), val);
}

/**
 * For the specified 'axis' set the Maximum Acceleration to the given value (mm/s^2).
 * The value may be limited with warning feedback, if configured.
 * Calls refresh_acceleration_rates to precalculate planner terms in steps.
 *
 * This hard limit is applied as a block is being added to the planner queue.
 */
void Planner::set_max_acceleration(const AxisEnum axis, float inMaxAccelMMS2) {
  #if ENABLED(LIMITED_MAX_ACCEL_EDITING)
    #ifdef MAX_ACCEL_EDIT_VALUES
      constexpr xyze_float_t max_accel_edit = MAX_ACCEL_EDIT_VALUES;
      const xyze_float_t &max_acc_edit_scaled = max_accel_edit;
    #else
      constexpr xyze_float_t max_accel_edit = DEFAULT_MAX_ACCELERATION;
      const xyze_float_t max_acc_edit_scaled = max_accel_edit * 2;
    #endif
    limit_and_warn(inMaxAccelMMS2, axis, F("Acceleration"), max_acc_edit_scaled);
  #endif
  settings.max_acceleration_mm_per_s2[axis] = inMaxAccelMMS2;

  // Update steps per s2 to agree with the units per s2 (since they are used in the planner)
  refresh_acceleration_rates();
}

/**
 * For the specified 'axis' set the Maximum Feedrate to the given value (mm/s).
 * The value may be limited with warning feedback, if configured.
 *
 * This hard limit is applied as a block is being added to the planner queue.
 */
void Planner::set_max_feedrate(const AxisEnum axis, float inMaxFeedrateMMS) {
  #if ENABLED(LIMITED_MAX_FR_EDITING)
    #ifdef MAX_FEEDRATE_EDIT_VALUES
      constexpr xyze_float_t max_fr_edit = MAX_FEEDRATE_EDIT_VALUES;
      const xyze_float_t &max_fr_edit_scaled = max_fr_edit;
    #else
      constexpr xyze_float_t max_fr_edit = DEFAULT_MAX_FEEDRATE;
      const xyze_float_t max_fr_edit_scaled = max_fr_edit * 2;
    #endif
    limit_and_warn(inMaxFeedrateMMS, axis, F("Feedrate"), max_fr_edit_scaled);
  #endif
  settings.max_feedrate_mm_s[axis] = inMaxFeedrateMMS;
}

#if ENABLED(CLASSIC_JERK)

  /**
   * For the specified 'axis' set the Maximum Jerk (instant change) to the given value (mm/s).
   * The value may be limited with warning feedback, if configured.
   *
   * This hard limit is applied (to the block start speed) as the block is being added to the planner queue.
   */
  void Planner::set_max_jerk(const AxisEnum axis, float inMaxJerkMMS) {
    #if ENABLED(LIMITED_JERK_EDITING)
      constexpr xyze_float_t max_jerk_edit =
        #ifdef MAX_JERK_EDIT_VALUES
          MAX_JERK_EDIT_VALUES
        #else
          LOGICAL_AXIS_ARRAY(
            (DEFAULT_EJERK) * 2,
            (DEFAULT_XJERK) * 2, (DEFAULT_YJERK) * 2, (DEFAULT_ZJERK) * 2,
            (DEFAULT_IJERK) * 2, (DEFAULT_JJERK) * 2, (DEFAULT_KJERK) * 2,
            (DEFAULT_UJERK) * 2, (DEFAULT_VJERK) * 2, (DEFAULT_WJERK) * 2
          )
        #endif
      ;
      limit_and_warn(inMaxJerkMMS, axis, F("Jerk"), max_jerk_edit);
    #endif
    max_jerk[axis] = inMaxJerkMMS;
  }

#endif

#if HAS_WIRED_LCD

  uint16_t Planner::block_buffer_runtime() {
    #ifdef __AVR__
      // Protect the access to the variable. Only required for AVR, as
      // any 32bit CPU offers atomic access to 32bit variables
      const bool was_enabled = stepper.suspend();
    #endif

    uint32_t bbru = block_buffer_runtime_us;

    #ifdef __AVR__
      // Reenable Stepper ISR
      if (was_enabled) stepper.wake_up();
    #endif

    // To translate µs to ms a division by 1000 would be required.
    // We introduce 2.4% error here by dividing by 1024.
    // Doesn't matter because block_buffer_runtime_us is already too small an estimation.
    bbru >>= 10;
    // limit to about a minute.
    return _MIN(bbru, 0x0000FFFFUL);
  }

  void Planner::clear_block_buffer_runtime() {
    #ifdef __AVR__
      // Protect the access to the variable. Only required for AVR, as
      // any 32bit CPU offers atomic access to 32bit variables
      const bool was_enabled = stepper.suspend();
    #endif

    block_buffer_runtime_us = 0;

    #ifdef __AVR__
      // Reenable Stepper ISR
      if (was_enabled) stepper.wake_up();
    #endif
  }

#endif
