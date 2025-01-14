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
 * parser.h - Parser for a G-Code line, providing a parameter interface.
 *           Codes like M149 control the way the G-Code parser behaves,
 *           so settings for these codes are located in this class.
 */

#include "../inc/MarlinConfig.h"

//#define DEBUG_GCODE_PARSER
#if ENABLED(DEBUG_GCODE_PARSER)
  #include "../libs/hex_print.h"
#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  typedef enum : uint8_t { TEMPUNIT_C, TEMPUNIT_K, TEMPUNIT_F } TempUnit;
#endif

#if ENABLED(INCH_MODE_SUPPORT)
  typedef enum : uint8_t { LINEARUNIT_MM, LINEARUNIT_INCH } LinearUnit;
#endif

/**
 * G-Code parser
 *
 *  - Parse a single G-code line for its letter, code, subcode, and parameters
 *  - FASTER_GCODE_PARSER:
 *    - Flags existing params (1 bit each)
 *    - Stores value offsets (1 byte each)
 *  - Provide accessors for parameters:
 *    - Parameter exists
 *    - Parameter has value
 *    - Parameter value in different units and types
 */
class GCodeParser {

private:
  static char *value_ptr;           // Set by seen, used to fetch the value

  #if ENABLED(FASTER_GCODE_PARSER)
    static uint32_t codebits;       // Parameters pre-scanned
    static uint8_t param[26];       // For A-Z, offsets into command args
  #else
    static char *command_args;      // Args start here, for slow scan
  #endif

public:

  // Global states for G-Code-level units features

  static bool volumetric_enabled;

  #if ENABLED(INCH_MODE_SUPPORT)
    static float linear_unit_factor, volumetric_unit_factor;
  #endif

  #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
    static TempUnit input_temp_units;
  #endif

  // Command line state
  static char *command_ptr,               // The command, so it can be echoed
              *string_arg,                // string of command line
              command_letter;             // G, M, or T
  static uint16_t codenum;                // 123
  #if USE_GCODE_SUBCODES
    static uint8_t subcode;               // .1
  #endif

  #if ENABLED(GCODE_MOTION_MODES)
    static int16_t motion_mode_codenum;
    #if USE_GCODE_SUBCODES
      static uint8_t motion_mode_subcode;
    #endif
    FORCE_INLINE static void cancel_motion_mode() { motion_mode_codenum = -1; }
  #endif

  #if ENABLED(DEBUG_GCODE_PARSER)
    static void debug();
  #endif

  // Reset is done before parsing
  static void reset();

  #define LETTER_BIT(N) ((N) - 'A')

  FORCE_INLINE static bool valid_signless(const char * const p) {
    return NUMERIC(p[0]) || (p[0] == '.' && NUMERIC(p[1])); // .?[0-9]
  }

  FORCE_INLINE static bool valid_float(const char * const p) {
    return valid_signless(p) || ((p[0] == '-' || p[0] == '+') && valid_signless(&p[1])); // [-+]?.?[0-9]
  }

  FORCE_INLINE static bool valid_number(const char * const p) {
    // TODO: With MARLIN_DEV_MODE allow HEX values starting with "x"
    return valid_float(p);
  }

  #if ENABLED(FASTER_GCODE_PARSER)

    FORCE_INLINE static bool valid_int(const char * const p) {
      return NUMERIC(p[0]) || ((p[0] == '-' || p[0] == '+') && NUMERIC(p[1])); // [-+]?[0-9]
    }

    // Set the flag and pointer for a parameter
    static void set(const char c, char * const ptr) {
      const uint8_t ind = LETTER_BIT(c);
      if (ind >= COUNT(param)) return;           // Only A-Z
      SBI32(codebits, ind);                      // parameter exists
      param[ind] = ptr ? ptr - command_ptr : 0;  // parameter offset or 0
      #if ENABLED(DEBUG_GCODE_PARSER)
        if (codenum == 800) {
          SERIAL_ECHOPGM("Set bit ", ind, " of codebits (", hex_address((void*)(codebits >> 16)));
          print_hex_word((uint16_t)(codebits & 0xFFFF));
          SERIAL_ECHOLNPGM(") | param = ", param[ind]);
        }
      #endif
    }

    // Code seen bit was set. If not found, value_ptr is unchanged.
    // This allows "if (seen('A')||seen('B'))" to use the last-found value.
    static bool seen(const char c) {
      const uint8_t ind = LETTER_BIT(c);
      if (ind >= COUNT(param)) return false; // Only A-Z
      const bool b = TEST32(codebits, ind);
      if (b) {
        if (param[ind]) {
          char * const ptr = command_ptr + param[ind];
          value_ptr = valid_number(ptr) ? ptr : nullptr;
        }
        else
          value_ptr = nullptr;
      }
      return b;
    }

    FORCE_INLINE static constexpr uint32_t letter_bits(const char * const str) {
      return  (str[0] ? _BV32(LETTER_BIT(str[0])) |
              (str[1] ? _BV32(LETTER_BIT(str[1])) |
              (str[2] ? _BV32(LETTER_BIT(str[2])) |
              (str[3] ? _BV32(LETTER_BIT(str[3])) |
              (str[4] ? _BV32(LETTER_BIT(str[4])) |
              (str[5] ? _BV32(LETTER_BIT(str[5])) |
              (str[6] ? _BV32(LETTER_BIT(str[6])) |
              (str[7] ? _BV32(LETTER_BIT(str[7])) |
              (str[8] ? _BV32(LETTER_BIT(str[8])) |
              (str[9] ? _BV32(LETTER_BIT(str[9]))
            : 0) : 0) : 0) : 0) : 0) : 0) : 0) : 0) : 0) : 0);
    }

    // At least one of a list of code letters was seen
    #ifdef CPU_32_BIT
      FORCE_INLINE static bool seen(const char * const str) { return !!(codebits & letter_bits(str)); }
    #else
      FORCE_INLINE static bool seen(const char * const str) {
        const uint32_t letrbits = letter_bits(str);
        const uint8_t * const cb = (uint8_t*)&codebits;
        const uint8_t * const lb = (uint8_t*)&letrbits;
        return (cb[0] & lb[0]) || (cb[1] & lb[1]) || (cb[2] & lb[2]) || (cb[3] & lb[3]);
      }
    #endif

    static bool seen_any() { return !!codebits; }

    FORCE_INLINE static bool seen_test(const char c) { return TEST32(codebits, LETTER_BIT(c)); }

  #else // !FASTER_GCODE_PARSER

    #if ENABLED(GCODE_CASE_INSENSITIVE)
      FORCE_INLINE static char* strgchr(char *p, char g) {
        auto uppercase = [](char c) {
          return TERN0(GCODE_CASE_INSENSITIVE, WITHIN(c, 'a', 'z')) ? c + 'A' - 'a' : c;
        };
        const char d = uppercase(g);
        for (char cc; (cc = uppercase(*p)); p++) if (cc == d) return p;
        return nullptr;
      }
    #else
      #define strgchr strchr
    #endif

    // Code is found in the string. If not found, value_ptr is unchanged.
    // This allows "if (seen('A')||seen('B'))" to use the last-found value.
    static bool seen(const char c) {
      char *p = strgchr(command_args, c);
      const bool b = !!p;
      if (b) value_ptr = valid_number(&p[1]) ? &p[1] : nullptr;
      return b;
    }

    static bool seen_any() { return *command_args == '\0'; }

    FORCE_INLINE static bool seen_test(const char c) { return (bool)strgchr(command_args, c); }

    // At least one of a list of code letters was seen
    static bool seen(const char * const str) {
      for (uint8_t i = 0; const char c = str[i]; i++)
        if (seen_test(c)) return true;
      return false;
    }

  #endif // !FASTER_GCODE_PARSER

  // Seen any axis parameter
  static bool seen_axis() { return seen(STR_AXES_LOGICAL); }

  #if ENABLED(GCODE_QUOTED_STRINGS)
    static char* unescape_string(char* &src);
  #else
    FORCE_INLINE static char* unescape_string(char* &src) { return src; }
  #endif

  // Populate all fields by parsing a single line of G-Code
  // This uses 54 bytes of SRAM to speed up seen/value
  static void parse(char * p);

  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    // Parse the next parameter as a new command
    static bool chain();
  #endif

  // Test whether the parsed command matches the input
  static bool is_command(const char ltr, const uint16_t num) { return command_letter == ltr && codenum == num; }

  // The code value pointer was set
  FORCE_INLINE static bool has_value() { return !!value_ptr; }

  // Seen a parameter with a value
  static bool seenval(const char c) { return seen(c) && has_value(); }

  // The value as a string
  static char* value_string() { return value_ptr; }

  // Float removes 'E' to prevent scientific notation interpretation
  static float value_float() {
    if (!value_ptr) return 0;
    char *e = value_ptr;
    for (;;) {
      const char c = *e;
      if (c == '\0' || c == ' ') break;
      if (c == 'E' || c == 'e' || c == 'X' || c == 'x') {
        *e = '\0';
        const float ret = strtof(value_ptr, nullptr);
        *e = c;
        return ret;
      }
      ++e;
    }
    return strtof(value_ptr, nullptr);
  }

  // Code value as a long or ulong
  static int32_t value_long() { return value_ptr ? strtol(value_ptr, nullptr, 10) : 0L; }
  static uint32_t value_ulong() { return value_ptr ? strtoul(value_ptr, nullptr, 10) : 0UL; }

  // Code value for use as time
  static millis_t value_millis() { return value_ulong(); }
  static millis_t value_millis_from_seconds() { return (millis_t)SEC_TO_MS(value_float()); }

  // Reduce to fewer bits
  static int16_t value_int() { return (int16_t)value_long(); }
  static uint16_t value_ushort() { return (uint16_t)value_long(); }
  static uint8_t value_byte() { return (uint8_t)constrain(value_long(), 0, 255); }

  // Bool is true with no value or non-zero
  static bool value_bool() { return !has_value() || !!value_byte(); }

  static constexpr bool axis_is_rotational(const AxisEnum axis) {
    return (false
      || TERN0(AXIS4_ROTATES, axis == I_AXIS)
      || TERN0(AXIS5_ROTATES, axis == J_AXIS)
      || TERN0(AXIS6_ROTATES, axis == K_AXIS)
      || TERN0(AXIS7_ROTATES, axis == U_AXIS)
      || TERN0(AXIS8_ROTATES, axis == V_AXIS)
      || TERN0(AXIS9_ROTATES, axis == W_AXIS)
    );
  }

  // Units modes: Inches, Fahrenheit, Kelvin

  #if ENABLED(INCH_MODE_SUPPORT)
    static float mm_to_linear_unit(const_float_t mm)     { return mm / linear_unit_factor; }
    static float mm_to_volumetric_unit(const_float_t mm) { return mm / (volumetric_enabled ? volumetric_unit_factor : linear_unit_factor); }

    // Init linear units by constructor
    GCodeParser() { set_input_linear_units(LINEARUNIT_MM); }

    static void set_input_linear_units(const LinearUnit units) {
      switch (units) {
        default:
        case LINEARUNIT_MM:   linear_unit_factor =  1.0f; break;
        case LINEARUNIT_INCH: linear_unit_factor = 25.4f; break;
      }
      volumetric_unit_factor = POW(linear_unit_factor, 3);
    }

    static float axis_unit_factor(const AxisEnum axis) {
      if (axis_is_rotational(axis)) return 1.0f;
      #if HAS_EXTRUDERS
        if (axis >= E_AXIS && volumetric_enabled) return volumetric_unit_factor;
      #endif
      return linear_unit_factor;
    }

    static float linear_value_to_mm(const_float_t v)                  { return v * linear_unit_factor; }
    static float axis_value_to_mm(const AxisEnum axis, const float v) { return v * axis_unit_factor(axis); }
    static float per_axis_value(const AxisEnum axis, const float v)   { return v / axis_unit_factor(axis); }

  #else

    static constexpr float mm_to_linear_unit(const_float_t mm)     { return mm; }
    static constexpr float mm_to_volumetric_unit(const_float_t mm) { return mm; }

    static constexpr float linear_value_to_mm(const_float_t v)             { return v; }
    static constexpr float axis_value_to_mm(const AxisEnum, const float v) { return v; }
    static constexpr float per_axis_value(const AxisEnum, const float v)   { return v; }

  #endif

  static bool using_inch_units() { return mm_to_linear_unit(1.0f) != 1.0f; }

  #define IN_TO_MM(I)        ((I) * 25.4f)
  #define MM_TO_IN(M)        ((M) / 25.4f)
  #define LINEAR_UNIT(V)     parser.mm_to_linear_unit(V)
  #define VOLUMETRIC_UNIT(V) parser.mm_to_volumetric_unit(V)

  #define X_AXIS_UNIT LINEAR_UNIT
  #define Y_AXIS_UNIT LINEAR_UNIT
  #define Z_AXIS_UNIT LINEAR_UNIT
  #define I_AXIS_UNIT(V) TERN(AXIS4_ROTATES, (V), LINEAR_UNIT(V))
  #define J_AXIS_UNIT(V) TERN(AXIS5_ROTATES, (V), LINEAR_UNIT(V))
  #define K_AXIS_UNIT(V) TERN(AXIS6_ROTATES, (V), LINEAR_UNIT(V))
  #define U_AXIS_UNIT(V) TERN(AXIS7_ROTATES, (V), LINEAR_UNIT(V))
  #define V_AXIS_UNIT(V) TERN(AXIS8_ROTATES, (V), LINEAR_UNIT(V))
  #define W_AXIS_UNIT(V) TERN(AXIS9_ROTATES, (V), LINEAR_UNIT(V))

  static float value_linear_units()                      { return linear_value_to_mm(value_float()); }
  static float value_axis_units(const AxisEnum axis)     { return axis_value_to_mm(axis, value_float()); }
  static float value_per_axis_units(const AxisEnum axis) { return per_axis_value(axis, value_float()); }

  #if ENABLED(TEMPERATURE_UNITS_SUPPORT)

    static void set_input_temp_units(const TempUnit units) { input_temp_units = units; }

    static char temp_units_code() {
      return input_temp_units == TEMPUNIT_K ? 'K' : input_temp_units == TEMPUNIT_F ? 'F' : 'C';
    }
    static FSTR_P temp_units_name() {
      return input_temp_units == TEMPUNIT_K ? F("Kelvin") : input_temp_units == TEMPUNIT_F ? F("Fahrenheit") : F("Celsius");
    }

    #if HAS_MARLINUI_MENU && DISABLED(DISABLE_M503)

      static float to_temp_units(celsius_t c) {
        switch (input_temp_units) {
          default:
          case TEMPUNIT_C: return c;
          case TEMPUNIT_K: return c + 273.15f;
          case TEMPUNIT_F: return c * 0.5555555556f + 32;
        }
      }

    #endif // HAS_MARLINUI_MENU && !DISABLE_M503

    static celsius_t value_celsius() {
      float f = value_float();
      switch (input_temp_units) {
        default:
        case TEMPUNIT_C: break;
        case TEMPUNIT_K: f -= 273.15f;
        case TEMPUNIT_F: f = (f - 32) * 0.5555555556f;
      }
      return LROUND(f);
    }

    static celsius_t value_celsius_diff() {
      float f = value_float();
      switch (input_temp_units) {
        default:
        case TEMPUNIT_C:
        case TEMPUNIT_K: break;
        case TEMPUNIT_F: f *= 0.5555555556f;
      }
      return LROUND(f);
    }

  #else // !TEMPERATURE_UNITS_SUPPORT

    static constexpr float to_temp_units(int16_t c) { return (float)c; }

    static celsius_t value_celsius()      { return value_int(); }
    static celsius_t value_celsius_diff() { return value_int(); }

  #endif // !TEMPERATURE_UNITS_SUPPORT

  static feedRate_t value_feedrate() { return MMM_TO_MMS(value_linear_units()); }

  void unknown_command_warning();

  // Provide simple value accessors with default option
  static char*     stringval(const char c, char * const dval=nullptr) { return seenval(c) ? value_string()   : dval; }
  static float     floatval(const char c, const float dval=0.0)   { return seenval(c) ? value_float()        : dval; }
  static bool      boolval(const char c, const bool dval=false)   { return seenval(c) ? value_bool()         : (seen(c) ? true : dval); }
  static uint8_t   byteval(const char c, const uint8_t dval=0)    { return seenval(c) ? value_byte()         : dval; }
  static int16_t   intval(const char c, const int16_t dval=0)     { return seenval(c) ? value_int()          : dval; }
  static uint16_t  ushortval(const char c, const uint16_t dval=0) { return seenval(c) ? value_ushort()       : dval; }
  static int32_t   longval(const char c, const int32_t dval=0)    { return seenval(c) ? value_long()         : dval; }
  static uint32_t  ulongval(const char c, const uint32_t dval=0)  { return seenval(c) ? value_ulong()        : dval; }
  static float     linearval(const char c, const float dval=0)    { return seenval(c) ? value_linear_units() : dval; }
  static float     axisunitsval(const char c, const AxisEnum a, const float dval=0)
                                                                         { return seenval(c) ? value_axis_units(a)  : dval; }
  static celsius_t celsiusval(const char c, const celsius_t dval=0)    { return seenval(c) ? value_celsius() : dval; }
  static feedRate_t feedrateval(const char c, const feedRate_t dval=0) { return seenval(c) ? value_feedrate() : dval; }

  #if ENABLED(MARLIN_DEV_MODE)

    static uint8_t* hex_adr_val(const char c, uint8_t * const dval=nullptr) {
      if (!seen(c) || *value_ptr != 'x') return dval;
      uint8_t *out = nullptr;
      for (char *vp = value_ptr + 1; HEXCHR(*vp) >= 0; vp++)
        out = (uint8_t*)((uintptr_t(out) << 8) | HEXCHR(*vp));
      return out;
    }

    static uint16_t hex_val(const char c, uint16_t const dval=0) {
      if (!seen(c) || *value_ptr != 'x') return dval;
      uint16_t out = 0;
      for (char *vp = value_ptr + 1; HEXCHR(*vp) >= 0; vp++)
        out = ((out) << 8) | HEXCHR(*vp);
      return out;
    }

  #endif
};

extern GCodeParser parser;
