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
 * parser.cpp - Parser for a G-Code line, providing a parameter interface.
 */

#include "parser.h"

#include "../MarlinCore.h"

// Must be declared for allocation and to satisfy the linker
// Zero values need no initialization.

bool GCodeParser::volumetric_enabled;

#if ENABLED(INCH_MODE_SUPPORT)
  float GCodeParser::linear_unit_factor, GCodeParser::volumetric_unit_factor;
#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  TempUnit GCodeParser::input_temp_units = TEMPUNIT_C;
#endif

char *GCodeParser::command_ptr,
     *GCodeParser::string_arg,
     *GCodeParser::value_ptr;
char GCodeParser::command_letter;
uint16_t GCodeParser::codenum;

#if USE_GCODE_SUBCODES
  uint8_t GCodeParser::subcode;
#endif

#if ENABLED(GCODE_MOTION_MODES)
  int16_t GCodeParser::motion_mode_codenum = -1;
  #if USE_GCODE_SUBCODES
    uint8_t GCodeParser::motion_mode_subcode;
  #endif
#endif

#if ENABLED(FASTER_GCODE_PARSER)
  // Optimized Parameters
  uint32_t GCodeParser::codebits;  // found bits
  uint8_t GCodeParser::param[26];  // parameter offsets from command_ptr
#else
  char *GCodeParser::command_args; // start of parameters
#endif

// Create a global instance of the G-Code parser singleton
GCodeParser parser;

/**
 * Clear all code-seen (and value pointers)
 *
 * Since each param is set/cleared on seen codes,
 * this may be optimized by commenting out ZERO(param)
 */
void GCodeParser::reset() {
  string_arg = nullptr;                 // No whole line argument
  command_letter = '?';                 // No command letter
  codenum = 0;                          // No command code
  TERN_(USE_GCODE_SUBCODES, subcode = 0); // No command sub-code
  #if ENABLED(FASTER_GCODE_PARSER)
    codebits = 0;                       // No codes yet
    //ZERO(param);                      // No parameters (should be safe to comment out this line)
  #endif
}

#if ENABLED(GCODE_QUOTED_STRINGS)

  // Pass the address after the first quote (if any)
  char* GCodeParser::unescape_string(char* &src) {
    if (*src == '"') ++src;     // Skip the leading quote
    char * const out = src;     // Start of the string
    char *dst = src;            // Prepare to unescape and terminate
    for (;;) {
      char c = *src++;          // Get the next char
      switch (c) {
        case '\\': c = *src++; break; // Get the escaped char
        case '"' : c = '\0'; break;   // Convert bare quote to nul
      }
      if (!(*dst++ = c)) break; // Copy and break on nul
    }
    return out;
  }

#endif

/**
 * Populate the command line state (command_letter, codenum, subcode, and string_arg)
 * by parsing a single line of G-Code. 58 bytes of SRAM are used to speed up seen/value.
 */
void GCodeParser::parse(char *p) {

  reset(); // No codes to report

  auto uppercase = [](char c) {
    return TERN0(GCODE_CASE_INSENSITIVE, WITHIN(c, 'a', 'z')) ? c + 'A' - 'a' : c;
  };

  // Skip spaces
  while (*p == ' ') ++p;

  // Skip N[-0-9] if included in the command line
  if (uppercase(*p) == 'N' && NUMERIC_SIGNED(p[1])) {
    //TERN_(FASTER_GCODE_PARSER, set('N', p + 1)); // (optional) Set the 'N' parameter value
    p += 2;                  // skip N[-0-9]
    while (NUMERIC(*p)) ++p; // skip [0-9]*
    while (*p == ' ')   ++p; // skip [ ]*
  }

  // *p now points to the current command, which should be G, M, or T
  command_ptr = p;

  // Get the command letter, which must be G, M, or T
  const char letter = uppercase(*p++);

  // Nullify asterisk and trailing whitespace
  char *starpos = strchr(p, '*');
  if (starpos) {
    --starpos;                          // *
    while (*starpos == ' ') --starpos;  // spaces...
    starpos[1] = '\0';
  }

  #if ANY(MARLIN_DEV_MODE, SWITCHING_TOOLHEAD, MAGNETIC_SWITCHING_TOOLHEAD, ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
    #define SIGNED_CODENUM 1
  #endif

  /**
   * Screen for good command letters.
   * With Realtime Reporting, commands S000, P000, and R000 are allowed.
   */
  #if ENABLED(REALTIME_REPORTING_COMMANDS)
    switch (letter) {
      case 'P': case 'R' ... 'S': {
        uint8_t digits = 0;
        char *a = p;
        while (*a++ == '0') digits++; // Count up '0' characters
        if (digits == 3) {            // Three '0' digits is a good command
          codenum = 0;
          command_letter = letter;
          return;
        }
      }
    }
  #endif

  /**
   * Screen for good command letters. G, M, and T are always accepted.
   * With Motion Modes enabled any axis letter can come first.
   */
  switch (letter) {
    case 'G': case 'M': case 'T': TERN_(MARLIN_DEV_MODE, case 'D':) {
      // Skip spaces to get the numeric part
      while (*p == ' ') p++;

      #if HAS_PRUSA_MMU2 || HAS_PRUSA_MMU3
        if (letter == 'T') {
          // check for special MMU2 T?/Tx/Tc commands
          if (*p == '?' || *p == 'x' || *p == 'c') {
            command_letter = letter;
            string_arg = p;
            return;
          }
        }
      #endif

      // Bail if there's no command code number
      if (!TERN(SIGNED_CODENUM, NUMERIC_SIGNED(*p), NUMERIC(*p))) {
        if (TERN0(HAS_MULTI_EXTRUDER, letter == 'T')) {
          p[0] = '*'; p[1] = '\0'; string_arg = p; // Convert 'T' alone into 'T*'
          command_letter = letter;
        }
        return;
      }

      // Save the command letter at this point
      // A '?' signifies an unknown command
      command_letter = letter;

      #if ENABLED(SIGNED_CODENUM)
        int sign = 1; // Allow for a negative code like D-1 or T-1
        if (*p == '-') { sign = -1; ++p; }
      #endif

      // Get the code number - integer digits only
      codenum = 0;

      do { codenum = codenum * 10 + *p++ - '0'; } while (NUMERIC(*p));

      // Apply the sign, if any
      TERN_(SIGNED_CODENUM, codenum *= sign);

      // Allow for decimal point in command
      #if USE_GCODE_SUBCODES
        if (*p == '.') {
          p++;
          while (NUMERIC(*p))
            subcode = subcode * 10 + *p++ - '0';
        }
      #endif

      // Skip all spaces to get to the first argument, or nul
      while (*p == ' ') p++;

      #if ENABLED(GCODE_MOTION_MODES)
        if (letter == 'G'
          && (codenum <= TERN(ARC_SUPPORT, 3, 1) || TERN0(BEZIER_CURVE_SUPPORT, codenum == 5) || TERN0(G38_PROBE_TARGET, codenum == 38))
        ) {
          motion_mode_codenum = codenum;
          TERN_(USE_GCODE_SUBCODES, motion_mode_subcode = subcode);
        }
      #endif

    } break;

    #if ENABLED(GCODE_MOTION_MODES)

      #if ANY(BEZIER_CURVE_SUPPORT, ARC_SUPPORT)
        case 'I' ... 'J': case 'P':
          if (TERN1(BEZIER_CURVE_SUPPORT, motion_mode_codenum != 5)
            && TERN1(ARC_P_CIRCLES, !WITHIN(motion_mode_codenum, 2, 3))
          ) return;
      #endif

      #if ENABLED(BEZIER_CURVE_SUPPORT)
        case 'Q': if (motion_mode_codenum != 5) return;
      #endif

      #if ENABLED(ARC_SUPPORT)
        case 'R': if (!WITHIN(motion_mode_codenum, 2, 3)) return;
      #endif

      LOGICAL_AXIS_GANG(case 'E':, case 'X':, case 'Y':, case 'Z':, case AXIS4_NAME:, case AXIS5_NAME:, case AXIS6_NAME:, case AXIS7_NAME:, case AXIS8_NAME:, case AXIS9_NAME:)
      case 'F':
        if (motion_mode_codenum < 0) return;
        command_letter = 'G';
        codenum = motion_mode_codenum;
        TERN_(USE_GCODE_SUBCODES, subcode = motion_mode_subcode);
        p--; // Back up one character to use the current parameter
        break;

    #endif

    default: return;
  }

  // The command parameters (if any) start here, for sure!

  IF_DISABLED(FASTER_GCODE_PARSER, command_args = p); // Scan for parameters in seen()

  // Only use string_arg for these M codes
  if (letter == 'M') switch (codenum) {
    TERN_(EXPECTED_PRINTER_CHECK, case 16:)
    TERN_(HAS_MEDIA, case 23: case 28: case 30: case 928:)
    TERN_(HAS_STATUS_MESSAGE, case 117:)
    TERN_(HAS_RS485_SERIAL, case 485:)
    TERN_(GCODE_MACROS, case 810 ... 819:)
    case 118:
      string_arg = unescape_string(p);
      return;
    default: break;
  }

  #if ENABLED(DEBUG_GCODE_PARSER)
    const bool debug = codenum == 800;
  #endif

  /**
   * Find all parameters, set flags and pointers for fast parsing
   *
   * Most codes ignore 'string_arg', but those that want a string will get the right pointer.
   * The following loop assigns the first "parameter" having no numeric value to 'string_arg'.
   * This allows M0/M1 with expire time to work: "M0 S5 You Win!"
   * For 'M118' you must use 'E1' and 'A1' rather than just 'E' or 'A'
   */
  #if ENABLED(GCODE_QUOTED_STRINGS)
    bool quoted_string_arg = false;
  #endif
  string_arg = nullptr;
  while (const char param = uppercase(*p++)) {  // Get the next parameter. A NUL ends the loop

    // Special handling for M32 [P] !/path/to/file.g#
    // The path must be the last parameter
    if (param == '!' && is_command('M', 32)) {
      string_arg = p;                           // Name starts after '!'
      char * const lb = strchr(p, '#');         // Already seen '#' as SD char (to pause buffering)
      if (lb) *lb = '\0';                       // Safe to mark the end of the filename
      return;
    }

    #if ENABLED(GCODE_QUOTED_STRINGS)
      if (!quoted_string_arg && param == '"') {
        quoted_string_arg = true;
        string_arg = unescape_string(p);
      }
    #endif

    #if ENABLED(FASTER_GCODE_PARSER)
      // Arguments MUST be uppercase for fast G-Code parsing
      #define PARAM_OK(P) WITHIN((P), 'A', 'Z')
    #else
      #define PARAM_OK(P) true
    #endif

    if (PARAM_OK(param)) {

      while (*p == ' ') p++;                    // Skip spaces between parameters & values

      #if ENABLED(GCODE_QUOTED_STRINGS)
        const bool is_str = (*p == '"'), has_val = is_str || valid_float(p);
        char * const valptr = has_val ? is_str ? unescape_string(p) : p : nullptr;
      #else
        const bool has_val = valid_float(p);
        #if ENABLED(FASTER_GCODE_PARSER)
          char * const valptr = has_val ? p : nullptr;
        #endif
      #endif

      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) {
          SERIAL_ECHOPGM("Got param ", C(param), " at index ", p - command_ptr - 1);
          if (has_val) SERIAL_ECHOPGM(" (has_val)");
        }
      #endif

      if (!has_val && !string_arg) {            // No value? First time, keep as string_arg
        string_arg = p - 1;
        #if ENABLED(DEBUG_GCODE_PARSER)
          if (debug) SERIAL_ECHOPGM(" string_arg: ", hex_address(string_arg)); // DEBUG
        #endif
      }

      if (TERN0(DEBUG_GCODE_PARSER, debug)) SERIAL_EOL();

      TERN_(FASTER_GCODE_PARSER, set(param, valptr)); // Set parameter exists and pointer (nullptr for no value)
    }
    else if (!string_arg) {                     // Not A-Z? First time, keep as the string_arg
      string_arg = p - 1;
      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) SERIAL_ECHOPGM(" string_arg: ", hex_address(string_arg)); // DEBUG
      #endif
    }

    if (!WITHIN(*p, 'A', 'Z')) {                // Another parameter right away?
      while (*p && DECIMAL_SIGNED(*p)) p++;     // Skip over the value section of a parameter
      while (*p == ' ') p++;                    // Skip over all spaces
    }
  }
}

#if ENABLED(CNC_COORDINATE_SYSTEMS)

  // Parse the next parameter as a new command
  bool GCodeParser::chain() {
    #if ENABLED(FASTER_GCODE_PARSER)
      char *next_command = command_ptr;
      if (next_command) {
        while (*next_command && *next_command != ' ') ++next_command;
        while (*next_command == ' ') ++next_command;
        if (!*next_command) next_command = nullptr;
      }
    #else
      const char *next_command = command_args;
    #endif
    if (next_command) parse(next_command);
    return !!next_command;
  }

#endif // CNC_COORDINATE_SYSTEMS

void GCodeParser::unknown_command_warning() {
  SERIAL_ECHO_MSG(STR_UNKNOWN_COMMAND, command_ptr, "\"");
}

#if ENABLED(DEBUG_GCODE_PARSER)

  void GCodeParser::debug() {
    SERIAL_ECHOPGM("Command: ", command_ptr, " (", command_letter);
    SERIAL_ECHO(codenum);
    SERIAL_ECHOLNPGM(")");
    #if ENABLED(FASTER_GCODE_PARSER)
      SERIAL_ECHOPGM(" args: { ");
      for (char c = 'A'; c <= 'Z'; ++c) if (seen(c)) SERIAL_CHAR(c, ' ');
      SERIAL_CHAR('}');
    #else
      SERIAL_ECHOPGM(" args: { ", command_args, " }");
    #endif
    if (string_arg) {
      SERIAL_ECHOPGM(" string: \"", string_arg);
      SERIAL_CHAR('"');
    }
    SERIAL_ECHOLNPGM("\n");
    for (char c = 'A'; c <= 'Z'; ++c) {
      if (seen(c)) {
        SERIAL_ECHOPGM("Code '", c); SERIAL_ECHOPGM("':");
        if (has_value()) {
          SERIAL_ECHOLNPGM(
            "\n    float: ", value_float(),
            "\n     long: ", value_long(),
            "\n    ulong: ", value_ulong(),
            "\n   millis: ", value_millis(),
            "\n   sec-ms: ", value_millis_from_seconds(),
            "\n      int: ", value_int(),
            "\n   ushort: ", value_ushort(),
            "\n     byte: ", value_byte(),
            "\n     bool: ", value_bool(),
            "\n   linear: ", value_linear_units(),
            "\n  celsius: ", value_celsius()
          );
        }
        else
          SERIAL_ECHOLNPGM(" (no value)");
      }
    }
  }

#endif // DEBUG_GCODE_PARSER
