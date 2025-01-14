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

#include "../inc/MarlinConfig.h"
#include "serial_hook.h"

#if HAS_MEATPACK
  #include "../feature/meatpack.h"
#endif

//
// Debugging flags for use by M111
//
enum MarlinDebugFlags : uint8_t {
  MARLIN_DEBUG_NONE          = 0,
  MARLIN_DEBUG_ECHO          = TERN0(DEBUG_FLAGS_GCODE,      _BV(0)), //!< Echo commands in order as they are processed
  MARLIN_DEBUG_INFO          = TERN0(DEBUG_FLAGS_GCODE,      _BV(1)), //!< Print messages for code that has debug output
  MARLIN_DEBUG_ERRORS        = TERN0(DEBUG_FLAGS_GCODE,      _BV(2)), //!< Not implemented
  MARLIN_DEBUG_DRYRUN        =                               _BV(3),  //!< Ignore temperature setting and E movement commands
  MARLIN_DEBUG_COMMUNICATION = TERN0(DEBUG_FLAGS_GCODE,      _BV(4)), //!< Not implemented
  MARLIN_DEBUG_LEVELING      = TERN0(DEBUG_LEVELING_FEATURE, _BV(5)), //!< Print detailed output for homing and leveling
  MARLIN_DEBUG_MESH_ADJUST   = TERN0(DEBUG_LEVELING_FEATURE, _BV(6)), //!< UBL bed leveling
  MARLIN_DEBUG_ALL           = MARLIN_DEBUG_ECHO|MARLIN_DEBUG_INFO|MARLIN_DEBUG_ERRORS|MARLIN_DEBUG_COMMUNICATION|MARLIN_DEBUG_LEVELING|MARLIN_DEBUG_MESH_ADJUST
};

extern uint8_t marlin_debug_flags;
#define DEBUGGING(F) (marlin_debug_flags & (MARLIN_DEBUG_## F))

//
// Serial redirection
//
// Step 1: Find out what the first serial leaf is
#if HAS_MULTI_SERIAL && defined(SERIAL_CATCHALL)
  #define _SERIAL_LEAF_1 MYSERIAL
#else
  #define _SERIAL_LEAF_1 MYSERIAL1
#endif

// Hook Meatpack if it's enabled on the first leaf
#if ENABLED(MEATPACK_ON_SERIAL_PORT_1)
  typedef MeatpackSerial<decltype(_SERIAL_LEAF_1)> SerialLeafT1;
  extern SerialLeafT1 mpSerial1;
  #define SERIAL_LEAF_1 mpSerial1
#else
  #define SERIAL_LEAF_1 _SERIAL_LEAF_1
#endif

// Step 2: For multiserial wrap all serial ports in a single
//         interface with the ability to output to multiple serial ports.
#if HAS_MULTI_SERIAL
  #define _PORT_REDIRECT(n,p) REMEMBER(n,multiSerial.portMask,p)
  #define _PORT_RESTORE(n)    RESTORE(n)
  #define SERIAL_ASSERT(P)    if (multiSerial.portMask!=(P)) { debugger(); }
  // If we have a catchall, use that directly
  #ifdef SERIAL_CATCHALL
    #define _SERIAL_LEAF_2 SERIAL_CATCHALL
  #elif HAS_ETHERNET
    typedef ConditionalSerial<decltype(MYSERIAL2)> SerialLeafT2;  // We need to create an instance here
    extern SerialLeafT2 msSerial2;
    #define _SERIAL_LEAF_2 msSerial2
  #else
    #define _SERIAL_LEAF_2 MYSERIAL2 // Don't create a useless instance here, directly use the existing instance
  #endif

  // Nothing complicated here
  #define _SERIAL_LEAF_3 MYSERIAL3

  // Hook Meatpack if it's enabled on the second leaf
  #if ENABLED(MEATPACK_ON_SERIAL_PORT_2)
    typedef MeatpackSerial<decltype(_SERIAL_LEAF_2)> SerialLeafT2;
    extern SerialLeafT2 mpSerial2;
    #define SERIAL_LEAF_2 mpSerial2
  #else
    #define SERIAL_LEAF_2 _SERIAL_LEAF_2
  #endif

  // Hook Meatpack if it's enabled on the third leaf
  #if ENABLED(MEATPACK_ON_SERIAL_PORT_3)
    typedef MeatpackSerial<decltype(_SERIAL_LEAF_3)> SerialLeafT3;
    extern SerialLeafT3 mpSerial3;
    #define SERIAL_LEAF_3 mpSerial3
  #else
    #define SERIAL_LEAF_3 _SERIAL_LEAF_3
  #endif

  #define __S_MULTI(N) decltype(SERIAL_LEAF_##N),
  #define _S_MULTI(N) __S_MULTI(N)

  typedef MultiSerial< REPEAT_1(NUM_SERIAL, _S_MULTI) 0> SerialOutputT;

  #undef __S_MULTI
  #undef _S_MULTI

  extern SerialOutputT        multiSerial;
  #define SERIAL_IMPL         multiSerial
#else
  #define _PORT_REDIRECT(n,p) NOOP
  #define _PORT_RESTORE(n)    NOOP
  #define SERIAL_ASSERT(P)    NOOP
  #define SERIAL_IMPL         SERIAL_LEAF_1
#endif

#define PORT_REDIRECT(p)   _PORT_REDIRECT(1,p)
#define PORT_RESTORE()     _PORT_RESTORE(1)
#define SERIAL_PORTMASK(P) SerialMask::from(P)

//
// SERIAL_CHAR - Print one or more individual chars
//
void SERIAL_CHAR(char a);
template <typename ... Args>
void SERIAL_CHAR(char a, Args ... args) { SERIAL_IMPL.write(a); SERIAL_CHAR(args ...); }

/**
 * SERIAL_ECHO / SERIAL_ECHOLN - Print a single string or value.
 *   Any numeric parameter (including char) is printed as a base-10 number.
 *   A string pointer or literal will be output as a string.
 *
 * NOTE: Use SERIAL_CHAR to print char as a single character.
 */
template <typename T> void SERIAL_ECHO(T x)   { SERIAL_IMPL.print(x); }
template <typename T> void SERIAL_ECHOLN(T x) { SERIAL_IMPL.println(x); }

// Wrapper for ECHO commands to interpret a char
void SERIAL_ECHO(serial_char_t x);
#define AS_DIGIT(n) C('0' + (n))

// Print an integer with a numeric base such as PrintBase::Hex
template <typename T> void SERIAL_PRINT(T x, PrintBase y)   { SERIAL_IMPL.print(x, y); }
template <typename T> void SERIAL_PRINTLN(T x, PrintBase y) { SERIAL_IMPL.println(x, y); }

// Flush the serial port
void SERIAL_FLUSH();
void SERIAL_FLUSHTX();

// Start an echo: or error: output
void SERIAL_ECHO_START();
void SERIAL_ERROR_START();
void SERIAL_WARN_START();

// Serial end-of-line
void SERIAL_EOL();

// Print a single PROGMEM, PGM_P, or PSTR() string.
void SERIAL_ECHO_P(PGM_P pstr);
void SERIAL_ECHOLN_P(PGM_P pstr);

// Specializations for float, p_float_t, and w_float_t
template<> void SERIAL_ECHO(const float f);
template<> void SERIAL_ECHO(const p_float_t pf);
template<> void SERIAL_ECHO(const w_float_t wf);

// Specializations for F-string
template<> void SERIAL_ECHO(FSTR_P const fstr);
template<> void SERIAL_ECHOLN(FSTR_P const fstr);

// Print any number of items with arbitrary types (except loose PROGMEM strings)
template <typename T, typename ... Args>
void SERIAL_ECHO(T arg1, Args ... args) { SERIAL_ECHO(arg1); SERIAL_ECHO(args ...); }
template <typename T, typename ... Args>
void SERIAL_ECHOLN(T arg1, Args ... args) { SERIAL_ECHO(arg1); SERIAL_ECHO(args ...); SERIAL_EOL(); }

//
// SERIAL_ECHOPGM... macros are used to output string-value pairs, wrapping
//                   all the odd loose string elements as PROGMEM strings.
//

// Print up to 20 pairs of values. Odd elements must be literal strings.
#define __SEP_N(N,V...)           _SEP_##N(V)
#define _SEP_N(N,V...)            __SEP_N(N,V)
#define _SEP_N_REF()              _SEP_N
#define _SEP_1(s)                 SERIAL_ECHO(F(s));
#define _SEP_2(s,v)               SERIAL_ECHO(F(s),v);
#define _SEP_3(s,v,V...)          _SEP_2(s,v); DEFER2(_SEP_N_REF)()(TWO_ARGS(V),V);
#define SERIAL_ECHOPGM(V...)      do{ EVAL(_SEP_N(TWO_ARGS(V),V)); }while(0)

// Print up to 20 pairs of values followed by newline. Odd elements must be literal strings.
#define __SELP_N(N,V...)          _SELP_##N(V)
#define _SELP_N(N,V...)           __SELP_N(N,V)
#define _SELP_N_REF()             _SELP_N
#define _SELP_1(s)                SERIAL_ECHO(F(s "\n"));
#define _SELP_2(s,v)              SERIAL_ECHOLN(F(s),v);
#define _SELP_3(s,v,V...)         _SEP_2(s,v); DEFER2(_SELP_N_REF)()(TWO_ARGS(V),V);
#define SERIAL_ECHOLNPGM(V...)    do{ EVAL(_SELP_N(TWO_ARGS(V),V)); }while(0)

// Print up to 20 pairs of values. Odd elements must be PSTR pointers.
#define __SEP_N_P(N,V...)         _SEP_##N##_P(V)
#define _SEP_N_P(N,V...)          __SEP_N_P(N,V)
#define _SEP_N_P_REF()            _SEP_N_P
#define _SEP_1_P(p)               SERIAL_ECHO(FPSTR(p));
#define _SEP_2_P(p,v)             SERIAL_ECHO(FPSTR(p),v);
#define _SEP_3_P(p,v,V...)        _SEP_2_P(p,v); DEFER2(_SEP_N_P_REF)()(TWO_ARGS(V),V);
#define SERIAL_ECHOPGM_P(V...)    do{ EVAL(_SEP_N_P(TWO_ARGS(V),V)); }while(0)

// Print up to 20 pairs of values followed by newline. Odd elements must be PSTR pointers.
#define __SELP_N_P(N,V...)        _SELP_##N##_P(V)
#define _SELP_N_P(N,V...)         __SELP_N_P(N,V)
#define _SELP_N_P_REF()           _SELP_N_P
#define _SELP_1_P(p)              SERIAL_ECHOLN(FPSTR(p));
#define _SELP_2_P(p,v)            SERIAL_ECHOLN(FPSTR(p),v);
#define _SELP_3_P(p,v,V...)       { _SEP_2_P(p,v); DEFER2(_SELP_N_P_REF)()(TWO_ARGS(V),V); }
#define SERIAL_ECHOLNPGM_P(V...)  do{ EVAL(_SELP_N_P(TWO_ARGS(V),V)); }while(0)

#define SERIAL_ECHO_MSG(V...)  do{ SERIAL_ECHO_START();  SERIAL_ECHOLNPGM(V); }while(0)
#define SERIAL_ERROR_MSG(V...) do{ SERIAL_ERROR_START(); SERIAL_ECHOLNPGM(V); }while(0)
#define SERIAL_WARN_MSG(V...)  do{ SERIAL_WARN_START();  SERIAL_ECHOLNPGM(V); }while(0)

// Print a prefix, conditional string, and suffix
void serial_ternary(FSTR_P const pre, const bool onoff, FSTR_P const on, FSTR_P const off, FSTR_P const post=nullptr);
// Shorthand to put loose strings in PROGMEM
#define SERIAL_ECHO_TERNARY(TF, PRE, ON, OFF, POST) serial_ternary(F(PRE), TF, F(ON), F(OFF), F(POST))

// Print up to 255 spaces
void SERIAL_ECHO_SP(uint8_t count);

void serialprint_onoff(const bool onoff);
void serialprintln_onoff(const bool onoff);
void serialprint_truefalse(const bool tf);
void serial_offset(const_float_t v, const uint8_t sp=0); // For v==0 draw space (sp==1) or plus (sp==2)

void print_bin(const uint16_t val);

void print_xyz(NUM_AXIS_ARGS_(const_float_t) FSTR_P const prefix=nullptr, FSTR_P const suffix=nullptr);
inline void print_xyz(const xyz_pos_t &xyz, FSTR_P const prefix=nullptr, FSTR_P const suffix=nullptr) {
  print_xyz(NUM_AXIS_ELEM_(xyz) prefix, suffix);
}

void print_xyze(LOGICAL_AXIS_ARGS_(const_float_t) FSTR_P const prefix=nullptr, FSTR_P const suffix=nullptr);
inline void print_xyze(const xyze_pos_t &xyze, FSTR_P const prefix=nullptr, FSTR_P const suffix=nullptr) {
  print_xyze(LOGICAL_AXIS_ELEM_LC_(xyze) prefix, suffix);
}

#define SERIAL_POS(SUFFIX,VAR) do { print_xyz(VAR, F("  " STRINGIFY(VAR) "="), F(" : " SUFFIX "\n")); }while(0)
#define SERIAL_XYZ(PREFIX,V...) do { print_xyz(V, F(PREFIX)); }while(0)

/**
 * Extended string that can echo itself to serial
 */
template <int SIZE=DEFAULT_MSTRING_SIZE>
class SString : public MString<SIZE> {
public:
  typedef MString<SIZE> super;
  using super::str;
  using super::debug;

  SString() : super() {}

  template <typename T, typename... Args>
  SString(T arg1, Args... more) : super(arg1, more...) {}

  SString& set() { super::set(); return *this; }

  template<typename... Args>
  SString& setf_P(PGM_P const pfmt, Args... more) { super::setf_P(pfmt, more...); return *this; }

  template<typename... Args>
  SString& setf(const char *fmt, Args... more)    { super::setf(fmt, more...); return *this; }

  template<typename... Args>
  SString& setf(FSTR_P const ffmt, Args... more)  { super::setf(ffmt, more...); return *this; }

  template <typename T>
  SString& set(const T &v) { super::set(v); return *this; }

  template <typename T>
  SString& append(const T &v) { super::append(v); return *this; }

  template<typename T, typename... Args>
  SString& set(T arg1, Args... more) { set(arg1).append(more...); return *this; }

  template<typename T, typename... Args>
  SString& append(T arg1, Args... more) { append(arg1).append(more...); return *this; }

  SString& clear() { set(); return *this; }
  SString& eol() { append('\n'); return *this; }
  SString& trunc(const int &i) { super::trunc(i); return *this; }

  // Extended with methods to print to serial
  SString& echo()   { SERIAL_ECHO(str);   return *this; }
  SString& echoln() { SERIAL_ECHOLN(str); return *this; }
};

#define TSS(V...) SString<>(V)

//
// Commonly-used strings in serial output
//

#define _N_STR(N) N##_STR
#define _N_LBL(N) N##_LBL
#define _N_STR_A(N) _N_STR(N)[]
#define _N_LBL_A(N) _N_LBL(N)[]
#define _SP_N_STR(N) SP_##N##_STR
#define _SP_N_LBL(N) SP_##N##_LBL
#define _SP_N_STR_A(N) _SP_N_STR(N)[]
#define _SP_N_LBL_A(N) _SP_N_LBL(N)[]

extern const char SP_A_STR[], SP_B_STR[], SP_C_STR[], SP_P_STR[], SP_T_STR[], NUL_STR[],
                  MAPLIST(_N_STR_A, LOGICAL_AXIS_NAMES), MAPLIST(_SP_N_STR_A, LOGICAL_AXIS_NAMES),
                  MAPLIST(_N_LBL_A, LOGICAL_AXIS_NAMES), MAPLIST(_SP_N_LBL_A, LOGICAL_AXIS_NAMES);

PGM_P const SP_AXIS_LBL[] PROGMEM = { MAPLIST(_SP_N_LBL, LOGICAL_AXIS_NAMES) };
PGM_P const SP_AXIS_STR[] PROGMEM = { MAPLIST(_SP_N_STR, LOGICAL_AXIS_NAMES) };

#undef _N_STR
#undef _N_LBL
#undef _N_STR_A
#undef _N_LBL_A
#undef _SP_N_STR
#undef _SP_N_LBL
#undef _SP_N_STR_A
#undef _SP_N_LBL_A
