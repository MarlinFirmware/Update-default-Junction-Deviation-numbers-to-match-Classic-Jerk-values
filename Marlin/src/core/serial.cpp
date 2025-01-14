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

#include "serial.h"
#include "../inc/MarlinConfig.h"

#if HAS_ETHERNET
  #include "../feature/ethernet.h"
#endif

#include <stdlib.h> // dtostrf

// Echo commands to the terminal by default in dev mode
uint8_t marlin_debug_flags = TERN(MARLIN_DEV_MODE, MARLIN_DEBUG_ECHO, MARLIN_DEBUG_NONE);

// Commonly-used strings in serial output
PGMSTR(SP_A_STR, " A"); PGMSTR(SP_B_STR, " B"); PGMSTR(SP_C_STR, " C");
PGMSTR(SP_P_STR, " P"); PGMSTR(SP_T_STR, " T"); PGMSTR(NUL_STR,   "");

#define _N_STR(N) PGMSTR(N##_STR, STR_##N);
#define _N_LBL(N) PGMSTR(N##_LBL, STR_##N ":");
#define _SP_N_STR(N) PGMSTR(SP_##N##_STR, " " STR_##N);
#define _SP_N_LBL(N) PGMSTR(SP_##N##_LBL, " " STR_##N ":");
MAP(_N_STR, LOGICAL_AXIS_NAMES); MAP(_SP_N_STR, LOGICAL_AXIS_NAMES);
MAP(_N_LBL, LOGICAL_AXIS_NAMES); MAP(_SP_N_LBL, LOGICAL_AXIS_NAMES);

// Hook Meatpack if it's enabled on the first leaf
#if ENABLED(MEATPACK_ON_SERIAL_PORT_1)
  SerialLeafT1 mpSerial1(false, _SERIAL_LEAF_1);
#endif
#if ENABLED(MEATPACK_ON_SERIAL_PORT_2)
  SerialLeafT2 mpSerial2(false, _SERIAL_LEAF_2);
#endif
#if ENABLED(MEATPACK_ON_SERIAL_PORT_3)
  SerialLeafT3 mpSerial3(false, _SERIAL_LEAF_3);
#endif

// Step 2: For multiserial, handle the second serial port as well
#if HAS_MULTI_SERIAL
  #if HAS_ETHERNET
    // We need a definition here
    SerialLeafT2 msSerial2(ethernet.have_telnet_client, MYSERIAL2, false);
  #endif

  #define __S_LEAF(N) ,SERIAL_LEAF_##N
  #define _S_LEAF(N) __S_LEAF(N)

  SerialOutputT multiSerial( SERIAL_LEAF_1 REPEAT_S(2, INCREMENT(NUM_SERIAL), _S_LEAF) );

  #undef __S_LEAF
  #undef _S_LEAF

#endif

// Specializations for float, p_float_t, w_float_t
template <> void SERIAL_ECHO(const float f)      { SERIAL_IMPL.print(f, SERIAL_FLOAT_PRECISION); }
template <> void SERIAL_ECHO(const p_float_t pf) { SERIAL_IMPL.print(pf.value, pf.prec); }
template <> void SERIAL_ECHO(const w_float_t wf) { char f1[20]; SERIAL_IMPL.print(dtostrf(wf.value, wf.width, wf.prec, f1)); }

// Specializations for F-string
template <> void SERIAL_ECHO(FSTR_P const fstr)   { SERIAL_ECHO_P(FTOP(fstr)); }
template <> void SERIAL_ECHOLN(FSTR_P const fstr) { SERIAL_ECHOLN_P(FTOP(fstr)); }

void SERIAL_CHAR(char a) { SERIAL_IMPL.write(a); }
void SERIAL_EOL() { SERIAL_CHAR('\n'); }

void SERIAL_ECHO(serial_char_t x) { SERIAL_IMPL.write(x.c); }

void SERIAL_FLUSH()    { SERIAL_IMPL.flush(); }
void SERIAL_FLUSHTX()  { SERIAL_IMPL.flushTX(); }

void SERIAL_ECHO_P(PGM_P pstr) {
  while (const char c = pgm_read_byte(pstr++)) SERIAL_CHAR(c);
}
void SERIAL_ECHOLN_P(PGM_P pstr) { SERIAL_ECHO_P(pstr); SERIAL_EOL(); }

void SERIAL_ECHO_START()  { SERIAL_ECHO(F("echo:")); }
void SERIAL_ERROR_START() { SERIAL_ECHO(F("Error:")); }
void SERIAL_WARN_START()  { SERIAL_ECHO(F("Warning:")); }

void SERIAL_ECHO_SP(uint8_t count) { count *= (PROPORTIONAL_FONT_RATIO); while (count--) SERIAL_CHAR(' '); }

void serial_offset(const_float_t v, const uint8_t sp/*=0*/) {
  if (v == 0 && sp == 1)
    SERIAL_CHAR(' ');
  else if (v > 0 || (v == 0 && sp == 2))
    SERIAL_CHAR('+');
  SERIAL_ECHO(v);
}

void serial_ternary(FSTR_P const pre, const bool onoff, FSTR_P const on, FSTR_P const off, FSTR_P const post/*=nullptr*/) {
  if (pre)           SERIAL_ECHO(pre);
  if (onoff && on)   SERIAL_ECHO(on);
  if (!onoff && off) SERIAL_ECHO(off);
  if (post)          SERIAL_ECHO(post);
}

void serialprint_onoff(const bool onoff) { SERIAL_ECHO(onoff ? F(STR_ON) : F(STR_OFF)); }
void serialprintln_onoff(const bool onoff) { serialprint_onoff(onoff); SERIAL_EOL(); }
void serialprint_truefalse(const bool tf) { SERIAL_ECHO(tf ? F("true") : F("false")); }

void print_bin(uint16_t val) {
  for (uint8_t i = 16; i--;) {
    SERIAL_CHAR('0' + TEST(val, i));
    if (!(i & 0x3) && i) SERIAL_CHAR(' ');
  }
}

void _print_xyz(NUM_AXIS_ARGS_(const_float_t) FSTR_P const prefix) {
  if (prefix) SERIAL_ECHO(prefix);
  #if NUM_AXES
    SERIAL_ECHOPGM_P(
      LIST_N(DOUBLE(NUM_AXES), SP_X_STR, x, SP_Y_STR, y, SP_Z_STR, z, SP_I_STR, i, SP_J_STR, j, SP_K_STR, k, SP_U_STR, u, SP_V_STR, v, SP_W_STR, w)
    );
  #endif
}

void print_xyz(NUM_AXIS_ARGS_(const_float_t) FSTR_P const prefix/*=nullptr*/, FSTR_P const suffix/*=nullptr*/) {
  _print_xyz(NUM_AXIS_LIST_(x, y, z, i, j, k, u, v, w) prefix);
  if (suffix) SERIAL_ECHO(suffix); else SERIAL_EOL();
}

void print_xyze(LOGICAL_AXIS_ARGS_(const_float_t) FSTR_P const prefix/*=nullptr*/, FSTR_P const suffix/*=nullptr*/) {
  _print_xyz(NUM_AXIS_LIST_(x, y, z, i, j, k, u, v, w) prefix);
  #if HAS_EXTRUDERS
    SERIAL_ECHOPGM_P(SP_E_STR, e);
  #endif
  if (suffix) SERIAL_ECHO(suffix); else SERIAL_EOL();
}
