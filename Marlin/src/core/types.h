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

#include <math.h>
#include <stddef.h>

#include "../inc/MarlinConfigPre.h"

//
// Conditional type assignment magic. For example...
//
// typename IF<(MYOPT==12), int, float>::type myvar;
//
template <bool, class L, class R> struct IF { typedef R type; };
template <class L, class R> struct IF<true, L, R> { typedef L type; };

#define ALL_AXIS_NAMES X, X2, Y, Y2, Z, Z2, Z3, Z4, I, J, K, U, V, W, E0, E1, E2, E3, E4, E5, E6, E7

#define NUM_AXIS_GANG(V...)   GANG_N(NUM_AXES, V)
#define NUM_AXIS_CODE(V...)   CODE_N(NUM_AXES, V)
#define NUM_AXIS_LIST(V...)   LIST_N(NUM_AXES, V)
#define NUM_AXIS_LIST_1(V)    LIST_N_1(NUM_AXES, V)
#define NUM_AXIS_ARRAY(V...)  { NUM_AXIS_LIST(V) }
#define NUM_AXIS_ARRAY_1(V)   { NUM_AXIS_LIST_1(V) }
#define NUM_AXIS_ARGS(T)      NUM_AXIS_LIST(T X, T Y, T Z, T I, T J, T K, T U, T V, T W)
#define NUM_AXIS_ARGS_LC(T)   NUM_AXIS_LIST(T x, T y, T z, T i, T j, T k, T u, T v, T w)
#define NUM_AXIS_ELEM(O)      NUM_AXIS_LIST(O.X, O.Y, O.Z, O.I, O.J, O.K, O.U, O.V, O.W)
#define NUM_AXIS_ELEM_LC(O)   NUM_AXIS_LIST(O.x, O.y, O.z, O.i, O.j, O.k, O.u, O.v, O.w)
#define NUM_AXIS_DECL(T,V)    NUM_AXIS_LIST(T X=V, T Y=V, T Z=V, T I=V, T J=V, T K=V, T U=V, T V=V, T W=V)
#define NUM_AXIS_DECL_LC(T,V) NUM_AXIS_LIST(T x=V, T y=V, T z=V, T i=V, T j=V, T k=V, T u=V, T v=V, T w=V)
#define MAIN_AXIS_NAMES       NUM_AXIS_LIST(X, Y, Z, I, J, K, U, V, W)
#define MAIN_AXIS_NAMES_LC    NUM_AXIS_LIST(x, y, z, i, j, k, u, v, w)
#define STR_AXES_MAIN         NUM_AXIS_GANG("X", "Y", "Z", STR_I, STR_J, STR_K, STR_U, STR_V, STR_W)

#define LOGICAL_AXIS_GANG(N,V...)  NUM_AXIS_GANG(V) GANG_ITEM_E(N)
#define LOGICAL_AXIS_CODE(N,V...)  NUM_AXIS_CODE(V) CODE_ITEM_E(N)
#define LOGICAL_AXIS_LIST(N,V...)  NUM_AXIS_LIST(V) LIST_ITEM_E(N)
#define LOGICAL_AXIS_LIST_1(V)     NUM_AXIS_LIST_1(V) LIST_ITEM_E(V)
#define LOGICAL_AXIS_ARRAY(N,V...) { LOGICAL_AXIS_LIST(N,V) }
#define LOGICAL_AXIS_ARRAY_1(V)    { LOGICAL_AXIS_LIST_1(V) }
#define LOGICAL_AXIS_ARGS(T)       LOGICAL_AXIS_LIST(T E, T X, T Y, T Z, T I, T J, T K, T U, T V, T W)
#define LOGICAL_AXIS_ARGS_LC(T)    LOGICAL_AXIS_LIST(T e, T x, T y, T z, T i, T j, T k, T u, T v, T w)
#define LOGICAL_AXIS_ELEM(O)       LOGICAL_AXIS_LIST(O.E, O.X, O.Y, O.Z, O.I, O.J, O.K, O.U, O.V, O.W)
#define LOGICAL_AXIS_ELEM_LC(O)    LOGICAL_AXIS_LIST(O.e, O.x, O.y, O.z, O.i, O.j, O.k, O.u, O.v, O.w)
#define LOGICAL_AXIS_DECL(T,V)     LOGICAL_AXIS_LIST(T E=V, T X=V, T Y=V, T Z=V, T I=V, T J=V, T K=V, T U=V, T V=V, T W=V)
#define LOGICAL_AXIS_DECL_LC(T,V)  LOGICAL_AXIS_LIST(T e=V, T x=V, T y=V, T z=V, T i=V, T j=V, T k=V, T u=V, T v=V, T w=V)
#define LOGICAL_AXIS_NAMES         LOGICAL_AXIS_LIST(E, X, Y, Z, I, J, K, U, V, W)
#define LOGICAL_AXIS_NAMES_LC      LOGICAL_AXIS_LIST(e, x, y, z, i, j, k, u, v, w)
#define LOGICAL_AXIS_MAP(F)        MAP(F, LOGICAL_AXIS_NAMES)
#define LOGICAL_AXIS_MAP_LC(F)     MAP(F, LOGICAL_AXIS_NAMES_LC)
#define STR_AXES_LOGICAL           LOGICAL_AXIS_GANG("E", "X", "Y", "Z", STR_I, STR_J, STR_K, STR_U, STR_V, STR_W)

#if NUM_AXES
  #define NUM_AXES_SEP ,
  #define MAIN_AXIS_MAP(F)    MAP(F, MAIN_AXIS_NAMES)
  #define MAIN_AXIS_MAP_LC(F) MAP(F, MAIN_AXIS_NAMES_LC)
  #define OPTARGS_NUM(T)      , NUM_AXIS_ARGS_LC(T)
  #define OPTARGS_LOGICAL(T)  , LOGICAL_AXIS_ARGS_LC(T)
#else
  #define NUM_AXES_SEP
  #define MAIN_AXIS_MAP(F)
  #define MAIN_AXIS_MAP_LC(F)
  #define OPTARGS_NUM(T)
  #define OPTARGS_LOGICAL(T)
#endif

#define NUM_AXIS_GANG_(V...)    NUM_AXIS_GANG(V) NUM_AXES_SEP
#define NUM_AXIS_LIST_(V...)    NUM_AXIS_LIST(V) NUM_AXES_SEP
#define NUM_AXIS_LIST_1_(V...)  NUM_AXIS_LIST_1(V) NUM_AXES_SEP
#define NUM_AXIS_ARGS_(T)       NUM_AXIS_ARGS_LC(T) NUM_AXES_SEP
#define NUM_AXIS_ELEM_(T)       NUM_AXIS_ELEM_LC(T) NUM_AXES_SEP
#define MAIN_AXIS_NAMES_        MAIN_AXIS_NAMES NUM_AXES_SEP
#define MAIN_AXIS_NAMES_LC_     MAIN_AXIS_NAMES_LC NUM_AXES_SEP

#if LOGICAL_AXES
  #define LOGICAL_AXES_SEP ,
#else
  #define LOGICAL_AXES_SEP
#endif

#define LOGICAL_AXIS_GANG_(V...)    LOGICAL_AXIS_GANG(V) LOGICAL_AXES_SEP
#define LOGICAL_AXIS_LIST_(V...)    LOGICAL_AXIS_LIST(V) LOGICAL_AXES_SEP
#define LOGICAL_AXIS_LIST_1_(V...)  LOGICAL_AXIS_LIST_1(V) LOGICAL_AXES_SEP
#define LOGICAL_AXIS_ARGS_(T)       LOGICAL_AXIS_ARGS_LC(T) LOGICAL_AXES_SEP
#define LOGICAL_AXIS_ELEM_(T)       LOGICAL_AXIS_ELEM(T) LOGICAL_AXES_SEP
#define LOGICAL_AXIS_ELEM_LC_(T)    LOGICAL_AXIS_ELEM_LC(T) LOGICAL_AXES_SEP
#define LOGICAL_AXIS_NAMES_         LOGICAL_AXIS_NAMES LOGICAL_AXES_SEP
#define LOGICAL_AXIS_NAMES_LC_      LOGICAL_AXIS_NAMES_LC LOGICAL_AXES_SEP

#define SECONDARY_AXIS_GANG(V...) GANG_N(SECONDARY_AXES, V)
#define SECONDARY_AXIS_CODE(V...) CODE_N(SECONDARY_AXES, V)
#define SECONDARY_AXIS_LIST(V...) LIST_N(SECONDARY_AXES, V)
#if SECONDARY_AXES
  #define SECONDARY_AXIS_NAMES      SECONDARY_AXIS_LIST(I, J, K, U, V, W)
  #define SECONDARY_AXIS_NAMES_LC   SECONDARY_AXIS_LIST(i, j, k, u, v, w)
  #define SECONDARY_AXIS_ARGS(T)    SECONDARY_AXIS_LIST(T I, T J, T K, T U, T V, T W)
  #define SECONDARY_AXIS_ARGS_LC(T) SECONDARY_AXIS_LIST(T i, T j, T k, T u, T v, T w)
  #define SECONDARY_AXIS_MAP(F)     MAP(F, SECONDARY_AXIS_NAMES)
  #define SECONDARY_AXIS_MAP_LC(F)  MAP(F, SECONDARY_AXIS_NAMES_LC)
#else
  #define SECONDARY_AXIS_MAP(F)
  #define SECONDARY_AXIS_MAP_LC(F)
#endif

// Just the XY or XYZ elements
#if HAS_Z_AXIS
  #define XYZ_COUNT 3
  #define XY_COUNT 2
#elif HAS_Y_AXIS
  #define XY_COUNT 2
#elif HAS_X_AXIS
  #define XY_COUNT 1
#else
  #define XY_COUNT 0
#endif
#ifndef XYZ_COUNT
  #define XYZ_COUNT XY_COUNT
#endif
#define XY_LIST(V...)    LIST_N(XY_COUNT, V)
#define XY_ARRAY(V...)  ARRAY_N(XY_COUNT, V)
#define XY_CODE(V...)    CODE_N(XY_COUNT, V)
#define XY_GANG(V...)    GANG_N(XY_COUNT, V)
#define XYZ_LIST(V...)   LIST_N(XYZ_COUNT, V)
#define XYZ_ARRAY(V...) ARRAY_N(XYZ_COUNT, V)
#define XYZ_CODE(V...)   CODE_N(XYZ_COUNT, V)
#define XYZ_GANG(V...)   GANG_N(XYZ_COUNT, V)

#if HAS_ROTATIONAL_AXES
  #define ROTATIONAL_AXIS_GANG(V...) GANG_N(ROTATIONAL_AXES, V)
#endif

#if HAS_EXTRUDERS
  #if NUM_AXES
    #define LIST_ITEM_E(N) , N
    #define CODE_ITEM_E(N) ; N
  #else
    #define LIST_ITEM_E(N) N
    #define CODE_ITEM_E(N) N
  #endif
  #define GANG_ITEM_E(N) N
#else
  #define LIST_ITEM_E(N)
  #define CODE_ITEM_E(N)
  #define GANG_ITEM_E(N)
#endif

#define AXIS_COLLISION(L) (AXIS4_NAME == L || AXIS5_NAME == L || AXIS6_NAME == L || AXIS7_NAME == L || AXIS8_NAME == L || AXIS9_NAME == L)

// Helpers
#define _RECIP(N) ((N) ? 1.0f / static_cast<float>(N) : 0.0f)
#define _ABS(N) ((N) < 0 ? -(N) : (N))
#define _LS(N)       T(uint32_t(N) << p)
#define _RS(N)       T(uint32_t(N) >> p)
#define _LSE(N)  N = T(uint32_t(N) << p)
#define _RSE(N)  N = T(uint32_t(N) >> p)
#define FI FORCE_INLINE

// Define types based on largest bit width stored value required
#define bits_t(W)   typename IF<((W)>   32), uint64_t, typename IF<((W)> 16), uint32_t, typename IF<((W)>8), uint16_t, uint8_t>::type>::type>::type
#define uvalue_t(V) typename IF<((V)>65535), uint32_t, typename IF<((V)>255), uint16_t, uint8_t>::type>::type
#define value_t(V)  typename IF<((V)>32767),  int32_t, typename IF<((V)>127),  int16_t,  int8_t>::type>::type

class BitProxy;

// Define a template for a bit field of N bits, using the smallest type that can hold N bits
template<size_t N, bool UseArray = (N > 64)>
struct Flags;

// Flag bits for <= 64 states
template<size_t N>
struct Flags<N, false> {
  typedef bits_t(N) flagbits_t;
  flagbits_t b;

  class BitProxy {
  public:
    BitProxy(flagbits_t& data, int bit) : data_(data), bit_(bit) {}

    BitProxy& operator=(const bool value) {
      if (value)
        data_ |=  (flagbits_t(1) << bit_);
      else
        data_ &= ~(flagbits_t(1) << bit_);
      return *this;
    }

    operator bool() const { return bool(data_ & (flagbits_t(1) << bit_)); }

  private:
    flagbits_t& data_;
    uint8_t bit_;
  };

  FI void reset()                            { b = 0; }
  FI void set(const int n, const bool onoff) { onoff ? set(n) : clear(n); }
  FI void set(const int n)                   { b |=  (flagbits_t(1) << n); }
  FI void clear(const int n)                 { b &= ~(flagbits_t(1) << n); }
  FI bool test(const int n) const            { return bool(b & (flagbits_t(1) << n)); }
  FI BitProxy operator[](const int n)        { return BitProxy(b, n); }
  FI bool operator[](const int n) const      { return test(n); }
  FI int size() const                        { return sizeof(b); }
  FI operator bool() const                   { return b != 0; }
};

// Flag bits for more than 64 states
template<size_t N>
struct Flags<N, true> {
  uint8_t bitmask[(N+7)>>3];
  // Proxy class for handling bit assignment
  class BitProxy {
  public:
    BitProxy(uint8_t data[], int n) : data_(data[n >> 3]), bit_(n & 7) {}

    // Assignment operator
    BitProxy& operator=(const bool value) {
      if (value)
        data_ |=  _BV(bit_);
      else
        data_ &= ~_BV(bit_);
      return *this;
    }

    // Conversion operator to bool
    operator bool() const { return TEST(data_, bit_); }

  private:
    uint8_t& data_;
    uint8_t bit_;
  };

  FI void reset()                            { for (uint8_t b = 0; b < sizeof(bitmask); ++b) bitmask[b] = 0; }
  FI void set(const int n, const bool onoff) { onoff ? set(n) : clear(n); }
  FI void set(const int n)                   { bitmask[n >> 3] |=  _BV(n & 7); }
  FI void clear(const int n)                 { bitmask[n >> 3] &= ~_BV(n & 7); }
  FI bool test(const int n) const            { return TEST(bitmask[n >> 3], n & 7); }
  FI BitProxy operator[](const int n)        { return BitProxy(bitmask, n); }
  FI bool operator[](const int n) const      { return test(n); }
  FI int size() const                        { return sizeof(bitmask); }
  FI operator bool() const                   { for (uint8_t b : bitmask) if (b) return true; return false; }
};

// Specialization for a single bool flag
template<>
struct Flags<1, false> {
  bool b;
  FI void reset()                            { b = false; }
  FI void set(const int n, const bool onoff) { onoff ? set(n) : clear(n); }
  FI void set(const int)                     { b = true; }
  FI void clear(const int)                   { b = false; }
  FI bool test(const int) const              { return b; }
  FI bool& operator[](const int)             { return b; }
  FI bool  operator[](const int) const       { return b; }
  FI int size() const                        { return sizeof(b); }
  FI operator bool() const                   { return b; }
};

typedef Flags<8> flags_8_t;
typedef Flags<16> flags_16_t;

// Flags for some axis states, with per-axis aliases xyzijkuvwe
typedef struct {
  union {
    struct Flags<LOGICAL_AXES> flags;
    struct { bool LOGICAL_AXIS_LIST(e:1, x:1, y:1, z:1, i:1, j:1, k:1, u:1, v:1, w:1); };
  };
  FI void reset()                            { flags.reset(); }
  FI void set(const int n)                   { flags.set(n); }
  FI void set(const int n, const bool onoff) { flags.set(n, onoff); }
  FI void clear(const int n)                 { flags.clear(n); }
  FI bool test(const int n) const            { return flags.test(n); }
  FI bool operator[](const int n)            { return flags[n]; }
  FI bool operator[](const int n) const      { return flags[n]; }
  FI int size() const                        { return sizeof(flags); }
  FI operator bool() const                   { return (bool)flags; }
} AxisFlags;

//
// Enumerated axis indices
//
//  - X_AXIS, Y_AXIS, and Z_AXIS should be used for axes in Cartesian space
//  - A_AXIS, B_AXIS, and C_AXIS should be used for Steppers, corresponding to XYZ on Cartesians
//  - X_HEAD, Y_HEAD, and Z_HEAD should be used for axes on Core kinematics
//
enum AxisEnum : uint8_t {

  // Linear axes may be controlled directly or indirectly
  NUM_AXIS_LIST_(X_AXIS, Y_AXIS, Z_AXIS, I_AXIS, J_AXIS, K_AXIS, U_AXIS, V_AXIS, W_AXIS)

  #define _EN_ITEM(N) E##N##_AXIS,
  REPEAT(EXTRUDERS, _EN_ITEM)
  #undef _EN_ITEM

  // Core also keeps toolhead directions
  #if ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX)
    X_HEAD, Y_HEAD, Z_HEAD,
  #endif

  // Distinct axes, including all E and Core
  NUM_AXIS_HEADS,

  // Most of the time we refer only to the single E_AXIS
  #if HAS_EXTRUDERS
    E_AXIS = E0_AXIS,
  #endif

  // A, B, and C are for DELTA, SCARA, etc.
  #if HAS_X_AXIS
    A_AXIS = X_AXIS,
  #endif
  #if HAS_Y_AXIS
    B_AXIS = Y_AXIS,
  #endif
  #if HAS_Z_AXIS
    C_AXIS = Z_AXIS,
  #endif

  // To refer to all or none
  ALL_AXES_ENUM = 0xFE, NO_AXIS_ENUM = 0xFF
};

//
// Loop over axes
//
#define LOOP_ABC(VAR) for (uint8_t VAR = A_AXIS; VAR <= C_AXIS; ++VAR)
#define LOOP_NUM_AXES(VAR) for (uint8_t VAR = 0; VAR < NUM_AXES; ++VAR)
#define LOOP_LOGICAL_AXES(VAR) for (uint8_t VAR = 0; VAR < LOGICAL_AXES; ++VAR)
#define LOOP_DISTINCT_AXES(VAR) for (uint8_t VAR = 0; VAR < DISTINCT_AXES; ++VAR)
#define LOOP_DISTINCT_E(VAR) for (uint8_t VAR = 0; VAR < DISTINCT_E; ++VAR)

//
// feedRate_t is just a humble float that can represent mm/s or mm/min
//
typedef float feedRate_t;

//
// celsius_t is the native unit of temperature. Signed to handle a disconnected thermistor value (-14).
// For more resolition (e.g., for a chocolate printer) this may later be changed to Celsius x 100
//
typedef uint16_t raw_adc_t;
typedef int16_t celsius_t;
typedef float celsius_float_t;

//
// On AVR pointers are only 2 bytes so use 'const float &' for 'const float'
//
#ifdef __AVR__
  typedef const float & const_float_t;
#else
  typedef const float const_float_t;
#endif
typedef const_float_t const_feedRate_t;
typedef const_float_t const_celsius_float_t;

// Type large enough to count leveling grid points
typedef IF<TERN0(ABL_USES_GRID, (GRID_MAX_POINTS > 255)), uint16_t, uint8_t>::type grid_count_t;

// Conversion macros
#define MMM_TO_MMS(MM_M) feedRate_t(static_cast<float>(MM_M) / 60.0f)
#define MMS_TO_MMM(MM_S) (static_cast<float>(MM_S) * 60.0f)

// Packaged character for C macro and other usage
typedef struct SerialChar { char c; SerialChar(char n) : c(n) { } } serial_char_t;
#define C(c) serial_char_t(c)

// Packaged types: float with precision and/or width; a repeated space/character
typedef struct WFloat { float value; char width; char prec;
                        WFloat(float v, char w, char p) : value(v), width(w), prec(p) {}
                      } w_float_t;
typedef struct PFloat { float value; char prec;
                        PFloat(float v, char p) : value(v), prec(p) {}
                      } p_float_t;
typedef struct RepChr { char asc; int8_t count;
                        RepChr(char a, uint8_t c) : asc(a), count(c) {}
                      } repchr_t;
typedef struct Spaces { int8_t count;
                        Spaces(uint8_t c) : count(c) {}
                      } spaces_t;

#ifdef __AVR__
  typedef w_float_t w_double_t;
  typedef p_float_t p_double_t;
#else
  typedef struct WDouble { double value; char width; char prec;
                          WDouble(double v, char w, char p) : value(v), width(w), prec(p) {}
                        } w_double_t;
  typedef struct PDouble { double value; char prec;
                          PDouble(double v, char p) : value(v), prec(p) {}
                        } p_double_t;
#endif

//
// Coordinates structures for XY, XYZ, XYZE...
//

// Forward declarations
template<typename T> struct XYval;
template<typename T> struct XYZval;
template<typename T> struct XYZEval;

typedef struct XYval<bool>          xy_bool_t;
typedef struct XYZval<bool>        xyz_bool_t;
typedef struct XYZEval<bool>      xyze_bool_t;

typedef struct XYval<char>          xy_char_t;
typedef struct XYZval<char>        xyz_char_t;
typedef struct XYZEval<char>      xyze_char_t;

typedef struct XYval<unsigned char>     xy_uchar_t;
typedef struct XYZval<unsigned char>   xyz_uchar_t;
typedef struct XYZEval<unsigned char> xyze_uchar_t;

typedef struct XYval<int8_t>        xy_int8_t;
typedef struct XYZval<int8_t>      xyz_int8_t;
typedef struct XYZEval<int8_t>    xyze_int8_t;

typedef struct XYval<uint8_t>      xy_uint8_t;
typedef struct XYZval<uint8_t>    xyz_uint8_t;
typedef struct XYZEval<uint8_t>  xyze_uint8_t;

typedef struct XYval<int16_t>        xy_int_t;
typedef struct XYZval<int16_t>      xyz_int_t;
typedef struct XYZEval<int16_t>    xyze_int_t;

typedef struct XYval<uint16_t>      xy_uint_t;
typedef struct XYZval<uint16_t>    xyz_uint_t;
typedef struct XYZEval<uint16_t>  xyze_uint_t;

typedef struct XYval<int32_t>       xy_long_t;
typedef struct XYZval<int32_t>     xyz_long_t;
typedef struct XYZEval<int32_t>   xyze_long_t;

typedef struct XYval<uint32_t>     xy_ulong_t;
typedef struct XYZval<uint32_t>   xyz_ulong_t;
typedef struct XYZEval<uint32_t> xyze_ulong_t;

typedef struct XYZval<volatile int32_t>   xyz_vlong_t;
typedef struct XYZEval<volatile int32_t> xyze_vlong_t;

typedef struct XYval<float>        xy_float_t;
typedef struct XYZval<float>      xyz_float_t;
typedef struct XYZEval<float>    xyze_float_t;

typedef struct XYval<feedRate_t>     xy_feedrate_t;
typedef struct XYZval<feedRate_t>   xyz_feedrate_t;
typedef struct XYZEval<feedRate_t> xyze_feedrate_t;

typedef xy_uint8_t xy_byte_t;
typedef xyz_uint8_t xyz_byte_t;
typedef xyze_uint8_t xyze_byte_t;

typedef xyz_long_t abc_long_t;
typedef xyze_long_t abce_long_t;
typedef xyz_ulong_t abc_ulong_t;
typedef xyze_ulong_t abce_ulong_t;

typedef xy_float_t xy_pos_t;
typedef xyz_float_t xyz_pos_t;
typedef xyze_float_t xyze_pos_t;

typedef xy_float_t ab_float_t;
typedef xyz_float_t abc_float_t;
typedef xyze_float_t abce_float_t;

typedef ab_float_t ab_pos_t;
typedef abc_float_t abc_pos_t;
typedef abce_float_t abce_pos_t;

// External conversion methods
void toLogical(xy_pos_t &raw);
void toLogical(xyz_pos_t &raw);
void toLogical(xyze_pos_t &raw);
void toNative(xy_pos_t &lpos);
void toNative(xyz_pos_t &lpos);
void toNative(xyze_pos_t &lpos);

//
// Paired XY coordinates, counters, flags, etc.
// Always has XY elements regardless of the number of configured axes.
//
template<typename T>
struct XYval {
  union {
    struct { T x, y; };
    struct { T X, Y; };
    struct { T a, b; };
    struct { T A, B; };
    T pos[2];
  };

  // Set all to 0
  FI void reset()                                 { x = y = 0; }

  // Setters taking struct types and arrays
  #if HAS_X_AXIS
    FI void set(const T px)                       { x = px; }
  #endif
  #if HAS_Y_AXIS
    FI void set(const T px, const T py)           { x = px; y = py; }
    FI void set(const T (&arr)[XY])               { x = arr[0]; y = arr[1]; }
  #endif
  #if NUM_AXES > XY
    FI void set(const T (&arr)[NUM_AXES])         { x = arr[0]; y = arr[1]; }
  #endif
  #if LOGICAL_AXES > NUM_AXES
    FI void set(const T (&arr)[LOGICAL_AXES])     { x = arr[0]; y = arr[1]; }
    #if DISTINCT_AXES > LOGICAL_AXES
      FI void set(const T (&arr)[DISTINCT_AXES])  { x = arr[0]; y = arr[1]; }
    #endif
  #endif

  // Length reduced to one dimension
  FI constexpr T magnitude()    const { return (T)sqrtf(x*x + y*y); }
  // Pointer to the data as a simple array
  explicit FI operator T* ()          { return pos; }
  // If any element is true then it's true
  FI constexpr operator bool()  const { return x || y; }
  // Smallest element
  FI constexpr T small()        const { return _MIN(x, y); }
  // Largest element
  FI constexpr T large()        const { return _MAX(x, y); }

  // Explicit copy and copies with conversion
  FI constexpr XYval<T>           copy()  const { return *this; }
  FI constexpr XYval<T>            ABS()  const { return { T(_ABS(x)), T(_ABS(y)) }; }
  FI constexpr XYval<int16_t>    asInt()  const { return { int16_t(x), int16_t(y) }; }
  FI constexpr XYval<int32_t>   asLong()  const { return { int32_t(x), int32_t(y) }; }
  FI constexpr XYval<int32_t>   ROUNDL()  const { return { int32_t(LROUND(x)), int32_t(LROUND(y)) }; }
  FI constexpr XYval<float>    asFloat()  const { return { static_cast<float>(x), static_cast<float>(y) }; }
  FI constexpr XYval<float> reciprocal()  const { return { _RECIP(x), _RECIP(y) }; }

  // Marlin workspace shifting is done with G92 and M206
  FI XYval<float> asLogical() const { XYval<float> o = asFloat(); toLogical(o); return o; }
  FI XYval<float>  asNative() const { XYval<float> o = asFloat(); toNative(o);  return o; }

  // Cast to a type with more fields by making a new object
  FI constexpr operator XYZval<T>()  const { return { x, y }; }
  FI constexpr operator XYZEval<T>() const { return { x, y }; }

  // Accessor via an AxisEnum (or any integer) [index]
  FI       T&  operator[](const int n)       { return pos[n]; }
  FI const T&  operator[](const int n) const { return pos[n]; }

  // Assignment operator overrides do the expected thing
  FI XYval<T>& operator= (const T v)              { set(v, v); return *this; }
  FI XYval<T>& operator= (const XYZval<T>  &rs)   { set(XY_LIST(rs.x, rs.y)); return *this; }
  FI XYval<T>& operator= (const XYZEval<T> &rs)   { set(XY_LIST(rs.x, rs.y)); return *this; }

  // Override other operators to get intuitive behaviors
  #define XY_OP(OP) { x TERN_(HAS_X_AXIS, OP rs.x), y TERN_(HAS_Y_AXIS, OP rs.y) }
  FI constexpr XYval<T> operator+ (const XYval<T>   &rs) const { return { x + rs.x, y + rs.y }; }
  FI constexpr XYval<T> operator- (const XYval<T>   &rs) const { return { x - rs.x, y - rs.y }; }
  FI constexpr XYval<T> operator* (const XYval<T>   &rs) const { return { x * rs.x, y * rs.y }; }
  FI constexpr XYval<T> operator/ (const XYval<T>   &rs) const { return { x / rs.x, y / rs.y }; }
  FI constexpr XYval<T> operator+ (const XYZval<T>  &rs) const { return { XY_OP(+) }; }
  FI constexpr XYval<T> operator- (const XYZval<T>  &rs) const { return { XY_OP(-) }; }
  FI constexpr XYval<T> operator* (const XYZval<T>  &rs) const { return { XY_OP(*) }; }
  FI constexpr XYval<T> operator/ (const XYZval<T>  &rs) const { return { XY_OP(/) }; }
  FI constexpr XYval<T> operator+ (const XYZEval<T> &rs) const { return { XY_OP(+) }; }
  FI constexpr XYval<T> operator- (const XYZEval<T> &rs) const { return { XY_OP(-) }; }
  FI constexpr XYval<T> operator* (const XYZEval<T> &rs) const { return { XY_OP(*) }; }
  FI constexpr XYval<T> operator/ (const XYZEval<T> &rs) const { return { XY_OP(/) }; }
  FI constexpr XYval<T> operator* (const float &p)       const { return { (T)(x * p), (T)(y * p) }; }
  FI constexpr XYval<T> operator* (const int &p)         const { return { x * p, y * p }; }
  FI constexpr XYval<T> operator/ (const float &p)       const { return { (T)(x / p), (T)(y / p) }; }
  FI constexpr XYval<T> operator/ (const int &p)         const { return { x / p, y / p }; }
  FI constexpr XYval<T> operator>>(const int &p)         const { return { _RS(x), _RS(y) }; }
  FI constexpr XYval<T> operator<<(const int &p)         const { return { _LS(x), _LS(y) }; }
  FI constexpr XYval<T> operator-()                      const { return { -x, -y }; }
  #undef XY_OP

  // Modifier operators
  FI XYval<T>& operator+=(const XYval<T>   &rs) { x += rs.x; y += rs.y; return *this; }
  FI XYval<T>& operator-=(const XYval<T>   &rs) { x -= rs.x; y -= rs.y; return *this; }
  FI XYval<T>& operator*=(const XYval<T>   &rs) { x *= rs.x; y *= rs.y; return *this; }
  FI XYval<T>& operator/=(const XYval<T>   &rs) { x /= rs.x; y /= rs.y; return *this; }
  FI XYval<T>& operator+=(const XYZval<T>  &rs) { XY_CODE(x += rs.x, y += rs.y); return *this; }
  FI XYval<T>& operator-=(const XYZval<T>  &rs) { XY_CODE(x -= rs.x, y -= rs.y); return *this; }
  FI XYval<T>& operator*=(const XYZval<T>  &rs) { XY_CODE(x *= rs.x, y *= rs.y); return *this; }
  FI XYval<T>& operator/=(const XYZval<T>  &rs) { XY_CODE(x /= rs.x, y /= rs.y); return *this; }
  FI XYval<T>& operator+=(const XYZEval<T> &rs) { XY_CODE(x += rs.x, y += rs.y); return *this; }
  FI XYval<T>& operator-=(const XYZEval<T> &rs) { XY_CODE(x -= rs.x, y -= rs.y); return *this; }
  FI XYval<T>& operator*=(const XYZEval<T> &rs) { XY_CODE(x *= rs.x, y *= rs.y); return *this; }
  FI XYval<T>& operator/=(const XYZEval<T> &rs) { XY_CODE(x /= rs.x, y /= rs.y); return *this; }
  FI XYval<T>& operator*=(const float &p)       { x *= p; y *= p; return *this; }
  FI XYval<T>& operator*=(const int &p)         { x *= p; y *= p; return *this; }
  FI XYval<T>& operator>>=(const int &p)        { _RSE(x); _RSE(y); return *this; }
  FI XYval<T>& operator<<=(const int &p)        { _LSE(x); _LSE(y); return *this; }

  // Absolute difference between two objects
  FI constexpr XYval<T> diff(const XYZEval<T> &rs) const { return { TERN(HAS_X_AXIS, T(_ABS(x - rs.x)), x), TERN(HAS_Y_AXIS, T(_ABS(y - rs.y)), y) }; }
  FI constexpr XYval<T> diff(const XYZval<T>  &rs) const { return { TERN(HAS_X_AXIS, T(_ABS(x - rs.x)), x), TERN(HAS_Y_AXIS, T(_ABS(y - rs.y)), y) }; }
  FI constexpr XYval<T> diff(const XYval<T>   &rs) const { return { T(_ABS(x - rs.x)), T(_ABS(y - rs.y)) }; }

  // Exact comparisons. For floats a "NEAR" operation may be better.
  FI bool operator==(const XYval<T>   &rs) const { return x == rs.x && y == rs.y; }
  FI bool operator==(const XYZval<T>  &rs) const { return ENABLED(HAS_X_AXIS) XY_GANG(&& x == rs.x, && y == rs.y); }
  FI bool operator==(const XYZEval<T> &rs) const { return ENABLED(HAS_X_AXIS) XY_GANG(&& x == rs.x, && y == rs.y); }
  FI bool operator!=(const XYval<T>   &rs) const { return !operator==(rs); }
  FI bool operator!=(const XYZval<T>  &rs) const { return !operator==(rs); }
  FI bool operator!=(const XYZEval<T> &rs) const { return !operator==(rs); }
};

//
// Linear Axes coordinates, counters, flags, etc.
// May have any number of axes according to configuration, including zero axes.
//
template<typename T>
struct XYZval {
  union {
    #if NUM_AXES
      struct { NUM_AXIS_CODE(T x, T y, T z, T i, T j, T k, T u, T v, T w); };
      struct { NUM_AXIS_CODE(T X, T Y, T Z, T I, T J, T K, T U, T V, T W); };
      struct { NUM_AXIS_CODE(T a, T b, T c, T _i, T _j, T _k, T _u, T _v, T _w); };
      struct { NUM_AXIS_CODE(T A, T B, T C, T II, T JJ, T KK, T UU, T VV, T WW); };
    #endif
    T pos[NUM_AXES];
  };

  // Set all to 0
  FI void reset()                                { NUM_AXIS_CODE(x = 0, y = 0, z = 0, i = 0, j = 0, k = 0, u = 0, v = 0, w = 0); }

  // Setters taking struct types and arrays
  FI void set(const XYval<T> pxy)                { XY_CODE(x = pxy.x, y = pxy.y); }
  FI void set(const XYval<T> pxy, const T pz)    { XYZ_CODE(x = pxy.x, y = pxy.y, z = pz); }
  FI void set(const T (&arr)[NUM_AXES])          { NUM_AXIS_CODE(x = arr[0], y = arr[1], z = arr[2], i = arr[3], j = arr[4], k = arr[5], u = arr[6], v = arr[7], w = arr[8]); }
  #if LOGICAL_AXES > NUM_AXES
    FI void set(const T (&arr)[LOGICAL_AXES])    { NUM_AXIS_CODE(x = arr[0], y = arr[1], z = arr[2], i = arr[3], j = arr[4], k = arr[5], u = arr[6], v = arr[7], w = arr[8]); }
    FI void set(LOGICAL_AXIS_ARGS_LC(const T))   { NUM_AXIS_CODE(a = x,  b = y,  c = z, _i = i, _j = j, _k = k, _u = u, _v = v, _w = w); }
    #if DISTINCT_AXES > LOGICAL_AXES
      FI void set(const T (&arr)[DISTINCT_AXES]) { NUM_AXIS_CODE(x = arr[0], y = arr[1], z = arr[2], i = arr[3], j = arr[4], k = arr[5], u = arr[6], v = arr[7], w = arr[8]); }
    #endif
  #endif

  // Setter for all individual args
  FI void set(NUM_AXIS_ARGS_LC(const T)) { NUM_AXIS_CODE(a = x, b = y, c = z, _i = i, _j = j, _k = k, _u = u, _v = v, _w = w); }

  // Setters with fewer elements leave the rest untouched
  #if HAS_Y_AXIS
    FI void set(const T px) { x = px; }
  #endif
  #if HAS_Z_AXIS
    FI void set(const T px, const T py) { x = px; y = py; }
  #endif
  #if HAS_I_AXIS
    FI void set(const T px, const T py, const T pz) { x = px; y = py; z = pz; }
  #endif
  #if HAS_J_AXIS
    FI void set(const T px, const T py, const T pz, const T pi) { x = px; y = py; z = pz; i = pi; }
  #endif
  #if HAS_K_AXIS
    FI void set(const T px, const T py, const T pz, const T pi, const T pj) { x = px; y = py; z = pz; i = pi; j = pj; }
  #endif
  #if HAS_U_AXIS
    FI void set(const T px, const T py, const T pz, const T pi, const T pj, const T pk) { x = px; y = py; z = pz; i = pi; j = pj; k = pk; }
  #endif
  #if HAS_V_AXIS
    FI void set(const T px, const T py, const T pz, const T pi, const T pj, const T pk, const T pu) { x = px; y = py; z = pz; i = pi; j = pj; k = pk; u = pu; }
  #endif
  #if HAS_W_AXIS
    FI void set(const T px, const T py, const T pz, const T pi, const T pj, const T pk, const T pu, const T pv) { x = px; y = py; z = pz; i = pi; j = pj; k = pk; u = pu; v = pv; }
  #endif

  // Length reduced to one dimension
  FI constexpr T magnitude()    const { return (T)TERN(HAS_X_AXIS, sqrtf(NUM_AXIS_GANG(x*x, + y*y, + z*z, + i*i, + j*j, + k*k, + u*u, + v*v, + w*w)), 0); }
  // Pointer to the data as a simple array
  explicit FI operator T* ()          { return pos; }
  // If any element is true then it's true
  FI constexpr operator bool()  const { return 0 NUM_AXIS_GANG(|| x, || y, || z, || i, || j, || k, || u, || v, || w); }
  // Smallest element
  FI constexpr T small()        const { return TERN0(HAS_X_AXIS, _MIN(NUM_AXIS_LIST(x, y, z, i, j, k, u, v, w))); }
  // Largest element
  FI constexpr T large()        const { return TERN0(HAS_X_AXIS, _MAX(NUM_AXIS_LIST(x, y, z, i, j, k, u, v, w))); }

  // Explicit copy and copies with conversion
  FI constexpr XYZval<T>           copy() const { XYZval<T> o = *this; return o; }
  FI constexpr XYZval<T>            ABS() const { return NUM_AXIS_ARRAY(T(_ABS(x)), T(_ABS(y)), T(_ABS(z)), T(_ABS(i)), T(_ABS(j)), T(_ABS(k)), T(_ABS(u)), T(_ABS(v)), T(_ABS(w))); }
  FI constexpr XYZval<int16_t>    asInt() const { return NUM_AXIS_ARRAY(int16_t(x), int16_t(y), int16_t(z), int16_t(i), int16_t(j), int16_t(k), int16_t(u), int16_t(v), int16_t(w)); }
  FI constexpr XYZval<int32_t>   asLong() const { return NUM_AXIS_ARRAY(int32_t(x), int32_t(y), int32_t(z), int32_t(i), int32_t(j), int32_t(k), int32_t(u), int32_t(v), int32_t(w)); }
  FI constexpr XYZval<int32_t>   ROUNDL() const { return NUM_AXIS_ARRAY(int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)), int32_t(LROUND(i)), int32_t(LROUND(j)), int32_t(LROUND(k)), int32_t(LROUND(u)), int32_t(LROUND(v)), int32_t(LROUND(w))); }
  FI constexpr XYZval<float>    asFloat() const { return NUM_AXIS_ARRAY(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(i), static_cast<float>(j), static_cast<float>(k), static_cast<float>(u), static_cast<float>(v), static_cast<float>(w)); }
  FI constexpr XYZval<float> reciprocal() const { return NUM_AXIS_ARRAY(_RECIP(x),  _RECIP(y),  _RECIP(z),  _RECIP(i),  _RECIP(j),  _RECIP(k),  _RECIP(u),  _RECIP(v),  _RECIP(w)); }

  // Marlin workspace shifting is done with G92 and M206
  FI XYZval<float> asLogical() const { XYZval<float> o = asFloat(); toLogical(o); return o; }
  FI XYZval<float>  asNative() const { XYZval<float> o = asFloat(); toNative(o);  return o; }

  // In-place cast to types having fewer fields
  FI operator       XYval<T>&()       { return *(XYval<T>*)this; }
  FI operator const XYval<T>&() const { return *(const XYval<T>*)this; }

  // Cast to a type with more fields by making a new object
  FI constexpr operator XYZEval<T>()  const { return NUM_AXIS_ARRAY(x, y, z, i, j, k, u, v, w); }

  // Accessor via an AxisEnum (or any integer) [index]
  FI       T& operator[](const int n)       { return pos[n]; }
  FI const T& operator[](const int n) const { return pos[n]; }

  // Assignment operator overrides do the expected thing
  FI XYZval<T>& operator= (const T v)            { set(ARRAY_N_1(NUM_AXES, v)); return *this; }
  FI XYZval<T>& operator= (const XYval<T>   &rs) { set(rs.x, rs.y); return *this; }
  FI XYZval<T>& operator= (const XYZEval<T> &rs) { set(NUM_AXIS_ELEM_LC(rs)); return *this; }

  // Override other operators to get intuitive behaviors
  FI constexpr XYZval<T> operator+ (const XYval<T>   &rs) const { return NUM_AXIS_ARRAY(x + rs.x, y + rs.y, z, i, j, k, u, v, w ); }
  FI constexpr XYZval<T> operator- (const XYval<T>   &rs) const { return NUM_AXIS_ARRAY(x - rs.x, y - rs.y, z, i, j, k, u, v, w ); }
  FI constexpr XYZval<T> operator* (const XYval<T>   &rs) const { return NUM_AXIS_ARRAY(x * rs.x, y * rs.y, z, i, j, k, u, v, w ); }
  FI constexpr XYZval<T> operator/ (const XYval<T>   &rs) const { return NUM_AXIS_ARRAY(x / rs.x, y / rs.y, z, i, j, k, u, v, w ); }
  FI constexpr XYZval<T> operator+ (const XYZval<T>  &rs) const { return NUM_AXIS_ARRAY(x + rs.x, y + rs.y, z + rs.z, i + rs.i, j + rs.j, k + rs.k, u + rs.u, v + rs.v, w + rs.w ); }
  FI constexpr XYZval<T> operator- (const XYZval<T>  &rs) const { return NUM_AXIS_ARRAY(x - rs.x, y - rs.y, z - rs.z, i - rs.i, j - rs.j, k - rs.k, u - rs.u, v - rs.v, w - rs.w ); }
  FI constexpr XYZval<T> operator* (const XYZval<T>  &rs) const { return NUM_AXIS_ARRAY(x * rs.x, y * rs.y, z * rs.z, i * rs.i, j * rs.j, k * rs.k, u * rs.u, v * rs.v, w * rs.w ); }
  FI constexpr XYZval<T> operator/ (const XYZval<T>  &rs) const { return NUM_AXIS_ARRAY(x / rs.x, y / rs.y, z / rs.z, i / rs.i, j / rs.j, k / rs.k, u / rs.u, v / rs.v, w / rs.w ); }
  FI constexpr XYZval<T> operator+ (const XYZEval<T> &rs) const { return NUM_AXIS_ARRAY(x + rs.x, y + rs.y, z + rs.z, i + rs.i, j + rs.j, k + rs.k, u + rs.u, v + rs.v, w + rs.w ); }
  FI constexpr XYZval<T> operator- (const XYZEval<T> &rs) const { return NUM_AXIS_ARRAY(x - rs.x, y - rs.y, z - rs.z, i - rs.i, j - rs.j, k - rs.k, u - rs.u, v - rs.v, w - rs.w ); }
  FI constexpr XYZval<T> operator* (const XYZEval<T> &rs) const { return NUM_AXIS_ARRAY(x * rs.x, y * rs.y, z * rs.z, i * rs.i, j * rs.j, k * rs.k, u * rs.u, v * rs.v, w * rs.w ); }
  FI constexpr XYZval<T> operator/ (const XYZEval<T> &rs) const { return NUM_AXIS_ARRAY(x / rs.x, y / rs.y, z / rs.z, i / rs.i, j / rs.j, k / rs.k, u / rs.u, v / rs.v, w / rs.w ); }
  FI constexpr XYZval<T> operator* (const float &p)       const { return NUM_AXIS_ARRAY((T)(x * p), (T)(y * p), (T)(z * p), (T)(i * p), (T)(j * p), (T)(k * p), (T)(u * p), (T)(v * p), (T)(w * p)); }
  FI constexpr XYZval<T> operator* (const int &p)         const { return NUM_AXIS_ARRAY(x * p, y * p, z * p, i * p, j * p, k * p, u * p, v * p, w * p); }
  FI constexpr XYZval<T> operator/ (const float &p)       const { return NUM_AXIS_ARRAY((T)(x / p), (T)(y / p), (T)(z / p), (T)(i / p), (T)(j / p), (T)(k / p), (T)(u / p), (T)(v / p), (T)(w / p)); }
  FI constexpr XYZval<T> operator/ (const int &p)         const { return NUM_AXIS_ARRAY(x / p, y / p, z / p, i / p, j / p, k / p, u / p, v / p, w / p); }
  FI constexpr XYZval<T> operator>>(const int &p)         const { return NUM_AXIS_ARRAY(_RS(x), _RS(y), _RS(z), _RS(i), _RS(j), _RS(k), _RS(u), _RS(v), _RS(w)); }
  FI constexpr XYZval<T> operator<<(const int &p)         const { return NUM_AXIS_ARRAY(_LS(x), _LS(y), _LS(z), _LS(i), _LS(j), _LS(k), _LS(u), _LS(v), _LS(w)); }
  FI constexpr XYZval<T> operator-()                      const { return NUM_AXIS_ARRAY(-x, -y, -z, -i, -j, -k, -u, -v, -w); }

  // Absolute difference between two objects
  FI constexpr XYZval<T> diff(const XYZEval<T> &rs) const { return NUM_AXIS_ARRAY(T(_ABS(x - rs.x)), T(_ABS(y - rs.y)), T(_ABS(z - rs.z)), T(_ABS(i - rs.i)), T(_ABS(j - rs.j)), T(_ABS(k - rs.k)), T(_ABS(u - rs.u)), T(_ABS(v - rs.v)), T(_ABS(w - rs.w)) ); }
  FI constexpr XYZval<T> diff(const XYZval<T> &rs)  const { return NUM_AXIS_ARRAY(T(_ABS(x - rs.x)), T(_ABS(y - rs.y)), T(_ABS(z - rs.z)), T(_ABS(i - rs.i)), T(_ABS(j - rs.j)), T(_ABS(k - rs.k)), T(_ABS(u - rs.u)), T(_ABS(v - rs.v)), T(_ABS(w - rs.w)) ); }
  FI constexpr XYZval<T> diff(const XYval<T>   &rs) const { return NUM_AXIS_ARRAY(T(_ABS(x - rs.x)), T(_ABS(y - rs.y)), z, i, j, k, u, v, w ); }

  // Modifier operators
  FI XYZval<T>& operator+=(const XYval<T>   &rs) { XY_CODE(x += rs.x, y += rs.y); return *this; }
  FI XYZval<T>& operator-=(const XYval<T>   &rs) { XY_CODE(x -= rs.x, y -= rs.y); return *this; }
  FI XYZval<T>& operator*=(const XYval<T>   &rs) { XY_CODE(x *= rs.x, y *= rs.y); return *this; }
  FI XYZval<T>& operator/=(const XYval<T>   &rs) { XY_CODE(x /= rs.x, y /= rs.y); return *this; }
  FI XYZval<T>& operator+=(const XYZval<T>  &rs) { NUM_AXIS_CODE(x += rs.x, y += rs.y, z += rs.z, i += rs.i, j += rs.j, k += rs.k, u += rs.u, v += rs.v, w += rs.w); return *this; }
  FI XYZval<T>& operator-=(const XYZval<T>  &rs) { NUM_AXIS_CODE(x -= rs.x, y -= rs.y, z -= rs.z, i -= rs.i, j -= rs.j, k -= rs.k, u -= rs.u, v -= rs.v, w -= rs.w); return *this; }
  FI XYZval<T>& operator*=(const XYZval<T>  &rs) { NUM_AXIS_CODE(x *= rs.x, y *= rs.y, z *= rs.z, i *= rs.i, j *= rs.j, k *= rs.k, u *= rs.u, v *= rs.v, w *= rs.w); return *this; }
  FI XYZval<T>& operator/=(const XYZval<T>  &rs) { NUM_AXIS_CODE(x /= rs.x, y /= rs.y, z /= rs.z, i /= rs.i, j /= rs.j, k /= rs.k, u /= rs.u, v /= rs.v, w /= rs.w); return *this; }
  FI XYZval<T>& operator+=(const XYZEval<T> &rs) { NUM_AXIS_CODE(x += rs.x, y += rs.y, z += rs.z, i += rs.i, j += rs.j, k += rs.k, u += rs.u, v += rs.v, w += rs.w); return *this; }
  FI XYZval<T>& operator-=(const XYZEval<T> &rs) { NUM_AXIS_CODE(x -= rs.x, y -= rs.y, z -= rs.z, i -= rs.i, j -= rs.j, k -= rs.k, u -= rs.u, v -= rs.v, w -= rs.w); return *this; }
  FI XYZval<T>& operator*=(const XYZEval<T> &rs) { NUM_AXIS_CODE(x *= rs.x, y *= rs.y, z *= rs.z, i *= rs.i, j *= rs.j, k *= rs.k, u *= rs.u, v *= rs.v, w *= rs.w); return *this; }
  FI XYZval<T>& operator/=(const XYZEval<T> &rs) { NUM_AXIS_CODE(x /= rs.x, y /= rs.y, z /= rs.z, i /= rs.i, j /= rs.j, k /= rs.k, u /= rs.u, v /= rs.v, w /= rs.w); return *this; }
  FI XYZval<T>& operator*=(const float &p)       { NUM_AXIS_CODE(x *= p, y *= p, z *= p, i *= p, j *= p, k *= p, u *= p, v *= p, w *= p); return *this; }
  FI XYZval<T>& operator*=(const int &p)         { NUM_AXIS_CODE(x *= p, y *= p, z *= p, i *= p, j *= p, k *= p, u *= p, v *= p, w *= p); return *this; }
  FI XYZval<T>& operator>>=(const int &p)        { NUM_AXIS_CODE(_RSE(x), _RSE(y), _RSE(z), _RSE(i), _RSE(j), _RSE(k), _RSE(u), _RSE(v), _RSE(w)); return *this; }
  FI XYZval<T>& operator<<=(const int &p)        { NUM_AXIS_CODE(_LSE(x), _LSE(y), _LSE(z), _LSE(i), _LSE(j), _LSE(k), _LSE(u), _LSE(v), _LSE(w)); return *this; }

  // Exact comparisons. For floats a "NEAR" operation may be better.
  FI bool operator==(const XYZEval<T> &rs) const { return true NUM_AXIS_GANG(&& x == rs.x, && y == rs.y, && z == rs.z, && i == rs.i, && j == rs.j, && k == rs.k, && u == rs.u, && v == rs.v, && w == rs.w); }
  FI bool operator!=(const XYZEval<T> &rs) const { return !operator==(rs); }
};

//
// Logical Axes coordinates, counters, etc.
// May have any number of axes according to configuration, including zero axes.
// When there is no extruder, essentially identical to XYZval.
//
template<typename T>
struct XYZEval {
  union {
    struct { T LOGICAL_AXIS_ARGS_LC(); };
    struct { T LOGICAL_AXIS_ARGS(); };
    struct { T LOGICAL_AXIS_LIST(_e, a, b, c, _i, _j, _k, _u, _v, _w); };
    struct { T LOGICAL_AXIS_LIST(EE, A, B, C, II, JJ, KK, UU, VV, WW); };
    T pos[LOGICAL_AXES];
  };
  // Reset all to 0
  FI void reset()                                { LOGICAL_AXIS_GANG(e =, x =, y =, z =, i =, j =, k =, u =, v =, w =) 0; }

  // Setters taking struct types and arrays
  FI void set(const XYval<T> pxy)                           { XY_CODE(x = pxy.x, y = pxy.y); }
  FI void set(const XYval<T> pxy, const T pz)               { XYZ_CODE(x = pxy.x, y = pxy.y, z = pz); }
  FI void set(const XYZval<T> pxyz)                         { set(NUM_AXIS_ELEM_LC(pxyz)); }
  FI void set(const T (&arr)[NUM_AXES])                     { NUM_AXIS_CODE(x = arr[0], y = arr[1], z = arr[2], i = arr[3], j = arr[4], k = arr[5], u = arr[6], v = arr[7], w = arr[8]); }
  #if LOGICAL_AXES > NUM_AXES
    FI void set(const T (&arr)[LOGICAL_AXES])               { LOGICAL_AXIS_CODE(e = arr[LOGICAL_AXES-1], x = arr[0], y = arr[1], z = arr[2], i = arr[3], j = arr[4], k = arr[5], u = arr[6], v = arr[7], w = arr[8]); }
    FI void set(const XYval<T> pxy, const T pz, const T pe) { set(pxy, pz); e = pe; }
    FI void set(const XYZval<T> pxyz, const T pe)           { set(pxyz); e = pe; }
    FI void set(LOGICAL_AXIS_ARGS_LC(const T))              { LOGICAL_AXIS_CODE(_e = e, a = x, b = y, c = z, _i = i, _j = j, _k = k, _u = u, _v = v, _w = w); }
    #if DISTINCT_AXES > LOGICAL_AXES
      FI void set(const T (&arr)[DISTINCT_AXES])            { LOGICAL_AXIS_CODE(e = arr[LOGICAL_AXES-1], x = arr[0], y = arr[1], z = arr[2], i = arr[3], j = arr[4], k = arr[5], u = arr[6], v = arr[7], w = arr[8]); }
    #endif
  #endif

  // Setter for all individual args
  FI void set(NUM_AXIS_ARGS_LC(const T)) { NUM_AXIS_CODE(a = x, b = y, c = z, _i = i, _j = j, _k = k, _u = u, _v = v, _w = w); }

  // Setters with fewer elements leave the rest untouched
  #if HAS_Y_AXIS
    FI void set(const T px)                                                                                     { x = px; }
  #endif
  #if HAS_Z_AXIS
    FI void set(const T px, const T py)                                                                         { x = px; y = py; }
  #endif
  #if HAS_I_AXIS
    FI void set(const T px, const T py, const T pz)                                                             { x = px; y = py; z = pz; }
  #endif
  #if HAS_J_AXIS
    FI void set(const T px, const T py, const T pz, const T pi)                                                 { x = px; y = py; z = pz; i = pi; }
  #endif
  #if HAS_K_AXIS
    FI void set(const T px, const T py, const T pz, const T pi, const T pj)                                     { x = px; y = py; z = pz; i = pi; j = pj; }
  #endif
  #if HAS_U_AXIS
    FI void set(const T px, const T py, const T pz, const T pi, const T pj, const T pk)                         { x = px; y = py; z = pz; i = pi; j = pj; k = pk; }
  #endif
  #if HAS_V_AXIS
    FI void set(const T px, const T py, const T pz, const T pi, const T pj, const T pk, const T pu)             { x = px; y = py; z = pz; i = pi; j = pj; k = pk; u = pu; }
  #endif
  #if HAS_W_AXIS
    FI void set(const T px, const T py, const T pz, const T pi, const T pj, const T pk, const T pu, const T pv) { x = px; y = py; z = pz; i = pi; j = pj; k = pk; u = pu; v = pv; }
  #endif

  // Length reduced to one dimension
  FI constexpr T magnitude()    const { return (T)sqrtf(LOGICAL_AXIS_GANG(+ e*e, + x*x, + y*y, + z*z, + i*i, + j*j, + k*k, + u*u, + v*v, + w*w)); }
  // Pointer to the data as a simple array
  explicit FI operator T* ()          { return pos; }
  // If any element is true then it's true
  FI constexpr operator bool()  const { return 0 LOGICAL_AXIS_GANG(|| e, || x, || y, || z, || i, || j, || k, || u, || v, || w); }
  // Smallest element
  FI constexpr T small()        const { return _MIN(LOGICAL_AXIS_LIST(e, x, y, z, i, j, k, u, v, w)); }
  // Largest element
  FI constexpr T large()        const { return _MAX(LOGICAL_AXIS_LIST(e, x, y, z, i, j, k, u, v, w)); }

  // Explicit copy and copies with conversion
  FI constexpr XYZEval<T>           copy() const { XYZEval<T> v = *this; return v; }
  FI constexpr XYZEval<T>            ABS() const { return LOGICAL_AXIS_ARRAY(T(_ABS(e)), T(_ABS(x)), T(_ABS(y)), T(_ABS(z)), T(_ABS(i)), T(_ABS(j)), T(_ABS(k)), T(_ABS(u)), T(_ABS(v)), T(_ABS(w))); }
  FI constexpr XYZEval<int16_t>    asInt() const { return LOGICAL_AXIS_ARRAY(int16_t(e), int16_t(x), int16_t(y), int16_t(z), int16_t(i), int16_t(j), int16_t(k), int16_t(u), int16_t(v), int16_t(w)); }
  FI constexpr XYZEval<int32_t>   asLong() const { return LOGICAL_AXIS_ARRAY(int32_t(e), int32_t(x), int32_t(y), int32_t(z), int32_t(i), int32_t(j), int32_t(k), int32_t(u), int32_t(v), int32_t(w)); }
  FI constexpr XYZEval<int32_t>   ROUNDL() const { return LOGICAL_AXIS_ARRAY(int32_t(LROUND(e)), int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)), int32_t(LROUND(i)), int32_t(LROUND(j)), int32_t(LROUND(k)), int32_t(LROUND(u)), int32_t(LROUND(v)), int32_t(LROUND(w))); }
  FI constexpr XYZEval<float>    asFloat() const { return LOGICAL_AXIS_ARRAY(static_cast<float>(e), static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(i), static_cast<float>(j), static_cast<float>(k), static_cast<float>(u), static_cast<float>(v), static_cast<float>(w)); }
  FI constexpr XYZEval<float> reciprocal() const { return LOGICAL_AXIS_ARRAY(_RECIP(e),  _RECIP(x),  _RECIP(y),  _RECIP(z),  _RECIP(i),  _RECIP(j),  _RECIP(k),  _RECIP(u),  _RECIP(v),  _RECIP(w)); }

  // Marlin workspace shifting is done with G92 and M206
  FI XYZEval<float> asLogical() const { XYZEval<float> o = asFloat(); toLogical(o); return o; }
  FI XYZEval<float>  asNative() const { XYZEval<float> o = asFloat(); toNative(o);  return o; }

  // In-place cast to types having fewer fields
  FI operator       XYval<T>&()        { return *(XYval<T>*)this; }
  FI operator const XYval<T>&()  const { return *(const XYval<T>*)this; }
  FI operator       XYZval<T>&()       { return *(XYZval<T>*)this; }
  FI operator const XYZval<T>&() const { return *(const XYZval<T>*)this; }

  // Accessor via an AxisEnum (or any integer) [index]
  FI       T& operator[](const int n)       { return pos[n]; }
  FI const T& operator[](const int n) const { return pos[n]; }

  // Assignment operator overrides do the expected thing
  FI XYZEval<T>& operator= (const T v)           { set(LOGICAL_AXIS_LIST_1(v)); return *this; }
  FI XYZEval<T>& operator= (const XYval<T>  &rs) { set(rs.x, rs.y); return *this; }
  FI XYZEval<T>& operator= (const XYZval<T> &rs) { set(NUM_AXIS_ELEM_LC(rs)); return *this; }

  // Override other operators to get intuitive behaviors
  FI constexpr XYZEval<T> operator+ (const XYval<T>  &rs)  const { return LOGICAL_AXIS_ARRAY(e, x + rs.x, y + rs.y, z, i, j, k, u, v, w); }
  FI constexpr XYZEval<T> operator- (const XYval<T>  &rs)  const { return LOGICAL_AXIS_ARRAY(e, x - rs.x, y - rs.y, z, i, j, k, u, v, w); }
  FI constexpr XYZEval<T> operator* (const XYval<T>  &rs)  const { return LOGICAL_AXIS_ARRAY(e, x * rs.x, y * rs.y, z, i, j, k, u, v, w); }
  FI constexpr XYZEval<T> operator/ (const XYval<T>  &rs)  const { return LOGICAL_AXIS_ARRAY(e, x / rs.x, y / rs.y, z, i, j, k, u, v, w); }
  FI constexpr XYZEval<T> operator+ (const XYZval<T> &rs)  const { return LOGICAL_AXIS_ARRAY(e, x + rs.x, y + rs.y, z + rs.z, i + rs.i, j + rs.j, k + rs.k, u + rs.u, v + rs.v, w + rs.w); }
  FI constexpr XYZEval<T> operator- (const XYZval<T> &rs)  const { return LOGICAL_AXIS_ARRAY(e, x - rs.x, y - rs.y, z - rs.z, i - rs.i, j - rs.j, k - rs.k, u - rs.u, v - rs.v, w - rs.w); }
  FI constexpr XYZEval<T> operator* (const XYZval<T> &rs)  const { return LOGICAL_AXIS_ARRAY(e, x * rs.x, y * rs.y, z * rs.z, i * rs.i, j * rs.j, k * rs.k, u * rs.u, v * rs.v, w * rs.w); }
  FI constexpr XYZEval<T> operator/ (const XYZval<T> &rs)  const { return LOGICAL_AXIS_ARRAY(e, x / rs.x, y / rs.y, z / rs.z, i / rs.i, j / rs.j, k / rs.k, u / rs.u, v / rs.v, w / rs.w); }
  FI constexpr XYZEval<T> operator+ (const XYZEval<T> &rs) const { return LOGICAL_AXIS_ARRAY(e + rs.e, x + rs.x, y + rs.y, z + rs.z, i + rs.i, j + rs.j, k + rs.k, u + rs.u, v + rs.v, w + rs.w); }
  FI constexpr XYZEval<T> operator- (const XYZEval<T> &rs) const { return LOGICAL_AXIS_ARRAY(e - rs.e, x - rs.x, y - rs.y, z - rs.z, i - rs.i, j - rs.j, k - rs.k, u - rs.u, v - rs.v, w - rs.w); }
  FI constexpr XYZEval<T> operator* (const XYZEval<T> &rs) const { return LOGICAL_AXIS_ARRAY(e * rs.e, x * rs.x, y * rs.y, z * rs.z, i * rs.i, j * rs.j, k * rs.k, u * rs.u, v * rs.v, w * rs.w); }
  FI constexpr XYZEval<T> operator/ (const XYZEval<T> &rs) const { return LOGICAL_AXIS_ARRAY(e / rs.e, x / rs.x, y / rs.y, z / rs.z, i / rs.i, j / rs.j, k / rs.k, u / rs.u, v / rs.v, w / rs.w); }
  FI constexpr XYZEval<T> operator* (const float &p)       const { return LOGICAL_AXIS_ARRAY((T)(e * p), (T)(x * p), (T)(y * p), (T)(z * p), (T)(i * p), (T)(j * p), (T)(k * p), (T)(u * p), (T)(v * p), (T)(w * p)); }
  FI constexpr XYZEval<T> operator* (const int &p)         const { return LOGICAL_AXIS_ARRAY(e * p, x * p, y * p, z * p, i * p, j * p, k * p, u * p, v * p, w * p); }
  FI constexpr XYZEval<T> operator/ (const float &p)       const { return LOGICAL_AXIS_ARRAY((T)(e / p), (T)(x / p), (T)(y / p), (T)(z / p), (T)(i / p), (T)(j / p), (T)(k / p), (T)(u / p), (T)(v / p), (T)(w / p)); }
  FI constexpr XYZEval<T> operator/ (const int &p)         const { return LOGICAL_AXIS_ARRAY(e / p, x / p, y / p, z / p, i / p, j / p, k / p, u / p, v / p, w / p); }
  FI constexpr XYZEval<T> operator>>(const int &p)         const { return LOGICAL_AXIS_ARRAY(_RS(e), _RS(x), _RS(y), _RS(z), _RS(i), _RS(j), _RS(k), _RS(u), _RS(v), _RS(w)); }
  FI constexpr XYZEval<T> operator<<(const int &p)         const { return LOGICAL_AXIS_ARRAY(_LS(e), _LS(x), _LS(y), _LS(z), _LS(i), _LS(j), _LS(k), _LS(u), _LS(v), _LS(w)); }
  FI constexpr XYZEval<T> operator-()                      const { return LOGICAL_AXIS_ARRAY(-e, -x, -y, -z, -i, -j, -k, -u, -v, -w); }

  // Absolute difference between two objects
  FI constexpr XYZEval<T> diff(const XYZEval<T> &rs) const { return LOGICAL_AXIS_ARRAY(T(_ABS(e - rs.e)), T(_ABS(x - rs.x)), T(_ABS(y - rs.y)), T(_ABS(z - rs.z)), T(_ABS(i - rs.i)), T(_ABS(j - rs.j)), T(_ABS(k - rs.k)), T(_ABS(u - rs.u)), T(_ABS(v - rs.v)), T(_ABS(w - rs.w)) ); }
  FI constexpr XYZEval<T> diff(const XYZval<T>  &rs) const { return LOGICAL_AXIS_ARRAY(0 , T(_ABS(x - rs.x)), T(_ABS(y - rs.y)), T(_ABS(z - rs.z)), T(_ABS(i - rs.i)), T(_ABS(j - rs.j)), T(_ABS(k - rs.k)), T(_ABS(u - rs.u)), T(_ABS(v - rs.v)), T(_ABS(w - rs.w)) ); }
  FI constexpr XYZEval<T> diff(const XYval<T>   &rs) const { return LOGICAL_AXIS_ARRAY(0 , T(_ABS(x - rs.x)), T(_ABS(y - rs.y)), z, i, j, k, u, v, w ); }

  // Modifier operators
  FI XYZEval<T>& operator+=(const XYval<T>   &rs) { XY_CODE(x += rs.x, y += rs.y); return *this; }
  FI XYZEval<T>& operator-=(const XYval<T>   &rs) { XY_CODE(x -= rs.x, y -= rs.y); return *this; }
  FI XYZEval<T>& operator*=(const XYval<T>   &rs) { XY_CODE(x *= rs.x, y *= rs.y); return *this; }
  FI XYZEval<T>& operator/=(const XYval<T>   &rs) { XY_CODE(x /= rs.x, y /= rs.y); return *this; }
  FI XYZEval<T>& operator+=(const XYZval<T>  &rs) { NUM_AXIS_CODE(x += rs.x, y += rs.y, z += rs.z, i += rs.i, j += rs.j, k += rs.k, u += rs.u, v += rs.v, w += rs.w); return *this; }
  FI XYZEval<T>& operator-=(const XYZval<T>  &rs) { NUM_AXIS_CODE(x -= rs.x, y -= rs.y, z -= rs.z, i -= rs.i, j -= rs.j, k -= rs.k, u -= rs.u, v -= rs.v, w -= rs.w); return *this; }
  FI XYZEval<T>& operator*=(const XYZval<T>  &rs) { NUM_AXIS_CODE(x *= rs.x, y *= rs.y, z *= rs.z, i *= rs.i, j *= rs.j, k *= rs.k, u *= rs.u, v *= rs.v, w *= rs.w); return *this; }
  FI XYZEval<T>& operator/=(const XYZval<T>  &rs) { NUM_AXIS_CODE(x /= rs.x, y /= rs.y, z /= rs.z, i /= rs.i, j /= rs.j, k /= rs.k, u /= rs.u, v /= rs.v, w /= rs.w); return *this; }
  FI XYZEval<T>& operator+=(const XYZEval<T> &rs) { LOGICAL_AXIS_CODE(e += rs.e, x += rs.x, y += rs.y, z += rs.z, i += rs.i, j += rs.j, k += rs.k, u += rs.u, v += rs.v, w += rs.w); return *this; }
  FI XYZEval<T>& operator-=(const XYZEval<T> &rs) { LOGICAL_AXIS_CODE(e -= rs.e, x -= rs.x, y -= rs.y, z -= rs.z, i -= rs.i, j -= rs.j, k -= rs.k, u -= rs.u, v -= rs.v, w -= rs.w); return *this; }
  FI XYZEval<T>& operator*=(const XYZEval<T> &rs) { LOGICAL_AXIS_CODE(e *= rs.e, x *= rs.x, y *= rs.y, z *= rs.z, i *= rs.i, j *= rs.j, k *= rs.k, u *= rs.u, v *= rs.v, w *= rs.w); return *this; }
  FI XYZEval<T>& operator/=(const XYZEval<T> &rs) { LOGICAL_AXIS_CODE(e /= rs.e, x /= rs.x, y /= rs.y, z /= rs.z, i /= rs.i, j /= rs.j, k /= rs.k, u /= rs.u, v /= rs.v, w /= rs.w); return *this; }
  FI XYZEval<T>& operator*=(const T &p)           { LOGICAL_AXIS_CODE(e *= p, x *= p, y *= p, z *= p, i *= p, j *= p, k *= p, u *= p, v *= p, w *= p); return *this; }
  FI XYZEval<T>& operator>>=(const int &p)        { LOGICAL_AXIS_CODE(_RSE(e), _RSE(x), _RSE(y), _RSE(z), _RSE(i), _RSE(j), _RSE(k), _RSE(u), _RSE(v), _RSE(w)); return *this; }
  FI XYZEval<T>& operator<<=(const int &p)        { LOGICAL_AXIS_CODE(_LSE(e), _LSE(x), _LSE(y), _LSE(z), _LSE(i), _LSE(j), _LSE(k), _LSE(u), _LSE(v), _LSE(w)); return *this; }

  // Exact comparisons. For floats a "NEAR" operation may be better.
  FI bool operator==(const XYZval<T>  &rs) const { return true NUM_AXIS_GANG(&& x == rs.x, && y == rs.y, && z == rs.z, && i == rs.i, && j == rs.j, && k == rs.k, && u == rs.u, && v == rs.v, && w == rs.w); }
  FI bool operator==(const XYZEval<T> &rs) const { return true LOGICAL_AXIS_GANG(&& e == rs.e, && x == rs.x, && y == rs.y, && z == rs.z, && i == rs.i, && j == rs.j, && k == rs.k, && u == rs.u, && v == rs.v, && w == rs.w); }
  FI bool operator!=(const XYZval<T>  &rs) const { return !operator==(rs); }
  FI bool operator!=(const XYZEval<T> &rs) const { return !operator==(rs); }
};

#include <string.h> // for memset

template<typename T, int SIZE>
struct XYZarray {
  typedef T el[SIZE];
  union {
    el data[LOGICAL_AXES];
    struct { NUM_AXIS_CODE(T x, T y, T z, T i, T j, T k, T u, T v, T w); };
    struct { NUM_AXIS_CODE(T X, T Y, T Z, T I, T J, T K, T U, T V, T W); };
    struct { NUM_AXIS_CODE(T a, T b, T c, T _i, T _j, T _k, T _u, T _v, T _w); };
    struct { NUM_AXIS_CODE(T A, T B, T C, T II, T JJ, T KK, T UU, T VV, T WW); };
  };
  FI void reset() { ZERO(data); }

  FI void set(const int n, const XYval<T>   p) { NUM_AXIS_CODE(x[n]=p.x, y[n]=p.y,,,,,,,); }
  FI void set(const int n, const XYZval<T>  p) { NUM_AXIS_CODE(x[n]=p.x, y[n]=p.y, z[n]=p.z, i[n]=p.i, j[n]=p.j, k[n]=p.k, u[n]=p.u, v[n]=p.v, w[n]=p.w ); }
  FI void set(const int n, const XYZEval<T> p) { NUM_AXIS_CODE(x[n]=p.x, y[n]=p.y, z[n]=p.z, i[n]=p.i, j[n]=p.j, k[n]=p.k, u[n]=p.u, v[n]=p.v, w[n]=p.w ); }

  // Setter for all individual args
  FI void set(const int n OPTARGS_NUM(const T)) { NUM_AXIS_CODE(a[n] = x, b[n] = y, c[n] = z, _i[n] = i, _j[n] = j, _k[n] = k, _u[n] = u, _v[n] = v, _w[n] = w); }

  // Setters with fewer elements leave the rest untouched
  #if HAS_Y_AXIS
    FI void set(const int n, const T px) { x[n] = px; }
  #endif
  #if HAS_Z_AXIS
    FI void set(const int n, const T px, const T py) { x[n] = px; y[n] = py; }
  #endif
  #if HAS_I_AXIS
    FI void set(const int n, const T px, const T py, const T pz) { x[n] = px; y[n] = py; z[n] = pz; }
  #endif
  #if HAS_J_AXIS
    FI void set(const int n, const T px, const T py, const T pz, const T pi) { x[n] = px; y[n] = py; z[n] = pz; i[n] = pi; }
  #endif
  #if HAS_K_AXIS
    FI void set(const int n, const T px, const T py, const T pz, const T pi, const T pj) { x[n] = px; y[n] = py; z[n] = pz; i[n] = pi; j[n] = pj; }
  #endif
  #if HAS_U_AXIS
    FI void set(const int n, const T px, const T py, const T pz, const T pi, const T pj, const T pk) { x[n] = px; y[n] = py; z[n] = pz; i[n] = pi; j[n] = pj; k[n] = pk; }
  #endif
  #if HAS_V_AXIS
    FI void set(const int n, const T px, const T py, const T pz, const T pi, const T pj, const T pk, const T pu) { x[n] = px; y[n] = py; z[n] = pz; i[n] = pi; j[n] = pj; k[n] = pk; u[n] = pu; }
  #endif
  #if HAS_W_AXIS
    FI void set(const int n, const T px, const T py, const T pz, const T pi, const T pj, const T pk, const T pu, const T pv) { x[n] = px; y[n] = py; z[n] = pz; i[n] = pi; j[n] = pj; k[n] = pk; u[n] = pu; v[n] = pv; }
  #endif

  FI XYZval<T> operator[](const int n) const { return XYZval<T>(NUM_AXIS_ARRAY(x[n], y[n], z[n], i[n], j[n], k[n], u[n], v[n], w[n])); }
};

template<typename T, int SIZE>
struct XYZEarray {
  typedef T el[SIZE];
  union {
    el data[LOGICAL_AXES];
    struct { el LOGICAL_AXIS_ARGS(); };
    struct { el LOGICAL_AXIS_ARGS_LC(); };
    struct { el LOGICAL_AXIS_LIST(EE, A, B, C, II, JJ, KK, UU, VV, WW); };
    struct { el LOGICAL_AXIS_LIST(_e, a, b, c, _i, _j, _k, _u, _v, _w); };
  };
  FI void reset() { ZERO(data); }

  FI void set(const int n, const XYval<T>   p) { NUM_AXIS_CODE(x[n]=p.x, y[n]=p.y,,,,,,,); }
  FI void set(const int n, const XYZval<T>  p) { NUM_AXIS_CODE(x[n]=p.x, y[n]=p.y, z[n]=p.z, i[n]=p.i, j[n]=p.j, k[n]=p.k, u[n]=p.u, v[n]=p.v, w[n]=p.w ); }
  FI void set(const int n, const XYZEval<T> p) { LOGICAL_AXIS_CODE(e[n]=p.e, x[n]=p.x, y[n]=p.y, z[n]=p.z, i[n]=p.i, j[n]=p.j, k[n]=p.k, u[n]=p.u, v[n]=p.v, w[n]=p.w ); }

  // Setter for all individual args
  FI void set(const int n OPTARGS_NUM(const T)) { NUM_AXIS_CODE(a[n] = x, b[n] = y, c[n] = z, _i[n] = i, _j[n] = j, _k[n] = k, _u[n] = u, _v[n] = v, _w[n] = w); }
  #if LOGICAL_AXES > NUM_AXES
    FI void set(const int n, LOGICAL_AXIS_ARGS_LC(const T)) { LOGICAL_AXIS_CODE(_e[n] = e, a[n] = x, b[n] = y, c[n] = z, _i[n] = i, _j[n] = j, _k[n] = k, _u[n] = u, _v[n] = v, _w[n] = w); }
  #endif

  // Setters with fewer elements leave the rest untouched
  #if HAS_Y_AXIS
    FI void set(const int n, const T px) { x[n] = px; }
  #endif
  #if HAS_Z_AXIS
    FI void set(const int n, const T px, const T py) { x[n] = px; y[n] = py; }
  #endif
  #if HAS_I_AXIS
    FI void set(const int n, const T px, const T py, const T pz) { x[n] = px; y[n] = py; z[n] = pz; }
  #endif
  #if HAS_J_AXIS
    FI void set(const int n, const T px, const T py, const T pz, const T pi) { x[n] = px; y[n] = py; z[n] = pz; i[n] = pi; }
  #endif
  #if HAS_K_AXIS
    FI void set(const int n, const T px, const T py, const T pz, const T pi, const T pj) { x[n] = px; y[n] = py; z[n] = pz; i[n] = pi; j[n] = pj; }
  #endif
  #if HAS_U_AXIS
    FI void set(const int n, const T px, const T py, const T pz, const T pi, const T pj, const T pk) { x[n] = px; y[n] = py; z[n] = pz; i[n] = pi; j[n] = pj; k[n] = pk; }
  #endif
  #if HAS_V_AXIS
    FI void set(const int n, const T px, const T py, const T pz, const T pi, const T pj, const T pk, const T pu) { x[n] = px; y[n] = py; z[n] = pz; i[n] = pi; j[n] = pj; k[n] = pk; u[n] = pu; }
  #endif
  #if HAS_W_AXIS
    FI void set(const int n, const T px, const T py, const T pz, const T pi, const T pj, const T pk, const T pu, const T pv) { x[n] = px; y[n] = py; z[n] = pz; i[n] = pi; j[n] = pj; k[n] = pk; u[n] = pu; v[n] = pv; }
  #endif

  FI XYZEval<T> operator[](const int n) const { return XYZval<T>(LOGICAL_AXIS_ARRAY(e[n], x[n], y[n], z[n], i[n], j[n], k[n], u[n], v[n], w[n])); }
};

class AxisBits {
public:
  typedef bits_t(NUM_AXIS_HEADS) el;
  union {
    el bits;
    // Axes x, y, z ... e0, e1, e2 ... hx, hy, hz
    struct {
      #if NUM_AXES
        bool NUM_AXIS_LIST(x:1, y:1, z:1, i:1, j:1, k:1, u:1, v:1, w:1);
      #endif
      #define _EN_ITEM(N) bool e##N:1;
      REPEAT(EXTRUDERS,_EN_ITEM)
      #undef _EN_ITEM
      #if ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX)
        bool hx:1, hy:1, hz:1;
      #endif
    };
    // Axes X, Y, Z ... E0, E1, E2 ... HX, HY, HZ
    struct {
      #if NUM_AXES
        bool NUM_AXIS_LIST(X:1, Y:1, Z:1, I:1, J:1, K:1, U:1, V:1, W:1);
      #endif
      #define _EN_ITEM(N) bool E##N:1;
      REPEAT(EXTRUDERS,_EN_ITEM)
      #undef _EN_ITEM
      #if ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX)
        bool HX:1, HY:1, HZ:1;
      #endif
    };
    // a, b, c, e ... ha, hb, hc
    struct {
      bool LOGICAL_AXIS_LIST(e:1, a:1, b:1, c:1, ii:1, jj:1, kk:1, uu:1, vv:1, ww:1);
      #if EXTRUDERS > 1
        #define _EN_ITEM(N) bool _e##N:1;
        REPEAT_S(1,EXTRUDERS,_EN_ITEM)
        #undef _EN_ITEM
      #endif
      #if ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX)
        bool ha:1, hb:1, hc:1;
      #endif
    };
    // A, B, C, E ... HA, HB, HC
    struct {
      bool LOGICAL_AXIS_LIST(E:1, A:1, B:1, C:1, II:1, JJ:1, KK:1, UU:1, VV:1, WW:1);
      #if EXTRUDERS > 1
        #define _EN_ITEM(N) bool _E##N:1;
        REPEAT_S(1,EXTRUDERS,_EN_ITEM)
        #undef _EN_ITEM
      #endif
      #if ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX)
        bool HA:1, HB:1, HC:1;
      #endif
    };
  };

  class BitProxy {
  public:
    BitProxy(el& data, int bit) : data_(data), bit_(bit) {}

    BitProxy& operator=(const bool value) {
      if (value)
        data_ |=  (el(1) << bit_);
      else
        data_ &= ~(el(1) << bit_);
      return *this;
    }

    operator bool() const { return bool(data_ & (el(1) << bit_)); }

  private:
    el& data_;
    uint8_t bit_;
  };

  AxisBits() { reset(); }

  // Constructor, setter, and operator= for bit mask
  AxisBits(const el p) { set(p); }
  FI void set(const el p) { bits = el(p); }
  FI AxisBits& operator=(const el p) { set(p); return *this; }

  FI void reset() { set(0); }
  FI void fill() { set(_BV(NUM_AXIS_HEADS) - 1); }

  #define MSET(pE,pX,pY,pZ,pI,pJ,pK,pU,pV,pW) LOGICAL_AXIS_CODE(e=pE, x=pX, y=pY, z=pZ, i=pI, j=pJ, k=pK, u=pU, v=pV, w=pW)

  // Constructor, setter, and operator= for XYZE type
  AxisBits(const xyze_bool_t &p) { set(p); }
  FI void set(const xyze_bool_t &p) {
    MSET(p.e, p.x, p.y, p.z, p.i, p.j, p.k, p.u, p.v, p.w);
  }
  FI AxisBits& operator=(const xyze_bool_t &p) { set(p); return *this; }

  // Constructor, setter, and operator= for bool array
  AxisBits(const bool (&p)[LOGICAL_AXES]) { set(p); }
  FI void set(const bool (&p)[LOGICAL_AXES]) {
    MSET(p[E_AXIS], p[X_AXIS], p[Y_AXIS], p[Z_AXIS],
                    p[I_AXIS], p[J_AXIS], p[K_AXIS],
                    p[U_AXIS], p[V_AXIS], p[W_AXIS]);
  }
  FI AxisBits& operator=(const bool (&p)[LOGICAL_AXES]) { set(p); return *this; }

  // Constructor, setter, and operator= for undersized bool arrays
  #if LOGICAL_AXES > 1
    AxisBits(const bool (&p)[1]) { set(p); }
    FI void set(const bool (&p)[1]) {
      MSET(0, p[X_AXIS], 0, 0, 0, 0, 0, 0, 0, 0);
    }
    FI AxisBits& operator=(const bool (&p)[1]) { set(p); return *this; }
  #endif
  #if LOGICAL_AXES > 2
    AxisBits(const bool (&p)[2]) { set(p); }
    FI void set(const bool (&p)[2]) {
      MSET(0, p[X_AXIS], p[Y_AXIS], 0, 0, 0, 0, 0, 0, 0);
    }
    FI AxisBits& operator=(const bool (&p)[2]) { set(p); return *this; }
  #endif
  #if LOGICAL_AXES > 3
    AxisBits(const bool (&p)[3]) { set(p); }
    FI void set(const bool (&p)[3]) {
      MSET(0, p[X_AXIS], p[Y_AXIS], p[Z_AXIS], 0, 0, 0, 0, 0, 0);
    }
    FI AxisBits& operator=(const bool (&p)[3]) { set(p); return *this; }
  #endif
  #if LOGICAL_AXES > 4
    AxisBits(const bool (&p)[4]) { set(p); }
    FI void set(const bool (&p)[4]) {
      MSET(0, p[X_AXIS], p[Y_AXIS], p[Z_AXIS], p[I_AXIS], 0, 0, 0, 0, 0);
    }
    FI AxisBits& operator=(const bool (&p)[4]) { set(p); return *this; }
  #endif
  #if LOGICAL_AXES > 5
    AxisBits(const bool (&p)[5]) { set(p); }
    FI void set(const bool (&p)[5]) {
      MSET(0, p[X_AXIS], p[Y_AXIS], p[Z_AXIS], p[I_AXIS], p[J_AXIS], 0, 0, 0, 0);
    }
    FI AxisBits& operator=(const bool (&p)[5]) { set(p); return *this; }
  #endif
  #if LOGICAL_AXES > 6
    AxisBits(const bool (&p)[6]) { set(p); }
    FI void set(const bool (&p)[6]) {
      MSET(0, p[X_AXIS], p[Y_AXIS], p[Z_AXIS], p[I_AXIS], p[J_AXIS], p[K_AXIS], 0, 0, 0);
    }
    FI AxisBits& operator=(const bool (&p)[6]) { set(p); return *this; }
  #endif
  #if LOGICAL_AXES > 7
    AxisBits(const bool (&p)[7]) { set(p); }
    FI void set(const bool (&p)[7]) {
      MSET(0, p[X_AXIS], p[Y_AXIS], p[Z_AXIS], p[I_AXIS], p[J_AXIS], p[K_AXIS], p[U_AXIS], 0, 0);
    }
    FI AxisBits& operator=(const bool (&p)[7]) { set(p); return *this; }
  #endif
  #if LOGICAL_AXES > 8
    AxisBits(const bool (&p)[8]) { set(p); }
    FI void set(const bool (&p)[8]) {
      MSET(0, p[X_AXIS], p[Y_AXIS], p[Z_AXIS], p[I_AXIS], p[J_AXIS], p[K_AXIS], p[U_AXIS], p[V_AXIS], 0);
    }
    FI AxisBits& operator=(const bool (&p)[8]) { set(p); return *this; }
  #endif
  #if LOGICAL_AXES > 9
    AxisBits(const bool (&p)[9]) { set(p); }
    FI void set(const bool (&p)[9]) {
      MSET(0, p[X_AXIS], p[Y_AXIS], p[Z_AXIS], p[I_AXIS], p[J_AXIS], p[K_AXIS], p[U_AXIS], p[V_AXIS], p[W_AXIS]);
    }
    FI AxisBits& operator=(const bool (&p)[9]) { set(p); return *this; }
  #endif
  #undef MSET

  FI bool toggle(const AxisEnum n) { TBI(bits, n); return TEST(bits, n); }
  FI void bset(const AxisEnum n) { SBI(bits, n); }
  FI void bclr(const AxisEnum n) { CBI(bits, n); }
  FI void bset(const AxisEnum n, const bool b) { if (b) bset(n); else bclr(n); }

  // Accessor via an AxisEnum (or any integer) [index]
  FI BitProxy operator[](const int n)        { return BitProxy(bits, n); }
  FI BitProxy operator[](const AxisEnum n)   { return BitProxy(bits, n); }
  FI bool operator[](const int n)      const { return TEST(bits, n); }
  FI bool operator[](const AxisEnum n) const { return TEST(bits, n); }

  FI AxisBits& operator|=(const el &p) { bits |= el(p); return *this; }
  FI AxisBits& operator&=(const el &p) { bits &= el(p); return *this; }
  FI AxisBits& operator^=(const el &p) { bits ^= el(p); return *this; }

  FI AxisBits& operator|=(const AxisBits &p) { bits |= p.bits; return *this; }
  FI AxisBits& operator&=(const AxisBits &p) { bits &= p.bits; return *this; }
  FI AxisBits& operator^=(const AxisBits &p) { bits ^= p.bits; return *this; }

  FI bool operator==(const AxisBits &p) const { return p.bits == bits; }
  FI bool operator!=(const AxisBits &p) const { return p.bits != bits; }

  FI el operator|(const el &p) const { return bits | el(p); }
  FI el operator&(const el &p) const { return bits & el(p); }
  FI el operator^(const el &p) const { return bits ^ el(p); }

  FI AxisBits operator|(const AxisBits &p) const { return AxisBits(bits | p.bits); }
  FI AxisBits operator&(const AxisBits &p) const { return AxisBits(bits & p.bits); }
  FI AxisBits operator^(const AxisBits &p) const { return AxisBits(bits ^ p.bits); }

  FI operator bool() const { return !!bits; }
  FI operator uint16_t() const { return uint16_t(bits & 0xFFFF); }
  FI operator uint32_t() const { return uint32_t(bits); }

};

#undef _RECIP
#undef _ABS
#undef _LS
#undef _RS
#undef _LSE
#undef _RSE
#undef FI
