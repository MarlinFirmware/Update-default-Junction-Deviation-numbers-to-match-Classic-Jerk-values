/**
 * \file
 *
 * \brief Commonly used includes, types and macros.
 *
 * Copyright (c) 2010-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 * Support and FAQ: visit <a href="https://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef UTILS_COMPILER_H
#define UTILS_COMPILER_H

#include <sam.h>
#include <chip.h>
#include "arduino_due_x.h"
#include "conf_clock.h"
#ifdef SAM3XA_SERIES
#define SAM3XA 1
#endif
#define UDD_NO_SLEEP_MGR 1
#define pmc_is_wakeup_clocks_restored() true

#undef udd_get_endpoint_size_max
#define UDD_USB_INT_FUN USBD_ISR

/**
 * \defgroup group_sam_utils Compiler abstraction layer and code utilities
 *
 * Compiler abstraction layer and code utilities for AT91SAM.
 * This module provides various abstraction layers and utilities to make code compatible between different compilers.
 *
 * \{
 */
#include <stddef.h>

#if (defined __ICCARM__)
#  include <intrinsics.h>
#endif

#include <sam.h>
#include "preprocessor.h"

//_____ D E C L A R A T I O N S ____________________________________________

#ifndef __ASSEMBLY__ // Not defined for assembling.

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __ICCARM__
/*! \name Compiler Keywords
 *
 * Port of some keywords from GCC to IAR Embedded Workbench.
 */
//! @{
#define __asm__             asm
#define __inline__          inline
#define __volatile__
//! @}

#endif

#define FUNC_PTR                            void *
/**
 * \def UNUSED
 * \brief Marking \a v as a unused parameter or value.
 */
#ifndef UNUSED
  #define UNUSED(x) ((void)(x))
#endif

/**
 * \def unused
 * \brief Marking \a v as a unused parameter or value.
 */
#define unused(v)          do { (void)(v); }while(0)

/**
 * \def barrier
 * \brief Memory barrier
 */
#define barrier()          __DMB()

/**
 * \brief Emit the compiler pragma \a arg.
 *
 * \param arg The pragma directive as it would appear after \e \#pragma
 * (i.e. not stringified).
 */
#define COMPILER_PRAGMA(arg)            _Pragma(#arg)

/**
 * \def COMPILER_PACK_SET(alignment)
 * \brief Set maximum alignment for subsequent struct and union
 * definitions to \a alignment.
 */
#define COMPILER_PACK_SET(alignment)   COMPILER_PRAGMA(pack(alignment))

/**
 * \def COMPILER_PACK_RESET()
 * \brief Set default alignment for subsequent struct and union
 * definitions.
 */
#define COMPILER_PACK_RESET()          COMPILER_PRAGMA(pack())

/**
 * \brief Set aligned boundary.
 */
#if (defined __GNUC__) || (defined __CC_ARM)
#   define COMPILER_ALIGNED(a)    __attribute__((__aligned__(a)))
#elif (defined __ICCARM__)
#   define COMPILER_ALIGNED(a)    COMPILER_PRAGMA(data_alignment = a)
#endif

/**
 * \brief Set word-aligned boundary.
 */
#if (defined __GNUC__) || defined(__CC_ARM)
#define COMPILER_WORD_ALIGNED    __attribute__((__aligned__(4)))
#elif (defined __ICCARM__)
#define COMPILER_WORD_ALIGNED    COMPILER_PRAGMA(data_alignment = 4)
#endif

/**
 * \def __always_inline
 * \brief The function should always be inlined.
 *
 * This annotation instructs the compiler to ignore its inlining
 * heuristics and inline the function no matter how big it thinks it
 * becomes.
 */
#ifdef __CC_ARM
#   define __always_inline   __forceinline
#elif (defined __GNUC__)
#ifdef __always_inline
# undef __always_inline
#endif
# define __always_inline   inline __attribute__((__always_inline__))
#elif (defined __ICCARM__)
# define __always_inline   _Pragma("inline=forced")
#endif

/**
 * \def __no_inline
 * \brief The function should not be inlined.
 *
 * This annotation instructs the compiler to ignore its inlining
 * heuristics and not inline the function.
 */
#ifdef __CC_ARM
# define __no_inline   __attribute__((noinline))
#elif (defined __GNUC__)
# define __no_inline   __attribute__((__noinline__))
#elif (defined __ICCARM__)
# define __no_inline   _Pragma("inline=never")
#endif

/*! \brief This macro is used to test fatal errors.
 *
 * The macro tests if the expression is false. If it is, a fatal error is
 * detected and the application hangs up. If TEST_SUITE_DEFINE_ASSERT_MACRO
 * is defined, a unit test version of the macro is used, to allow execution
 * of further tests after a false expression.
 *
 * \param expr  Expression to evaluate and supposed to be nonzero.
 */
#ifdef _ASSERT_ENABLE_
#  if defined(TEST_SUITE_DEFINE_ASSERT_MACRO)
     // Assert() is defined in unit_test/suite.h
#    include "unit_test/suite.h"
#  else
#undef TEST_SUITE_DEFINE_ASSERT_MACRO
#    define Assert(expr) \
  {\
    if (!(expr)) while (true);\
  }
#  endif
#else
#  define Assert(expr) ((void) 0)
#endif

/* Define WEAK attribute */
#if defined   ( __CC_ARM   ) /* Keil µVision 4 */
#   define WEAK __attribute__ ((weak))
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
#   define WEAK __weak
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68 */
#   define WEAK __attribute__ ((weak))
#endif

/* Define NO_INIT attribute */
#if 0 //ndef NO_INIT
#ifdef __CC_ARM
#   define NO_INIT __attribute__((zero_init))
#elif defined ( __ICCARM__ )
#   define NO_INIT __no_init
#elif defined (  __GNUC__  )
#   define NO_INIT __attribute__((section(".no_init")))
#endif
#endif

/* Define RAMFUNC attribute */
#if defined   ( __CC_ARM   ) /* Keil µVision 4 */
#   define RAMFUNC __attribute__ ((section(".ramfunc")))
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
#   define RAMFUNC __ramfunc
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68 */
#   define RAMFUNC __attribute__ ((section(".ramfunc")))
#endif

/* Define OPTIMIZE_HIGH attribute */
#if defined   ( __CC_ARM   ) /* Keil µVision 4 */
#   define OPTIMIZE_HIGH _Pragma("O3")
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
#   define OPTIMIZE_HIGH _Pragma("optimize=high")
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68 */
#   define OPTIMIZE_HIGH __attribute__((optimize("s")))
#endif

/*! \name Usual Types
 */
//! @{
typedef unsigned char           Bool; //!< Boolean.
#ifndef __cplusplus
#ifndef __bool_true_false_are_defined
typedef unsigned char           bool; //!< Boolean.
#endif
#endif
typedef int8_t                  S8 ;  //!< 8-bit signed integer.
typedef uint8_t                 U8 ;  //!< 8-bit unsigned integer.
typedef int16_t                 S16;  //!< 16-bit signed integer.
typedef uint16_t                U16;  //!< 16-bit unsigned integer.
typedef uint16_t                le16_t;
typedef uint16_t                be16_t;
typedef int32_t                 S32;  //!< 32-bit signed integer.
typedef uint32_t                U32;  //!< 32-bit unsigned integer.
typedef uint32_t                le32_t;
typedef uint32_t                be32_t;
typedef int64_t                 S64;  //!< 64-bit signed integer.
typedef uint64_t                U64;  //!< 64-bit unsigned integer.
typedef float                   F32;  //!< 32-bit floating-point number.
typedef double                  F64;  //!< 64-bit floating-point number.
typedef uint32_t                iram_size_t;
//! @}

/*! \name Status Types
 */
//! @{
typedef bool                Status_bool_t;  //!< Boolean status.
typedef U8                  Status_t;       //!< 8-bit-coded status.
//! @}

/*! \name Aliasing Aggregate Types
 */
//! @{

//! 16-bit union.
typedef union
{
  S16 s16   ;
  U16 u16   ;
  S8  s8 [2];
  U8  u8 [2];
} Union16;

//! 32-bit union.
typedef union
{
  S32 s32   ;
  U32 u32   ;
  S16 s16[2];
  U16 u16[2];
  S8  s8 [4];
  U8  u8 [4];
} Union32;

//! 64-bit union.
typedef union
{
  S64 s64   ;
  U64 u64   ;
  S32 s32[2];
  U32 u32[2];
  S16 s16[4];
  U16 u16[4];
  S8  s8 [8];
  U8  u8 [8];
} Union64;

//! Union of pointers to 64-, 32-, 16- and 8-bit unsigned integers.
typedef union
{
  S64 *s64ptr;
  U64 *u64ptr;
  S32 *s32ptr;
  U32 *u32ptr;
  S16 *s16ptr;
  U16 *u16ptr;
  S8  *s8ptr ;
  U8  *u8ptr ;
} UnionPtr;

//! Union of pointers to volatile 64-, 32-, 16- and 8-bit unsigned integers.
typedef union
{
  volatile S64 *s64ptr;
  volatile U64 *u64ptr;
  volatile S32 *s32ptr;
  volatile U32 *u32ptr;
  volatile S16 *s16ptr;
  volatile U16 *u16ptr;
  volatile S8  *s8ptr ;
  volatile U8  *u8ptr ;
} UnionVPtr;

//! Union of pointers to constant 64-, 32-, 16- and 8-bit unsigned integers.
typedef union
{
  const S64 *s64ptr;
  const U64 *u64ptr;
  const S32 *s32ptr;
  const U32 *u32ptr;
  const S16 *s16ptr;
  const U16 *u16ptr;
  const S8  *s8ptr ;
  const U8  *u8ptr ;
} UnionCPtr;

//! Union of pointers to constant volatile 64-, 32-, 16- and 8-bit unsigned integers.
typedef union
{
  const volatile S64 *s64ptr;
  const volatile U64 *u64ptr;
  const volatile S32 *s32ptr;
  const volatile U32 *u32ptr;
  const volatile S16 *s16ptr;
  const volatile U16 *u16ptr;
  const volatile S8  *s8ptr ;
  const volatile U8  *u8ptr ;
} UnionCVPtr;

//! Structure of pointers to 64-, 32-, 16- and 8-bit unsigned integers.
typedef struct
{
  S64 *s64ptr;
  U64 *u64ptr;
  S32 *s32ptr;
  U32 *u32ptr;
  S16 *s16ptr;
  U16 *u16ptr;
  S8  *s8ptr ;
  U8  *u8ptr ;
} StructPtr;

//! Structure of pointers to volatile 64-, 32-, 16- and 8-bit unsigned integers.
typedef struct
{
  volatile S64 *s64ptr;
  volatile U64 *u64ptr;
  volatile S32 *s32ptr;
  volatile U32 *u32ptr;
  volatile S16 *s16ptr;
  volatile U16 *u16ptr;
  volatile S8  *s8ptr ;
  volatile U8  *u8ptr ;
} StructVPtr;

//! Structure of pointers to constant 64-, 32-, 16- and 8-bit unsigned integers.
typedef struct
{
  const S64 *s64ptr;
  const U64 *u64ptr;
  const S32 *s32ptr;
  const U32 *u32ptr;
  const S16 *s16ptr;
  const U16 *u16ptr;
  const S8  *s8ptr ;
  const U8  *u8ptr ;
} StructCPtr;

//! Structure of pointers to constant volatile 64-, 32-, 16- and 8-bit unsigned integers.
typedef struct
{
  const volatile S64 *s64ptr;
  const volatile U64 *u64ptr;
  const volatile S32 *s32ptr;
  const volatile U32 *u32ptr;
  const volatile S16 *s16ptr;
  const volatile U16 *u16ptr;
  const volatile S8  *s8ptr ;
  const volatile U8  *u8ptr ;
} StructCVPtr;

//! @}

#endif  // #ifndef __ASSEMBLY__

/*! \name Usual Constants
 */
//! @{
#define DISABLE   0
#define ENABLE    1
#ifndef __cplusplus
#ifndef __bool_true_false_are_defined
#define false (1==0)
#define true (1==1)
#endif
#endif
#ifndef PASS
#define PASS      0
#endif
#ifndef FAIL
#define FAIL      1
#endif
#ifndef LOW
#define LOW  0x0
#endif
#ifndef HIGH
#define HIGH 0x1
#endif
//! @}

#ifndef __ASSEMBLY__ // not for assembling.

//! \name Optimization Control
//@{

/**
 * \def likely(exp)
 * \brief The expression \a exp is likely to be true
 */
#ifndef likely
#   define likely(exp)    (exp)
#endif

/**
 * \def unlikely(exp)
 * \brief The expression \a exp is unlikely to be true
 */
#ifndef unlikely
#   define unlikely(exp)  (exp)
#endif

/**
 * \def is_constant(exp)
 * \brief Determine if an expression evaluates to a constant value.
 *
 * \param exp Any expression
 *
 * \return true if \a exp is constant, false otherwise.
 */
#if (defined __GNUC__) || (defined __CC_ARM)
#   define is_constant(exp)       __builtin_constant_p(exp)
#else
#   define is_constant(exp)       (0)
#endif

//! @}

/*! \name Bit-Field Handling
 */
//! @{

/*! \brief Reads the bits of a value specified by a given bit-mask.
 *
 * \param value Value to read bits from.
 * \param mask  Bit-mask indicating bits to read.
 *
 * \return Read bits.
 */
#define Rd_bits( value, mask)        ((value) & (mask))

/*! \brief Writes the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue to write bits to.
 * \param mask    Bit-mask indicating bits to write.
 * \param bits    Bits to write.
 *
 * \return Resulting value with written bits.
 */
#define Wr_bits(lvalue, mask, bits)  ((lvalue) = ((lvalue) & ~(mask)) |\
                                                 ((bits  ) &  (mask)))

/*! \brief Tests the bits of a value specified by a given bit-mask.
 *
 * \param value Value of which to test bits.
 * \param mask  Bit-mask indicating bits to test.
 *
 * \return \c 1 if at least one of the tested bits is set, else \c 0.
 */
#define Tst_bits( value, mask)  (Rd_bits(value, mask) != 0)

/*! \brief Clears the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue of which to clear bits.
 * \param mask    Bit-mask indicating bits to clear.
 *
 * \return Resulting value with cleared bits.
 */
#define Clr_bits(lvalue, mask)  ((lvalue) &= ~(mask))

/*! \brief Sets the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue of which to set bits.
 * \param mask    Bit-mask indicating bits to set.
 *
 * \return Resulting value with set bits.
 */
#define Set_bits(lvalue, mask)  ((lvalue) |=  (mask))

/*! \brief Toggles the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue of which to toggle bits.
 * \param mask    Bit-mask indicating bits to toggle.
 *
 * \return Resulting value with toggled bits.
 */
#define Tgl_bits(lvalue, mask)  ((lvalue) ^=  (mask))

/*! \brief Reads the bit-field of a value specified by a given bit-mask.
 *
 * \param value Value to read a bit-field from.
 * \param mask  Bit-mask indicating the bit-field to read.
 *
 * \return Read bit-field.
 */
#define Rd_bitfield( value, mask)           (Rd_bits( value, mask) >> ctz(mask))

/*! \brief Writes the bit-field of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue    C lvalue to write a bit-field to.
 * \param mask      Bit-mask indicating the bit-field to write.
 * \param bitfield  Bit-field to write.
 *
 * \return Resulting value with written bit-field.
 */
#define Wr_bitfield(lvalue, mask, bitfield) (Wr_bits(lvalue, mask, (U32)(bitfield) << ctz(mask)))

//! @}

/*! \name Zero-Bit Counting
 *
 * Under GCC, __builtin_clz and __builtin_ctz behave like macros when
 * applied to constant expressions (values known at compile time), so they are
 * more optimized than the use of the corresponding assembly instructions and
 * they can be used as constant expressions e.g. to initialize objects having
 * static storage duration, and like the corresponding assembly instructions
 * when applied to non-constant expressions (values unknown at compile time), so
 * they are more optimized than an assembly periphrasis. Hence, clz and ctz
 * ensure a possible and optimized behavior for both constant and non-constant
 * expressions.
 */
//! @{

/*! \brief Counts the leading zero bits of the given value considered as a 32-bit integer.
 *
 * \param u Value of which to count the leading zero bits.
 *
 * \return The count of leading zero bits in \a u.
 */
#ifndef clz
#if (defined __GNUC__) || (defined __CC_ARM)
#   define clz(u)              ((u) ? __builtin_clz(u) : 32)
#elif (defined __ICCARM__)
#   define clz(u)              ((u) ? __CLZ(u) : 32)
#else
#   define clz(u)              (((u) == 0)          ? 32 : \
                                ((u) & (1UL << 31)) ?  0 : \
                                ((u) & (1UL << 30)) ?  1 : \
                                ((u) & (1UL << 29)) ?  2 : \
                                ((u) & (1UL << 28)) ?  3 : \
                                ((u) & (1UL << 27)) ?  4 : \
                                ((u) & (1UL << 26)) ?  5 : \
                                ((u) & (1UL << 25)) ?  6 : \
                                ((u) & (1UL << 24)) ?  7 : \
                                ((u) & (1UL << 23)) ?  8 : \
                                ((u) & (1UL << 22)) ?  9 : \
                                ((u) & (1UL << 21)) ? 10 : \
                                ((u) & (1UL << 20)) ? 11 : \
                                ((u) & (1UL << 19)) ? 12 : \
                                ((u) & (1UL << 18)) ? 13 : \
                                ((u) & (1UL << 17)) ? 14 : \
                                ((u) & (1UL << 16)) ? 15 : \
                                ((u) & (1UL << 15)) ? 16 : \
                                ((u) & (1UL << 14)) ? 17 : \
                                ((u) & (1UL << 13)) ? 18 : \
                                ((u) & (1UL << 12)) ? 19 : \
                                ((u) & (1UL << 11)) ? 20 : \
                                ((u) & (1UL << 10)) ? 21 : \
                                ((u) & (1UL <<  9)) ? 22 : \
                                ((u) & (1UL <<  8)) ? 23 : \
                                ((u) & (1UL <<  7)) ? 24 : \
                                ((u) & (1UL <<  6)) ? 25 : \
                                ((u) & (1UL <<  5)) ? 26 : \
                                ((u) & (1UL <<  4)) ? 27 : \
                                ((u) & (1UL <<  3)) ? 28 : \
                                ((u) & (1UL <<  2)) ? 29 : \
                                ((u) & (1UL <<  1)) ? 30 : \
                                31)
#endif
#endif

/*! \brief Counts the trailing zero bits of the given value considered as a 32-bit integer.
 *
 * \param u Value of which to count the trailing zero bits.
 *
 * \return The count of trailing zero bits in \a u.
 */
#ifndef ctz
#if (defined __GNUC__) || (defined __CC_ARM)
#   define ctz(u)              ((u) ? __builtin_ctz(u) : 32)
#else
#   define ctz(u)              ((u) & (1UL <<  0) ?  0 : \
                                (u) & (1UL <<  1) ?  1 : \
                                (u) & (1UL <<  2) ?  2 : \
                                (u) & (1UL <<  3) ?  3 : \
                                (u) & (1UL <<  4) ?  4 : \
                                (u) & (1UL <<  5) ?  5 : \
                                (u) & (1UL <<  6) ?  6 : \
                                (u) & (1UL <<  7) ?  7 : \
                                (u) & (1UL <<  8) ?  8 : \
                                (u) & (1UL <<  9) ?  9 : \
                                (u) & (1UL << 10) ? 10 : \
                                (u) & (1UL << 11) ? 11 : \
                                (u) & (1UL << 12) ? 12 : \
                                (u) & (1UL << 13) ? 13 : \
                                (u) & (1UL << 14) ? 14 : \
                                (u) & (1UL << 15) ? 15 : \
                                (u) & (1UL << 16) ? 16 : \
                                (u) & (1UL << 17) ? 17 : \
                                (u) & (1UL << 18) ? 18 : \
                                (u) & (1UL << 19) ? 19 : \
                                (u) & (1UL << 20) ? 20 : \
                                (u) & (1UL << 21) ? 21 : \
                                (u) & (1UL << 22) ? 22 : \
                                (u) & (1UL << 23) ? 23 : \
                                (u) & (1UL << 24) ? 24 : \
                                (u) & (1UL << 25) ? 25 : \
                                (u) & (1UL << 26) ? 26 : \
                                (u) & (1UL << 27) ? 27 : \
                                (u) & (1UL << 28) ? 28 : \
                                (u) & (1UL << 29) ? 29 : \
                                (u) & (1UL << 30) ? 30 : \
                                (u) & (1UL << 31) ? 31 : \
                                32)
#endif
#endif

//! @}

/*! \name Bit Reversing
 */
//! @{

/*! \brief Reverses the bits of \a u8.
 *
 * \param u8  U8 of which to reverse the bits.
 *
 * \return Value resulting from \a u8 with reversed bits.
 */
#define bit_reverse8(u8)    ((U8)(bit_reverse32((U8)(u8)) >> 24))

/*! \brief Reverses the bits of \a u16.
 *
 * \param u16 U16 of which to reverse the bits.
 *
 * \return Value resulting from \a u16 with reversed bits.
 */
#define bit_reverse16(u16)  ((U16)(bit_reverse32((U16)(u16)) >> 16))

/*! \brief Reverses the bits of \a u32.
 *
 * \param u32 U32 of which to reverse the bits.
 *
 * \return Value resulting from \a u32 with reversed bits.
 */
#define bit_reverse32(u32)   __RBIT(u32)

/*! \brief Reverses the bits of \a u64.
 *
 * \param u64 U64 of which to reverse the bits.
 *
 * \return Value resulting from \a u64 with reversed bits.
 */
#define bit_reverse64(u64)  ((U64)(((U64)bit_reverse32((U64)(u64) >> 32)) |\
                                   ((U64)bit_reverse32((U64)(u64)) << 32)))

//! @}

/*! \name Alignment
 */
//! @{

/*! \brief Tests alignment of the number \a val with the \a n boundary.
 *
 * \param val Input value.
 * \param n   Boundary.
 *
 * \return \c 1 if the number \a val is aligned with the \a n boundary, else \c 0.
 */
#define Test_align(val, n     ) (!Tst_bits( val, (n) - 1     )   )

/*! \brief Gets alignment of the number \a val with respect to the \a n boundary.
 *
 * \param val Input value.
 * \param n   Boundary.
 *
 * \return Alignment of the number \a val with respect to the \a n boundary.
 */
#define Get_align( val, n     ) (  Rd_bits( val, (n) - 1     )   )

/*! \brief Sets alignment of the lvalue number \a lval to \a alg with respect to the \a n boundary.
 *
 * \param lval  Input/output lvalue.
 * \param n     Boundary.
 * \param alg   Alignment.
 *
 * \return New value of \a lval resulting from its alignment set to \a alg with respect to the \a n boundary.
 */
#define Set_align(lval, n, alg) (  Wr_bits(lval, (n) - 1, alg)   )

/*! \brief Aligns the number \a val with the upper \a n boundary.
 *
 * \param val Input value.
 * \param n   Boundary.
 *
 * \return Value resulting from the number \a val aligned with the upper \a n boundary.
 */
#define Align_up(  val, n     ) (((val) + ((n) - 1)) & ~((n) - 1))

/*! \brief Aligns the number \a val with the lower \a n boundary.
 *
 * \param val Input value.
 * \param n   Boundary.
 *
 * \return Value resulting from the number \a val aligned with the lower \a n boundary.
 */
#define Align_down(val, n     ) ( (val)              & ~((n) - 1))

//! @}

/*! \brief Calls the routine at address \a addr.
 *
 * It generates a long call opcode.
 *
 * For example, `Long_call(0x80000000)' generates a software reset on a UC3 if
 * it is invoked from the CPU supervisor mode.
 *
 * \param addr  Address of the routine to call.
 *
 * \note It may be used as a long jump opcode in some special cases.
 */
#define Long_call(addr)                   ((*(void (*)(void))(addr))())

/*! \name MCU Endianism Handling
 * ARM is MCU little endianism.
 */
//! @{
#define  MSB(u16)       (((U8  *)&(u16))[1]) //!< Most significant byte of \a u16.
#define  LSB(u16)       (((U8  *)&(u16))[0]) //!< Least significant byte of \a u16.

#define  MSH(u32)       (((U16 *)&(u32))[1]) //!< Most significant half-word of \a u32.
#define  LSH(u32)       (((U16 *)&(u32))[0]) //!< Least significant half-word of \a u32.
#define  MSB0W(u32)     (((U8  *)&(u32))[3]) //!< Most significant byte of 1st rank of \a u32.
#define  MSB1W(u32)     (((U8  *)&(u32))[2]) //!< Most significant byte of 2nd rank of \a u32.
#define  MSB2W(u32)     (((U8  *)&(u32))[1]) //!< Most significant byte of 3rd rank of \a u32.
#define  MSB3W(u32)     (((U8  *)&(u32))[0]) //!< Most significant byte of 4th rank of \a u32.
#define  LSB3W(u32)     MSB0W(u32)           //!< Least significant byte of 4th rank of \a u32.
#define  LSB2W(u32)     MSB1W(u32)           //!< Least significant byte of 3rd rank of \a u32.
#define  LSB1W(u32)     MSB2W(u32)           //!< Least significant byte of 2nd rank of \a u32.
#define  LSB0W(u32)     MSB3W(u32)           //!< Least significant byte of 1st rank of \a u32.

#define  MSW(u64)       (((U32 *)&(u64))[1]) //!< Most significant word of \a u64.
#define  LSW(u64)       (((U32 *)&(u64))[0]) //!< Least significant word of \a u64.
#define  MSH0(u64)      (((U16 *)&(u64))[3]) //!< Most significant half-word of 1st rank of \a u64.
#define  MSH1(u64)      (((U16 *)&(u64))[2]) //!< Most significant half-word of 2nd rank of \a u64.
#define  MSH2(u64)      (((U16 *)&(u64))[1]) //!< Most significant half-word of 3rd rank of \a u64.
#define  MSH3(u64)      (((U16 *)&(u64))[0]) //!< Most significant half-word of 4th rank of \a u64.
#define  LSH3(u64)      MSH0(u64)            //!< Least significant half-word of 4th rank of \a u64.
#define  LSH2(u64)      MSH1(u64)            //!< Least significant half-word of 3rd rank of \a u64.
#define  LSH1(u64)      MSH2(u64)            //!< Least significant half-word of 2nd rank of \a u64.
#define  LSH0(u64)      MSH3(u64)            //!< Least significant half-word of 1st rank of \a u64.
#define  MSB0D(u64)     (((U8  *)&(u64))[7]) //!< Most significant byte of 1st rank of \a u64.
#define  MSB1D(u64)     (((U8  *)&(u64))[6]) //!< Most significant byte of 2nd rank of \a u64.
#define  MSB2D(u64)     (((U8  *)&(u64))[5]) //!< Most significant byte of 3rd rank of \a u64.
#define  MSB3D(u64)     (((U8  *)&(u64))[4]) //!< Most significant byte of 4th rank of \a u64.
#define  MSB4D(u64)     (((U8  *)&(u64))[3]) //!< Most significant byte of 5th rank of \a u64.
#define  MSB5D(u64)     (((U8  *)&(u64))[2]) //!< Most significant byte of 6th rank of \a u64.
#define  MSB6D(u64)     (((U8  *)&(u64))[1]) //!< Most significant byte of 7th rank of \a u64.
#define  MSB7D(u64)     (((U8  *)&(u64))[0]) //!< Most significant byte of 8th rank of \a u64.
#define  LSB7D(u64)     MSB0D(u64)           //!< Least significant byte of 8th rank of \a u64.
#define  LSB6D(u64)     MSB1D(u64)           //!< Least significant byte of 7th rank of \a u64.
#define  LSB5D(u64)     MSB2D(u64)           //!< Least significant byte of 6th rank of \a u64.
#define  LSB4D(u64)     MSB3D(u64)           //!< Least significant byte of 5th rank of \a u64.
#define  LSB3D(u64)     MSB4D(u64)           //!< Least significant byte of 4th rank of \a u64.
#define  LSB2D(u64)     MSB5D(u64)           //!< Least significant byte of 3rd rank of \a u64.
#define  LSB1D(u64)     MSB6D(u64)           //!< Least significant byte of 2nd rank of \a u64.
#define  LSB0D(u64)     MSB7D(u64)           //!< Least significant byte of 1st rank of \a u64.

#define  BE16(x)        swap16(x)
#define  LE16(x)        (x)

#define  le16_to_cpu(x) (x)
#define  cpu_to_le16(x) (x)
#define  LE16_TO_CPU(x) (x)
#define  CPU_TO_LE16(x) (x)

#define  be16_to_cpu(x) swap16(x)
#define  cpu_to_be16(x) swap16(x)
#define  BE16_TO_CPU(x) swap16(x)
#define  CPU_TO_BE16(x) swap16(x)

#define  le32_to_cpu(x) (x)
#define  cpu_to_le32(x) (x)
#define  LE32_TO_CPU(x) (x)
#define  CPU_TO_LE32(x) (x)

#define  be32_to_cpu(x) swap32(x)
#define  cpu_to_be32(x) swap32(x)
#define  BE32_TO_CPU(x) swap32(x)
#define  CPU_TO_BE32(x) swap32(x)
//! @}

/*! \name Endianism Conversion
 *
 * The same considerations as for clz and ctz apply here but GCC's
 * __builtin_bswap_32 and __builtin_bswap_64 do not behave like macros when
 * applied to constant expressions, so two sets of macros are defined here:
 *   - Swap16, Swap32 and Swap64 to apply to constant expressions (values known
 *     at compile time);
 *   - swap16, swap32 and swap64 to apply to non-constant expressions (values
 *     unknown at compile time).
 */
//! @{

/*! \brief Toggles the endianism of \a u16 (by swapping its bytes).
 *
 * \param u16 U16 of which to toggle the endianism.
 *
 * \return Value resulting from \a u16 with toggled endianism.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Swap16(u16) ((U16)(((U16)(u16) >> 8) |\
                           ((U16)(u16) << 8)))

/*! \brief Toggles the endianism of \a u32 (by swapping its bytes).
 *
 * \param u32 U32 of which to toggle the endianism.
 *
 * \return Value resulting from \a u32 with toggled endianism.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Swap32(u32) ((U32)(((U32)Swap16((U32)(u32) >> 16)) |\
                           ((U32)Swap16((U32)(u32)) << 16)))

/*! \brief Toggles the endianism of \a u64 (by swapping its bytes).
 *
 * \param u64 U64 of which to toggle the endianism.
 *
 * \return Value resulting from \a u64 with toggled endianism.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Swap64(u64) ((U64)(((U64)Swap32((U64)(u64) >> 32)) |\
                           ((U64)Swap32((U64)(u64)) << 32)))

/*! \brief Toggles the endianism of \a u16 (by swapping its bytes).
 *
 * \param u16 U16 of which to toggle the endianism.
 *
 * \return Value resulting from \a u16 with toggled endianism.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#define swap16(u16) Swap16(u16)

/*! \brief Toggles the endianism of \a u32 (by swapping its bytes).
 *
 * \param u32 U32 of which to toggle the endianism.
 *
 * \return Value resulting from \a u32 with toggled endianism.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#if (defined __GNUC__)
#   define swap32(u32) ((U32)__builtin_bswap32((U32)(u32)))
#else
#   define swap32(u32) Swap32(u32)
#endif

/*! \brief Toggles the endianism of \a u64 (by swapping its bytes).
 *
 * \param u64 U64 of which to toggle the endianism.
 *
 * \return Value resulting from \a u64 with toggled endianism.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#if (defined __GNUC__)
#   define swap64(u64) ((U64)__builtin_bswap64((U64)(u64)))
#else
#   define swap64(u64) ((U64)(((U64)swap32((U64)(u64) >> 32)) |\
                           ((U64)swap32((U64)(u64)) << 32)))
#endif

//! @}

/*! \name Target Abstraction
 */
//! @{

#define _GLOBEXT_           extern      //!< extern storage-class specifier.
#define _CONST_TYPE_        const       //!< const type qualifier.
#define _MEM_TYPE_SLOW_                 //!< Slow memory type.
#define _MEM_TYPE_MEDFAST_              //!< Fairly fast memory type.
#define _MEM_TYPE_FAST_                 //!< Fast memory type.

typedef U8                  Byte;       //!< 8-bit unsigned integer.

#define memcmp_ram2ram      memcmp      //!< Target-specific memcmp of RAM to RAM.
#define memcmp_code2ram     memcmp      //!< Target-specific memcmp of RAM to NVRAM.
#define memcpy_ram2ram      memcpy      //!< Target-specific memcpy from RAM to RAM.
#define memcpy_code2ram     memcpy      //!< Target-specific memcpy from NVRAM to RAM.

#define LSB0(u32)           LSB0W(u32)  //!< Least significant byte of 1st rank of \a u32.
#define LSB1(u32)           LSB1W(u32)  //!< Least significant byte of 2nd rank of \a u32.
#define LSB2(u32)           LSB2W(u32)  //!< Least significant byte of 3rd rank of \a u32.
#define LSB3(u32)           LSB3W(u32)  //!< Least significant byte of 4th rank of \a u32.
#define MSB3(u32)           MSB3W(u32)  //!< Most significant byte of 4th rank of \a u32.
#define MSB2(u32)           MSB2W(u32)  //!< Most significant byte of 3rd rank of \a u32.
#define MSB1(u32)           MSB1W(u32)  //!< Most significant byte of 2nd rank of \a u32.
#define MSB0(u32)           MSB0W(u32)  //!< Most significant byte of 1st rank of \a u32.

//! @}

/**
 * \brief Calculate \f$ \left\lceil \frac{a}{b} \right\rceil \f$ using
 * integer arithmetic.
 *
 * \param a An integer
 * \param b Another integer
 *
 * \return (\a a / \a b) rounded up to the nearest integer.
 */
#define div_ceil(a, b)      (((a) + (b) - 1) / (b))

#endif  // #ifndef __ASSEMBLY__

#ifdef __ICCARM__
#define SHORTENUM           __packed
#elif defined(__GNUC__)
#define SHORTENUM           __attribute__((packed))
#endif

/* No operation */
#ifdef __ICCARM__
#define nop()               __no_operation()
#elif defined(__GNUC__)
#define nop()               (__NOP())
#endif

#define FLASH_DECLARE(x)  const x
#define FLASH_EXTERN(x) extern const x
#define PGM_READ_BYTE(x) *(x)
#define PGM_READ_WORD(x) *(x)
#define PGM_READ_DWORD(x) *(x)
#define MEMCPY_ENDIAN memcpy
#define PGM_READ_BLOCK(dst, src, len) memcpy((dst), (src), (len))

/*Defines the Flash Storage for the request and response of MAC*/
#define CMD_ID_OCTET    (0)

/* Converting of values from CPU endian to little endian. */
#define CPU_ENDIAN_TO_LE16(x)   (x)
#define CPU_ENDIAN_TO_LE32(x)   (x)
#define CPU_ENDIAN_TO_LE64(x)   (x)

/* Converting of values from little endian to CPU endian. */
#define LE16_TO_CPU_ENDIAN(x)   (x)
#define LE32_TO_CPU_ENDIAN(x)   (x)
#define LE64_TO_CPU_ENDIAN(x)   (x)

/* Converting of constants from little endian to CPU endian. */
#define CLE16_TO_CPU_ENDIAN(x)  (x)
#define CLE32_TO_CPU_ENDIAN(x)  (x)
#define CLE64_TO_CPU_ENDIAN(x)  (x)

/* Converting of constants from CPU endian to little endian. */
#define CCPU_ENDIAN_TO_LE16(x)  (x)
#define CCPU_ENDIAN_TO_LE32(x)  (x)
#define CCPU_ENDIAN_TO_LE64(x)  (x)

#define ADDR_COPY_DST_SRC_16(dst, src)  ((dst) = (src))
#define ADDR_COPY_DST_SRC_64(dst, src)  ((dst) = (src))

/**
 * @brief Converts a 64-Bit value into  a 8 Byte array
 *
 * @param[in] value 64-Bit value
 * @param[out] data Pointer to the 8 Byte array to be updated with 64-Bit value
 * @ingroup apiPalApi
 */
static inline void convert_64_bit_to_byte_array(uint64_t value, uint8_t *data)
{
    uint8_t val_index = 0;

    while (val_index < 8)
    {
        data[val_index++] = value & 0xFF;
        value >>= 8;
    }
}

/**
 * @brief Converts a 16-Bit value into  a 2 Byte array
 *
 * @param[in] value 16-Bit value
 * @param[out] data Pointer to the 2 Byte array to be updated with 16-Bit value
 * @ingroup apiPalApi
 */
static inline void convert_16_bit_to_byte_array(uint16_t value, uint8_t *data)
{
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
}

/* Converts a 16-Bit value into a 2 Byte array */
static inline void convert_spec_16_bit_to_byte_array(uint16_t value, uint8_t *data)
{
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
}

/* Converts a 16-Bit value into a 2 Byte array */
static inline void convert_16_bit_to_byte_address(uint16_t value, uint8_t *data)
{
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
}

/**
 * @brief Converts a 2 Byte array into a 16-Bit value
 *
 * @param data Specifies the pointer to the 2 Byte array
 *
 * @return 16-Bit value
 * @ingroup apiPalApi
 */
static inline uint16_t convert_byte_array_to_16_bit(uint8_t *data)
{
    return (data[0] | ((uint16_t)data[1] << 8));
}

/* Converts a 8 Byte array into a 32-Bit value */
static inline uint32_t convert_byte_array_to_32_bit(uint8_t *data)
{
  union
  {
    uint32_t u32;
    uint8_t u8[8];
  }long_addr;
  uint8_t index;
  for (index = 0; index < 4; index++) {
    long_addr.u8[index] = *data++;
  }
  return long_addr.u32;
}

/**
 * @brief Converts a 8 Byte array into a 64-Bit value
 *
 * @param data Specifies the pointer to the 8 Byte array
 *
 * @return 64-Bit value
 * @ingroup apiPalApi
 */
static inline uint64_t convert_byte_array_to_64_bit(uint8_t *data)
{
    union
    {
        uint64_t u64;
        uint8_t u8[8];
    } long_addr;

    uint8_t val_index;

    for (val_index = 0; val_index < 8; val_index++)
    {
        long_addr.u8[val_index] = *data++;
    }

    return long_addr.u64;
}
/**
 * \}
 */

#endif // UTILS_COMPILER_H
