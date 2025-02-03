/* Copyright (C) 2015-2016 Andrew J. Kroll
   and
Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Contact information
-------------------

Circuits At Home, LTD
Web      :  https://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */

#ifndef MACROS_H
#define MACROS_H
#include "macro_logic.h"

/**
 * Universal Arduino(tm) "IDE" fixups.
 */

// Just in case...
#ifndef SERIAL_PORT_MONITOR
#define SERIAL_PORT_MONITOR Serial
#endif

#ifndef INT16_MIN
#define INT16_MIN -32768
#endif
// require 10607+
#if defined(ARDUINO) && ARDUINO >=10607
// nop :-)
#else
#error "Arduino version too old, and must be at least 1.6.7"
#endif

// Nuke screwed up macro junk from the IDE.
#ifdef __cplusplus
#ifdef true
#undef true
#endif
#ifdef false
#undef false
#endif
#endif

#ifndef UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE

#ifndef UHS_BIG_FLASH

#if defined(FLASHEND) && defined(FLASHSTART)
#if (FLASHEND - FLASHSTART) > 0x0FFFFU
#define UHS_BIG_FLASH 1
#else
#define UHS_BIG_FLASH 0
#endif

#elif defined(__PIC32_FLASH_SIZE)
#if __PIC32_FLASH_SIZE > 511
#define UHS_BIG_FLASH 1
#else
#define UHS_BIG_FLASH 0
#endif

#elif defined(FLASHEND) && !defined(FLASHSTART)
// Assumes flash starts at 0x00000, is this a safe assumption?
// 192K + should be OK
#if FLASHEND > 0x02FFFFU
#define UHS_BIG_FLASH 1
#else
#define UHS_BIG_FLASH 0
#endif

#elif defined(IFLASH_SIZE)
#if IFLASH_SIZE > 0x0FFFFU
#define UHS_BIG_FLASH 1
#else
#define UHS_BIG_FLASH 0
#endif

#elif defined(ESP8266)
#define UHS_BIG_FLASH 1
#define SYSTEM_OR_SPECIAL_YIELD(...) yield()

#elif defined(__arm__) && defined(CORE_TEENSY)
#define UHS_BIG_FLASH 1

#elif defined(ARDUINO_spresense_ast)
#define UHS_BIG_FLASH 1
#else
// safe default
#warning Small flash?
#define UHS_BIG_FLASH 0
#endif
#endif

#if UHS_BIG_FLASH
#define UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE 1
#else
#define UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE 0
#endif
#endif

#if defined(__arm__) && defined(CORE_TEENSY)
#define UHS_PIN_WRITE(p, v) digitalWriteFast(p, v)
#define UHS_PIN_READ(p) digitalReadFast(p)
#endif
// TODO: Fast inline code for AVR and SAM based microcontrollers
//       This can be done pretty easily.
//       For now, this will just work out-of-the-box.
#ifndef UHS_PIN_WRITE
#define UHS_PIN_WRITE(p, v) digitalWrite(p, v)
#endif
#ifndef UHS_PIN_READ
#define UHS_PIN_READ(p) digitalRead(p)
#endif

#if defined( __PIC32MX__ ) && !defined(interrupts) // compiling with Microchip XC32 compiler
#define interrupts() __builtin_enable_interrupts()
#edfine noInterrupts() __builtin_disable_interrupts()
#endif

#ifndef ARDUINO_SAMD_ZERO
#ifdef ARDUINO_AVR_ADK
#define UHS_GET_DPI(x) (x == 54 ? 6 : digitalPinToInterrupt(x))
#else
#define UHS_GET_DPI(x) digitalPinToInterrupt(x)
#endif
#else
#define UHS_GET_DPI(x) (x)
#endif

#include "../../../../HAL/shared/progmem.h"

////////////////////////////////////////////////////////////////////////////////
// HANDY MACROS
////////////////////////////////////////////////////////////////////////////////

// Atomically set/clear single bits using bitbands.
// Believe it or not, this boils down to a constant,
// and is less code than using |= &= operators.
// Bonus, it makes code easier to read too.
// Bitbanding is a wonderful thing.
#define BITNR(i) (i&0x1?0:i&0x2?1:i&0x4?2:i&0x8?3:i&0x10?4:i&0x20?5:i&0x40?6:i&0x80?7:i&0x100?8:i&0x200?9:i&0x400?10:i&0x800?11:i&0x1000?12:i&0x2000?13:i&0x4000?14:i&0x8000?15:i&0x10000?16:i&0x20000?17:i&0x40000?18:i&0x80000?19:i&0x100000?20:i&0x200000?21:i&0x400000?22:i&0x800000?23:i&0x1000000?24:i&0x2000000?25:i&0x4000000?26:i&0x8000000?27:i&0x10000000?28:i&0x20000000?29:i&0x40000000?30:i&0x80000000?31:32)
#define UHS_KIO_BITBAND_ADDR(r, i) (((uint32_t)&(r) - 0x40000000) * 32 + (i) * 4 + 0x42000000)
#define UHS_KIO_SETBIT_ATOMIC(r, m) (*(uint32_t *)UHS_KIO_BITBAND_ADDR((r), BITNR((m)))) = 1
#define UHS_KIO_CLRBIT_ATOMIC(r, m) (*(uint32_t *)UHS_KIO_BITBAND_ADDR((r), BITNR((m)))) = 0

#define VALUE_BETWEEN(v,l,h) (((v)>(l)) && ((v)<(h)))
#define VALUE_WITHIN(v,l,h) (((v)>=(l)) && ((v)<=(h)))
#define output_pgm_message(wa,fp,mp,el) wa = &mp, fp((char *)pgm_read_pointer(wa), el)
#define output_if_between(v,l,h,wa,fp,mp,el) if(VALUE_BETWEEN(v,l,h)) output_pgm_message(wa,fp,mp[v-(l+1)],el);

#define UHS_SWAP_VALUES(a, b) (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b)))
#ifndef __BYTE_GRABBING_DEFINED__
#define __BYTE_GRABBING_DEFINED__ 1
#ifdef BROKEN_OPTIMIZER_LITTLE_ENDIAN
// NOTE: Use this if your compiler generates horrible assembler!
#define UHS_UINT8_BYTE0(__usi__)  (((uint8_t *)&(__usi__))[0])
#define UHS_UINT8_BYTE1(__usi__)  (((uint8_t *)&(__usi__))[1])
#define UHS_UINT8_BYTE2(__usi__)  (((uint8_t *)&(__usi__))[2])
#define UHS_UINT8_BYTE3(__usi__)  (((uint8_t *)&(__usi__))[3])
#define UHS_UINT8_BYTE4(__usi__)  (((uint8_t *)&(__usi__))[4])
#define UHS_UINT8_BYTE5(__usi__)  (((uint8_t *)&(__usi__))[5])
#define UHS_UINT8_BYTE6(__usi__)  (((uint8_t *)&(__usi__))[6])
#define UHS_UINT8_BYTE7(__usi__)  (((uint8_t *)&(__usi__))[7])
#else
// NOTE: The cast alone to uint8_t is actually enough.
// GCC throws out the "& 0xFF", and the size is no different.
// Some compilers need it.
#define UHS_UINT8_BYTE0(__usi__)  ((uint8_t)((__usi__) & 0xFF ))
#define UHS_UINT8_BYTE1(__usi__)  ((uint8_t)(((__usi__) >> 8) & 0xFF))
#define UHS_UINT8_BYTE2(__usi__)  ((uint8_t)(((__usi__) >> 16) & 0xFF))
#define UHS_UINT8_BYTE3(__usi__)  ((uint8_t)(((__usi__) >> 24) & 0xFF))
#define UHS_UINT8_BYTE4(__usi__)  ((uint8_t)(((__usi__) >> 32) & 0xFF))
#define UHS_UINT8_BYTE5(__usi__)  ((uint8_t)(((__usi__) >> 40) & 0xFF))
#define UHS_UINT8_BYTE6(__usi__)  ((uint8_t)(((__usi__) >> 48) & 0xFF))
#define UHS_UINT8_BYTE7(__usi__)  ((uint8_t)(((__usi__) >> 56) & 0xFF))
#endif
#define UHS_UINT16_SET_BYTE1(__usi__)  ((uint16_t)(__usi__) << 8)
#define UHS_UINT32_SET_BYTE1(__usi__)  ((uint32_t)(__usi__) << 8)
#define UHS_UINT64_SET_BYTE1(__usi__)  ((uint64_t)(__usi__) << 8)
#define UHS_UINT32_SET_BYTE2(__usi__)  ((uint32_t)(__usi__) << 16)
#define UHS_UINT64_SET_BYTE2(__usi__)  ((uint64_t)(__usi__) << 16)
#define UHS_UINT32_SET_BYTE3(__usi__)  ((uint32_t)(__usi__) << 24)
#define UHS_UINT64_SET_BYTE3(__usi__)  ((uint64_t)(__usi__) << 24)
#define UHS_UINT64_SET_BYTE4(__usi__)  ((uint64_t)(__usi__) << 32)
#define UHS_UINT64_SET_BYTE5(__usi__)  ((uint64_t)(__usi__) << 40)
#define UHS_UINT64_SET_BYTE6(__usi__)  ((uint64_t)(__usi__) << 48)
#define UHS_UINT64_SET_BYTE7(__usi__)  ((uint64_t)(__usi__) << 56)

// These are the smallest and fastest ways I have found so far in pure C/C++.
#define UHS_BYTES_TO_UINT16(__usc1__,__usc0__) ((uint16_t)((uint16_t)(__usc0__) | (uint16_t)UHS_UINT16_SET_BYTE1(__usc1__)))
#define UHS_BYTES_TO_UINT32(__usc3__,__usc2__,__usc1__,__usc0__) ((uint32_t)((uint32_t)(__usc0__) | UHS_UINT32_SET_BYTE1(__usc1__) | UHS_UINT32_SET_BYTE2(__usc2__) | UHS_UINT32_SET_BYTE3(__usc3__)))
#define UHS_BYTES_TO_UINT64(__usc7__,__usc6__,__usc5__,__usc4__,__usc3__,__usc2__,__usc1__,__usc0__) ((uint64_t)((uint64_t)__usc0__ | UHS_UINT64_SET_BYTE1(__usc1__) | UHS_UINT64_SET_BYTE2(__usc2__) | UHS_UINT64_SET_BYTE3(__usc3__) | UHS_UINT64_SET_BYTE4(__usc4__) | UHS_UINT64_SET_BYTE5(__usc5__) | UHS_UINT64_SET_BYTE6(__usc6__) | UHS_UINT64_SET_BYTE7(__usc7__)))
#endif
/**
 * Debug macros.
 * Useful when porting from UHS2.
 * Do not use these for any new code.
 * Change to better debugging after port is completed.
 * Strings are stored in progmem (flash) instead of RAM.
 */
#define USBTRACE1(s,l) (Notify(PSTR(s), l))
#define USBTRACE(s) (USBTRACE1((s), 0x80)); USB_HOST_SERIAL.flush()
#define USBTRACE3(s,r,l) (Notify(PSTR(s), l), D_PrintHex((r), l), Notify(PSTR("\r\n"), l))
#define USBTRACE3X(s,r,l) (Notify(PSTR(s), l), D_PrintHex((r), l))
#define USBTRACE2(s,r) (USBTRACE3((s),(r),0x80)); USB_HOST_SERIAL.flush()
#define USBTRACE2X(s,r) (USBTRACE3X((s),(r),0x80)); USB_HOST_SERIAL.flush()

#define VOID0 ((void)0)
#ifndef NOTUSED
#define NOTUSED(...)  __VA_ARGS__ __attribute__((unused))
#endif
#endif // MACROS_H
