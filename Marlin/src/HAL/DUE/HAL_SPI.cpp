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
 * Software SPI functions originally from Arduino Sd2Card Library
 * Copyright (c) 2009 by William Greiman
 *
 * Completely rewritten and tuned by Eduardo José Tagle in 2017/2018
 * in ARM thumb2 inline assembler and tuned for maximum speed and performance
 * allowing SPI clocks of up to 12 Mhz to increase SD card read/write performance
 */

/**
 * HAL for Arduino Due and compatible (SAM3X8E)
 */

#ifdef ARDUINO_ARCH_SAM

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"

// ------------------------
// Public functions
// ------------------------

#if ANY(SOFTWARE_SPI, FORCE_SOFT_SPI)

  // ------------------------
  // Software SPI
  // ------------------------

  // Make sure GCC optimizes this file.
  // Note that this line triggers a bug in GCC which is fixed by casting.
  // See the note below.
  #pragma GCC optimize (3)

  typedef uint8_t (*pfnSpiTransfer)(uint8_t b);
  typedef void    (*pfnSpiRxBlock)(uint8_t *buf, uint32_t nbyte);
  typedef void    (*pfnSpiTxBlock)(const uint8_t *buf, uint32_t nbyte);

  /* ---------------- Macros to be able to access definitions from asm */
  #define _PORT(IO) DIO ##  IO ## _WPORT
  #define _PIN_MASK(IO) MASK(DIO ## IO ## _PIN)
  #define _PIN_SHIFT(IO) DIO ## IO ## _PIN
  #define PORT(IO) _PORT(IO)
  #define PIN_MASK(IO) _PIN_MASK(IO)
  #define PIN_SHIFT(IO) _PIN_SHIFT(IO)

  // run at ~8 .. ~10Mhz - Tx version (Rx data discarded)
  static uint8_t spiTransferTx0(uint8_t bout) { // using Mode 0
    uint32_t MOSI_PORT_PLUS30 = ((uint32_t) PORT(SD_MOSI_PIN)) + 0x30;  /* SODR of port */
    uint32_t MOSI_MASK = PIN_MASK(SD_MOSI_PIN);
    uint32_t SCK_PORT_PLUS30 = ((uint32_t) PORT(SD_SCK_PIN)) + 0x30;    /* SODR of port */
    uint32_t SCK_MASK = PIN_MASK(SD_SCK_PIN);
    uint32_t idx = 0;

    /* Negate bout, as the assembler requires a negated value */
    bout = ~bout;

    /* The software SPI routine */
    __asm__ __volatile__(
      A(".syntax unified") // is to prevent CM0,CM1 non-unified syntax

      /* Bit 7 */
      A("ubfx %[idx],%[txval],#7,#1")                      /* Place bit 7 in bit 0 of idx*/

      A("str %[mosi_mask],[%[mosi_port], %[idx],LSL #2]")  /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[idx],%[txval],#6,#1")                      /* Place bit 6 in bit 0 of idx*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 6 */
      A("str %[mosi_mask],[%[mosi_port], %[idx],LSL #2]")  /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[idx],%[txval],#5,#1")                      /* Place bit 5 in bit 0 of idx*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 5 */
      A("str %[mosi_mask],[%[mosi_port], %[idx],LSL #2]")  /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[idx],%[txval],#4,#1")                      /* Place bit 4 in bit 0 of idx*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 4 */
      A("str %[mosi_mask],[%[mosi_port], %[idx],LSL #2]")  /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[idx],%[txval],#3,#1")                      /* Place bit 3 in bit 0 of idx*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 3 */
      A("str %[mosi_mask],[%[mosi_port], %[idx],LSL #2]")  /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[idx],%[txval],#2,#1")                      /* Place bit 2 in bit 0 of idx*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 2 */
      A("str %[mosi_mask],[%[mosi_port], %[idx],LSL #2]")  /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[idx],%[txval],#1,#1")                      /* Place bit 1 in bit 0 of idx*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 1 */
      A("str %[mosi_mask],[%[mosi_port], %[idx],LSL #2]")  /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[idx],%[txval],#0,#1")                      /* Place bit 0 in bit 0 of idx*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 0 */
      A("str %[mosi_mask],[%[mosi_port], %[idx],LSL #2]")  /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("nop")                                             /* Result will be 0 */
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      : [idx]"+r"( idx )
      : [txval]"r"( bout ) ,
        [mosi_mask]"r"( MOSI_MASK ),
        [mosi_port]"r"( MOSI_PORT_PLUS30 ),
        [sck_mask]"r"( SCK_MASK ),
        [sck_port]"r"( SCK_PORT_PLUS30 )
      : "cc"
    );

    return 0;
  }

   // Calculates the bit band alias address and returns a pointer address to word.
   // addr: The byte address of bitbanding bit.
   // bit:  The bit position of bitbanding bit.
  #define BITBAND_ADDRESS(addr, bit) \
    (((uint32_t)(addr) & 0xF0000000) + 0x02000000 + ((uint32_t)(addr)&0xFFFFF)*32 + (bit)*4)

  // run at ~8 .. ~10Mhz - Rx version (Tx line not altered)
  static uint8_t spiTransferRx0(uint8_t) { // using Mode 0
    uint32_t bin = 0;
    uint32_t work = 0;
    uint32_t BITBAND_MISO_PORT = BITBAND_ADDRESS( ((uint32_t)PORT(SD_MISO_PIN))+0x3C, PIN_SHIFT(SD_MISO_PIN));  /* PDSR of port in bitband area */
    uint32_t SCK_PORT_PLUS30 = ((uint32_t) PORT(SD_SCK_PIN)) + 0x30;    /* SODR of port */
    uint32_t SCK_MASK = PIN_MASK(SD_SCK_PIN);

    /* The software SPI routine */
    __asm__ __volatile__(
      A(".syntax unified") // is to prevent CM0,CM1 non-unified syntax

      /* bit 7 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#7,#1")                /* Store read bit as the bit 7 */

      /* bit 6 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#6,#1")                /* Store read bit as the bit 6 */

      /* bit 5 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#5,#1")                /* Store read bit as the bit 5 */

      /* bit 4 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#4,#1")                /* Store read bit as the bit 4 */

      /* bit 3 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#3,#1")                /* Store read bit as the bit 3 */

      /* bit 2 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#2,#1")                /* Store read bit as the bit 2 */

      /* bit 1 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#1,#1")                /* Store read bit as the bit 1 */

      /* bit 0 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#0,#1")                /* Store read bit as the bit 0 */

      : [bin]"+r"(bin),
        [work]"+r"(work)
      : [bitband_miso_port]"r"( BITBAND_MISO_PORT ),
        [sck_mask]"r"( SCK_MASK ),
        [sck_port]"r"( SCK_PORT_PLUS30 )
      : "cc"
    );

    return bin;
  }

  // run at ~4Mhz
  static uint8_t spiTransfer1(uint8_t b) { // using Mode 0
    int bits = 8;
    do {
      WRITE(SD_MOSI_PIN, b & 0x80);
      b <<= 1;        // little setup time

      WRITE(SD_SCK_PIN, HIGH);
      DELAY_NS(125);  // 10 cycles @ 84mhz

      b |= (READ(SD_MISO_PIN) != 0);

      WRITE(SD_SCK_PIN, LOW);
      DELAY_NS(125);  // 10 cycles @ 84mhz
    } while (--bits);
    return b;
  }

  // all the others
  static uint16_t spiDelayNS = 4000; // 4000ns => 125khz

  static uint8_t spiTransferX(uint8_t b) { // using Mode 0
    int bits = 8;
    do {
      WRITE(SD_MOSI_PIN, b & 0x80);
      b <<= 1; // little setup time

      WRITE(SD_SCK_PIN, HIGH);
      DELAY_NS_VAR(spiDelayNS);

      b |= (READ(SD_MISO_PIN) != 0);

      WRITE(SD_SCK_PIN, LOW);
      DELAY_NS_VAR(spiDelayNS);
    } while (--bits);
    return b;
  }

  // Pointers to generic functions for byte transfers

  /**
   * Note: The cast is unnecessary, but without it, this file triggers a GCC 4.8.3-2014 bug.
   * Later GCC versions do not have this problem, but at this time (May 2018) Arduino still
   * uses that buggy and obsolete GCC version!!
   */
  static pfnSpiTransfer spiTransferRx = (pfnSpiTransfer)spiTransferX;
  static pfnSpiTransfer spiTransferTx = (pfnSpiTransfer)spiTransferX;

  // Block transfers run at ~8 .. ~10Mhz - Tx version (Rx data discarded)
  static void spiTxBlock0(const uint8_t *ptr, uint32_t todo) {
    uint32_t MOSI_PORT_PLUS30 = ((uint32_t) PORT(SD_MOSI_PIN)) + 0x30;  /* SODR of port */
    uint32_t MOSI_MASK = PIN_MASK(SD_MOSI_PIN);
    uint32_t SCK_PORT_PLUS30 = ((uint32_t) PORT(SD_SCK_PIN)) + 0x30;    /* SODR of port */
    uint32_t SCK_MASK = PIN_MASK(SD_SCK_PIN);
    uint32_t work = 0;
    uint32_t txval = 0;

    /* The software SPI routine */
    __asm__ __volatile__(
      A(".syntax unified") // is to prevent CM0,CM1 non-unified syntax

      L("loop%=")
      A("ldrb.w %[txval], [%[ptr]], #1")                   /* Load value to send, increment buffer */
      A("mvn %[txval],%[txval]")                           /* Negate value */

      /* Bit 7 */
      A("ubfx %[work],%[txval],#7,#1")                     /* Place bit 7 in bit 0 of work*/

      A("str %[mosi_mask],[%[mosi_port], %[work],LSL #2]") /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[work],%[txval],#6,#1")                     /* Place bit 6 in bit 0 of work*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 6 */
      A("str %[mosi_mask],[%[mosi_port], %[work],LSL #2]") /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[work],%[txval],#5,#1")                     /* Place bit 5 in bit 0 of work*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 5 */
      A("str %[mosi_mask],[%[mosi_port], %[work],LSL #2]") /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[work],%[txval],#4,#1")                     /* Place bit 4 in bit 0 of work*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 4 */
      A("str %[mosi_mask],[%[mosi_port], %[work],LSL #2]") /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[work],%[txval],#3,#1")                     /* Place bit 3 in bit 0 of work*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 3 */
      A("str %[mosi_mask],[%[mosi_port], %[work],LSL #2]") /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[work],%[txval],#2,#1")                     /* Place bit 2 in bit 0 of work*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 2 */
      A("str %[mosi_mask],[%[mosi_port], %[work],LSL #2]") /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[work],%[txval],#1,#1")                     /* Place bit 1 in bit 0 of work*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 1 */
      A("str %[mosi_mask],[%[mosi_port], %[work],LSL #2]") /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("ubfx %[work],%[txval],#0,#1")                     /* Place bit 0 in bit 0 of work*/
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */

      /* Bit 0 */
      A("str %[mosi_mask],[%[mosi_port], %[work],LSL #2]") /* Access the proper SODR or CODR registers based on that bit */
      A("str %[sck_mask],[%[sck_port]]")                   /* SODR */
      A("subs %[todo],#1")                                 /* Decrement count of pending words to send, update status */
      A("str %[sck_mask],[%[sck_port],#0x4]")              /* CODR */
      A("bne.n loop%=")                                    /* Repeat until done */

      : [ptr]"+r" ( ptr ) ,
        [todo]"+r" ( todo ) ,
        [work]"+r"( work ) ,
        [txval]"+r"( txval )
      : [mosi_mask]"r"( MOSI_MASK ),
        [mosi_port]"r"( MOSI_PORT_PLUS30 ),
        [sck_mask]"r"( SCK_MASK ),
        [sck_port]"r"( SCK_PORT_PLUS30 )
      : "cc"
    );
  }

  static void spiRxBlock0(uint8_t *ptr, uint32_t todo) {
    uint32_t bin = 0;
    uint32_t work = 0;
    uint32_t BITBAND_MISO_PORT = BITBAND_ADDRESS( ((uint32_t)PORT(SD_MISO_PIN))+0x3C, PIN_SHIFT(SD_MISO_PIN));  /* PDSR of port in bitband area */
    uint32_t SCK_PORT_PLUS30 = ((uint32_t) PORT(SD_SCK_PIN)) + 0x30;    /* SODR of port */
    uint32_t SCK_MASK = PIN_MASK(SD_SCK_PIN);

    /* The software SPI routine */
    __asm__ __volatile__(
      A(".syntax unified")                  // is to prevent CM0,CM1 non-unified syntax

      L("loop%=")

      /* bit 7 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#7,#1")                /* Store read bit as the bit 7 */

      /* bit 6 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#6,#1")                /* Store read bit as the bit 6 */

      /* bit 5 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#5,#1")                /* Store read bit as the bit 5 */

      /* bit 4 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#4,#1")                /* Store read bit as the bit 4 */

      /* bit 3 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#3,#1")                /* Store read bit as the bit 3 */

      /* bit 2 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#2,#1")                /* Store read bit as the bit 2 */

      /* bit 1 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#1,#1")                /* Store read bit as the bit 1 */

      /* bit 0 */
      A("str %[sck_mask],[%[sck_port]]")           /* SODR */
      A("ldr %[work],[%[bitband_miso_port]]")      /* PDSR on bitband area for required bit: work will be 1 or 0 based on port */
      A("str %[sck_mask],[%[sck_port],#0x4]")      /* CODR */
      A("bfi %[bin],%[work],#0,#1")                /* Store read bit as the bit 0 */

      A("subs %[todo],#1")                         /* Decrement count of pending words to send, update status */
      A("strb.w %[bin], [%[ptr]], #1")             /* Store read value into buffer, increment buffer pointer */
      A("bne.n loop%=")                            /* Repeat until done */

      : [ptr]"+r"(ptr),
        [todo]"+r"(todo),
        [bin]"+r"(bin),
        [work]"+r"(work)
      : [bitband_miso_port]"r"( BITBAND_MISO_PORT ),
        [sck_mask]"r"( SCK_MASK ),
        [sck_port]"r"( SCK_PORT_PLUS30 )
      : "cc"
    );
  }

  static void spiTxBlockX(const uint8_t *buf, uint32_t todo) {
    do {
      (void)spiTransferTx(*buf++);
    } while (--todo);
  }

  static void spiRxBlockX(uint8_t *buf, uint32_t todo) {
    do {
      *buf++ = spiTransferRx(0xFF);
    } while (--todo);
  }

  // Pointers to generic functions for block transfers
  static pfnSpiTxBlock spiTxBlock = (pfnSpiTxBlock)spiTxBlockX;
  static pfnSpiRxBlock spiRxBlock = (pfnSpiRxBlock)spiRxBlockX;

  #if MB(ALLIGATOR)
    #define _SS_WRITE(S) WRITE(SD_SS_PIN, S)
  #else
    #define _SS_WRITE(S) NOOP
  #endif

  void spiBegin() {
    SET_OUTPUT(SD_SS_PIN);
    _SS_WRITE(HIGH);
    SET_OUTPUT(SD_SCK_PIN);
    SET_INPUT(SD_MISO_PIN);
    SET_OUTPUT(SD_MOSI_PIN);
  }

  uint8_t spiRec() {
    _SS_WRITE(LOW);
    WRITE(SD_MOSI_PIN, HIGH); // Output 1s 1
    uint8_t b = spiTransferRx(0xFF);
    _SS_WRITE(HIGH);
    return b;
  }

  void spiRead(uint8_t *buf, uint16_t nbyte) {
    if (nbyte) {
      _SS_WRITE(LOW);
      WRITE(SD_MOSI_PIN, HIGH); // Output 1s 1
      spiRxBlock(buf, nbyte);
      _SS_WRITE(HIGH);
    }
  }

  void spiSend(uint8_t b) {
    _SS_WRITE(LOW);
    (void)spiTransferTx(b);
    _SS_WRITE(HIGH);
  }

  void spiSendBlock(uint8_t token, const uint8_t *buf) {
    _SS_WRITE(LOW);
    (void)spiTransferTx(token);
    spiTxBlock(buf, 512);
    _SS_WRITE(HIGH);
  }

  /**
   * spiRate should be
   *  0 :  8 - 10 MHz
   *  1 :  4 - 5 MHz
   *  2 :  2 - 2.5 MHz
   *  3 :  1 - 1.25 MHz
   *  4 :  500 - 625 kHz
   *  5 :  250 - 312 kHz
   *  6 :  125 - 156 kHz
   */
  void spiInit(uint8_t spiRate) {
    switch (spiRate) {
      case 0:
        spiTransferTx = (pfnSpiTransfer)spiTransferTx0;
        spiTransferRx = (pfnSpiTransfer)spiTransferRx0;
        spiTxBlock = (pfnSpiTxBlock)spiTxBlock0;
        spiRxBlock = (pfnSpiRxBlock)spiRxBlock0;
        break;
      case 1:
        spiTransferTx = (pfnSpiTransfer)spiTransfer1;
        spiTransferRx = (pfnSpiTransfer)spiTransfer1;
        spiTxBlock = (pfnSpiTxBlock)spiTxBlockX;
        spiRxBlock = (pfnSpiRxBlock)spiRxBlockX;
        break;
      default:
        spiDelayNS = 4000 >> (6 - spiRate); // spiRate of 2 gives the maximum error with current CPU
        spiTransferTx = (pfnSpiTransfer)spiTransferX;
        spiTransferRx = (pfnSpiTransfer)spiTransferX;
        spiTxBlock = (pfnSpiTxBlock)spiTxBlockX;
        spiRxBlock = (pfnSpiRxBlock)spiRxBlockX;
        break;
    }

    _SS_WRITE(HIGH);
    WRITE(SD_MOSI_PIN, HIGH);
    WRITE(SD_SCK_PIN, LOW);
  }

  /** Begin SPI transaction, set clock, bit order, data mode */
  void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode) {
    // TODO: to be implemented
  }

  #pragma GCC reset_options

#else // !SOFTWARE_SPI

  #define WHILE_TX(N) while ((SPI0->SPI_SR & SPI_SR_TDRE) == (N))
  #define WHILE_RX(N) while ((SPI0->SPI_SR & SPI_SR_RDRF) == (N))
  #define FLUSH_TX() do{ WHILE_RX(1) SPI0->SPI_RDR; }while(0)

  #if MB(ALLIGATOR)

    // slave selects controlled by SPI controller
    // doesn't support changing SPI speeds for SD card

    // ------------------------
    // hardware SPI
    // ------------------------
    static bool spiInitialized = false;

    void spiInit(uint8_t spiRate) {
      if (spiInitialized) return;

      // 8.4 MHz, 4 MHz, 2 MHz, 1 MHz, 0.5 MHz, 0.329 MHz, 0.329 MHz
      constexpr int spiDivider[] = { 10, 21, 42, 84, 168, 255, 255 };
      if (spiRate > 6) spiRate = 1;

      // Set SPI mode 1, clock, select not active after transfer, with delay between transfers
      SPI_ConfigureNPCS(SPI0, SPI_CHAN_DAC,
                        SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDivider[spiRate]) |
                        SPI_CSR_DLYBCT(1));
      // Set SPI mode 0, clock, select not active after transfer, with delay between transfers
      SPI_ConfigureNPCS(SPI0, SPI_CHAN_EEPROM1, SPI_CSR_NCPHA |
                        SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDivider[spiRate]) |
                        SPI_CSR_DLYBCT(1));

      // Set SPI mode 0, clock, select not active after transfer, with delay between transfers
      SPI_ConfigureNPCS(SPI0, SPI_CHAN, SPI_CSR_NCPHA |
                        SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDivider[spiRate]) |
                        SPI_CSR_DLYBCT(1));
      SPI_Enable(SPI0);
      spiInitialized = true;
    }

    void spiBegin() {
      if (spiInitialized) return;

      // Configure SPI pins
      PIO_Configure(
         g_APinDescription[SD_SCK_PIN].pPort,
         g_APinDescription[SD_SCK_PIN].ulPinType,
         g_APinDescription[SD_SCK_PIN].ulPin,
         g_APinDescription[SD_SCK_PIN].ulPinConfiguration);
      PIO_Configure(
         g_APinDescription[SD_MOSI_PIN].pPort,
         g_APinDescription[SD_MOSI_PIN].ulPinType,
         g_APinDescription[SD_MOSI_PIN].ulPin,
         g_APinDescription[SD_MOSI_PIN].ulPinConfiguration);
      PIO_Configure(
         g_APinDescription[SD_MISO_PIN].pPort,
         g_APinDescription[SD_MISO_PIN].ulPinType,
         g_APinDescription[SD_MISO_PIN].ulPin,
         g_APinDescription[SD_MISO_PIN].ulPinConfiguration);

      // set master mode, peripheral select, fault detection
      SPI_Configure(SPI0, ID_SPI0, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PS);
      SPI_Enable(SPI0);

      SET_OUTPUT(DAC0_SYNC_PIN);
      #if HAS_MULTI_EXTRUDER
        OUT_WRITE(DAC1_SYNC_PIN, HIGH);
      #endif
      WRITE(DAC0_SYNC_PIN, HIGH);
      OUT_WRITE(SPI_EEPROM1_CS_PIN, HIGH);
      OUT_WRITE(SPI_EEPROM2_CS_PIN, HIGH);
      OUT_WRITE(SPI_FLASH_CS_PIN, HIGH);
      OUT_WRITE(SD_SS_PIN, HIGH);
      WRITE(SD_SS_PIN, LOW);

      PIO_Configure(
        g_APinDescription[SPI_PIN].pPort,
        g_APinDescription[SPI_PIN].ulPinType,
        g_APinDescription[SPI_PIN].ulPin,
        g_APinDescription[SPI_PIN].ulPinConfiguration
      );

      spiInit(1);
    }

    // Read single byte from SPI
    uint8_t spiRec() {
      // write dummy byte with address and end transmission flag
      SPI0->SPI_TDR = 0x000000FF | SPI_PCS(SPI_CHAN) | SPI_TDR_LASTXFER;

      WHILE_TX(0);
      WHILE_RX(0);

      //DELAY_US(1U);
      return SPI0->SPI_RDR;
    }

    uint8_t spiRec(uint32_t chan) {

      WHILE_TX(0);
      FLUSH_RX();

      // write dummy byte with address and end transmission flag
      SPI0->SPI_TDR = 0x000000FF | SPI_PCS(chan) | SPI_TDR_LASTXFER;
      WHILE_RX(0);

      return SPI0->SPI_RDR;
    }

    // Read from SPI into buffer
    void spiRead(uint8_t *buf, uint16_t nbyte) {
      if (!nbyte) return;
      --nbyte;
      for (int i = 0; i < nbyte; i++) {
        //WHILE_TX(0);
        SPI0->SPI_TDR = 0x000000FF | SPI_PCS(SPI_CHAN);
        WHILE_RX(0);
        buf[i] = SPI0->SPI_RDR;
        //DELAY_US(1U);
      }
      buf[nbyte] = spiRec();
    }

    // Write single byte to SPI
    void spiSend(const byte b) {
      // write byte with address and end transmission flag
      SPI0->SPI_TDR = (uint32_t)b | SPI_PCS(SPI_CHAN) | SPI_TDR_LASTXFER;
      WHILE_TX(0);
      WHILE_RX(0);
      SPI0->SPI_RDR;
      //DELAY_US(1U);
    }

    void spiSend(const uint8_t *buf, size_t nbyte) {
      if (!nbyte) return;
      --nbyte;
      for (size_t i = 0; i < nbyte; i++) {
        SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(SPI_CHAN);
        WHILE_TX(0);
        WHILE_RX(0);
        SPI0->SPI_RDR;
        //DELAY_US(1U);
      }
      spiSend(buf[nbyte]);
    }

    void spiSend(uint32_t chan, byte b) {
      WHILE_TX(0);
      // write byte with address and end transmission flag
      SPI0->SPI_TDR = (uint32_t)b | SPI_PCS(chan) | SPI_TDR_LASTXFER;
      WHILE_RX(0);
      FLUSH_RX();
    }

    void spiSend(uint32_t chan, const uint8_t *buf, size_t nbyte) {
      if (!nbyte) return;
      --nbyte;
      for (size_t i = 0; i < nbyte; i++) {
        WHILE_TX(0);
        SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(chan);
        WHILE_RX(0);
        FLUSH_RX();
      }
      spiSend(chan, buf[nbyte]);
    }

    // Write from buffer to SPI
    void spiSendBlock(uint8_t token, const uint8_t *buf) {
      SPI0->SPI_TDR = (uint32_t)token | SPI_PCS(SPI_CHAN);
      WHILE_TX(0);
      //WHILE_RX(0);
      //SPI0->SPI_RDR;
      for (int i = 0; i < 511; i++) {
        SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(SPI_CHAN);
        WHILE_TX(0);
        WHILE_RX(0);
        SPI0->SPI_RDR;
        //DELAY_US(1U);
      }
      spiSend(buf[511]);
    }

    /** Begin SPI transaction, set clock, bit order, data mode */
    void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode) {
      // TODO: to be implemented
    }

  #else // U8G compatible hardware SPI

    #define SPI_MODE_0_DUE_HW 2  // DUE CPHA control bit is inverted
    #define SPI_MODE_1_DUE_HW 3
    #define SPI_MODE_2_DUE_HW 0
    #define SPI_MODE_3_DUE_HW 1

    /**
     *  The DUE SPI controller is set up so the upper word of the longword
     *  written to the transmit data register selects which SPI Chip Select
     *  Register is used. This allows different streams to have different SPI
     *  settings.
     *
     *  In practice it's spooky. Some combinations hang the system, while others
     *  upset the peripheral device.
     *
     *  SPI mode should be the same for all streams. The FYSETC_MINI_12864 gets
     *  upset if the clock phase changes after chip select goes active.
     *
     *  SPI_CSR_CSAAT should be set for all streams. If not the WHILE_TX(0)
     *  macro returns immediately which can result in the SPI chip select going
     *  inactive before all the data has been sent.
     *
     *  The TMC2130 library uses SPI0->SPI_CSR[3].
     *
     *  The U8G hardware SPI uses SPI0->SPI_CSR[0]. The system hangs and/or the
     *  FYSETC_MINI_12864 gets upset if lower baud rates are used and the SD card
     *  is inserted or removed.
     *
     *  The SD card uses SPI0->SPI_CSR[3]. Efforts were made to use [1] and [2]
     *  but they all resulted in hangs or garbage on the LCD.
     *
     *  The SPI controlled chip selects are NOT enabled in the GPIO controller.
     *  The application must control the chip select.
     *
     *  All of the above can be avoided by defining FORCE_SOFT_SPI to force the
     *  display to use software SPI.
     */

    void spiInit(uint8_t spiRate=6) {  // Default to slowest rate if not specified)
                                       // Also sets U8G SPI rate to 4MHz and the SPI mode to 3

      // 8.4 MHz, 4 MHz, 2 MHz, 1 MHz, 0.5 MHz, 0.329 MHz, 0.329 MHz
      constexpr int spiDivider[] = { 10, 21, 42, 84, 168, 255, 255 };
      if (spiRate > 6) spiRate = 1;

      // Enable PIOA and SPI0
      REG_PMC_PCER0 = (1UL << ID_PIOA) | (1UL << ID_SPI0);

      // Disable PIO on A26 and A27
      REG_PIOA_PDR = 0x0C000000;
      OUT_WRITE(SD_SS_PIN, HIGH);

      // Reset SPI0 (from sam lib)
      SPI0->SPI_CR = SPI_CR_SPIDIS;
      SPI0->SPI_CR = SPI_CR_SWRST;
      SPI0->SPI_CR = SPI_CR_SWRST;
      SPI0->SPI_CR = SPI_CR_SPIEN;

      // TMC2103 compatible setup
      // Master mode, no fault detection, PCS bits in data written to TDR select CSR register
      SPI0->SPI_MR = SPI_MR_MSTR | SPI_MR_PS | SPI_MR_MODFDIS;
      // SPI mode 3, 8 Bit data transfer, baud rate
      SPI0->SPI_CSR[3] = SPI_CSR_SCBR(spiDivider[spiRate]) | SPI_CSR_CSAAT | SPI_MODE_3_DUE_HW;  // use same CSR as TMC2130
      SPI0->SPI_CSR[0] = SPI_CSR_SCBR(spiDivider[1]) | SPI_CSR_CSAAT | SPI_MODE_3_DUE_HW;  // U8G default to 4MHz
    }

    void spiBegin() { spiInit(); }

    static uint8_t spiTransfer(uint8_t data) {
      WHILE_TX(0);
      SPI0->SPI_TDR = (uint32_t)data | 0x00070000UL;  // Add TMC2130 PCS bits to every byte (use SPI0->SPI_CSR[3])
      WHILE_TX(0);
      WHILE_RX(0);
      return SPI0->SPI_RDR;
    }

    uint8_t spiRec() { return (uint8_t)spiTransfer(0xFF); }

    void spiRead(uint8_t *buf, uint16_t nbyte) {
      for (int i = 0; i < nbyte; i++)
        buf[i] = spiTransfer(0xFF);
    }

    void spiSend(uint8_t data) { spiTransfer(data); }

    void spiSend(const uint8_t *buf, size_t nbyte) {
      for (uint16_t i = 0; i < nbyte; i++)
        spiTransfer(buf[i]);
    }

    void spiSendBlock(uint8_t token, const uint8_t *buf) {
      spiTransfer(token);
      for (uint16_t i = 0; i < 512; i++)
        spiTransfer(buf[i]);
    }

  #endif // !ALLIGATOR
#endif // !SOFTWARE_SPI

#endif // ARDUINO_ARCH_SAM
