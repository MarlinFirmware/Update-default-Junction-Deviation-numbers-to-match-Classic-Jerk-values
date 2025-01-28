/**
 * File:   dyn_SWI.h
 * Author: xxxajk@gmail.com
 *
 * Created on December 5, 2014, 9:12 AM
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef DYN_SWI_H
#define DYN_SWI_H

#if defined(__arm__) || defined(ARDUINO_ARCH_PIC32)
#ifdef ARDUINO_ARCH_PIC32
#include <p32xxxx.h>
#endif
#ifdef __cplusplus

#ifdef true
#undef true
#endif

#ifdef false
#undef false
#endif

#endif

#ifdef ARDUINO_spresense_ast
#define SWI_IRQ_NUM 666 // because this board is totally evil.
#elif defined(ARDUINO_ARCH_PIC32)
#ifndef SWI_IRQ_NUM
#ifdef _DSPI0_IPL_ISR
#define SWI_IPL _DSPI0_IPL_ISR
#define SWI_VECTOR _DSPI0_ERR_IRQ
#define SWI_IRQ_NUM _DSPI0_ERR_IRQ
#elif defined(_PMP_ERROR_IRQ)
#define SWI_IRQ_NUM _PMP_ERROR_IRQ
#define SWI_VECTOR _PMP_VECTOR
#else
#error SWI_IRQ_NUM and SWI_VECTOR need a definition
#endif
#ifdef __cplusplus
extern "C"
{
        void
#ifdef __PIC32MZXX__
                __attribute__((nomips16,at_vector(SWI_VECTOR),interrupt(SWI_IPL)))
#else
                __attribute__((interrupt(),nomips16))
#endif
                softISR();
}
#endif
#endif
#elif !defined(NVIC_NUM_INTERRUPTS)
// Assume CMSIS
#define __USE_CMSIS_VECTORS__
#ifdef NUMBER_OF_INT_VECTORS
#define NVIC_NUM_INTERRUPTS (NUMBER_OF_INT_VECTORS-16)
#else
#define NVIC_NUM_INTERRUPTS ((int)PERIPH_COUNT_IRQn)
#endif
#define VECTORTABLE_SIZE (NVIC_NUM_INTERRUPTS+16)
#define VECTORTABLE_ALIGNMENT (0x100UL)
#define NVIC_GET_ACTIVE(n) NVIC_GetActive((IRQn_Type)n)
#define NVIC_GET_PENDING(n) NVIC_GetPendingIRQ((IRQn_Type)n)
#define NVIC_SET_PENDING(n) NVIC_SetPendingIRQ((IRQn_Type)n)
#define NVIC_ENABLE_IRQ(n) NVIC_EnableIRQ((IRQn_Type)n)
#define NVIC_SET_PRIORITY(n ,p) NVIC_SetPriority((IRQn_Type)n, (uint32_t) p)
//extern "C" {
//        extern uint32_t _VectorsRam[VECTORTABLE_SIZE] __attribute__((aligned(VECTORTABLE_ALIGNMENT)));
//}

#ifndef SWI_IRQ_NUM
#if defined(__SAM3X8E__) && defined(_VARIANT_ARDUINO_DUE_X_)
// DUE
// Choices available:
// HSMCI_IRQn Multimedia Card Interface (HSMCI)
// EMAC_IRQn Ethernet MAC (EMAC)
// EMAC is not broken out on the official DUE, but is on clones.
// SPI0_IRQn Serial Peripheral Interface (SPI0)
// SPI0_IRQn seems to be the best choice, as long as nobody uses an ISR based master
#define SWI_IRQ_NUM SPI0_IRQn
#elif defined(ARDUINO_SAMD_ZERO)
// Just use sercom4's unused IRQ vector.
#define SWI_IRQ_NUM I2S_IRQn
//#define SWI_IRQ_NUM SERCOM4_IRQn
#endif
#endif

#ifndef SWI_IRQ_NUM
#error SWI_IRQ_NUM not defined (CMSIS)
#endif

#elif defined(CORE_TEENSY)

#ifndef NVIC_GET_ACTIVE
#define NVIC_GET_ACTIVE(n)  (*((volatile uint32_t *)0xE000E300 + ((n) >> 5)) & (1 << ((n) & 31)))
#endif
#ifndef NVIC_GET_PENDING
#define NVIC_GET_PENDING(n) (*((volatile uint32_t *)0xE000E200 + ((n) >> 5)) & (1 << ((n) & 31)))
#ifndef SWI_IRQ_NUM
#ifdef __MK20DX256__
#define SWI_IRQ_NUM 17
#elif defined(__MK20DX128__)
#define SWI_IRQ_NUM 5
#elif defined(__MKL26Z64__)
#define SWI_IRQ_NUM 4
#elif defined(__MK66FX1M0__)
#define SWI_IRQ_NUM 30
#elif defined(__MK64FX512__)
#define SWI_IRQ_NUM 30
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)
#define SWI_IRQ_NUM 71
#else
#error Do not know how to relocate IRQ vectors for this pjrc product
#endif
#endif
#endif
#else // Not CMSIS or PJRC CORE_TEENSY or PIC32 or SPRESENSE
#error Do not know how to relocate IRQ vectors or perform SWI
#endif // SWI_IRQ_NUM

#ifndef SWI_IRQ_NUM
#error SWI_IRQ_NUM not defined
#else
/**
 * Use this class to extend your class, in order to provide
 * a C++ context callable SWI.
 */
class dyn_SWI {
public:

        /**
         * Override this method with your code.
         */
        virtual void dyn_SWISR() {
        };
};

extern int exec_SWI(const dyn_SWI* klass);

#include "SWI_INLINE.h"

// IMPORTANT! Define this so that you do NOT end up with a NULL stub!
#define SWI_NO_STUB
#endif // SWI_IRQ_NUM
#endif // __arm__

// if no SWI for CPU (e.g. AVR) make a void stub.
#ifndef SWI_NO_STUB
#define Init_dyn_SWI() (void(0))
#ifndef DDSB
#define DDSB() (void(0))
#endif
#endif
#endif // DYN_SWI_H
