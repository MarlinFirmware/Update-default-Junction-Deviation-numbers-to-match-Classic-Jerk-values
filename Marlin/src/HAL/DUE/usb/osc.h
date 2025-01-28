/**
 * \file
 *
 * \brief Chip-specific oscillator management functions.
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
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

#ifndef CHIP_OSC_H_INCLUDED
#define CHIP_OSC_H_INCLUDED

#include "compiler.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * Below BOARD_XXX macros are related to the specific board, and
 * should be defined by the board code, otherwise default value are used.
 */
#ifndef BOARD_FREQ_SLCK_XTAL
  #warning The board slow clock xtal frequency has not been defined.
  #define BOARD_FREQ_SLCK_XTAL      (32768UL)
#endif

#ifndef BOARD_FREQ_SLCK_BYPASS
  #warning The board slow clock bypass frequency has not been defined.
  #define BOARD_FREQ_SLCK_BYPASS    (32768UL)
#endif

#ifndef BOARD_FREQ_MAINCK_XTAL
  #warning The board main clock xtal frequency has not been defined.
  #define BOARD_FREQ_MAINCK_XTAL    (12000000UL)
#endif

#ifndef BOARD_FREQ_MAINCK_BYPASS
  #warning The board main clock bypass frequency has not been defined.
  #define BOARD_FREQ_MAINCK_BYPASS  (12000000UL)
#endif

#ifndef BOARD_OSC_STARTUP_US
  #warning The board main clock xtal startup time has not been defined.
  #define BOARD_OSC_STARTUP_US      (15625UL)
#endif

/**
 * \weakgroup osc_group
 * @{
 */

//! \name Oscillator identifiers
//@{
#define OSC_SLCK_32K_RC             0    //!< Internal 32kHz RC oscillator.
#define OSC_SLCK_32K_XTAL           1    //!< External 32kHz crystal oscillator.
#define OSC_SLCK_32K_BYPASS         2    //!< External 32kHz bypass oscillator.
#define OSC_MAINCK_4M_RC            3    //!< Internal 4MHz RC oscillator.
#define OSC_MAINCK_8M_RC            4    //!< Internal 8MHz RC oscillator.
#define OSC_MAINCK_12M_RC           5    //!< Internal 12MHz RC oscillator.
#define OSC_MAINCK_XTAL             6    //!< External crystal oscillator.
#define OSC_MAINCK_BYPASS           7    //!< External bypass oscillator.
//@}

//! \name Oscillator clock speed in hertz
//@{
#define OSC_SLCK_32K_RC_HZ          CHIP_FREQ_SLCK_RC               //!< Internal 32kHz RC oscillator.
#define OSC_SLCK_32K_XTAL_HZ        BOARD_FREQ_SLCK_XTAL            //!< External 32kHz crystal oscillator.
#define OSC_SLCK_32K_BYPASS_HZ      BOARD_FREQ_SLCK_BYPASS          //!< External 32kHz bypass oscillator.
#define OSC_MAINCK_4M_RC_HZ         CHIP_FREQ_MAINCK_RC_4MHZ        //!< Internal 4MHz RC oscillator.
#define OSC_MAINCK_8M_RC_HZ         CHIP_FREQ_MAINCK_RC_8MHZ        //!< Internal 8MHz RC oscillator.
#define OSC_MAINCK_12M_RC_HZ        CHIP_FREQ_MAINCK_RC_12MHZ       //!< Internal 12MHz RC oscillator.
#define OSC_MAINCK_XTAL_HZ          BOARD_FREQ_MAINCK_XTAL          //!< External crystal oscillator.
#define OSC_MAINCK_BYPASS_HZ        BOARD_FREQ_MAINCK_BYPASS        //!< External bypass oscillator.
//@}

static inline void osc_enable(uint32_t ul_id) {
  switch (ul_id) {
  case OSC_SLCK_32K_RC:
    break;

  case OSC_SLCK_32K_XTAL:
    pmc_switch_sclk_to_32kxtal(PMC_OSC_XTAL);
    break;

  case OSC_SLCK_32K_BYPASS:
    pmc_switch_sclk_to_32kxtal(PMC_OSC_BYPASS);
    break;

  case OSC_MAINCK_4M_RC:
    pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_4_MHz);
    break;

  case OSC_MAINCK_8M_RC:
    pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_8_MHz);
    break;

  case OSC_MAINCK_12M_RC:
    pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_12_MHz);
    break;

  case OSC_MAINCK_XTAL:
    pmc_switch_mainck_to_xtal(PMC_OSC_XTAL/*,
      pmc_us_to_moscxtst(BOARD_OSC_STARTUP_US,
        OSC_SLCK_32K_RC_HZ)*/);
    break;

  case OSC_MAINCK_BYPASS:
    pmc_switch_mainck_to_xtal(PMC_OSC_BYPASS/*,
      pmc_us_to_moscxtst(BOARD_OSC_STARTUP_US,
        OSC_SLCK_32K_RC_HZ)*/);
    break;
  }
}

static inline void osc_disable(uint32_t ul_id) {
  switch (ul_id) {
  case OSC_SLCK_32K_RC:
  case OSC_SLCK_32K_XTAL:
  case OSC_SLCK_32K_BYPASS:
    break;

  case OSC_MAINCK_4M_RC:
  case OSC_MAINCK_8M_RC:
  case OSC_MAINCK_12M_RC:
    pmc_osc_disable_fastrc();
    break;

  case OSC_MAINCK_XTAL:
    pmc_osc_disable_xtal(PMC_OSC_XTAL);
    break;

  case OSC_MAINCK_BYPASS:
    pmc_osc_disable_xtal(PMC_OSC_BYPASS);
    break;
  }
}

static inline bool osc_is_ready(uint32_t ul_id) {
  switch (ul_id) {
  case OSC_SLCK_32K_RC:
    return 1;

  case OSC_SLCK_32K_XTAL:
  case OSC_SLCK_32K_BYPASS:
    return pmc_osc_is_ready_32kxtal();

  case OSC_MAINCK_4M_RC:
  case OSC_MAINCK_8M_RC:
  case OSC_MAINCK_12M_RC:
  case OSC_MAINCK_XTAL:
  case OSC_MAINCK_BYPASS:
    return pmc_osc_is_ready_mainck();
  }

  return 0;
}

static inline uint32_t osc_get_rate(uint32_t ul_id) {
  switch (ul_id) {
  case OSC_SLCK_32K_RC:
    return OSC_SLCK_32K_RC_HZ;

  case OSC_SLCK_32K_XTAL:
    return BOARD_FREQ_SLCK_XTAL;

  case OSC_SLCK_32K_BYPASS:
    return BOARD_FREQ_SLCK_BYPASS;

  case OSC_MAINCK_4M_RC:
    return OSC_MAINCK_4M_RC_HZ;

  case OSC_MAINCK_8M_RC:
    return OSC_MAINCK_8M_RC_HZ;

  case OSC_MAINCK_12M_RC:
    return OSC_MAINCK_12M_RC_HZ;

  case OSC_MAINCK_XTAL:
    return BOARD_FREQ_MAINCK_XTAL;

  case OSC_MAINCK_BYPASS:
    return BOARD_FREQ_MAINCK_BYPASS;
  }

  return 0;
}

/**
 * \brief Wait until the oscillator identified by \a id is ready
 *
 * This function will busy-wait for the oscillator identified by \a id
 * to become stable and ready to use as a clock source.
 *
 * \param id A number identifying the oscillator to wait for.
 */
static inline void osc_wait_ready(uint8_t id) {
  while (!osc_is_ready(id)) {
    /* Do nothing */
  }
}

//! @}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif // CHIP_OSC_H_INCLUDED
