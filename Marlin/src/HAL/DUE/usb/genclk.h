/**
 * \file
 *
 * \brief Chip-specific generic clock management.
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

#ifndef CHIP_GENCLK_H_INCLUDED
#define CHIP_GENCLK_H_INCLUDED

#include <osc.h>
#include <pll.h>

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \weakgroup genclk_group
 * @{
 */

//! \name Programmable Clock Identifiers (PCK)
//@{
#define GENCLK_PCK_0      0 //!< PCK0 ID
#define GENCLK_PCK_1      1 //!< PCK1 ID
#define GENCLK_PCK_2      2 //!< PCK2 ID
//@}

//! \name Programmable Clock Sources (PCK)
//@{

enum genclk_source {
  GENCLK_PCK_SRC_SLCK_RC       = 0, //!< Internal 32kHz RC oscillator as PCK source clock
  GENCLK_PCK_SRC_SLCK_XTAL     = 1, //!< External 32kHz crystal oscillator as PCK source clock
  GENCLK_PCK_SRC_SLCK_BYPASS   = 2, //!< External 32kHz bypass oscillator as PCK source clock
  GENCLK_PCK_SRC_MAINCK_4M_RC  = 3, //!< Internal 4MHz RC oscillator as PCK source clock
  GENCLK_PCK_SRC_MAINCK_8M_RC  = 4, //!< Internal 8MHz RC oscillator as PCK source clock
  GENCLK_PCK_SRC_MAINCK_12M_RC = 5, //!< Internal 12MHz RC oscillator as PCK source clock
  GENCLK_PCK_SRC_MAINCK_XTAL   = 6, //!< External crystal oscillator as PCK source clock
  GENCLK_PCK_SRC_MAINCK_BYPASS = 7, //!< External bypass oscillator as PCK source clock
  GENCLK_PCK_SRC_PLLACK        = 8, //!< Use PLLACK as PCK source clock
  GENCLK_PCK_SRC_PLLBCK        = 9, //!< Use PLLBCK as PCK source clock
  GENCLK_PCK_SRC_MCK           = 10, //!< Use Master Clk as PCK source clock
};

//@}

//! \name Programmable Clock Prescalers (PCK)
//@{

enum genclk_divider {
  GENCLK_PCK_PRES_1  = PMC_PCK_PRES_CLK_1, //!< Set PCK clock prescaler to 1
  GENCLK_PCK_PRES_2  = PMC_PCK_PRES_CLK_2, //!< Set PCK clock prescaler to 2
  GENCLK_PCK_PRES_4  = PMC_PCK_PRES_CLK_4, //!< Set PCK clock prescaler to 4
  GENCLK_PCK_PRES_8  = PMC_PCK_PRES_CLK_8, //!< Set PCK clock prescaler to 8
  GENCLK_PCK_PRES_16 = PMC_PCK_PRES_CLK_16, //!< Set PCK clock prescaler to 16
  GENCLK_PCK_PRES_32 = PMC_PCK_PRES_CLK_32, //!< Set PCK clock prescaler to 32
  GENCLK_PCK_PRES_64 = PMC_PCK_PRES_CLK_64, //!< Set PCK clock prescaler to 64
};

//@}

struct genclk_config {
  uint32_t ctrl;
};

static inline void genclk_config_defaults(struct genclk_config *p_cfg, uint32_t ul_id) {
  ul_id = ul_id;
  p_cfg->ctrl = 0;
}

static inline void genclk_config_read(struct genclk_config *p_cfg, uint32_t ul_id) {
  p_cfg->ctrl = PMC->PMC_PCK[ul_id];
}

static inline void genclk_config_write(const struct genclk_config *p_cfg, uint32_t ul_id) {
  PMC->PMC_PCK[ul_id] = p_cfg->ctrl;
}

//! \name Programmable Clock Source and Prescaler configuration
//@{

static inline void genclk_config_set_source(struct genclk_config *p_cfg, enum genclk_source e_src) {
  p_cfg->ctrl &= (~PMC_PCK_CSS_Msk);

  switch (e_src) {
  case GENCLK_PCK_SRC_SLCK_RC:
  case GENCLK_PCK_SRC_SLCK_XTAL:
  case GENCLK_PCK_SRC_SLCK_BYPASS:
    p_cfg->ctrl |= (PMC_PCK_CSS_SLOW_CLK);
    break;

  case GENCLK_PCK_SRC_MAINCK_4M_RC:
  case GENCLK_PCK_SRC_MAINCK_8M_RC:
  case GENCLK_PCK_SRC_MAINCK_12M_RC:
  case GENCLK_PCK_SRC_MAINCK_XTAL:
  case GENCLK_PCK_SRC_MAINCK_BYPASS:
    p_cfg->ctrl |= (PMC_PCK_CSS_MAIN_CLK);
    break;

  case GENCLK_PCK_SRC_PLLACK:
    p_cfg->ctrl |= (PMC_PCK_CSS_PLLA_CLK);
    break;

  case GENCLK_PCK_SRC_PLLBCK:
    p_cfg->ctrl |= (PMC_PCK_CSS_UPLL_CLK);
    break;

  case GENCLK_PCK_SRC_MCK:
    p_cfg->ctrl |= (PMC_PCK_CSS_MCK);
    break;
  }
}

static inline void genclk_config_set_divider(struct genclk_config *p_cfg, uint32_t e_divider) {
  p_cfg->ctrl &= ~PMC_PCK_PRES_Msk;
  p_cfg->ctrl |= e_divider;
}

//@}

static inline void genclk_enable(const struct genclk_config *p_cfg, uint32_t ul_id) {
  PMC->PMC_PCK[ul_id] = p_cfg->ctrl;
  pmc_enable_pck(ul_id);
}

static inline void genclk_disable(uint32_t ul_id) {
  pmc_disable_pck(ul_id);
}

static inline void genclk_enable_source(enum genclk_source e_src) {
  switch (e_src) {
  case GENCLK_PCK_SRC_SLCK_RC:
    if (!osc_is_ready(OSC_SLCK_32K_RC)) {
      osc_enable(OSC_SLCK_32K_RC);
      osc_wait_ready(OSC_SLCK_32K_RC);
    }
    break;

  case GENCLK_PCK_SRC_SLCK_XTAL:
    if (!osc_is_ready(OSC_SLCK_32K_XTAL)) {
      osc_enable(OSC_SLCK_32K_XTAL);
      osc_wait_ready(OSC_SLCK_32K_XTAL);
    }
    break;

  case GENCLK_PCK_SRC_SLCK_BYPASS:
    if (!osc_is_ready(OSC_SLCK_32K_BYPASS)) {
      osc_enable(OSC_SLCK_32K_BYPASS);
      osc_wait_ready(OSC_SLCK_32K_BYPASS);
    }
    break;

  case GENCLK_PCK_SRC_MAINCK_4M_RC:
    if (!osc_is_ready(OSC_MAINCK_4M_RC)) {
      osc_enable(OSC_MAINCK_4M_RC);
      osc_wait_ready(OSC_MAINCK_4M_RC);
    }
    break;

  case GENCLK_PCK_SRC_MAINCK_8M_RC:
    if (!osc_is_ready(OSC_MAINCK_8M_RC)) {
      osc_enable(OSC_MAINCK_8M_RC);
      osc_wait_ready(OSC_MAINCK_8M_RC);
    }
    break;

  case GENCLK_PCK_SRC_MAINCK_12M_RC:
    if (!osc_is_ready(OSC_MAINCK_12M_RC)) {
      osc_enable(OSC_MAINCK_12M_RC);
      osc_wait_ready(OSC_MAINCK_12M_RC);
    }
    break;

  case GENCLK_PCK_SRC_MAINCK_XTAL:
    if (!osc_is_ready(OSC_MAINCK_XTAL)) {
      osc_enable(OSC_MAINCK_XTAL);
      osc_wait_ready(OSC_MAINCK_XTAL);
    }
    break;

  case GENCLK_PCK_SRC_MAINCK_BYPASS:
    if (!osc_is_ready(OSC_MAINCK_BYPASS)) {
      osc_enable(OSC_MAINCK_BYPASS);
      osc_wait_ready(OSC_MAINCK_BYPASS);
    }
    break;

  #ifdef CONFIG_PLL0_SOURCE
    case GENCLK_PCK_SRC_PLLACK:
      pll_enable_config_defaults(0);
      break;
  #endif

  #ifdef CONFIG_PLL1_SOURCE
    case GENCLK_PCK_SRC_PLLBCK:
      pll_enable_config_defaults(1);
      break;
  #endif

  case GENCLK_PCK_SRC_MCK:
    break;

  default:
    Assert(false);
    break;
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

#endif // CHIP_GENCLK_H_INCLUDED
