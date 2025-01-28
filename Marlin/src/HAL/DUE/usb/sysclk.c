/**
 * \file
 *
 * \brief Chip-specific system clock management functions.
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

#ifdef ARDUINO_ARCH_SAM

#include "sysclk.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \weakgroup sysclk_group
 * @{
 */

#if defined(CONFIG_USBCLK_SOURCE) || defined(__DOXYGEN__)
/**
 * \brief Enable full speed USB clock.
 *
 * \note The SAM3X PMC hardware interprets div as div+1. For readability the hardware div+1
 * is hidden in this implementation. Use div as div effective value.
 *
 * \param pll_id Source of the USB clock.
 * \param div Actual clock divisor. Must be superior to 0.
 */
void sysclk_enable_usb(void)
{
	Assert(CONFIG_USBCLK_DIV > 0);

#ifdef CONFIG_PLL0_SOURCE
	if (CONFIG_USBCLK_SOURCE == USBCLK_SRC_PLL0) {
		struct pll_config pllcfg;

		pll_enable_source(CONFIG_PLL0_SOURCE);
		pll_config_defaults(&pllcfg, 0);
		pll_enable(&pllcfg, 0);
		pll_wait_for_lock(0);
		pmc_switch_udpck_to_pllack(CONFIG_USBCLK_DIV - 1);
		pmc_enable_udpck();
		return;
	}
#endif

	if (CONFIG_USBCLK_SOURCE == USBCLK_SRC_UPLL) {

		pmc_enable_upll_clock();
		pmc_switch_udpck_to_upllck(CONFIG_USBCLK_DIV - 1);
		pmc_enable_udpck();
		return;
	}
}

/**
 * \brief Disable full speed USB clock.
 *
 * \note This implementation does not switch off the PLL, it just turns off the USB clock.
 */
void sysclk_disable_usb(void)
{
	pmc_disable_udpck();
}
#endif // CONFIG_USBCLK_SOURCE

//! @}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif // ARDUINO_ARCH_SAM
