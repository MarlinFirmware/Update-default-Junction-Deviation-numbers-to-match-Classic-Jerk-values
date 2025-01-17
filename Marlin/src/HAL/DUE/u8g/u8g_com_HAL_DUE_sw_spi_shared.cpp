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
 * Based on u8g_com_st7920_hw_spi.c
 *
 * Universal 8bit Graphics Library
 *
 * Copyright (c) 2011, olikraus@gmail.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list
 *    of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef ARDUINO_ARCH_SAM

#include "../../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_U8GLIB

#include "../../../inc/MarlinConfig.h"
#include "../../shared/Delay.h"

#include <U8glib-HAL.h>

#include "u8g_com_HAL_DUE_sw_spi_shared.h"

void u8g_SetPIOutput_DUE(u8g_t *u8g, uint8_t pin_index) {
  PIO_Configure(g_APinDescription[u8g->pin_list[pin_index]].pPort, PIO_OUTPUT_1,
    g_APinDescription[u8g->pin_list[pin_index]].ulPin, g_APinDescription[u8g->pin_list[pin_index]].ulPinConfiguration);  // OUTPUT
}

void u8g_SetPILevel_DUE(u8g_t *u8g, uint8_t pin_index, uint8_t level) {
  volatile Pio* port = g_APinDescription[u8g->pin_list[pin_index]].pPort;
  uint32_t mask = g_APinDescription[u8g->pin_list[pin_index]].ulPin;
  if (level) port->PIO_SODR = mask; else port->PIO_CODR = mask;
}

Pio *SCK_pPio, *MOSI_pPio;
uint32_t SCK_dwMask, MOSI_dwMask;

void u8g_spiSend_sw_DUE_mode_0(uint8_t val) { // 3MHz
  for (uint_fast8_t i = 0; i < 8; ++i) {
    if (val & 0x80)
      MOSI_pPio->PIO_SODR = MOSI_dwMask;
    else
      MOSI_pPio->PIO_CODR = MOSI_dwMask;
    DELAY_NS(48);
    SCK_pPio->PIO_SODR = SCK_dwMask;
    DELAY_NS(905);
    val <<= 1;
    SCK_pPio->PIO_CODR = SCK_dwMask;
  }
}

void u8g_spiSend_sw_DUE_mode_3(uint8_t val) { // 3.5MHz
  for (uint_fast8_t i = 0; i < 8; ++i) {
    SCK_pPio->PIO_CODR = SCK_dwMask;
    DELAY_NS(50);
    if (val & 0x80)
      MOSI_pPio->PIO_SODR = MOSI_dwMask;
    else
      MOSI_pPio->PIO_CODR = MOSI_dwMask;
    val <<= 1;
    DELAY_NS(10);
    SCK_pPio->PIO_SODR = SCK_dwMask;
    DELAY_NS(70);
  }
}

#endif // HAS_MARLINUI_U8GLIB
#endif // ARDUINO_ARCH_SAM
