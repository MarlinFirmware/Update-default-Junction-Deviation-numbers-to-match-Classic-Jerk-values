/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#ifdef __PLAT_NATIVE_SIM__

#include "../../../inc/MarlinConfig.h"

#if IS_U8GLIB_ST7920

#include <U8glib-HAL.h>
#include "../../shared/Delay.h"

#undef SPI_SPEED
#define SPI_SPEED 6
#define SPI_DELAY_CYCLES (1 + SPI_SPEED * 10)

static pin_t SCK_pin_ST7920_HAL, MOSI_pin_ST7920_HAL_HAL;
static uint8_t SPI_speed = 0;

static uint8_t swSpiTransfer(uint8_t b, const uint8_t spi_speed, const pin_t sck_pin, const pin_t miso_pin, const pin_t mosi_pin) {
  for (uint_fast8_t i = 0; i < 8; i++) {
    WRITE_PIN(mosi_pin, !!(b & 0x80));
    DELAY_CYCLES(SPI_SPEED);
    WRITE_PIN(sck_pin, HIGH);
    DELAY_CYCLES(SPI_SPEED);
    b <<= 1;
    if (miso_pin >= 0 && READ_PIN(miso_pin)) b |= 1;
    WRITE_PIN(sck_pin, LOW);
    DELAY_CYCLES(SPI_SPEED);
  }
  return b;
}

static uint8_t swSpiInit(const uint8_t spiRate, const pin_t sck_pin, const pin_t mosi_pin) {
  WRITE_PIN(mosi_pin, HIGH);
  WRITE_PIN(sck_pin, LOW);
  return spiRate;
}

static void u8g_com_st7920_write_byte_sw_spi(uint8_t rs, uint8_t val) {
  static uint8_t rs_last_state = 255;
  if (rs != rs_last_state) {
    // Transfer Data (FA) or Command (F8)
    swSpiTransfer(rs ? 0x0FA : 0x0F8, SPI_speed, SCK_pin_ST7920_HAL, -1, MOSI_pin_ST7920_HAL_HAL);
    rs_last_state = rs;
    DELAY_US(40); // Give the controller time to process the data: 20 is bad, 30 is OK, 40 is safe
  }
  swSpiTransfer(val & 0x0F0, SPI_speed, SCK_pin_ST7920_HAL, -1, MOSI_pin_ST7920_HAL_HAL);
  swSpiTransfer(val << 4, SPI_speed, SCK_pin_ST7920_HAL, -1, MOSI_pin_ST7920_HAL_HAL);
}

#ifdef __cplusplus
  extern "C" {
#endif

uint8_t u8g_com_ST7920_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
  switch (msg) {
    case U8G_COM_MSG_INIT:
      SCK_pin_ST7920_HAL = u8g->pin_list[U8G_PI_SCK];
      MOSI_pin_ST7920_HAL_HAL = u8g->pin_list[U8G_PI_MOSI];

      u8g_SetPIOutput(u8g, U8G_PI_CS);
      u8g_SetPIOutput(u8g, U8G_PI_SCK);
      u8g_SetPIOutput(u8g, U8G_PI_MOSI);
      u8g_Delay(5);

      SPI_speed = swSpiInit(SPI_SPEED, SCK_pin_ST7920_HAL, MOSI_pin_ST7920_HAL_HAL);

      u8g_SetPILevel(u8g, U8G_PI_CS, 0);
      u8g_SetPILevel(u8g, U8G_PI_SCK, 0);
      u8g_SetPILevel(u8g, U8G_PI_MOSI, 0);

      u8g->pin_list[U8G_PI_A0_STATE] = 0;       /* initial RS state: command mode */
      break;

    case U8G_COM_MSG_STOP:
      break;

    case U8G_COM_MSG_RESET:
      if (U8G_PIN_NONE != u8g->pin_list[U8G_PI_RESET]) u8g_SetPILevel(u8g, U8G_PI_RESET, arg_val);
      break;

    case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      u8g->pin_list[U8G_PI_A0_STATE] = arg_val;
      break;

    case U8G_COM_MSG_CHIP_SELECT:
      if (U8G_PIN_NONE != u8g->pin_list[U8G_PI_CS]) u8g_SetPILevel(u8g, U8G_PI_CS, arg_val);  //note: the st7920 has an active high chip select
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      u8g_com_st7920_write_byte_sw_spi(u8g->pin_list[U8G_PI_A0_STATE], arg_val);
      break;

    case U8G_COM_MSG_WRITE_SEQ: {
        uint8_t *ptr = (uint8_t*) arg_ptr;
        while (arg_val > 0) {
          u8g_com_st7920_write_byte_sw_spi(u8g->pin_list[U8G_PI_A0_STATE], *ptr++);
          arg_val--;
        }
      }
      break;

      case U8G_COM_MSG_WRITE_SEQ_P: {
        uint8_t *ptr = (uint8_t*) arg_ptr;
        while (arg_val > 0) {
          u8g_com_st7920_write_byte_sw_spi(u8g->pin_list[U8G_PI_A0_STATE], *ptr++);
          arg_val--;
        }
      }
      break;
  }
  return 1;
}

#ifdef __cplusplus
  }
#endif

#endif // IS_U8GLIB_ST7920
#endif // __PLAT_NATIVE_SIM__
