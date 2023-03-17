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

#ifdef __STM32F1__

#include "../../../inc/MarlinConfig.h"

#if ALL(HAS_MARLINUI_U8GLIB, FORCE_SOFT_SPI)

#include <U8glib-HAL.h>
#include "../../shared/HAL_SPI.h"

#ifndef LCD_SPI_SPEED
  #define LCD_SPI_SPEED SPI_FULL_SPEED    // Fastest
  //#define LCD_SPI_SPEED SPI_QUARTER_SPEED // Slower
#endif

static uint8_t SPI_speed = LCD_SPI_SPEED;

static inline uint8_t swSpiTransfer_mode_0(uint8_t b, const uint8_t spi_speed, const pin_t miso_pin=-1) {
  for (uint_fast8_t i = 0; i < 8; ++i) {
    if (spi_speed == 0) {
      WRITE(DOGLCD_MOSI, !!(b & 0x80));
      WRITE(DOGLCD_SCK, HIGH);
      b <<= 1;
      if (miso_pin >= 0 && READ(miso_pin)) b |= 1;
      WRITE(DOGLCD_SCK, LOW);
    }
    else {
      const uint8_t state = (b & 0x80) ? HIGH : LOW;
      for (uint_fast8_t j = 0; j < spi_speed; ++j)
        WRITE(DOGLCD_MOSI, state);

      for (uint_fast8_t j = 0; j < spi_speed + (miso_pin >= 0 ? 0 : 1); ++j)
        WRITE(DOGLCD_SCK, HIGH);

      b <<= 1;
      if (miso_pin >= 0 && READ(miso_pin)) b |= 1;

      for (uint_fast8_t j = 0; j < spi_speed; ++j)
        WRITE(DOGLCD_SCK, LOW);
    }
  }
  return b;
}

static inline uint8_t swSpiTransfer_mode_3(uint8_t b, const uint8_t spi_speed, const pin_t miso_pin=-1) {
  for (uint_fast8_t i = 0; i < 8; ++i) {
    const uint8_t state = (b & 0x80) ? HIGH : LOW;
    if (spi_speed == 0) {
      WRITE(DOGLCD_SCK, LOW);
      WRITE(DOGLCD_MOSI, state);
      WRITE(DOGLCD_MOSI, state);  // need some setup time
      WRITE(DOGLCD_SCK, HIGH);
    }
    else {
      for (uint_fast8_t j = 0; j < spi_speed + (miso_pin >= 0 ? 0 : 1); ++j)
        WRITE(DOGLCD_SCK, LOW);

      for (uint_fast8_t j = 0; j < spi_speed; ++j)
        WRITE(DOGLCD_MOSI, state);

      for (uint_fast8_t j = 0; j < spi_speed; ++j)
        WRITE(DOGLCD_SCK, HIGH);
    }
    b <<= 1;
    if (miso_pin >= 0 && READ(miso_pin)) b |= 1;
  }
  return b;
}

static void u8g_sw_spi_shift_out(uint8_t val) {
  #if U8G_SPI_USE_MODE_3
    swSpiTransfer_mode_3(val, SPI_speed);
  #else
    swSpiTransfer_mode_0(val, SPI_speed);
  #endif
}

static uint8_t swSpiInit(const uint8_t spi_speed) {
  #if PIN_EXISTS(LCD_RESET)
    SET_OUTPUT(LCD_RESET_PIN);
  #endif
  SET_OUTPUT(DOGLCD_A0);
  OUT_WRITE(DOGLCD_SCK, LOW);
  OUT_WRITE(DOGLCD_MOSI, LOW);
  OUT_WRITE(DOGLCD_CS, HIGH);
  return spi_speed;
}

uint8_t u8g_com_HAL_STM32F1_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
  switch (msg) {
    case U8G_COM_MSG_INIT:
      SPI_speed = swSpiInit(LCD_SPI_SPEED);
      break;

    case U8G_COM_MSG_STOP:
      break;

    case U8G_COM_MSG_RESET:
      #if PIN_EXISTS(LCD_RESET)
        WRITE(LCD_RESET_PIN, arg_val);
      #endif
      break;

    case U8G_COM_MSG_CHIP_SELECT:
      #if U8G_SPI_USE_MODE_3        // This LCD SPI is running mode 3 while SD card is running mode 0
        if (arg_val) {              // SCK idle state needs to be set to the proper idle state before
                                    //  the next chip select goes active
          WRITE(DOGLCD_SCK, HIGH);  // Set SCK to mode 3 idle state before CS goes active
          WRITE(DOGLCD_CS, LOW);
        }
        else {
          WRITE(DOGLCD_CS, HIGH);
          WRITE(DOGLCD_SCK, LOW);   // Set SCK to mode 0 idle state after CS goes inactive
        }
      #else
        WRITE(DOGLCD_CS, !arg_val);
      #endif
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      u8g_sw_spi_shift_out(arg_val);
      break;

    case U8G_COM_MSG_WRITE_SEQ: {
      uint8_t *ptr = (uint8_t *)arg_ptr;
      while (arg_val > 0) {
        u8g_sw_spi_shift_out(*ptr++);
        arg_val--;
      }
    } break;

    case U8G_COM_MSG_WRITE_SEQ_P: {
      uint8_t *ptr = (uint8_t *)arg_ptr;
      while (arg_val > 0) {
        u8g_sw_spi_shift_out(u8g_pgm_read(ptr));
        ptr++;
        arg_val--;
      }
    } break;

    case U8G_COM_MSG_ADDRESS: /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      WRITE(DOGLCD_A0, arg_val);
      break;
  }
  return 1;
}

#endif // HAS_MARLINUI_U8GLIB && FORCE_SOFT_SPI
#endif // STM32F1
