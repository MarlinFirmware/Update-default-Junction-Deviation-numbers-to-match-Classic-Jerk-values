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
 * u8g_dev_tft_320x240_upscale_from_128x64.cpp
 *
 * Universal 8bit Graphics Library
 *
 * Copyright (c) 2011, olikraus@gmail.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
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

#include "../../../inc/MarlinConfig.h"

#if HAS_MARLINUI_U8GLIB && (PIN_EXISTS(FSMC_CS) || HAS_SPI_GRAPHICAL_TFT || HAS_LTDC_GRAPHICAL_TFT)

#include "HAL_LCD_com_defines.h"
#include "../marlinui_DOGM.h"

#include <string.h>

#if ANY(LCD_USE_DMA_FSMC, LCD_USE_DMA_SPI, HAS_LTDC_GRAPHICAL_TFT)
  #define HAS_LCD_IO 1
#endif

#include "../../tft_io/tft_io.h"
TFT_IO tftio;

#define WIDTH  LCD_PIXEL_WIDTH
#define HEIGHT LCD_PIXEL_HEIGHT
#define PAGE_HEIGHT 8

#if ENABLED(TOUCH_SCREEN_CALIBRATION)
  #include "../../tft_io/touch_calibration.h"
  #include "../../marlinui.h"
#endif

#include "../../touch/touch_buttons.h"
#include "../../scaled_tft.h"

#define X_HI (UPSCALE(TFT_PIXEL_OFFSET_X, WIDTH) - 1)
#define Y_HI (UPSCALE(TFT_PIXEL_OFFSET_Y, HEIGHT) - 1)

// RGB565 color picker: https://rgbcolorpicker.com/565
// Hex code to color name: https://www.color-name.com/

#define COLOR_BLACK       0x0000  // #000000
#define COLOR_WHITE       0xFFFF  // #FFFFFF
#define COLOR_SILVER      0xC618  // #C0C0C0
#define COLOR_GREY        0x7BEF  // #808080
#define COLOR_DARKGREY    0x4208  // #404040
#define COLOR_DARKGREY2   0x39E7  // #303030
#define COLOR_DARK        0x0003  // #000019

#define COLOR_RED         0xF800  // #FF0000
#define COLOR_LIME        0x7E00  // #00FF00
#define COLOR_BLUE        0x001F  // #0000FF
#define COLOR_YELLOW      0xFFE0  // #FFFF00
#define COLOR_MAGENTA     0xF81F  // #FF00FF
#define COLOR_CYAN        0x07FF  // #00FFFF

#define COLOR_MAROON      0x7800  // #800000
#define COLOR_GREEN       0x03E0  // #008000
#define COLOR_NAVY        0x000F  // #000080
#define COLOR_OLIVE       0x8400  // #808000
#define COLOR_PURPLE      0x8010  // #800080
#define COLOR_TEAL        0x0410  // #008080

#define COLOR_ORANGE      0xFC00  // #FF7F00

#ifndef TFT_MARLINUI_COLOR
  #define TFT_MARLINUI_COLOR COLOR_WHITE
#endif
#ifndef TFT_MARLINBG_COLOR
  #define TFT_MARLINBG_COLOR COLOR_BLACK
#endif
#ifndef TFT_DISABLED_COLOR
  #define TFT_DISABLED_COLOR COLOR_DARK
#endif
#ifndef TFT_BTCANCEL_COLOR
  #define TFT_BTCANCEL_COLOR COLOR_RED
#endif
#ifndef TFT_BTARROWS_COLOR
  #define TFT_BTARROWS_COLOR COLOR_BLUE
#endif
#ifndef TFT_BTOKMENU_COLOR
  #define TFT_BTOKMENU_COLOR COLOR_RED
#endif

static void setWindow(u8g_t *u8g, u8g_dev_t *dev, uint16_t xMin, uint16_t yMin, uint16_t xMax, uint16_t yMax) {
  tftio.set_window(xMin, yMin, xMax, yMax);
}

#if HAS_TOUCH_BUTTONS

  static const uint8_t buttonD[] = {
    B01111111,B11111111,B11111111,B11111110,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00011000,B00110000,B00000001,
    B10000000,B00001100,B01100000,B00000001,
    B10000000,B00000110,B11000000,B00000001,
    B10000000,B00000011,B10000000,B00000001,
    B10000000,B00000011,B10000000,B00000001,
    B10000000,B00000110,B11000000,B00000001,
    B10000000,B00001100,B01100000,B00000001,
    B10000000,B00011000,B00110000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B01111111,B11111111,B11111111,B11111110,
  };

  #if ENABLED(REVERSE_MENU_DIRECTION)

    static const uint8_t buttonA[] = {
      B01111111,B11111111,B11111111,B11111110,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B11100000,B00000000,B00000001,
      B10000000,B11100000,B00000000,B00000001,
      B10000000,B11100000,B00000000,B00000001,
      B10000000,B11100000,B00000000,B00000001,
      B10000000,B11100000,B00111111,B11100001,
      B10000111,B11111100,B00111111,B11100001,
      B10000011,B11111000,B00000000,B00000001,
      B10000001,B11110000,B00000000,B00000001,
      B10000000,B11100000,B00000000,B00000001,
      B10000000,B01000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B01111111,B11111111,B11111111,B11111110,
    };
    static const uint8_t buttonB[] = {
      B01111111,B11111111,B11111111,B11111110,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B01100000,B00000010,B00000001,
      B10000000,B01100000,B00000111,B00000001,
      B10000000,B01100000,B00001111,B10000001,
      B10000000,B01100000,B00011111,B11000001,
      B10000111,B11111110,B00111111,B11100001,
      B10000111,B11111110,B00000111,B00000001,
      B10000000,B01100000,B00000111,B00000001,
      B10000000,B01100000,B00000111,B00000001,
      B10000000,B01100000,B00000111,B00000001,
      B10000000,B01100000,B00000111,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B01111111,B11111111,B11111111,B11111110,
    };

  #else

    static const uint8_t buttonA[] = {
      B01111111,B11111111,B11111111,B11111110,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B01000000,B00000000,B00000001,
      B10000000,B11100000,B00000000,B00000001,
      B10000001,B11110000,B00000000,B00000001,
      B10000011,B11111000,B00000000,B00000001,
      B10000111,B11111100,B00111111,B11100001,
      B10000000,B11100000,B00111111,B11100001,
      B10000000,B11100000,B00000000,B00000001,
      B10000000,B11100000,B00000000,B00000001,
      B10000000,B11100000,B00000000,B00000001,
      B10000000,B11100000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B01111111,B11111111,B11111111,B11111110,
    };

    static const uint8_t buttonB[] = {
      B01111111,B11111111,B11111111,B11111110,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B01100000,B00000111,B00000001,
      B10000000,B01100000,B00000111,B00000001,
      B10000000,B01100000,B00000111,B00000001,
      B10000000,B01100000,B00000111,B00000001,
      B10000111,B11111110,B00000111,B00000001,
      B10000111,B11111110,B00111111,B11100001,
      B10000000,B01100000,B00011111,B11000001,
      B10000000,B01100000,B00001111,B10000001,
      B10000000,B01100000,B00000111,B00000001,
      B10000000,B01100000,B00000010,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B10000000,B00000000,B00000000,B00000001,
      B01111111,B11111111,B11111111,B11111110,
    };

  #endif

  static const uint8_t buttonC[] = {
    B01111111,B11111111,B11111111,B11111110,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00011100,B00000001,
    B10000000,B00000100,B00011100,B00000001,
    B10000000,B00001100,B00011100,B00000001,
    B10000000,B00011111,B11111100,B00000001,
    B10000000,B00111111,B11111100,B00000001,
    B10000000,B00011111,B11111100,B00000001,
    B10000000,B00001100,B00000000,B00000001,
    B10000000,B00000100,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B10000000,B00000000,B00000000,B00000001,
    B01111111,B11111111,B11111111,B11111110,
  };

  void drawImage(const uint8_t *data, u8g_t *u8g, u8g_dev_t *dev, uint16_t length, uint16_t height, uint16_t color) {
    uint16_t buffer[BUTTON_WIDTH * sq(GRAPHICAL_TFT_UPSCALE)];

    if (length > BUTTON_WIDTH) return;

    for (uint16_t i = 0; i < height; i++) {
      uint16_t k = 0;
      for (uint16_t j = 0; j < length; j++) {
        uint16_t v = TFT_MARLINBG_COLOR;
        if (*(data + (i * (length >> 3) + (j >> 3))) & (0x80 >> (j & 7)))
          v = color;
        else
          v = TFT_MARLINBG_COLOR;
        for (uint8_t n = 0; n < GRAPHICAL_TFT_UPSCALE; ++n) buffer[k++] = v;
      }
      #if HAS_LCD_IO
        for (uint8_t n = 1; n < GRAPHICAL_TFT_UPSCALE; ++n)
          for (uint16_t l = 0; l < UPSCALE0(length); l++)
            buffer[l + n * UPSCALE0(length)] = buffer[l];

        tftio.writeSequence(buffer, length * sq(GRAPHICAL_TFT_UPSCALE));
      #else
        for (uint8_t i = GRAPHICAL_TFT_UPSCALE; i--;)
          u8g_WriteSequence(u8g, dev, k << 1, (uint8_t*)buffer);
      #endif
    }
  }

#endif // HAS_TOUCH_BUTTONS

// Used to fill RGB565 (16bits) background
inline void memset2(const void *ptr, uint16_t fill, size_t cnt) {
  uint16_t *wptr = (uint16_t*)ptr;
  for (size_t i = 0; i < cnt; i += 2) { *wptr = fill; wptr++; }
}

static bool preinit = true;
static uint8_t page;

#if HAS_TOUCH_BUTTONS

  static bool redrawTouchButtons = true;
  static void drawTouchButtons(u8g_t *u8g, u8g_dev_t *dev) {
    if (!redrawTouchButtons) return;
    redrawTouchButtons = false;

    // Bottom buttons
    setWindow(u8g, dev, BUTTOND_X_LO, BUTTON_Y_LO, BUTTOND_X_HI, BUTTON_Y_HI);
    drawImage(buttonD, u8g, dev, BUTTON_DRAW_WIDTH, BUTTON_DRAW_HEIGHT, TFT_BTCANCEL_COLOR);

    setWindow(u8g, dev, BUTTONA_X_LO, BUTTON_Y_LO, BUTTONA_X_HI, BUTTON_Y_HI);
    drawImage(buttonA, u8g, dev, BUTTON_DRAW_WIDTH, BUTTON_DRAW_HEIGHT, TFT_BTARROWS_COLOR);

    setWindow(u8g, dev, BUTTONB_X_LO, BUTTON_Y_LO, BUTTONB_X_HI, BUTTON_Y_HI);
    drawImage(buttonB, u8g, dev, BUTTON_DRAW_WIDTH, BUTTON_DRAW_HEIGHT, TFT_BTARROWS_COLOR);

    setWindow(u8g, dev, BUTTONC_X_LO, BUTTON_Y_LO, BUTTONC_X_HI, BUTTON_Y_HI);
    drawImage(buttonC, u8g, dev, BUTTON_DRAW_WIDTH, BUTTON_DRAW_HEIGHT, TFT_BTOKMENU_COLOR);
  }

#endif // HAS_TOUCH_BUTTONS

static void u8g_upscale_clear_lcd(u8g_t *u8g, u8g_dev_t *dev, uint16_t *buffer) {
  setWindow(u8g, dev, 0, 0, (TFT_WIDTH) - 1, (TFT_HEIGHT) - 1);
  #if HAS_LCD_IO
    UNUSED(buffer);
    tftio.writeMultiple(TFT_MARLINBG_COLOR, (TFT_WIDTH) * (TFT_HEIGHT));
  #else
    memset2(buffer, TFT_MARLINBG_COLOR, (TFT_WIDTH) / 2);
    for (uint16_t i = 0; i < (TFT_HEIGHT) * sq(GRAPHICAL_TFT_UPSCALE); i++)
      u8g_WriteSequence(u8g, dev, (TFT_WIDTH) / 2, (uint8_t *)buffer);
  #endif
}

static uint8_t msgInitCount = 2; // Ignore all messages until 2nd U8G_COM_MSG_INIT

uint8_t u8g_dev_tft_320x240_upscale_from_128x64_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg) {
  u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);

  #if HAS_LCD_IO
    static uint16_t bufferA[WIDTH * sq(GRAPHICAL_TFT_UPSCALE)], bufferB[WIDTH * sq(GRAPHICAL_TFT_UPSCALE)];
    uint16_t *buffer = &bufferA[0];
  #else
    uint16_t buffer[WIDTH * GRAPHICAL_TFT_UPSCALE]; // 16-bit RGB 565 pixel line buffer
  #endif

  switch (msg) {
    case U8G_DEV_MSG_INIT:
      dev->com_fn(u8g, U8G_COM_MSG_INIT, U8G_SPI_CLK_CYCLE_NONE, nullptr);

      if (preinit) {
        preinit = false;
        return u8g_dev_pb8v1_base_fn(u8g, dev, msg, arg);
      }

      if (msgInitCount) return -1;
      tftio.init();
      tftio.initTFT();
      TERN_(TOUCH_SCREEN_CALIBRATION, touch_calibration.calibration_reset());
      u8g_upscale_clear_lcd(u8g, dev, buffer);
      return 0;

    case U8G_DEV_MSG_STOP: preinit = true; break;

    case U8G_DEV_MSG_PAGE_FIRST: {
      page = 0;
      #if HAS_TOUCH_BUTTONS
        #if HAS_DISPLAY_SLEEP
          static bool sleepCleared;
          if (touchBt.isSleeping()) {
            if (!sleepCleared) {
              sleepCleared = true;
              u8g_upscale_clear_lcd(u8g, dev, buffer);
              redrawTouchButtons = true;
            }
            break;
          }
          else
            sleepCleared = false;
        #endif
        drawTouchButtons(u8g, dev);
      #endif
      setWindow(u8g, dev, TFT_PIXEL_OFFSET_X, TFT_PIXEL_OFFSET_Y, X_HI, Y_HI);
    } break;

    case U8G_DEV_MSG_PAGE_NEXT:
      #if HAS_TOUCH_BUTTONS && HAS_DISPLAY_SLEEP
        if (touchBt.isSleeping()) break;
      #endif
      if (++page > (HEIGHT / PAGE_HEIGHT)) return 1;

      for (uint8_t y = 0; y < PAGE_HEIGHT; ++y) {
        uint32_t k = 0;
        TERN_(HAS_LCD_IO, buffer = (y & 1) ? bufferB : bufferA);
        for (uint16_t i = 0; i < (uint32_t)pb->width; i++) {
          const uint8_t b = *(((uint8_t *)pb->buf) + i);
          const uint16_t c = TEST(b, y) ? TFT_MARLINUI_COLOR : TFT_MARLINBG_COLOR;
          for (uint8_t n = 0; n < GRAPHICAL_TFT_UPSCALE; ++n) buffer[k++] = c;
        }
        #if HAS_LCD_IO
          for (uint8_t n = 1; n < GRAPHICAL_TFT_UPSCALE; ++n)
            for (uint16_t l = 0; l < UPSCALE0(WIDTH); l++)
              buffer[l + n * UPSCALE0(WIDTH)] = buffer[l];

          tftio.writeSequence(buffer, COUNT(bufferA));
        #else
          uint8_t *bufptr = (uint8_t*) buffer;
          for (uint8_t i = GRAPHICAL_TFT_UPSCALE; i--;) {
            for (uint8_t n = 0; n < GRAPHICAL_TFT_UPSCALE * 2; ++n) {
              u8g_WriteSequence(u8g, dev, WIDTH, &bufptr[WIDTH * n]);
            }
          }
        #endif
      }
      break;

    case U8G_DEV_MSG_SLEEP_ON:
      // Enter Sleep Mode (10h)
      return 1;
    case U8G_DEV_MSG_SLEEP_OFF:
      // Sleep Out (11h)
      return 1;
  }
  return u8g_dev_pb8v1_base_fn(u8g, dev, msg, arg);
}

uint8_t u8g_com_hal_tft_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
  if (msgInitCount) {
    if (msg == U8G_COM_MSG_INIT) msgInitCount--;
    if (msgInitCount) return -1;
  }

  static uint8_t isCommand;

  switch (msg) {
    case U8G_COM_MSG_STOP: break;
    case U8G_COM_MSG_INIT:
      isCommand = 0;
      break;

    case U8G_COM_MSG_ADDRESS: // define cmd (arg_val = 0) or data mode (arg_val = 1)
      isCommand = arg_val == 0 ? 1 : 0;
      break;

    case U8G_COM_MSG_RESET:
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      tftio.dataTransferBegin(DATASIZE_8BIT);
      if (isCommand)
        tftio.writeReg(arg_val);
      else
        tftio.writeData((uint16_t)arg_val);
      tftio.dataTransferEnd();
      break;

    case U8G_COM_MSG_WRITE_SEQ:
      tftio.dataTransferBegin(DATASIZE_16BIT);
      for (uint8_t i = 0; i < arg_val; i += 2)
        tftio.writeData(*(uint16_t *)(((uintptr_t)arg_ptr) + i));
      tftio.dataTransferEnd();
      break;

  }
  return 1;
}

U8G_PB_DEV(u8g_dev_tft_320x240_upscale_from_128x64, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_tft_320x240_upscale_from_128x64_fn, U8G_COM_HAL_TFT_FN);

#if ENABLED(TOUCH_SCREEN_CALIBRATION)

  static void drawCross(uint16_t x, uint16_t y, uint16_t color) {
    tftio.set_window(x - 15, y, x + 15, y);
    tftio.writeMultiple(color, 31);
    tftio.set_window(x, y - 15, x, y + 15);
    tftio.writeMultiple(color, 31);
  }

  void MarlinUI::touch_calibration_screen() {
    uint16_t x, y;
    calibrationState stage = touch_calibration.get_calibration_state();

    if (stage == CALIBRATION_NONE) {
      // start and clear screen
      defer_status_screen(true);
      stage = touch_calibration.calibration_start();
      tftio.set_window(0, 0, (TFT_WIDTH) - 1, (TFT_HEIGHT) - 1);
      tftio.writeMultiple(TFT_MARLINBG_COLOR, uint32_t(TFT_WIDTH) * (TFT_HEIGHT));
    }
    else {
      // clear last cross
      x = touch_calibration.calibration_points[_MIN(stage - 1, CALIBRATION_BOTTOM_LEFT)].x;
      y = touch_calibration.calibration_points[_MIN(stage - 1, CALIBRATION_BOTTOM_LEFT)].y;
      drawCross(x, y, TFT_MARLINBG_COLOR);
    }

    FSTR_P str = nullptr;
    if (stage < CALIBRATION_SUCCESS) {
      // handle current state
      switch (stage) {
        case CALIBRATION_TOP_LEFT:     str = GET_TEXT_F(MSG_TOP_LEFT);     break;
        case CALIBRATION_TOP_RIGHT:    str = GET_TEXT_F(MSG_TOP_RIGHT);    break;
        case CALIBRATION_BOTTOM_RIGHT: str = GET_TEXT_F(MSG_BOTTOM_RIGHT); break;
        case CALIBRATION_BOTTOM_LEFT:  str = GET_TEXT_F(MSG_BOTTOM_LEFT);  break;
        default: break;
      }

      x = touch_calibration.calibration_points[stage].x;
      y = touch_calibration.calibration_points[stage].y;
      drawCross(x, y, TFT_MARLINUI_COLOR);
    }
    else {
      // end calibration
      str = stage == CALIBRATION_SUCCESS ? GET_TEXT_F(MSG_CALIBRATION_COMPLETED) : GET_TEXT_F(MSG_CALIBRATION_FAILED);
      defer_status_screen(false);
      touch_calibration.calibration_end();
      TERN_(HAS_TOUCH_BUTTONS, redrawTouchButtons = true);
    }

    // draw current message
    tftio.set_window(TFT_PIXEL_OFFSET_X, TFT_PIXEL_OFFSET_Y, X_HI, Y_HI);
    do {
      set_font(FONT_MENU);
      lcd_put_u8str(0, 7, GET_TEXT_F(MSG_TOUCH_CALIBRATION));
      lcd_put_u8str(0, LCD_PIXEL_HEIGHT / 2, str);
    } while (u8g.nextPage());
    drawing_screen = false;
    safe_delay(250);
    if (stage == CALIBRATION_SUCCESS) {
      safe_delay(500);
      ui.goto_previous_screen();
    }
  }

#endif // TOUCH_SCREEN_CALIBRATION

#endif // HAS_MARLINUI_U8GLIB && (FSMC_CS_PIN || HAS_SPI_GRAPHICAL_TFT || HAS_LTDC_GRAPHICAL_TFT)
