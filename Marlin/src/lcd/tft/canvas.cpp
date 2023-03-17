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

#include "../../inc/MarlinConfig.h"

#if HAS_GRAPHICAL_TFT

#include "canvas.h"

uint16_t Canvas::width, Canvas::height;
uint16_t Canvas::startLine, Canvas::endLine;
uint16_t Canvas::background_color;
uint16_t *Canvas::buffer = TFT::buffer;

void Canvas::instantiate(uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
  Canvas::width = width;
  Canvas::height = height;
  startLine = 0;
  endLine = 0;

  // The TFT handles DMA within the given canvas rectangle
  // so whatever is drawn will be offset on the screen by x,y.
  tft.set_window(x, y, x + width - 1, y + height - 1);
}

void Canvas::next() {
  startLine = endLine;
  endLine = (TFT_BUFFER_WORDS) < width * (height - startLine) ? startLine + (TFT_BUFFER_WORDS) / width : height;
}

bool Canvas::toScreen() {
  tft.write_sequence(buffer, width * (endLine - startLine));
  return endLine == height;
}

void Canvas::setBackground(uint16_t color) {
  /* TODO: test and optimize performance */
  /*
  uint32_t count = (endLine - startLine) * width;
  uint16_t *pixel = buffer;
  while (count--)
    *pixel++ = color;
  */
  const uint32_t two_pixels = (((uint32_t )color) << 16) | color;
  uint32_t count = ((endLine - startLine) * width + 1) >> 1;
  uint32_t *pointer = (uint32_t *)buffer;
  while (count--) *pointer++ = two_pixels;
  background_color = color;
}

extern uint16_t gradient(uint16_t colorA, uint16_t colorB, uint16_t factor);

void Canvas::addText(uint16_t x, uint16_t y, uint16_t color, uint16_t *string, uint16_t maxWidth) {
  if (endLine < y || startLine > y + getFontHeight()) return;

  if (maxWidth == 0) maxWidth = width - x;

  uint16_t colors[16];
  uint16_t stringWidth = 0;
  if (getFontType() == FONT_MARLIN_GLYPHS_2BPP) {
    for (uint_fast8_t i = 0; i < 3; i++) {
      colors[i] = gradient(ENDIAN_COLOR(color), ENDIAN_COLOR(background_color), ((i+1) << 8) / 3);
      colors[i] = ENDIAN_COLOR(colors[i]);
    }
  }
  for (uint16_t i = 0 ; *(string + i) ; i++) {
    glyph_t *pGlyph = glyph(string + i);
    if (stringWidth + pGlyph->bbxWidth > maxWidth) break;
    switch (getFontType()) {
      case FONT_MARLIN_GLYPHS_1BPP:
        addImage(x + stringWidth + pGlyph->bbxOffsetX, y + getFontAscent() - pGlyph->bbxHeight - pGlyph->bbxOffsetY, pGlyph->bbxWidth, pGlyph->bbxHeight, GREYSCALE1, ((uint8_t *)pGlyph) + sizeof(glyph_t), &color);
        break;
      case FONT_MARLIN_GLYPHS_2BPP:
        addImage(x + stringWidth + pGlyph->bbxOffsetX, y + getFontAscent() - pGlyph->bbxHeight - pGlyph->bbxOffsetY, pGlyph->bbxWidth, pGlyph->bbxHeight, GREYSCALE2, ((uint8_t *)pGlyph) + sizeof(glyph_t), colors);
        break;
    }
    stringWidth += pGlyph->dWidth;
  }
}

void Canvas::addImage(int16_t x, int16_t y, MarlinImage image, uint16_t *colors) {
  uint16_t *data = (uint16_t *)images[image].data;
  if (!data) return;

  const uint16_t image_width = images[image].width,
                image_height = images[image].height;
  colorMode_t color_mode = images[image].colorMode;

  if (color_mode == HIGHCOLOR) {
    // HIGHCOLOR - 16 bits per pixel
    int16_t line = y;
    for (int16_t i = 0; i < image_height; i++, line++) {
      if (WITHIN(line, startLine, endLine - 1)) {
        uint16_t *pixel = buffer + x + (line - startLine) * width;
        uint16_t cx = x;
        for (int16_t j = 0; j < image_width; j++, cx++) {
          if (WITHIN(cx, 0, width - 1)) {
            uint16_t color = ENDIAN_COLOR(*data);
            if (color == 0x0001) color = COLOR_BACKGROUND;
            *pixel = color;
          }
          pixel++;
          data++;
        }
      }
      else
        data += image_width;
    }
    return;
  }

  #if ENABLED(COMPACT_MARLIN_BOOT_LOGO)

    static struct {
      bool has_rle_state = false;
      int16_t dstx, dsty, srcx, srcy;
      uint32_t rle_offset;
    } rle_state;

    // RLE16 HIGHCOLOR - 16 bits per pixel
    if (color_mode == RLE16) {
      uint8_t *bytedata = (uint8_t *)images[image].data;
      if (!bytedata) return;

      // Loop through the image data advancing the row and column as needed
      int16_t srcy = 0, srcx = 0,                   // Image data line / column index
              dsty = y, dstx = x;                   // Destination line / column index

      uint16_t color = 0;                           // Persist the last fetched color value
      if (rle_state.has_rle_state) {                // do we have RLE position data?
        rle_state.has_rle_state = false;            // invalidate stored RLE state
        dstx = rle_state.dstx;                      // restore required states
        dsty = rle_state.dsty;
        srcx = rle_state.srcx;
        srcy = rle_state.srcy;
        bytedata = (uint8_t *)images[image].data + rle_state.rle_offset;  // Restart decode from here instead of the start of data
      }

      bool done = false;
      while (!done) {
        if (dsty >= endLine - 1 || srcy >= image_height - 1) { // Store state?
          rle_state.dstx = dstx;                    // Save required states
          rle_state.dsty = dsty;
          rle_state.srcx = srcx;
          rle_state.srcy = srcy;
          rle_state.rle_offset = bytedata - (uint8_t *)images[image].data;; // Keep these for skipping full RLE decode on future iteratons
        }

        uint8_t count = *bytedata++;                // Get the count byte
        const bool uniq = bool(count & 0x80);       // >= 128 is a distinct run; < 128 is a repeat run
        count = (count & 0x7F) + 1;                 // Actual count is 7-bit plus 1

        bool getcol = true;                         // Get at least one color word
        while (count--) {                           // Emit 'count' pixels

          if (getcol) {
            getcol = uniq;                          // Keep getting colors if not RLE
            const uint16_t msb = *bytedata++,       // Color most-significant bits
                           lsb = *bytedata++;       // Color least-significant bits
            color = ENDIAN_COLOR((msb << 8) | lsb); // Color with proper endianness
            if (color == 0x0001) color = COLOR_BACKGROUND; // 0x0001 is "transparent"
          }

          if (WITHIN(dsty, startLine, endLine - 1)) { // Dest pixel Y within the segment?
            if (WITHIN(dstx, 0, width - 1)) {       // Dest pixel X within the canvas?
              uint16_t * const pixel = buffer + dstx + (dsty - startLine) * width;
              *pixel = color;                       // Store the color in the pixel
            }
          }

          ++srcx; ++dstx;                           // Advance the pixel column
          if (srcx >= image_width) {                // Past the right edge of the source image?
            ++srcy; ++dsty;                         // Advance to the next line
            srcx = 0; dstx = x;                     // May be shifted within the canvas, but usually not
            if (dsty >= endLine || srcy >= image_height) { // Done with the segment or the image?
              done = true;                          // Set a flag to end the loop...
              rle_state.has_rle_state = true;       // RLE state is stored
              break;                                // ...and break out of while(count--)
            }
          }
        }
      }
      return;
    }
  #endif // COMPACT_MARLIN_BOOT_LOGO

  addImage(x, y, image_width, image_height, color_mode, (uint8_t *)data, colors);
}

void Canvas::addImage(int16_t x, int16_t y, uint8_t image_width, uint8_t image_height, colorMode_t color_mode, uint8_t *data, uint16_t *colors) {
  uint8_t bitsPerPixel;
  switch (color_mode) {
    case GREYSCALE1: bitsPerPixel = 1; break;
    case GREYSCALE2: bitsPerPixel = 2; break;
    case GREYSCALE4: bitsPerPixel = 4; break;
    default: return;
  }

  uint8_t mask = 0xFF >> (8 - bitsPerPixel),
          pixelsPerByte = 8 / bitsPerPixel;

  colors--;

  for (int16_t i = 0; i < image_height; i++) {
    const int16_t line = y + i;
    if (WITHIN(line, startLine, endLine - 1)) {
      uint16_t *pixel = buffer + x + (line - startLine) * width;
      uint8_t offset = 8 - bitsPerPixel;
      for (int16_t j = 0; j < image_width; j++) {
        if (offset > 8) {
          data++;
          offset = 8 - bitsPerPixel;
        }
        if (WITHIN(x + j, 0, width - 1)) {
          const uint8_t color = ((*data) >> offset) & mask;
          if (color) *pixel = *(colors + color);
        }
        pixel++;
        offset -= bitsPerPixel;
      }
      data++;
    }
    else
      data += (image_width + pixelsPerByte - 1) / pixelsPerByte;
  }
}

void Canvas::addRect(uint16_t x, uint16_t y, uint16_t rectangleWidth, uint16_t rectangleHeight, uint16_t color) {
  if (endLine < y || startLine > y + rectangleHeight) return;

  for (uint16_t i = 0; i < rectangleHeight; i++) {
    const uint16_t line = y + i;
    if (WITHIN(line, startLine, endLine - 1)) {
      uint16_t *pixel = buffer + x + (line - startLine) * width;
      if (i == 0 || i == rectangleHeight - 1) {
        for (uint16_t j = 0; j < rectangleWidth; j++) *pixel++ = color;
      }
      else {
        *pixel = color;
        pixel += rectangleWidth - 1;
        *pixel = color;
      }
    }
  }
}

void Canvas::addBar(uint16_t x, uint16_t y, uint16_t barWidth, uint16_t barHeight, uint16_t color) {
  if (endLine < y || startLine > y + barHeight) return;

  for (uint16_t i = 0; i < barHeight; i++) {
    const uint16_t line = y + i;
    if (WITHIN(line, startLine, endLine - 1)) {
      uint16_t *pixel = buffer + x + (line - startLine) * width;
      for (uint16_t j = 0; j < barWidth; j++) *pixel++ = color;
    }
  }
}

Canvas tftCanvas;

#endif // HAS_GRAPHICAL_TFT
