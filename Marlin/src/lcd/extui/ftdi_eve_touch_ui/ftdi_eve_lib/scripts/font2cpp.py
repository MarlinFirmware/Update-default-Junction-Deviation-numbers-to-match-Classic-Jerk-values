#!/usr/bin/python

# Written By Marcio Teixeira 2019 - Aleph Objects, Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# To view a copy of the GNU General Public License, go to the following
# location: <https://www.gnu.org/licenses/>.

from __future__ import print_function
from PIL import Image
import argparse, textwrap

def pack_rle(data):
  """Use run-length encoding to pack the bytes"""
  rle = []
  value = data[0]
  count = 0
  for i in data:
    if i != value or count == 255:
      rle.append(count)
      rle.append(value)
      value = i
      count = 1
    else:
      count += 1
  rle.append(count)
  rle.append(value)
  return rle

class WriteSource:
  def __init__(self, lines_in_blocks):
    self.blocks      = []
    self.values      = []
    self.block_size  = lines_in_blocks
    self.rows        = 0

  def add_pixel(self, value):
    self.values.append(value)

  def convert_to_4bpp(self, data, chunk_size = 0):
    # Invert the image
    data = list(map(lambda i: 255 - i, data))
    # Quanitize 8-bit values into 4-bits
    data = list(map(lambda i: i >> 4, data))
    # Make sure there is an even number of elements
    if (len(data) & 1) == 1:
      data.append(0)
    # Combine each two adjacent values into one
    i = iter(data)
    data = list(map(lambda a, b: a << 4 | b, i ,i))
    # Pack the data
    data = pack_rle(data)
    # Convert values into hex strings
    return list(map(lambda a: "0x" + format(a, '02x'), data))

  def end_row(self, y):
    # Pad each row into even number of values
    if len(self.values) & 1:
      self.values.append(0)

    self.rows += 1
    if self.block_size and (self.rows % self.block_size) == 0:
      self.blocks.append(self.values)
      self.values = []

  def write(self):
    if len(self.values):
      self.blocks.append(self.values)

    block_strs = []
    for b in self.blocks:
      data = self.convert_to_4bpp(b)
      data = ', '.join(data)
      data = textwrap.fill(data, 75, initial_indent = '  ', subsequent_indent = '  ')
      block_strs.append(data)

    print("const unsigned char font[] PROGMEM = {")
    for i, b in enumerate(block_strs):
      if i:
        print(',')
      print('\n  /* {} */'.format(i))
      print(b, end='')
    print("\n};")

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Converts a grayscale bitmap into a 16-level RLE packed C array for use as font data')
  parser.add_argument("input")
  parser.add_argument('--char_height', help='Adds a separator every so many lines', type=int)
  args = parser.parse_args()

  writer = WriteSource(args.char_height)

  img = Image.open(args.input).convert('L')
  for y in range(img.height):
    for x in range(img.width):
      writer.add_pixel(img.getpixel((x,y)))
    writer.end_row(y)
  writer.write()
