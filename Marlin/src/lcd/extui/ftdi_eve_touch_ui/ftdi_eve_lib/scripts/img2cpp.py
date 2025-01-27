#!/usr/bin/python

# Written By Marcio Teixeira 2021 - SynDaver Labs, Inc.
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
import argparse, textwrap, os, sys, zlib

class WriteSource:
  def __init__(self, mode):
    self.values      = []
    self.mode        = mode
    self.offset      = 8
    self.byte        = 0

  def finish_byte(self):
    if self.offset != 8:
      self.values.append(self.byte)
      self.offset = 8
      self.byte   = 0

  def add_bits_to_byte(self, value, size = 1):
    self.offset -= size
    self.byte = self.byte | value << self.offset
    if self.offset == 0:
      self.finish_byte()

  def append_rgb565(self, color):
    value = ((color[0] & 0xF8) << 8) + ((color[1] & 0xFC) << 3) + ((color[2] & 0xF8) >> 3)
    self.values.append((value & 0x00FF) >> 0)
    self.values.append((value & 0xFF00) >> 8)

  def append_rgb332(self, color):
    value = (color[0] & 0xE0) + ((color[1] & 0xE0) >> 3) + ((color[2] & 0xC0) >> 6)
    self.values.append(value)

  def append_grayscale(self, color, bits):
    luminance = int(0.2126 * color[0] + 0.7152 * color[1] + 0.0722 * color[2])
    self.add_bits_to_byte(luminance >> (8 - bits), bits)

  def deflate(self, data):
     return zlib.compress(data)

  def add_pixel(self, color):
    if self.mode == "l1":
      self.append_grayscale(color, 1)
    elif self.mode == "l2":
      self.append_grayscale(color, 2)
    elif self.mode == "l4":
      self.append_grayscale(color, 4)
    elif self.mode == "l8":
      self.append_grayscale(color, 8)
    elif self.mode == "rgb565":
      self.append_rgb565(color)
    elif self.mode == "rgb332":
      self.append_rgb332(color)

  def end_row(self, y):
    if self.mode in ["l1", "l2", "l3"]:
       self.finish_byte()

  def write(self, varname, deflate):
    print("Length of uncompressed data: ", len(self.values), file=sys.stderr)
    data = bytes(bytearray(self.values))
    if deflate:
      data = self.deflate(data)
      print("Length of data after compression: ", len(data), file=sys.stderr)
    data = bytearray(data)
    data = list(map(lambda a: "0x" + format(a, '02x'), data))
    nElements = len(data)
    data = ', '.join(data)
    data = textwrap.fill(data, 75, initial_indent = '  ', subsequent_indent = '  ')

    print("const unsigned char " + varname + "[" + format(nElements) + "] PROGMEM = {")
    print(data)
    print("};")

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Converts a bitmap into a C array')
  parser.add_argument("input")
  parser.add_argument("-d", "--deflate", action="store_true", help="Packs the data using the deflate algorithm")
  parser.add_argument("-m", "--mode", default="l1", help="Mode, can be l1, l2, l4, l8, rgb332 or rgb565")
  args = parser.parse_args()

  varname = os.path.splitext(os.path.basename(args.input))[0]

  writer = WriteSource(args.mode)

  img = Image.open(args.input)
  print("Image height: ", img.height, file=sys.stderr)
  print("Image width:  ", img.width,  file=sys.stderr)
  for y in range(img.height):
    for x in range(img.width):
      writer.add_pixel(img.getpixel((x,y)))
    writer.end_row(y)
  writer.write(varname, args.deflate)
