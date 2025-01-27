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
import argparse, textwrap, os, zlib

def deflate(data):
  return zlib.compress(data)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Converts a file into a packed C array for use as data')
  parser.add_argument("input")
  parser.add_argument("-d", "--deflate", action="store_true", help="Packs the data using the deflate algorithm")
  args = parser.parse_args()

  varname = os.path.splitext(os.path.basename(args.input))[0]

  with open(args.input, "rb") as in_file:
    data = in_file.read()
  if args.deflate:
    data = deflate(data)
  data = bytearray(data)
  data = list(map(lambda a: "0x" + format(a, '02x'), data))
  nElements = len(data)
  data = ', '.join(data)
  data = textwrap.fill(data, 75, initial_indent = '  ', subsequent_indent = '  ')

  print("const unsigned char " + varname + "[" + format(nElements) + "] PROGMEM = {")
  print(data)
  print("};")
