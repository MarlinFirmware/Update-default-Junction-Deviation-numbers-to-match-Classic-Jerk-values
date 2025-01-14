#!/usr/bin/env python3
#
# Marlin 3D Printer Firmware
# Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
#
# Based on Sprinter and grbl.
# Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

# Generate Marlin TFT Images from bitmaps/PNG/JPG

import sys, struct
from PIL import Image

def image2bin(image, output_file, transparency):
    w, h = image.size[0], image.size[1]
    print(f"Converting image with dimensions {w}x{h}...")
    if output_file.endswith(('.c', '.cpp')):
        is_cpp = True
        row_sp, item_sp = ("  ", "") if w >= 480 else (" ", " ")
        row_end, data_end = "\n", "};\n"
        f = open(output_file, 'wt')
        f.write("const uint16_t image[%d] = {\n" % (h * w))
    else:
        is_cpp = False
        row_sp, row_end, data_end = b"", b"", b""
        f = open(output_file, 'wb')
    tcolor, got_tcolor = 0, False
    pixs = image.load()
    for y in range(h):
        f.write(row_sp)
        for x in range(w):
            R = pixs[x, y][0] >> 3
            G = pixs[x, y][1] >> 2
            B = pixs[x, y][2] >> 3
            rgb = (R << 11) | (G << 5) | B
            if transparency:
                if not got_tcolor:
                    got_tcolor = True
                    tcolor = rgb            # First pixel color is transparent
                if rgb == tcolor: rgb = 1   # "color 1" is transparent
            if is_cpp:
                strHex = item_sp + "0x{0:04X},".format(rgb)
                f.write(strHex)
            else:
                f.write(struct.pack("B", (rgb & 0xFF)))
                f.write(struct.pack("B", (rgb >> 8) & 0xFF))
        f.write(row_end)
    f.write(data_end)
    f.close()

if len(sys.argv) <= 2:
    print("Utility to export a image in Marlin TFT friendly format.")
    print("It will dump a raw bin RGB565 image or create a CPP file with an array of 16 bit image pixels.")
    print("Usage: gen-tft-image.py INPUT_IMAGE.(png|bmp|jpg) OUTPUT_FILE.(cpp|bin) [--transparency]")
    print("Authors: rhapsodyv, thinkyhead")
    exit(1)

transparency = len(sys.argv) > 3 and sys.argv[3] == "--transparency"
output_img = sys.argv[2]
img = Image.open(sys.argv[1])
image2bin(img, output_img, transparency)
