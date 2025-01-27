#!/usr/bin/python3

# Written By Marcio Teixeira 2018 - Aleph Objects, Inc.
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
import argparse, re, sys

from html.parser import HTMLParser

usage = '''
This program extracts line segments from a SVG file and writes
them as coordinates in a C array. The x and y values will be
scaled from 0x0000 to 0xFFFE. 0xFFFF is used as path separator.

This program can only interpret straight segments, not curves.
It also cannot handle SVG transform attributes. To convert an
SVG file into the proper format, use the following procedure:

  - Load SVG file into Inkscape
  - Convert all Objects to Paths (Path -> Object to Path)
  - Convert all Strokes to Paths (Path -> Stroke to Path)
  - Combine all paths into one (Path -> Combine) [1]
  - Convert all curves into short line segments
            (Extensions -> Modify Paths -> Flatten Beziers...)
  - Save as new SVG
  - Convert into a header file using this utility
  - To give paths individual names, break apart paths and
    use the XML Editor to set the "id" attributes.

[1] Combining paths is necessary to remove transforms. You
could also use inkscape-applytransforms Inkscape extension.

'''

header = '''
/****************************************************************************
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

/**
 * This file was auto-generated using "svg2cpp.py"
 *
 * The encoding consists of x,y pairs with the min and max scaled to
 * 0x0000 and 0xFFFE. A single 0xFFFF in the data stream indicates the
 * start of a new closed path.
 */

#pragma once
'''

class ComputeBoundingBox:
  def reset(self):
    self.x_min    = float(" inf")
    self.y_min    = float(" inf")
    self.x_max    = float("-inf")
    self.y_max    = float("-inf")
    self.n_points = 0
    self.n_paths  = 0

  def command(self, type, x, y):
    self.x_min = min(self.x_min, x)
    self.x_max = max(self.x_max, x)
    self.y_min = min(self.y_min, y)
    self.y_max = max(self.y_max, y)

    if type == "M":
      self.n_paths += 1
    self.n_points += 1

  def scale(self, x, y):
    x -= self.x_min
    y -= self.y_min
    x /= self.x_max - self.x_min
    y /= self.y_max - self.y_min
    #y = 1 - y # Flip upside down
    return (x, y)

  def path_finished(self, id):
    pass

  def write(self):
    print("constexpr float x_min = %f;" % self.x_min)
    print("constexpr float x_max = %f;" % self.x_max)
    print("constexpr float y_min = %f;" % self.y_min)
    print("constexpr float y_max = %f;" % self.y_max)
    print()

  def from_svg_view_box(self, viewbox):
    m = re.search('([0-9-.]+) ([0-9-.]+) ([0-9-.]+) ([0-9-.]+)', viewbox)
    if m:
      self.x_min = float(m[1])
      self.y_min = float(m[2])
      self.x_max = float(m[3])
      self.y_max = float(m[4])
      return True
    return False

class WriteDataStructure:
  def __init__(self, bounding_box):
    self.bounds = bounding_box

  def reset(self, ):
    self.hex_words = []

  def push(self, value):
    self.hex_words.append("0x%04X" % (0xFFFF & int(value)))

  def command(self, type, x, y):
    if type == "M":
      self.push(0xFFFF)
    x, y = self.bounds.scale(x,y)
    self.push(x * 0xFFFE)
    self.push(y * 0xFFFE)

  def path_finished(self, id):
    if self.hex_words and self.hex_words[0] == "0xFFFF":
      self.hex_words.pop(0)
    print("const PROGMEM uint16_t", id + "[] = {" + ", ".join (self.hex_words) + "};")
    self.hex_words = []

class SVGParser(HTMLParser):
  def __init__(self, args):
    super().__init__()
    self.args   = args
    self.tags   = []
    self.groups = []
    self.op     = None
    self.restart()

  def set_consumer(self, op):
    self.op = op
    if self.op:
      self.op.reset()

  def restart(self):
    self.last_x = 0
    self.last_y = 0
    self.initial_x = 0
    self.initial_y = 0

  def process_svg_path_L_or_M(self, cmd, x, y):
    if self.op:
      self.op.command(cmd, x, y)
    self.last_x = x
    self.last_y = y
    if cmd == "M":
      self.initial_x = x
      self.initial_y = y

  def process_svg_path_data_cmd(self, id, cmd, a, b):
    """Converts the various types of moves into L or M commands
    and dispatches to process_svg_path_L_or_M for further processing."""
    if cmd == "Z" or cmd == "z":
      self.process_svg_path_L_or_M("L", self.initial_x, self.initial_y)
    elif cmd == "H":
      self.process_svg_path_L_or_M("L", a, self.last_y)
    elif cmd == "V":
      self.process_svg_path_L_or_M("L", self.last_x, a)
    elif cmd == "h":
      self.process_svg_path_L_or_M("L", self.last_x + a, self.last_y)
    elif cmd == "v":
      self.process_svg_path_L_or_M("L", self.last_x, self.last_y + a)
    elif cmd == "L":
      self.process_svg_path_L_or_M("L", a, b)
    elif cmd == "l":
      self.process_svg_path_L_or_M("L", self.last_x + a, self.last_y + b)
    elif cmd == "M":
      self.process_svg_path_L_or_M("M", a, b)
    elif cmd == "m":
      self.process_svg_path_L_or_M("M", self.last_x + a, self.last_y + b)
    else:
      print("Unsupported path data command:", cmd, "in path", id, "\n", file=sys.stderr)
      quit()

  def eat_token(self, regex):
    """Looks for a token at the start of self.d.
       If found, the token is removed."""
    self.m = re.match(regex,self.d)
    if self.m:
      self.d = self.d[self.m.end():]
    return self.m

  def process_svg_path_data(self, id, d):
    """Breaks up the "d" attribute into individual commands
       and calls "process_svg_path_data_cmd" for each"""

    self.d = d
    while (self.d):
      if self.eat_token('\s+'):
        pass # Just eat the spaces

      elif self.eat_token('([LMHVZlmhvz])'):
        cmd = self.m[1]
        # The following commands take no arguments
        if cmd == "Z" or cmd == "z":
          self.process_svg_path_data_cmd(id, cmd, 0, 0)

      elif self.eat_token('([CScsQqTtAa])'):
        print("Unsupported path data command:", self.m[1], "in path", id, "\n", file=sys.stderr)
        quit()

      elif self.eat_token('([ ,]*[-0-9e.]+)+'):
        # Process list of coordinates following command
        coords = re.split('[ ,]+', self.m[0])
        # The following commands take two arguments
        if cmd == "L" or cmd == "l":
          while coords:
            self.process_svg_path_data_cmd(id, cmd, float(coords.pop(0)), float(coords.pop(0)))
        elif cmd == "M":
          while coords:
            self.process_svg_path_data_cmd(id, cmd, float(coords.pop(0)), float(coords.pop(0)))
            # If a MOVETO has multiple points, the subsequent ones are assumed to be LINETO
            cmd = "L"
        elif cmd == "m":
          while coords:
            self.process_svg_path_data_cmd(id, cmd, float(coords.pop(0)), float(coords.pop(0)))
            # If a MOVETO has multiple points, the subsequent ones are assumed to be LINETO
            cmd = "l"
        # Assume all other commands are single argument
        else:
          while coords:
            self.process_svg_path_data_cmd(id, cmd, float(coords.pop(0)), 0)
      else:
        print("Syntax error:", d, "in path", id, "\n", file=sys.stderr)
        quit()

  def find_attr(attrs, what):
    for attr, value in attrs:
      if attr == what:
         return value

  def layer_matches(self):
    """ Are we in the correct layer?"""
    if not self.args.layer:
        return True
    for l in self.groups:
      if l and l.find(self.args.layer) != -1:
        return True
    return False

  def handle_starttag(self, tag, attrs):
    self.tags.append(tag)
    if tag == 'svg':
      self.viewbox = SVGParser.find_attr(attrs, 'viewbox')
    if tag == 'g':
      label = SVGParser.find_attr(attrs, 'inkscape:label')
      self.groups.append(label)
      if label and self.layer_matches():
        print("Reading layer:", label, file=sys.stderr)
    if tag == 'path' and self.layer_matches():
      id = SVGParser.find_attr(attrs, 'id')
      transform = SVGParser.find_attr(attrs, 'transform')
      if transform:
        print("Found transform in path", id, "! Cannot process file!", file=sys.stderr)
        quit()
      d = SVGParser.find_attr(attrs, 'd')
      if d:
        self.process_svg_path_data(id, d)
        if self.op:
          self.op.path_finished(id)
        self.restart()

  def handle_endtag(self, tag):
    if tag == 'g':
      self.groups.pop()
    if tag != self.tags.pop():
      print("Error popping tag off list")

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("filename")
  parser.add_argument('--layer', help='only include layers which have this string in their names')
  args = parser.parse_args()

  f = open(args.filename, "r")
  data = f.read()

  # First pass to grab viewbox
  p = SVGParser(args)
  p.feed(data)

  b = ComputeBoundingBox()
  if not b.from_svg_view_box(p.viewbox):
    # Can't find the view box, so use the bounding box of the elements themselves.
    p = SVGParser(args)
    p.set_consumer(b)
    p.feed(data)
    b.write()

  # Last pass to process paths
  w = WriteDataStructure(b)
  p = SVGParser(args)
  p.set_consumer(w)
  p.feed(data)
