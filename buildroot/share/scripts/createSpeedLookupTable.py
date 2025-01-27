#!/usr/bin/env python

from __future__ import print_function, division

""" Generate the stepper delay lookup table for Marlin firmware. """

import argparse

__author__ = "Ben Gamari <bgamari@gmail.com>"
__copyright__ = "Copyright 2012, Ben Gamari"
__license__ = "GPL"

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('-f', '--cpu-freq', type=int, default=16, help='CPU clockrate in MHz (default=16)')
parser.add_argument('-d', '--divider', type=int, default=8, help='Timer/counter pre-scale divider (default=8)')
args = parser.parse_args()

cpu_freq = args.cpu_freq * 1000000
timer_freq = cpu_freq / args.divider

print("#pragma once")
print()
print("#if F_CPU == %d" % cpu_freq)
print()

print("  const struct { uint16_t base; uint8_t gain; } speed_lookuptable_fast[256] PROGMEM = {")
a = [0 for i in range(8)] + [ int(0.5 + float(timer_freq) / (i*256)) for i in range(8, 256) ]
b = [0 for i in range(8)] + [ a[i] - a[i+1] for i in range(8, 255) ]
b.append(b[-1])
for i in range(32):
    print("    ", end='')
    for j in range(8):
        print("{ %5d, %5d }," % (a[8*i+j], b[8*i+j]), end='')
        if j < 7: print(" ", end='')
    if i == 0: print(" // dummy first row")
    else: print()
print("  };")
print()

print("  const uint16_t speed_lookuptable_slow[256][2] PROGMEM = {")
a = [ int(0.5 + float(timer_freq) / ((i*8)+(args.cpu_freq*2))) for i in range(256) ]
b = [ a[i] - a[i+1] for i in range(255) ]
b.append(b[-1])
for i in range(32):
    print("    ", end='')
    for j in range(8):
        print("{ %5d, %5d }," % (a[8*i+j], b[8*i+j]), end='')
        if j < 7: print(" ", end='')
    print()
print("  };")
print()

print("#endif")

