#!/usr/bin/env python
"""Thermistor Value Lookup Table Generator

Generates lookup to temperature values for use in a microcontroller in C format based on:
https://en.wikipedia.org/wiki/Steinhart-Hart_equation

The main use is for Arduino programs that read data from the circuit board described here:
https://reprap.org/wiki/Temperature_Sensor_v2.0

Usage: python createTemperatureLookupMarlin.py [options]

Options:
  -h, --help        show this help
  --rp=...          pull-up resistor
  --t1=ttt:rrr      low temperature temperature:resistance point (around 25 degC)
  --t2=ttt:rrr      middle temperature temperature:resistance point (around 150 degC)
  --t3=ttt:rrr      high temperature temperature:resistance point (around 250 degC)
  --num-temps=...   the number of temperature points to calculate (default: 36)
"""

from __future__ import print_function, division

from math import *
import sys, getopt

"Constants"
ZERO   = 273.15                             # zero point of Kelvin scale
VADC   = 5                                  # ADC voltage
VCC    = 5                                  # supply voltage
ARES   = pow(2,10)                          # 10 Bit ADC resolution
VSTEP  = VADC / ARES                        # ADC voltage resolution
TMIN   = 0                                  # lowest temperature in table
TMAX   = 350                                # highest temperature in table

class Thermistor:
    "Class to do the thermistor maths"
    def __init__(self, rp, t1, r1, t2, r2, t3, r3):
        l1 = log(r1)
        l2 = log(r2)
        l3 = log(r3)
        y1 = 1.0 / (t1 + ZERO)              # adjust scale
        y2 = 1.0 / (t2 + ZERO)
        y3 = 1.0 / (t3 + ZERO)
        x = (y2 - y1) / (l2 - l1)
        y = (y3 - y1) / (l3 - l1)
        c = (y - x) / ((l3 - l2) * (l1 + l2 + l3))
        b = x - c * (l1**2 + l2**2 + l1*l2)
        a = y1 - (b + l1**2 *c)*l1

        if c < 0:
            print("//////////////////////////////////////////////////////////////////////////////////////")
            print("// WARNING: Negative coefficient 'c'! Something may be wrong with the measurements! //")
            print("//////////////////////////////////////////////////////////////////////////////////////")
            c = -c
        self.c1 = a                         # Steinhart-Hart coefficients
        self.c2 = b
        self.c3 = c
        self.rp = rp                        # pull-up resistance

    def resol(self, adc):
        "Convert ADC reading into a resolution"
        res = self.temp(adc)-self.temp(adc+1)
        return res

    def voltage(self, adc):
        "Convert ADC reading into a Voltage"
        return adc * VSTEP                     # convert the 10 bit ADC value to a voltage

    def resist(self, adc):
        "Convert ADC reading into a resistance in Ohms"
        r = self.rp * self.voltage(adc) / (VCC - self.voltage(adc)) # resistance of thermistor
        return r

    def temp(self, adc):
        "Convert ADC reading into a temperature in Celsius"
        l = log(self.resist(adc))
        Tinv = self.c1 + self.c2*l + self.c3* l**3 # inverse temperature
        return (1/Tinv) - ZERO              # temperature

    def adc(self, temp):
        "Convert temperature into a ADC reading"
        x = (self.c1 - (1.0 / (temp+ZERO))) / (2*self.c3)
        y = sqrt((self.c2 / (3*self.c3))**3 + x**2)
        r = exp((y-x)**(1.0/3) - (y+x)**(1.0/3))
        return (r / (self.rp + r)) * ARES

def main(argv):
    "Default values"
    t1 = 25                                 # low temperature in Kelvin (25 degC)
    r1 = 100000                             # resistance at low temperature (10 kOhm)
    t2 = 150                                # middle temperature in Kelvin (150 degC)
    r2 = 1641.9                             # resistance at middle temperature (1.6 KOhm)
    t3 = 250                                # high temperature in Kelvin (250 degC)
    r3 = 226.15                             # resistance at high temperature (226.15 Ohm)
    rp = 4700                               # pull-up resistor (4.7 kOhm)
    num_temps = 36                          # number of entries for look-up table

    try:
        opts, args = getopt.getopt(argv, "h", ["help", "rp=", "t1=", "t2=", "t3=", "num-temps="])
    except getopt.GetoptError as err:
        print(str(err))
        usage()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            usage()
            sys.exit()
        elif opt == "--rp":
            rp = int(arg)
        elif opt == "--t1":
            arg =  arg.split(':')
            t1 = float(arg[0])
            r1 = float(arg[1])
        elif opt == "--t2":
            arg =  arg.split(':')
            t2 = float(arg[0])
            r2 = float(arg[1])
        elif opt == "--t3":
            arg =  arg.split(':')
            t3 = float(arg[0])
            r3 = float(arg[1])
        elif opt == "--num-temps":
            num_temps = int(arg)

    t = Thermistor(rp, t1, r1, t2, r2, t3, r3)
    increment = int((ARES - 1) / (num_temps - 1))
    step = int((TMIN - TMAX) / (num_temps - 1))
    low_bound = t.temp(ARES - 1)
    up_bound = t.temp(1)
    min_temp = int(TMIN if TMIN > low_bound else low_bound)
    max_temp = int(TMAX if TMAX < up_bound else up_bound)
    temps = list(range(max_temp, TMIN + step, step))

    print("// Thermistor lookup table for Marlin")
    print("// ./createTemperatureLookupMarlin.py --rp=%s --t1=%s:%s --t2=%s:%s --t3=%s:%s --num-temps=%s" % (rp, t1, r1, t2, r2, t3, r3, num_temps))
    print("// Steinhart-Hart Coefficients: a=%.15g, b=%.15g, c=%.15g " % (t.c1, t.c2, t.c3))
    print("// Theoretical limits of thermistor: %.2f to %.2f degC" % (low_bound, up_bound))
    print()
    print("const short temptable[][2] PROGMEM = {")

    for temp in temps:
        adc = t.adc(temp)
        print("    { OV(%7.2f), %4s }%s // v=%.3f\tr=%.3f\tres=%.3f degC/count" % (adc , temp, \
                        ',' if temp != temps[-1] else ' ', \
                        t.voltage(adc), \
                        t.resist( adc), \
                        t.resol(  adc) \
                    ))
    print("};")

def usage():
    print(__doc__)

if __name__ == "__main__":
    main(sys.argv[1:])
