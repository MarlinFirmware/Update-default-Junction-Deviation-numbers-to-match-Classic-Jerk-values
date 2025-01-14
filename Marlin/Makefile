# Marlin Firmware Arduino Project Makefile
#
# Makefile Based on:
# Arduino 0011 Makefile
# Arduino adaptation by mellis, eighthave, oli.keller
# Marlin adaption by Daid
# Marlin 2.0 support and RELOC_WORKAROUND by @marcio-ao
#
# This has been tested with Arduino 0022.
#
# This makefile allows you to build sketches from the command line
# without the Arduino environment (or Java).
#
# Detailed instructions for using the makefile:
#
#  1. Modify the line containing "ARDUINO_INSTALL_DIR" to point to the directory that
#     contains the Arduino installation (for example, under macOS, this
#     might be /Applications/Arduino.app/Contents/Resources/Java).
#
#  2. Modify the line containing "UPLOAD_PORT" to refer to the filename
#     representing the USB or serial connection to your Arduino board
#     (e.g. UPLOAD_PORT = /dev/tty.USB0).  If the exact name of this file
#     changes, you can use * as a wild card (e.g. UPLOAD_PORT = /dev/tty.usb*).
#
#  3. Set the line containing "MCU" to match your board's processor. Set
#     "PROG_MCU" as the AVR part name corresponding to "MCU". You can use the
#     following command to get a list of correspondences: `avrdude -c alf -p x`
#     Older boards are atmega8 based, newer ones like Arduino Mini, Bluetooth
#     or Diecimila have the atmega168.  If you're using a LilyPad Arduino,
#     change F_CPU to 8000000. If you are using Gen7 electronics, you
#     probably need to use 20000000. Either way, you must regenerate
#     the speed lookup table with create_speed_lookuptable.py.
#
#  4. Type "make" and press enter to compile/verify your program.
#
#  5. Type "make upload", reset your Arduino board, and press enter to
#     upload your program to the Arduino board.
#
# Note that all settings at the top of this file can be overridden from
# the command line with, for example, "make HARDWARE_MOTHERBOARD=71"
#
# To compile for RAMPS (atmega2560) with Arduino 1.6.9 at root/arduino you would use...
#
#   make ARDUINO_VERSION=10609 AVR_TOOLS_PATH=/root/arduino/hardware/tools/avr/bin/ \
#   HARDWARE_MOTHERBOARD=1200 ARDUINO_INSTALL_DIR=/root/arduino
#
# To compile and upload simply add "upload" to the end of the line...
#
#   make ARDUINO_VERSION=10609 AVR_TOOLS_PATH=/root/arduino/hardware/tools/avr/bin/ \
#   HARDWARE_MOTHERBOARD=1200 ARDUINO_INSTALL_DIR=/root/arduino upload
#
# If uploading doesn't work try adding the parameter "AVRDUDE_PROGRAMMER=wiring" or
# start upload manually (using stk500) like so:
#
#   avrdude -C /root/arduino/hardware/tools/avr/etc/avrdude.conf -v -p m2560 -c stk500 \
#   -U flash:w:applet/Marlin.hex:i -P /dev/ttyUSB0
#
# Or, try disconnecting USB to power down and then reconnecting before running avrdude.
#

# This defines the board to compile for (see boards.h for your board's ID)
HARDWARE_MOTHERBOARD ?= 1020

ifeq ($(OS),Windows_NT)
  # Windows
  ARDUINO_INSTALL_DIR ?= ${HOME}/AppData/Local/Arduino
  ARDUINO_USER_DIR ?= ${HOME}/Documents/Arduino
else
  UNAME_S := $(shell uname -s)
  ifeq ($(UNAME_S),Linux)
    # Linux
    ARDUINO_INSTALL_DIR ?= /usr/share/arduino
    ARDUINO_USER_DIR ?= ${HOME}/Arduino
  endif
  ifeq ($(UNAME_S),Darwin)
    # Darwin (macOS)
    ARDUINO_INSTALL_DIR ?= /Applications/Arduino.app/Contents/Java
    ARDUINO_USER_DIR ?= ${HOME}/Documents/Arduino
    AVR_TOOLS_PATH ?= /Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/
  endif
endif

# Arduino source install directory, and version number
# On most linuxes this will be /usr/share/arduino
ARDUINO_INSTALL_DIR  ?= ${HOME}/AppData/Local/Arduino       # C:/Users/${USERNAME}/AppData/Local/Arduino
ARDUINO_VERSION      ?= 10819

# The installed Libraries are in the User folder
ARDUINO_USER_DIR ?= ${HOME}/Documents/Arduino

# You can optionally set a path to the avr-gcc tools.
# Requires a trailing slash. For example, /usr/local/avr-gcc/bin/
AVR_TOOLS_PATH ?=

# Programmer configuration
UPLOAD_RATE        ?= 57600
AVRDUDE_PROGRAMMER ?= arduino
# On most linuxes this will be /dev/ttyACM0 or /dev/ttyACM1
UPLOAD_PORT        ?= /dev/ttyUSB0

# Directory used to build files in, contains all the build files, from object
# files to the final hex file on linux it is best to put an absolute path
# like /home/username/tmp .
BUILD_DIR          ?= applet

# This defines whether Liquid_TWI2 support will be built
LIQUID_TWI2        ?= 0

# This defines if Wire is needed
WIRE               ?= 0

# This defines if Tone is needed (i.e., SPEAKER is defined in Configuration.h)
# Disabling this (and SPEAKER) saves approximately 350 bytes of memory.
TONE               ?= 1

# This defines if U8GLIB is needed (may require RELOC_WORKAROUND)
U8GLIB             ?= 0

# This defines whether to include the Trinamic TMCStepper library
TMC                ?= 0

# This defines whether to include the AdaFruit NeoPixel library
NEOPIXEL           ?= 0

############
# Try to automatically determine whether RELOC_WORKAROUND is needed based
# on GCC versions:
#   https://www.avrfreaks.net/comment/1789106#comment-1789106

CC_MAJ:=$(shell $(CC) -dM -E - < /dev/null | grep __GNUC__ | cut -f3 -d\ )
CC_MIN:=$(shell $(CC) -dM -E - < /dev/null | grep __GNUC_MINOR__ | cut -f3 -d\ )
CC_PATCHLEVEL:=$(shell $(CC) -dM -E - < /dev/null | grep __GNUC_PATCHLEVEL__ | cut -f3 -d\ )
CC_VER:=$(shell echo $$(( $(CC_MAJ) * 10000 + $(CC_MIN) * 100 + $(CC_PATCHLEVEL) )))
ifeq ($(shell test $(CC_VER) -lt 40901 && echo 1),1)
  $(warning This GCC version $(CC_VER) is likely broken. Enabling relocation workaround.)
  RELOC_WORKAROUND = 1
endif

############################################################################
# Below here nothing should be changed...

# Here the Arduino variant is selected by the board type
# HARDWARE_VARIANT = "arduino", "Sanguino", "Gen7", ...
# MCU = "atmega1280", "Mega2560", "atmega2560", "atmega644p", ...

ifeq ($(HARDWARE_MOTHERBOARD),0)

  # No motherboard selected

#
# RAMPS 1.3 / 1.4 - ATmega1280, ATmega2560
#

# MEGA/RAMPS up to 1.2
else ifeq ($(HARDWARE_MOTHERBOARD),1000)

# RAMPS 1.3 (Power outputs: Hotend, Fan, Bed)
else ifeq ($(HARDWARE_MOTHERBOARD),1010)
# RAMPS 1.3 (Power outputs: Hotend0, Hotend1, Bed)
else ifeq ($(HARDWARE_MOTHERBOARD),1011)
# RAMPS 1.3 (Power outputs: Hotend, Fan0, Fan1)
else ifeq ($(HARDWARE_MOTHERBOARD),1012)
# RAMPS 1.3 (Power outputs: Hotend0, Hotend1, Fan)
else ifeq ($(HARDWARE_MOTHERBOARD),1013)
# RAMPS 1.3 (Power outputs: Spindle, Controller Fan)
else ifeq ($(HARDWARE_MOTHERBOARD),1014)

# RAMPS 1.4 (Power outputs: Hotend, Fan, Bed)
else ifeq ($(HARDWARE_MOTHERBOARD),1020)
# RAMPS 1.4 (Power outputs: Hotend0, Hotend1, Bed)
else ifeq ($(HARDWARE_MOTHERBOARD),1021)
# RAMPS 1.4 (Power outputs: Hotend, Fan0, Fan1)
else ifeq ($(HARDWARE_MOTHERBOARD),1022)
# RAMPS 1.4 (Power outputs: Hotend0, Hotend1, Fan)
else ifeq ($(HARDWARE_MOTHERBOARD),1023)
# RAMPS 1.4 (Power outputs: Spindle, Controller Fan)
else ifeq ($(HARDWARE_MOTHERBOARD),1024)

# RAMPS Plus 3DYMY (Power outputs: Hotend, Fan, Bed)
else ifeq ($(HARDWARE_MOTHERBOARD),1030)
# RAMPS Plus 3DYMY (Power outputs: Hotend0, Hotend1, Bed)
else ifeq ($(HARDWARE_MOTHERBOARD),1031)
# RAMPS Plus 3DYMY (Power outputs: Hotend, Fan0, Fan1)
else ifeq ($(HARDWARE_MOTHERBOARD),1032)
# RAMPS Plus 3DYMY (Power outputs: Hotend0, Hotend1, Fan)
else ifeq ($(HARDWARE_MOTHERBOARD),1033)
# RAMPS Plus 3DYMY (Power outputs: Spindle, Controller Fan)
else ifeq ($(HARDWARE_MOTHERBOARD),1034)

# RAMPS 1.6+ (Power outputs: Hotend, Fan, Bed)
else ifeq ($(HARDWARE_MOTHERBOARD),1035)
# RAMPS 1.6+ (Power outputs: Hotend0, Hotend1, Bed)
else ifeq ($(HARDWARE_MOTHERBOARD),1036)
# RAMPS 1.6+ (Power outputs: Hotend, Fan0, Fan1)
else ifeq ($(HARDWARE_MOTHERBOARD),1037)
# RAMPS 1.6+ (Power outputs: Hotend0, Hotend1, Fan)
else ifeq ($(HARDWARE_MOTHERBOARD),1038)
# RAMPS 1.6+ (Power outputs: Spindle, Controller Fan)
else ifeq ($(HARDWARE_MOTHERBOARD),1039)

#
# RAMPS Derivatives - ATmega1280, ATmega2560
#

# 3Drag Controller
else ifeq ($(HARDWARE_MOTHERBOARD),1100)
# Velleman K8200 Controller (derived from 3Drag Controller)
else ifeq ($(HARDWARE_MOTHERBOARD),1101)
# Velleman K8400 Controller (derived from 3Drag Controller)
else ifeq ($(HARDWARE_MOTHERBOARD),1102)
# Velleman K8600 Controller (Vertex Nano)
else ifeq ($(HARDWARE_MOTHERBOARD),1103)
# Velleman K8800 Controller (Vertex Delta)
else ifeq ($(HARDWARE_MOTHERBOARD),1104)
# 2PrintBeta BAM&DICE with STK drivers
else ifeq ($(HARDWARE_MOTHERBOARD),1105)
# 2PrintBeta BAM&DICE Due with STK drivers
else ifeq ($(HARDWARE_MOTHERBOARD),1106)
# MKS BASE v1.0
else ifeq ($(HARDWARE_MOTHERBOARD),1107)
# MKS BASE v1.4 with Allegro A4982 stepper drivers
else ifeq ($(HARDWARE_MOTHERBOARD),1108)
# MKS BASE v1.5 with Allegro A4982 stepper drivers
else ifeq ($(HARDWARE_MOTHERBOARD),1109)
# MKS BASE v1.6 with Allegro A4982 stepper drivers
else ifeq ($(HARDWARE_MOTHERBOARD),1110)
# MKS BASE 1.0 with Heroic HR4982 stepper drivers
else ifeq ($(HARDWARE_MOTHERBOARD),1111)
# MKS GEN v1.3 or 1.4
else ifeq ($(HARDWARE_MOTHERBOARD),1112)
# MKS GEN L
else ifeq ($(HARDWARE_MOTHERBOARD),1113)
# BigTreeTech or BIQU KFB2.0
else ifeq ($(HARDWARE_MOTHERBOARD),1114)
# Zonestar zrib V2.0 (Chinese RAMPS replica)
else ifeq ($(HARDWARE_MOTHERBOARD),1115)
# Zonestar zrib V5.2 (Chinese RAMPS replica)
else ifeq ($(HARDWARE_MOTHERBOARD),1116)
# Zonestar zrib V5.3 (Chinese RAMPS replica)
else ifeq ($(HARDWARE_MOTHERBOARD),1117)
# Felix 2.0+ Electronics Board (RAMPS like)
else ifeq ($(HARDWARE_MOTHERBOARD),1118)
# Invent-A-Part RigidBoard
else ifeq ($(HARDWARE_MOTHERBOARD),1119)
# Invent-A-Part RigidBoard V2
else ifeq ($(HARDWARE_MOTHERBOARD),1120)
# Sainsmart 2-in-1 board
else ifeq ($(HARDWARE_MOTHERBOARD),1121)
# Ultimaker
else ifeq ($(HARDWARE_MOTHERBOARD),1122)
# Ultimaker (Older electronics. Pre 1.5.4. This is rare)
else ifeq ($(HARDWARE_MOTHERBOARD),1123)
  MCU              ?= atmega1280
  PROG_MCU         ?= m1280
# Azteeg X3
else ifeq ($(HARDWARE_MOTHERBOARD),1124)
# Azteeg X3 Pro
else ifeq ($(HARDWARE_MOTHERBOARD),1125)
# Ultimainboard 2.x (Uses TEMP_SENSOR 20)
else ifeq ($(HARDWARE_MOTHERBOARD),1126)
# Rumba
else ifeq ($(HARDWARE_MOTHERBOARD),1127)
# Raise3D N series Rumba derivative
else ifeq ($(HARDWARE_MOTHERBOARD),1128)
# Rapide Lite 200 (v1, low-cost RUMBA clone with drv)
else ifeq ($(HARDWARE_MOTHERBOARD),1129)
# Formbot T-Rex 2 Plus
else ifeq ($(HARDWARE_MOTHERBOARD),1130)
# Formbot T-Rex 3
else ifeq ($(HARDWARE_MOTHERBOARD),1131)
# Formbot Raptor
else ifeq ($(HARDWARE_MOTHERBOARD),1132)
# Formbot Raptor 2
else ifeq ($(HARDWARE_MOTHERBOARD),1133)
# bq ZUM Mega 3D
else ifeq ($(HARDWARE_MOTHERBOARD),1134)
# MakeBoard Mini v2.1.2 by MicroMake
else ifeq ($(HARDWARE_MOTHERBOARD),1135)
# TriGorilla Anycubic version 1.3-based on RAMPS EFB
else ifeq ($(HARDWARE_MOTHERBOARD),1136)
#   ... Ver 1.4
else ifeq ($(HARDWARE_MOTHERBOARD),1137)
#   ... Rev 1.1 (new servo pin order)
else ifeq ($(HARDWARE_MOTHERBOARD),1138)
# Creality: Ender-4, CR-8
else ifeq ($(HARDWARE_MOTHERBOARD),1139)
# Creality: CR10S, CR20, CR-X
else ifeq ($(HARDWARE_MOTHERBOARD),1140)
# Dagoma F5
else ifeq ($(HARDWARE_MOTHERBOARD),1141)
# Dagoma D6 (as found in the Dagoma DiscoUltimate V2 TMC)
else ifeq ($(HARDWARE_MOTHERBOARD),1142)
# FYSETC F6 1.3
else ifeq ($(HARDWARE_MOTHERBOARD),1143)
# FYSETC F6 1.4
else ifeq ($(HARDWARE_MOTHERBOARD),1144)
# Wanhao Duplicator i3 Plus
else ifeq ($(HARDWARE_MOTHERBOARD),1145)
# VORON Design
else ifeq ($(HARDWARE_MOTHERBOARD),1146)
# Tronxy TRONXY-V3-1.0
else ifeq ($(HARDWARE_MOTHERBOARD),1147)
# Z-Bolt X Series
else ifeq ($(HARDWARE_MOTHERBOARD),1148)
# TT OSCAR
else ifeq ($(HARDWARE_MOTHERBOARD),1149)
# BIQU Tango V1
else ifeq ($(HARDWARE_MOTHERBOARD),1150)
# MKS GEN L V2
else ifeq ($(HARDWARE_MOTHERBOARD),1151)
# MKS GEN L V2.1
else ifeq ($(HARDWARE_MOTHERBOARD),1152)
# Copymaster 3D
else ifeq ($(HARDWARE_MOTHERBOARD),1153)
# Ortur 4
else ifeq ($(HARDWARE_MOTHERBOARD),1154)
# Tenlog D3 Hero IDEX printer
else ifeq ($(HARDWARE_MOTHERBOARD),1155)
# Tenlog D3, D5, D6 IDEX Printer
else ifeq ($(HARDWARE_MOTHERBOARD),1156)
# Ramps S 1.2 by Sakul.cz (Power outputs: Hotend0, Hotend1, Fan, Bed)
else ifeq ($(HARDWARE_MOTHERBOARD),1157)
# Ramps S 1.2 by Sakul.cz (Power outputs: Hotend0, Hotend1, Hotend2, Bed)
else ifeq ($(HARDWARE_MOTHERBOARD),1158)
# Ramps S 1.2 by Sakul.cz (Power outputs: Hotend, Fan0, Fan1, Bed)
else ifeq ($(HARDWARE_MOTHERBOARD),1159)
# Longer LK1 PRO / Alfawise U20 Pro (PRO version)
else ifeq ($(HARDWARE_MOTHERBOARD),1160)
# Longer LKx PRO / Alfawise Uxx Pro (PRO version)
else ifeq ($(HARDWARE_MOTHERBOARD),1161)
# Pxmalion Core I3
else ifeq ($(HARDWARE_MOTHERBOARD),1162)
# Panowin Cutlass (as found in the Panowin F1)
else ifeq ($(HARDWARE_MOTHERBOARD),1163)
# Kodama Bardo V1.x (as found in the Kodama Trinus)
else ifeq ($(HARDWARE_MOTHERBOARD),1164)
# XTLW MFF V1.0
else ifeq ($(HARDWARE_MOTHERBOARD),1165)
# XTLW MFF V2.0
else ifeq ($(HARDWARE_MOTHERBOARD),1166)


#
# RAMBo and derivatives
#

# Rambo
else ifeq ($(HARDWARE_MOTHERBOARD),1200)
# Mini-Rambo
else ifeq ($(HARDWARE_MOTHERBOARD),1201)
# Mini-Rambo 1.0a
else ifeq ($(HARDWARE_MOTHERBOARD),1202)
# Einsy Rambo
else ifeq ($(HARDWARE_MOTHERBOARD),1203)
# Einsy Retro
else ifeq ($(HARDWARE_MOTHERBOARD),1204)
# abee Scoovo X9H
else ifeq ($(HARDWARE_MOTHERBOARD),1205)
# ThinkerV2
else ifeq ($(HARDWARE_MOTHERBOARD),1206)

#
# Other ATmega1280, ATmega2560
#

# Cartesio CN Controls V11
else ifeq ($(HARDWARE_MOTHERBOARD),1300)
# Cartesio CN Controls V12
else ifeq ($(HARDWARE_MOTHERBOARD),1301)
# Cartesio CN Controls V15
else ifeq ($(HARDWARE_MOTHERBOARD),1302)
# Cheaptronic v1.0
else ifeq ($(HARDWARE_MOTHERBOARD),1303)
# Cheaptronic v2.0
else ifeq ($(HARDWARE_MOTHERBOARD),1304)
# Makerbot Mightyboard Revision E
else ifeq ($(HARDWARE_MOTHERBOARD),1305)
# Megatronics
else ifeq ($(HARDWARE_MOTHERBOARD),1306)
# Megatronics v2.0
else ifeq ($(HARDWARE_MOTHERBOARD),1307)
# Megatronics v3.0
else ifeq ($(HARDWARE_MOTHERBOARD),1308)
# Megatronics v3.1
else ifeq ($(HARDWARE_MOTHERBOARD),1309)
# Megatronics v3.2
else ifeq ($(HARDWARE_MOTHERBOARD),1310)
# Elefu Ra Board (v3)
else ifeq ($(HARDWARE_MOTHERBOARD),1311)
# Leapfrog
else ifeq ($(HARDWARE_MOTHERBOARD),1312)
# Mega controller
else ifeq ($(HARDWARE_MOTHERBOARD),1313)
# Geeetech GT2560 Rev A
else ifeq ($(HARDWARE_MOTHERBOARD),1314)
# Geeetech GT2560 Rev A+ (with auto level probe)
else ifeq ($(HARDWARE_MOTHERBOARD),1315)
# Geeetech GT2560 Rev B
else ifeq ($(HARDWARE_MOTHERBOARD),1316)
# Geeetech GT2560 Rev B for A10(M/T/D)
else ifeq ($(HARDWARE_MOTHERBOARD),1317)
# Geeetech GT2560 Rev B for Mecreator2
else ifeq ($(HARDWARE_MOTHERBOARD),1318)
# Geeetech GT2560 Rev B for A20(M/T/D)
else ifeq ($(HARDWARE_MOTHERBOARD),1319)
# Geeetech GT2560 Rev B for A10(M/T/D)
else ifeq ($(HARDWARE_MOTHERBOARD),1320)
# Geeetech GT2560 Rev B for A20(M/T/D)
else ifeq ($(HARDWARE_MOTHERBOARD),1321)
# Einstart retrofit
else ifeq ($(HARDWARE_MOTHERBOARD),1322)
# Wanhao 0ne+ i3 Mini
else ifeq ($(HARDWARE_MOTHERBOARD),1323)
# Overlord/Overlord Pro
else ifeq ($(HARDWARE_MOTHERBOARD),1324)
# ADIMLab Gantry v1
else ifeq ($(HARDWARE_MOTHERBOARD),1325)
# ADIMLab Gantry v2
else ifeq ($(HARDWARE_MOTHERBOARD),1326)
# Leapfrog Xeed 2015
else ifeq ($(HARDWARE_MOTHERBOARD),1327)
# PICA Shield (original version)
else ifeq ($(HARDWARE_MOTHERBOARD),1328)
# PICA Shield (rev C or later)
else ifeq ($(HARDWARE_MOTHERBOARD),1329)
# Intamsys 4.0 (Funmat HT)
else ifeq ($(HARDWARE_MOTHERBOARD),1330)
# Malyan M180 Mainboard Version 2 (no display function, direct G-code only)
else ifeq ($(HARDWARE_MOTHERBOARD),1331)
# Mega controller & Protoneer CNC Shield V3.00
else ifeq ($(HARDWARE_MOTHERBOARD),1332)
# WEEDO 62A board (TINA2, Monoprice Cadet, etc.)
else ifeq ($(HARDWARE_MOTHERBOARD),1333)
# Geeetech GT2560 V4.1B for A10(M/T/D)
else ifeq ($(HARDWARE_MOTHERBOARD),1334)

#
# ATmega1281, ATmega2561
#

# Minitronics v1.0/1.1
else ifeq ($(HARDWARE_MOTHERBOARD),1400)
  MCU              ?= atmega1281
  PROG_MCU         ?= m1281
# Silvergate v1.0
else ifeq ($(HARDWARE_MOTHERBOARD),1401)
  MCU              ?= atmega1281
  PROG_MCU         ?= m1281

#
# Sanguinololu and Derivatives - ATmega644P, ATmega1284P
#

# Sanguinololu < 1.2
else ifeq ($(HARDWARE_MOTHERBOARD),1500)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega644p
  PROG_MCU         ?= m644p
# Sanguinololu 1.2 and above
else ifeq ($(HARDWARE_MOTHERBOARD),1501)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega644p
  PROG_MCU         ?= m644p
# Melzi
else ifeq ($(HARDWARE_MOTHERBOARD),1502)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega644p
  PROG_MCU         ?= m644p
# Melzi V2
else ifeq ($(HARDWARE_MOTHERBOARD),1503)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega1284p
  PROG_MCU         ?= m1284p
# Melzi with ATmega1284 (MaKr3d version)
else ifeq ($(HARDWARE_MOTHERBOARD),1504)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega1284p
  PROG_MCU         ?= m1284p
# Melzi Creality3D (for CR-10 etc)
else ifeq ($(HARDWARE_MOTHERBOARD),1505)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega1284p
  PROG_MCU         ?= m1284p
# Melzi Creality3D (for Ender-2)
else ifeq ($(HARDWARE_MOTHERBOARD),1506)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega1284p
  PROG_MCU         ?= m1284p
# Melzi Malyan M150
else ifeq ($(HARDWARE_MOTHERBOARD),1507)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega1284p
  PROG_MCU         ?= m1284p
# Tronxy X5S
else ifeq ($(HARDWARE_MOTHERBOARD),1508)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega1284p
  PROG_MCU         ?= m1284p
# STB V1.1
else ifeq ($(HARDWARE_MOTHERBOARD),1509)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega1284p
  PROG_MCU         ?= m1284p
# Azteeg X1
else ifeq ($(HARDWARE_MOTHERBOARD),1510)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega1284p
  PROG_MCU         ?= m1284p
# Anet 1.0 (Melzi clone)
else ifeq ($(HARDWARE_MOTHERBOARD),1511)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega1284p
  PROG_MCU         ?= m1284p
# ZoneStar ZMIB V2
else ifeq ($(HARDWARE_MOTHERBOARD),1511)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega1284p
  PROG_MCU         ?= m1284p

#
# Other ATmega644P, ATmega644, ATmega1284P
#

# Gen3 Monolithic Electronics
else ifeq ($(HARDWARE_MOTHERBOARD),1600)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega644p
  PROG_MCU         ?= m644p
# Gen3+
else ifeq ($(HARDWARE_MOTHERBOARD),1601)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega644p
  PROG_MCU         ?= m644p
# Gen6
else ifeq ($(HARDWARE_MOTHERBOARD),1602)
  HARDWARE_VARIANT ?= Gen6
  MCU              ?= atmega644p
  PROG_MCU         ?= m644p
# Gen6 deluxe
else ifeq ($(HARDWARE_MOTHERBOARD),1603)
  HARDWARE_VARIANT ?= Gen6
  MCU              ?= atmega644p
  PROG_MCU         ?= m644p
# Gen7 custom (Alfons3 Version)
else ifeq ($(HARDWARE_MOTHERBOARD),1604)
  HARDWARE_VARIANT ?= Gen7
  MCU              ?= atmega644
  PROG_MCU         ?= m644
  F_CPU            ?= 20000000
# Gen7 v1.1, v1.2
else ifeq ($(HARDWARE_MOTHERBOARD),1605)
  HARDWARE_VARIANT ?= Gen7
  MCU              ?= atmega644p
  PROG_MCU         ?= m644p
  F_CPU            ?= 20000000
# Gen7 v1.3
else ifeq ($(HARDWARE_MOTHERBOARD),1606)
  HARDWARE_VARIANT ?= Gen7
  MCU              ?= atmega644p
  PROG_MCU         ?= m644p
  F_CPU            ?= 20000000
# Gen7 v1.4
else ifeq ($(HARDWARE_MOTHERBOARD),1607)
  HARDWARE_VARIANT ?= Gen7
  MCU              ?= atmega1284p
  PROG_MCU         ?= m1284p
  F_CPU            ?= 20000000
# Alpha OMCA board
else ifeq ($(HARDWARE_MOTHERBOARD),1608)
  HARDWARE_VARIANT ?= SanguinoA
  MCU              ?= atmega644
  PROG_MCU         ?= m644
# Final OMCA board
else ifeq ($(HARDWARE_MOTHERBOARD),1609)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega644p
  PROG_MCU         ?= m644p
# Sethi 3D_1
else ifeq ($(HARDWARE_MOTHERBOARD),1610)
  HARDWARE_VARIANT ?= Sanguino
  MCU              ?= atmega644p
  PROG_MCU         ?= m644p

#
# Teensyduino - AT90USB1286, AT90USB1286P
#

# Teensylu
else ifeq ($(HARDWARE_MOTHERBOARD),1700)
  HARDWARE_VARIANT ?= Teensy
  MCU              ?= at90usb1286
  PROG_MCU         ?= usb1286
# Printrboard (AT90USB1286)
else ifeq ($(HARDWARE_MOTHERBOARD),1701)
  HARDWARE_VARIANT ?= Teensy
  MCU              ?= at90usb1286
  PROG_MCU         ?= usb1286
# Printrboard Revision F (AT90USB1286)
else ifeq ($(HARDWARE_MOTHERBOARD),1702)
  HARDWARE_VARIANT ?= Teensy
  MCU              ?= at90usb1286
  PROG_MCU         ?= usb1286
# Brainwave (AT90USB646)
else ifeq ($(HARDWARE_MOTHERBOARD),1703)
  HARDWARE_VARIANT ?= Teensy
  MCU              ?= at90usb646
  PROG_MCU         ?= usb646
# Brainwave Pro (AT90USB1286)
else ifeq ($(HARDWARE_MOTHERBOARD),1704)
  HARDWARE_VARIANT ?= Teensy
  MCU              ?= at90usb1286
  PROG_MCU         ?= usb1286
# SAV Mk-I (AT90USB1286)
else ifeq ($(HARDWARE_MOTHERBOARD),1705)
  HARDWARE_VARIANT ?= Teensy
  MCU              ?= at90usb1286
  PROG_MCU         ?= usb1286
# Teensy++2.0 (AT90USB1286)
else ifeq ($(HARDWARE_MOTHERBOARD),1706)
  HARDWARE_VARIANT ?= Teensy
  MCU              ?= at90usb1286
  PROG_MCU         ?= usb1286
# 5DPrint D8 Driver Board
else ifeq ($(HARDWARE_MOTHERBOARD),1707)
  HARDWARE_VARIANT ?= Teensy
  MCU              ?= at90usb1286
  PROG_MCU         ?= usb1286

# UltiMachine Archim1 (with DRV8825 drivers)
else ifeq ($(HARDWARE_MOTHERBOARD),3023)
  HARDWARE_VARIANT ?= archim
  MCPU              = cortex-m3
  F_CPU             = 84000000
  IS_MCU            = 0
# UltiMachine Archim2 (with TMC2130 drivers)
else ifeq ($(HARDWARE_MOTHERBOARD),3024)
  HARDWARE_VARIANT ?= archim
  MCPU              = cortex-m3
  F_CPU             = 84000000
  IS_MCU            = 0
endif

# Be sure to regenerate speed_lookuptable.h with create_speed_lookuptable.py
# if you are setting this to something other than 16MHz
# Do not put the UL suffix, it's done later on.
# Set to 16Mhz if not yet set.
F_CPU ?= 16000000

# Set to microcontroller if IS_MCU not yet set
IS_MCU ?= 1

ifeq ($(IS_MCU),1)
  # Set to arduino, ATmega2560 if not yet set.
  HARDWARE_VARIANT ?= arduino
  MCU              ?= atmega2560
  PROG_MCU         ?= m2560

  TOOL_PREFIX = avr
  MCU_FLAGS   = -mmcu=$(MCU)
  SIZE_FLAGS  = --mcu=$(MCU) -C
else
  TOOL_PREFIX = arm-none-eabi
  CPU_FLAGS   = -mthumb -mcpu=$(MCPU)
  SIZE_FLAGS  = -A
endif

# Arduino contained the main source code for the Arduino
# Libraries, the "hardware variant" are for boards
# that derives from that, and their source are present in
# the main Marlin source directory

TARGET = $(notdir $(CURDIR))

# VPATH tells make to look into these directory for source files,
# there is no need to specify explicit pathnames as long as the
# directory is added here

# The Makefile for previous versions of Marlin used VPATH for all
# source files, but for Marlin 2.0, we use VPATH only for arduino
# library files.

VPATH = .
VPATH += $(BUILD_DIR)
VPATH += $(HARDWARE_SRC)

ifeq ($(HARDWARE_VARIANT), $(filter $(HARDWARE_VARIANT),arduino Teensy Sanguino))
  # Old libraries (avr-core 1.6.21 < / Arduino < 1.6.8)
  VPATH += $(ARDUINO_INSTALL_DIR)/hardware/arduino/avr/libraries/SPI
  # New libraries (avr-core >= 1.6.21 / Arduino >= 1.6.8)
  VPATH += $(ARDUINO_INSTALL_DIR)/packages/arduino/hardware/arduino/avr/1.8.6/libraries/SPI/src
endif

ifeq ($(IS_MCU),1)
  VPATH += $(ARDUINO_INSTALL_DIR)/packages/arduino/hardware/arduino/avr/1.8.6/cores/arduino

  # Old libraries (avr-core 1.6.21 < / Arduino < 1.6.8)
  VPATH += $(ARDUINO_INSTALL_DIR)/hardware/arduino/avr/libraries/SPI
  VPATH += $(ARDUINO_INSTALL_DIR)/hardware/arduino/avr/libraries/SoftwareSerial
  # New libraries (avr-core >= 1.6.21 / Arduino >= 1.6.8)
  VPATH += $(ARDUINO_INSTALL_DIR)/packages/arduino/hardware/arduino/avr/1.8.6/libraries/SPI/src
  VPATH += $(ARDUINO_INSTALL_DIR)/packages/arduino/hardware/arduino/avr/1.8.6/libraries/SoftwareSerial/src
endif

VPATH += $(ARDUINO_INSTALL_DIR)/libraries/LiquidCrystal/src

ifeq ($(LIQUID_TWI2), 1)
  WIRE   = 1
  VPATH += $(ARDUINO_INSTALL_DIR)/libraries/LiquidTWI2
endif
ifeq ($(WIRE), 1)
  # Old libraries (avr-core 1.6.21 / Arduino < 1.6.8)
  VPATH += $(ARDUINO_INSTALL_DIR)/hardware/arduino/avr/libraries/Wire
  VPATH += $(ARDUINO_INSTALL_DIR)/hardware/arduino/avr/libraries/Wire/utility
  # New libraries (avr-core >= 1.6.21 / Arduino >= 1.6.8)
  VPATH += $(ARDUINO_INSTALL_DIR)/packages/arduino/hardware/avr/1.8.6/libraries/Wire/src
  VPATH += $(ARDUINO_INSTALL_DIR)/packages/arduino/hardware/avr/1.8.6/libraries/Wire/src/utility
endif
ifeq ($(NEOPIXEL), 1)
VPATH += $(ARDUINO_INSTALL_DIR)/libraries/Adafruit_NeoPixel
endif
ifeq ($(U8GLIB), 1)
VPATH += $(ARDUINO_INSTALL_DIR)/libraries/U8glib-HAL
VPATH += $(ARDUINO_INSTALL_DIR)/libraries/U8glib-HAL/src
# VPATH += $(ARDUINO_INSTALL_DIR)/libraries/U8glib
# VPATH += $(ARDUINO_INSTALL_DIR)/libraries/U8glib/src
endif
ifeq ($(TMC), 1)
VPATH += $(ARDUINO_INSTALL_DIR)/libraries/TMCStepper/src
VPATH += $(ARDUINO_INSTALL_DIR)/libraries/TMCStepper/src/source
endif

ifeq ($(HARDWARE_VARIANT), arduino)
  HARDWARE_SUB_VARIANT ?= mega
  VPATH += $(ARDUINO_INSTALL_DIR)/packages/arduino/hardware/avr/1.8.6/variants/$(HARDWARE_SUB_VARIANT)
else ifeq ($(HARDWARE_VARIANT), Sanguino)
  VPATH += $(ARDUINO_INSTALL_DIR)/packages/arduino/hardware/avr/1.8.6/variants/sanguino
else ifeq ($(HARDWARE_VARIANT), archim)
  VPATH   += $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/system/libsam
  VPATH   += $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/system/CMSIS/CMSIS/Include/
  VPATH   += $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/system/CMSIS/Device/ATMEL/
  VPATH   += $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/cores/arduino
  VPATH   += $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/cores/arduino/avr
  VPATH   += $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/cores/arduino/USB
  VPATH   += $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/libraries/Wire/src
  VPATH   += $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/libraries/SPI/src
  VPATH   += $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/libraries/U8glib/src/clib
  VPATH   += $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/variants/archim
  LDSCRIPT = $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/variants/archim/linker_scripts/gcc/flash.ld
  LDLIBS   = $(ARDUINO_INSTALL_DIR)/packages/ultimachine/hardware/sam/1.6.9-b/variants/archim/libsam_sam3x8e_gcc_rel.a
else
  HARDWARE_SUB_VARIANT ?= standard
  VPATH += $(ARDUINO_INSTALL_DIR)/packages/arduino/hardware/avr/1.8.6/variants/$(HARDWARE_SUB_VARIANT)
endif

LIB_SRC = wiring.c \
  wiring_analog.c wiring_digital.c \
  wiring_shift.c WInterrupts.c hooks.c

ifeq ($(HARDWARE_VARIANT), archim)
  LIB_ASRC += wiring_pulse_asm.S
else
  LIB_SRC += wiring_pulse.c
endif

ifeq ($(HARDWARE_VARIANT), Teensy)
  LIB_SRC = wiring.c
  VPATH += $(ARDUINO_INSTALL_DIR)/packages/arduino/hardware/teensy/cores/teensy
endif

LIB_CXXSRC = WMath.cpp WString.cpp Print.cpp SPI.cpp

ifeq ($(NEOPIXEL), 1)
  LIB_CXXSRC += Adafruit_NeoPixel.cpp
endif

ifeq ($(LIQUID_TWI2), 0)
  LIB_CXXSRC += LiquidCrystal.cpp
else
  LIB_SRC += twi.c
  LIB_CXXSRC += Wire.cpp LiquidTWI2.cpp
endif

ifeq ($(WIRE), 1)
  LIB_SRC += twi.c
  LIB_CXXSRC += Wire.cpp
endif

ifeq ($(TONE), 1)
  LIB_CXXSRC += Tone.cpp
endif

ifeq ($(U8GLIB), 1)
  LIB_CXXSRC += U8glib.cpp
  LIB_SRC += u8g_ll_api.c u8g_bitmap.c u8g_clip.c u8g_com_null.c u8g_delay.c \
    u8g_page.c u8g_pb.c u8g_pb16h1.c u8g_rect.c u8g_state.c u8g_font.c \
    u8g_font_6x13.c u8g_font_04b_03.c u8g_font_5x8.c
endif

ifeq ($(TMC), 1)
  LIB_CXXSRC += TMCStepper.cpp COOLCONF.cpp DRV_STATUS.cpp IHOLD_IRUN.cpp \
    CHOPCONF.cpp GCONF.cpp PWMCONF.cpp DRV_CONF.cpp DRVCONF.cpp DRVCTRL.cpp \
    DRVSTATUS.cpp ENCMODE.cpp RAMP_STAT.cpp SGCSCONF.cpp SHORT_CONF.cpp \
    SMARTEN.cpp SW_MODE.cpp SW_SPI.cpp TMC2130Stepper.cpp TMC2208Stepper.cpp \
    TMC2209Stepper.cpp TMC2660Stepper.cpp TMC5130Stepper.cpp TMC5160Stepper.cpp
endif

ifeq ($(RELOC_WORKAROUND), 1)
  LD_PREFIX=-nodefaultlibs
  LD_SUFFIX=-lm -lgcc -lc -lgcc
endif

#Check for Arduino 1.0.0 or higher and use the correct source files for that version
ifeq ($(shell [ $(ARDUINO_VERSION) -ge 100 ] && echo true), true)
  LIB_CXXSRC += main.cpp
else
  LIB_SRC += pins_arduino.c main.c
endif

FORMAT = ihex

# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
DEBUG = stabs

OPT = s

DEFINES ?=

# Program settings
CC = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-gcc
CXX = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-g++
OBJCOPY = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-objdump
AR  = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-ar
SIZE = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-size
NM = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-nm
AVRDUDE = avrdude
REMOVE = rm -f
MV = mv -f

# Place -D or -U options here
CDEFS    = -DF_CPU=$(F_CPU)UL ${addprefix -D , $(DEFINES)} -DARDUINO=$(ARDUINO_VERSION)
CXXDEFS  = $(CDEFS)

ifeq ($(HARDWARE_VARIANT), Teensy)
  CDEFS      += -DUSB_SERIAL
  LIB_SRC    += usb.c pins_teensy.c
  LIB_CXXSRC += usb_api.cpp

else ifeq ($(HARDWARE_VARIANT), archim)
  CDEFS      += -DARDUINO_SAM_ARCHIM -DARDUINO_ARCH_SAM -D__SAM3X8E__
  CDEFS      += -DUSB_VID=0x27B1 -DUSB_PID=0x0001 -DUSBCON
  CDEFS      += '-DUSB_MANUFACTURER="UltiMachine"' '-DUSB_PRODUCT_STRING="Archim"'

  LIB_CXXSRC += variant.cpp IPAddress.cpp Reset.cpp RingBuffer.cpp Stream.cpp \
    UARTClass.cpp  USARTClass.cpp abi.cpp new.cpp watchdog.cpp CDC.cpp \
    PluggableUSB.cpp USBCore.cpp

  LIB_SRC    += cortex_handlers.c iar_calls_sam3.c syscalls_sam3.c dtostrf.c itoa.c

  ifeq ($(U8GLIB), 1)
    LIB_SRC += u8g_com_api.c u8g_pb32h1.c
  endif
endif

# Add all the source directories as include directories too
CINCS = ${addprefix -I ,${VPATH}}
CXXINCS = ${addprefix -I ,${VPATH}}

# Silence warnings for library code (won't work for .h files, unfortunately)
LIBWARN = -w -Wno-packed-bitfield-compat

# Compiler flag to set the C/CPP Standard level.
CSTANDARD = -std=gnu99
CXXSTANDARD = -std=gnu++11
CDEBUG = -g$(DEBUG)
CWARN   = -Wall -Wstrict-prototypes -Wno-packed-bitfield-compat -Wno-pragmas -Wunused-parameter
CXXWARN = -Wall                     -Wno-packed-bitfield-compat -Wno-pragmas -Wunused-parameter
CTUNING = -fsigned-char -funsigned-bitfields -fno-exceptions \
          -fshort-enums -ffunction-sections -fdata-sections
ifneq ($(HARDWARE_MOTHERBOARD),)
  CTUNING += -DMOTHERBOARD=${HARDWARE_MOTHERBOARD}
endif

#CEXTRA = -Wa,-adhlns=$(<:.c=.lst)
CXXEXTRA = -fno-use-cxa-atexit -fno-threadsafe-statics -fno-rtti
CFLAGS := $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CEXTRA)   $(CTUNING) $(CSTANDARD)
CXXFLAGS :=         $(CDEFS) $(CINCS) -O$(OPT) $(CXXEXTRA) $(CTUNING) $(CXXSTANDARD)
ASFLAGS :=          $(CDEFS)
#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs

ifeq ($(HARDWARE_VARIANT), archim)
  LD_PREFIX = -Wl,--gc-sections,-Map,Marlin.ino.map,--cref,--check-sections,--entry=Reset_Handler,--unresolved-symbols=report-all,--warn-common,--warn-section-align
  LD_SUFFIX = $(LDLIBS)

  LDFLAGS   = -lm -T$(LDSCRIPT) -u _sbrk -u link -u _close -u _fstat -u _isatty
  LDFLAGS  += -u _lseek -u _read -u _write -u _exit -u kill -u _getpid
else
  LD_PREFIX = -Wl,--gc-sections,--relax
  LDFLAGS   = -lm
  CTUNING   += -flto
endif

# Programming support using avrdude. Settings and variables.
AVRDUDE_PORT = $(UPLOAD_PORT)
AVRDUDE_WRITE_FLASH = -Uflash:w:$(BUILD_DIR)/$(TARGET).hex:i
ifeq ($(shell uname -s), Linux)
  AVRDUDE_CONF = /etc/avrdude/avrdude.conf
else
  AVRDUDE_CONF = $(ARDUINO_INSTALL_DIR)/packages/arduino/tools/avrdude/6.3.0-arduino17/etc/avrdude.conf
endif
AVRDUDE_FLAGS = -D -C$(AVRDUDE_CONF) \
  -p$(PROG_MCU) -P$(AVRDUDE_PORT) -c$(AVRDUDE_PROGRAMMER) \
  -b$(UPLOAD_RATE)

# Since Marlin 2.0, the source files may be distributed into several
# different directories, so it is necessary to find them recursively

SRC    = $(shell find src -name '*.c'   -type f)
CXXSRC = $(shell find src -name '*.cpp' -type f)

# Define all object files.
OBJ  = ${patsubst %.c,   $(BUILD_DIR)/arduino/%.o, ${LIB_SRC}}
OBJ += ${patsubst %.cpp, $(BUILD_DIR)/arduino/%.o, ${LIB_CXXSRC}}
OBJ += ${patsubst %.S,   $(BUILD_DIR)/arduino/%.o, ${LIB_ASRC}}
OBJ += ${patsubst %.c,   $(BUILD_DIR)/%.o, ${SRC}}
OBJ += ${patsubst %.cpp, $(BUILD_DIR)/%.o, ${CXXSRC}}

# Define all listing files.
LST = $(LIB_ASRC:.S=.lst) $(LIB_CXXSRC:.cpp=.lst) $(LIB_SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS   = $(MCU_FLAGS) $(CPU_FLAGS) $(CFLAGS) -I.
ALL_CXXFLAGS = $(MCU_FLAGS) $(CPU_FLAGS) $(CXXFLAGS)
ALL_ASFLAGS  = $(MCU_FLAGS) $(CPU_FLAGS) $(ASFLAGS) -x assembler-with-cpp

# set V=1 (eg, "make V=1") to print the full commands etc.
ifneq ($V,1)
  Pecho=@echo
  P=@
else
  Pecho=@:
  P=
endif

# Create required build hierarchy if it does not exist

$(shell mkdir -p $(dir $(OBJ)))

# Default target.
all: sizeafter

build: elf hex bin

elf: $(BUILD_DIR)/$(TARGET).elf
bin: $(BUILD_DIR)/$(TARGET).bin
hex: $(BUILD_DIR)/$(TARGET).hex
eep: $(BUILD_DIR)/$(TARGET).eep
lss: $(BUILD_DIR)/$(TARGET).lss
sym: $(BUILD_DIR)/$(TARGET).sym

# Program the device.
# Do not try to reset an Arduino if it's not one
upload: $(BUILD_DIR)/$(TARGET).hex
ifeq (${AVRDUDE_PROGRAMMER}, arduino)
	stty hup < $(UPLOAD_PORT); true
endif
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)
ifeq (${AVRDUDE_PROGRAMMER}, arduino)
	stty -hup < $(UPLOAD_PORT); true
endif

# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(BUILD_DIR)/$(TARGET).hex
ELFSIZE = $(SIZE)  $(SIZE_FLAGS) $(BUILD_DIR)/$(TARGET).elf; \
          $(SIZE)  $(BUILD_DIR)/$(TARGET).elf
sizebefore:
	$P if [ -f $(BUILD_DIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(HEXSIZE); echo; fi

sizeafter: build
	$P if [ -f $(BUILD_DIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi


# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
  --change-section-address .data-0x800000 \
  --change-section-address .bss-0x800000 \
  --change-section-address .noinit-0x800000 \
  --change-section-address .eeprom-0x810000


coff: $(BUILD_DIR)/$(TARGET).elf
	$(COFFCONVERT) -O coff-avr $(BUILD_DIR)/$(TARGET).elf $(TARGET).cof


extcoff: $(TARGET).elf
	$(COFFCONVERT) -O coff-ext-avr $(BUILD_DIR)/$(TARGET).elf $(TARGET).cof


.SUFFIXES: .elf .hex .eep .lss .sym .bin
.PRECIOUS: .o

.elf.hex:
	$(Pecho) "  COPY  $@"
	$P $(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

.elf.bin:
	$(Pecho) "  COPY  $@"
	$P $(OBJCOPY) -O binary -R .eeprom $< $@

.elf.eep:
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	  --change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
.elf.lss:
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
.elf.sym:
	$(NM) -n $< > $@

# Link: create ELF output file from library.
LDFLAGS+= -Wl,-V
$(BUILD_DIR)/$(TARGET).elf: $(OBJ) Configuration.h
	$(Pecho) "  CXX   $@"
	$P $(CXX) $(LD_PREFIX) $(ALL_CXXFLAGS) -o $@ -L. $(OBJ) $(LDFLAGS) $(LD_SUFFIX)

# Object files that were found in "src" will be stored in $(BUILD_DIR)
# in directories that mirror the structure of "src"

$(BUILD_DIR)/%.o: %.c Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CC    $<"
	$P $(CC) -MMD -c $(ALL_CFLAGS) $(CWARN) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CXX   $<"
	$P $(CXX) -MMD -c $(ALL_CXXFLAGS) $(CXXWARN) $< -o $@

# Object files for Arduino libs will be created in $(BUILD_DIR)/arduino

$(BUILD_DIR)/arduino/%.o: %.c Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CC    $<"
	$P $(CC) -MMD -c $(ALL_CFLAGS) $(LIBWARN) $< -o $@

$(BUILD_DIR)/arduino/%.o: %.cpp Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CXX   $<"
	$P $(CXX) -MMD -c $(ALL_CXXFLAGS)  $(LIBWARN) $< -o $@

$(BUILD_DIR)/arduino/%.o: %.S $(MAKEFILE)
	$(Pecho) "  CXX   $<"
	$P $(CXX) -MMD -c $(ALL_ASFLAGS) $< -o $@

# Target: clean project.
clean:
	$(Pecho) "  RMDIR $(BUILD_DIR)/"
	$P rm -rf $(BUILD_DIR)


.PHONY: all build elf hex eep lss sym program coff extcoff clean depend sizebefore sizeafter

# Automatically include the dependency files created by gcc
-include ${patsubst %.o, %.d, ${OBJ}}
