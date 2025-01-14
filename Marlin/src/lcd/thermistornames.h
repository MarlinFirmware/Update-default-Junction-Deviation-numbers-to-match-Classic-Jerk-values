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
 * thermistornames.h
 *
 * Used by LCD code to obtain a thermistor name
 *
 * Usage: Set THERMISTOR_ID then #include this file
 * to set a new value for THERMISTOR_NAME.
 */

#undef THERMISTOR_NAME

// User-specified thermistor parameters
#if THERMISTOR_ID == 1000
  #define THERMISTOR_NAME "User Parameters"

// SPI RTD/Thermocouple Boards
#elif THERMISTOR_ID == -5
  #define THERMISTOR_NAME "MAX31865"
#elif THERMISTOR_ID == -3
  #define THERMISTOR_NAME "MAX31855"
#elif THERMISTOR_ID == -2
  #define THERMISTOR_NAME "MAX6675"

// Analog Thermocouple Boards
#elif THERMISTOR_ID == -4
  #define THERMISTOR_NAME "AD8495"
#elif THERMISTOR_ID == -1
  #define THERMISTOR_NAME "AD595"

// Standard thermistors
#elif THERMISTOR_ID == 1
  #define THERMISTOR_NAME "EPCOS 100K"
#elif THERMISTOR_ID == 331
  #define THERMISTOR_NAME "3.3V EPCOS 100K (MEGA)"
#elif THERMISTOR_ID == 332
  #define THERMISTOR_NAME "3.3V EPCOS 100K (DUE)"
#elif THERMISTOR_ID == 2
  #define THERMISTOR_NAME "ATC 204GT-2"
#elif THERMISTOR_ID == 202
  #define THERMISTOR_NAME "200k Copymaster 3D"
#elif THERMISTOR_ID == 3
  #define THERMISTOR_NAME "Mendel-parts"
#elif THERMISTOR_ID == 4
  #define THERMISTOR_NAME "Generic 10K"
#elif THERMISTOR_ID == 5
  #define THERMISTOR_NAME "ATC 104GT-2"
#elif THERMISTOR_ID == 501
  #define THERMISTOR_NAME "Zonestar (Tronxy X3A)"
#elif THERMISTOR_ID == 502
  #define THERMISTOR_NAME "Zonestar (P802M Hot Bed)"
#elif THERMISTOR_ID == 503
  #define THERMISTOR_NAME "Zonestar (Z8XM2 Bed)"
#elif THERMISTOR_ID == 504
  #define THERMISTOR_NAME "Zonestar (P802QR2 Hot End)"
#elif THERMISTOR_ID == 505
  #define THERMISTOR_NAME "Zonestar (P802QR2 Bed)"
#elif THERMISTOR_ID == 512
  #define THERMISTOR_NAME "RPW-Ultra"
#elif THERMISTOR_ID == 6
  #define THERMISTOR_NAME "EPCOS (alt)"
#elif THERMISTOR_ID == 7
  #define THERMISTOR_NAME "HW 104LAG"
#elif THERMISTOR_ID == 71
  #define THERMISTOR_NAME "HW 104LAF"
#elif THERMISTOR_ID == 8
  #define THERMISTOR_NAME "E3104FXT"
#elif THERMISTOR_ID == 9
  #define THERMISTOR_NAME "GE AL03006"
#elif THERMISTOR_ID == 10
  #define THERMISTOR_NAME "RS 198-961"
#elif THERMISTOR_ID == 11
  #define THERMISTOR_NAME "1% beta 3950"
#elif THERMISTOR_ID == 12
  #define THERMISTOR_NAME "E3104FXT (alt)"
#elif THERMISTOR_ID == 13
  #define THERMISTOR_NAME "Hisens 3950"
#elif THERMISTOR_ID == 14
  #define THERMISTOR_NAME "100k Ender-5 S1"
#elif THERMISTOR_ID == 15
  #define THERMISTOR_NAME "100k JGAurora A5"
#elif THERMISTOR_ID == 17
  #define THERMISTOR_NAME "100k Dagoma NTC"
#elif THERMISTOR_ID == 18
  #define THERMISTOR_NAME "ATC Semitec 204GT-2"
#elif THERMISTOR_ID == 22
  #define THERMISTOR_NAME "GTM32 Pro vB (hotend)"
#elif THERMISTOR_ID == 23
  #define THERMISTOR_NAME "GTM32 Pro vB (bed)"
#elif THERMISTOR_ID == 60
  #define THERMISTOR_NAME "Makers Tool"
#elif THERMISTOR_ID == 70
  #define THERMISTOR_NAME "Hephestos 2"
#elif THERMISTOR_ID == 75
  #define THERMISTOR_NAME "MGB18"

// Analog Thermistors - 1kΩ pullup
#elif THERMISTOR_ID == 51
  #define THERMISTOR_NAME "EPCOS 1K"
#elif THERMISTOR_ID == 52
  #define THERMISTOR_NAME "ATC204GT-2 1K"
#elif THERMISTOR_ID == 55
  #define THERMISTOR_NAME "ATC104GT-2 1K"

// Analog Thermistors - 10kΩ pullup - Atypical
#elif THERMISTOR_ID == 99
  #define THERMISTOR_NAME "100k with 10k pull-up"

// Analog RTDs (Pt100/Pt1000)
#elif THERMISTOR_ID == 110
  #define THERMISTOR_NAME "Pt100 1K"
#elif THERMISTOR_ID == 147
  #define THERMISTOR_NAME "Pt100 4K7"
#elif THERMISTOR_ID == 1010
  #define THERMISTOR_NAME "PT1000 1K"
#elif THERMISTOR_ID == 1022
  #define THERMISTOR_NAME "PT1000 2K2"
#elif THERMISTOR_ID == 1047
  #define THERMISTOR_NAME "PT1000 4K7"
#elif THERMISTOR_ID == 20
  #define THERMISTOR_NAME "Pt100 UltiMB 5v"
#elif THERMISTOR_ID == 21
  #define THERMISTOR_NAME "Pt100 UltiMB 3.3v"
#elif THERMISTOR_ID == 201
  #define THERMISTOR_NAME "Pt100 OverLord"

// Modified thermistors
#elif THERMISTOR_ID == 30
  #define THERMISTOR_NAME "Kis3d EN AW NTC100K/3950"
#elif THERMISTOR_ID == 666
  #define THERMISTOR_NAME "Einstart S"
#elif THERMISTOR_ID == 2000
  #define THERMISTOR_NAME "TDK NTCG104LH104JT1"

// High Temperature thermistors
#elif THERMISTOR_ID == 61
  #define THERMISTOR_NAME "Formbot 350°C"
#elif THERMISTOR_ID == 66
  #define THERMISTOR_NAME "Dyze / TL 4.7M"
#elif THERMISTOR_ID == 67
  #define THERMISTOR_NAME "SliceEng 450°C"
#elif THERMISTOR_ID == 68
  #define THERMISTOR_NAME "PT-100 + Dyze amp"

// Dummies for dev testing
#elif THERMISTOR_ID == 998
  #define THERMISTOR_NAME "Dummy 1"
#elif THERMISTOR_ID == 999
  #define THERMISTOR_NAME "Dummy 2"
#elif THERMISTOR_ID == 1000
  #define THERMISTOR_NAME "Custom"

// SOC thermistor`
#elif THERMISTOR_ID == 100
  #define THERMISTOR_NAME "SOC"

#endif // THERMISTOR_ID

#undef THERMISTOR_ID
