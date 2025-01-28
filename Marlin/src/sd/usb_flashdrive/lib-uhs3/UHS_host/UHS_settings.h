/* Copyright (C) 2015-2016 Andrew J. Kroll
   and
Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Contact information
-------------------

Circuits At Home, LTD
Web      :  https://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */

#ifndef UHS_SETTINGS_H
#define UHS_SETTINGS_H

// TODO: Move specific settings to modules which use them.

////////////////////////////////////////////////////////////////////////////////
// Define any of these options at the top of your sketch to override
// the defaults contained herewith. Do NOT do modifications here.
// Individual Components have their own settings.
//
// Macro                        | Settings and notes    | Default
// -----------------------------+-----------------------+-----------------------
//                              |  Any class that does  |
// USB_HOST_SERIAL              |  text streaming       | SERIAL_PORT_MONITOR
//                              |  e.g. Serial2         |
// -----------------------------+-----------------------+-----------------------
// ENABLE_UHS_DEBUGGING         | 0 = off, 1 = on       | 0
// -----------------------------+-----------------------+-----------------------
//                              | 0 = off, 1 = on       |
//                              | Caution! Can make     |
// DEBUG_PRINTF_EXTRA_HUGE      | program too large!    | 0
//                              | Other modules depend  |
//                              | on this setting.      |
// -----------------------------+-----------------------+-----------------------
// USE_UHS_BLACK_WIDDOW         | 0 = no, 1 = yes       | 0
// -----------------------------+-----------------------+-----------------------
// ENABLE_WII_IR_CAMERA         | 0 = no, 1 = yes       | 0
// -----------------------------^-----------------------^-----------------------
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// DEBUGGING
////////////////////////////////////////////////////////////////////////////////

#ifndef USB_HOST_SERIAL
#if defined(SERIAL_PORT_USBVIRTUAL) && defined(LOAD_UHS_KINETIS_FS_HOST)
#define USB_HOST_SERIAL SERIAL_PORT_HARDWARE
#else
#define USB_HOST_SERIAL SERIAL_PORT_MONITOR
#endif
#endif

#ifndef ENABLE_UHS_DEBUGGING
#define ENABLE_UHS_DEBUGGING 0
#endif

#ifndef DEBUG_PRINTF_EXTRA_HUGE
#define DEBUG_PRINTF_EXTRA_HUGE 0
#endif

////////////////////////////////////////////////////////////////////////////////
// Manual board activation
////////////////////////////////////////////////////////////////////////////////

/* Set this to 1 if you are using a Black Widdow */
#ifndef USE_UHS_BLACK_WIDDOW
#define USE_UHS_BLACK_WIDDOW 0
#endif

////////////////////////////////////////////////////////////////////////////////
// Wii IR camera
////////////////////////////////////////////////////////////////////////////////

/* Set this to 1 to activate code for the Wii IR camera */
#ifndef ENABLE_WII_IR_CAMERA
#define ENABLE_WII_IR_CAMERA 0
#endif

////////////////////////////////////////////////////////////////////////////////
// Set to 1 to use the faster spi4teensy3 driver. (not used yet))
////////////////////////////////////////////////////////////////////////////////
#ifndef USE_SPI4TEENSY3
#define USE_SPI4TEENSY3 0
#endif

////////////////////////////////////////////////////////////////////////////////
// AUTOMATIC Settings
////////////////////////////////////////////////////////////////////////////////

// No user serviceable parts below this line.
// DO NOT change anything below here unless you are a developer!

#if defined(__GNUC__) && defined(__AVR__)
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif
#if GCC_VERSION < 40602 // Test for GCC < 4.6.2
#ifdef PROGMEM
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data"))) // Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734#c4
#ifdef PSTR
#undef PSTR
#define PSTR(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0];})) // Copied from pgmspace.h in avr-libc source
#endif
#endif
#endif
#endif

#if !defined(DEBUG_USB_HOST) && ENABLE_UHS_DEBUGGING
#define DEBUG_USB_HOST
#endif

#if !defined(WIICAMERA) && ENABLE_WII_IR_CAMERA
#define WIICAMERA
#endif

#define UHS_SLEEP_MS(v) pUsb->sof_delay(v)

#ifndef UHS_NI
#define UHS_NI __attribute__((noinline))
#endif

#endif // SETTINGS_H
