/**
 * Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Contact information
 * -------------------
 *
 * Circuits At Home, LTD
 * Web      :  https://www.circuitsathome.com
 * e-mail   :  support@circuitsathome.com
 */

#include "../../../inc/MarlinConfigPre.h"

#if HAS_USB_FLASH_DRIVE && DISABLED(USE_UHS3_USB)

#include "Usb.h"

// 0x80 is the default (i.e. trace) to turn off set this global to something lower.
// this allows for 126 other debugging levels.
// TO-DO: Allow assignment to a different serial port by software
int UsbDEBUGlvl = 0x80;

void E_Notifyc(char c, int lvl) {
  if (UsbDEBUGlvl < lvl) return;
  USB_HOST_SERIAL.print(c
    #if !defined(ARDUINO) && !defined(ARDUINO_ARCH_LPC176X)
      , BYTE
    #endif
  );
  //USB_HOST_SERIAL.flush();
}

void E_Notify(char const * msg, int lvl) {
  if (UsbDEBUGlvl < lvl) return;
  if (!msg) return;
  while (const char c = pgm_read_byte(msg++)) E_Notifyc(c, lvl);
}

void E_NotifyStr(char const * msg, int lvl) {
  if (UsbDEBUGlvl < lvl) return;
  if (!msg) return;
  while (const char c = *msg++) E_Notifyc(c, lvl);
}

void E_Notify(uint8_t b, int lvl) {
  if (UsbDEBUGlvl < lvl) return;
  USB_HOST_SERIAL.print(b
    #if !defined(ARDUINO) && !defined(ARDUINO_ARCH_LPC176X)
      , DEC
    #endif
  );
  //USB_HOST_SERIAL.flush();
}

void E_Notify(double d, int lvl) {
  if (UsbDEBUGlvl < lvl) return;
  USB_HOST_SERIAL.print(d);
  //USB_HOST_SERIAL.flush();
}

#ifdef DEBUG_USB_HOST

  void NotifyFailGetDevDescr() {
    Notify(PSTR("\r\ngetDevDescr "), 0x80);
  }

  void NotifyFailSetDevTblEntry() {
    Notify(PSTR("\r\nsetDevTblEn "), 0x80);
  }

  void NotifyFailGetConfDescr() {
    Notify(PSTR("\r\ngetConf "), 0x80);
  }

  void NotifyFailSetConfDescr() {
    Notify(PSTR("\r\nsetConf "), 0x80);
  }

  void NotifyFailGetDevDescr(uint8_t reason) {
    NotifyFailGetDevDescr();
    NotifyFail(reason);
  }

  void NotifyFailSetDevTblEntry(uint8_t reason) {
    NotifyFailSetDevTblEntry();
    NotifyFail(reason);

  }

  void NotifyFailGetConfDescr(uint8_t reason) {
    NotifyFailGetConfDescr();
    NotifyFail(reason);
  }

  void NotifyFailSetConfDescr(uint8_t reason) {
    NotifyFailSetConfDescr();
    NotifyFail(reason);
  }

  void NotifyFailUnknownDevice(uint16_t VID, uint16_t PID) {
    Notify(PSTR("\r\nUnknown Device Connected - VID: "), 0x80);
    D_PrintHex<uint16_t > (VID, 0x80);
    Notify(PSTR(" PID: "), 0x80);
    D_PrintHex<uint16_t > (PID, 0x80);
  }

  void NotifyFail(uint8_t rcode) {
    D_PrintHex<uint8_t > (rcode, 0x80);
    Notify(PSTR("\r\n"), 0x80);
  }

#endif // DEBUG_USB_HOST

#endif // HAS_USB_FLASH_DRIVE && !USE_UHS3_USB
