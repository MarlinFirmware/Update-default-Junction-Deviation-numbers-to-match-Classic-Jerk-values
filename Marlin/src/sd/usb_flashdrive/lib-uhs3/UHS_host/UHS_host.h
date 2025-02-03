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

// USB functions
#ifndef _UHS_host_h_
#define _UHS_host_h_

// WARNING: Do not change the order of includes, or stuff will break!
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>

#if DISABLED(USE_UHS3_USB)
#include <ISR_safe_memory.h>
#include <Wire.h>
#include <SPI.h>
#include <UHS_ByteBuffer.h>
#endif
#include "UHS_macros.h"

// None of these should ever be directly included by a driver, or a user's sketch.
#include "../dyn_SWI/dyn_SWI.h"
#include "UHS_USB_IDs.h"
#include "UHS_settings.h"
#include "UHS_usb_ch9.h"
#include "UHS_UsbCore.h"
#include "UHS_address.h"
#include "UHS_usbhost.h"
#include "UHS_printhex.h"
#include "UHS_message.h"

/**
 * Load system components as required
 */
#if defined(LOAD_USB_HOST_SYSTEM) && !defined(USB_HOST_SYSTEM_LOADED)
#include "UHS_util_INLINE.h"
#include "UHS_host_INLINE.h"
#include "UHS_printf_HELPER.h"

#ifdef LOAD_USB_HOST_SHIELD
#include "USB_HOST_SHIELD/USB_HOST_SHIELD.h"
#endif

#if defined(LOAD_UHS_KINETIS_FS_HOST) && !defined(UHS_KINETIS_FS_HOST_LOADED)
#include "UHS_KINETIS_FS_HOST/UHS_KINETIS_FS_HOST.h"
#endif

#if defined(LOAD_UHS_KINETIS_EHCI) && !defined(UHS_KINETIS_EHCI_LOADED)
#include "UHS_KINETIS_EHCI/UHS_KINETIS_EHCI.h"
#endif

/**
 * Load USB drivers and multiplexers
 */
#ifdef LOAD_UHS_HUB
#include "UHS_HUB/UHS_HUB.h"
#endif // HUB loaded

#ifdef LOAD_UHS_BULK_STORAGE
#include "UHS_BULK_STORAGE/UHS_BULK_STORAGE.h"
#endif

#ifdef LOAD_GENERIC_STORAGE
#include "../UHS_FS/UHS_FS.h"
#endif

// Add BT and optionally HID if directed to do so
#ifdef LOAD_UHS_BT
#include "UHS_BT/UHS_BT.h"
#endif // BT and optionally HID loaded

// Add HID
#ifdef LOAD_UHS_HID
#include "UHS_HID/UHS_HID.h"
#endif // HID loaded

// Add CDC multiplexers (currently only ACM)
#if defined(LOAD_UHS_CDC_ACM) || defined(LOAD_UHS_CDC_ACM_FTDI) || defined(LOAD_UHS_CDC_ACM_PROLIFIC) || defined(LOAD_UHS_CDC_ACM_XR21B1411)
#include "UHS_CDC/UHS_CDC.h"
#endif // CDC loaded

#ifdef LOAD_UHS_ADK
#include "UHS_ADK/UHS_ADK.h"
#endif

#ifdef LOAD_UHS_MIDI
#include "UHS_MIDI/UHS_MIDI.h"
#endif

#endif // System code loaded

#endif // _UHS_host_h_
