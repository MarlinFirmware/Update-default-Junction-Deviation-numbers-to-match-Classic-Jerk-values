/**
 * \file
 *
 * \brief Main functions for USB composite example
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

// Support and FAQ: visit <a href="https://www.atmel.com/design-support/">Atmel Support</a>

#ifdef ARDUINO_ARCH_SAM

#include <Arduino.h>
#include <Reset.h>

#include "conf_usb.h"
#include "udc.h"

#if HAS_MEDIA
  static volatile bool main_b_msc_enable = false;
#endif
static volatile bool main_b_cdc_enable = false;
static volatile bool main_b_dtr_active = false;

void usb_task_idle(void) {
  #if HAS_MEDIA
    // Attend SD card access from the USB MSD -- Prioritize access to improve speed
    int delay = 2;
    while (main_b_msc_enable && --delay > 0) {
      if (udi_msc_process_trans()) delay = 20;

      // Reset the watchdog, just to be sure
      REG_WDT_CR = WDT_CR_WDRSTT | WDT_CR_KEY(0xA5);
    }
  #endif
}

#if HAS_MEDIA
  bool usb_task_msc_enable(void)                { return ((main_b_msc_enable = true)); }
  void usb_task_msc_disable(void)               { main_b_msc_enable = false; }
  bool usb_task_msc_isenabled(void)             { return main_b_msc_enable; }
#endif

bool usb_task_cdc_enable(const uint8_t port)  { UNUSED(port); return ((main_b_cdc_enable = true)); }
void usb_task_cdc_disable(const uint8_t port) { UNUSED(port); main_b_cdc_enable = false; main_b_dtr_active = false; }
bool usb_task_cdc_isenabled(void)             { return main_b_cdc_enable; }

/*! \brief Called by CDC interface
 * Callback running when CDC device have received data
 */
void usb_task_cdc_rx_notify(const uint8_t port) { UNUSED(port); }

/*! \brief Configures communication line
 *
 * \param cfg      line configuration
 */
static uint16_t dwDTERate = 0;
void usb_task_cdc_config(const uint8_t port, usb_cdc_line_coding_t *cfg) {
  UNUSED(port);
  // Store last DTE rate
  dwDTERate = cfg->dwDTERate;
}

void usb_task_cdc_set_dtr(const uint8_t port, const bool b_enable) {
  UNUSED(port);
  // Keep DTR status
  main_b_dtr_active = b_enable;

  //  Implement Arduino-Compatible kludge to enter programming mode from
  // the native port:
  //  "Auto-reset into the bootloader is triggered when the port, already
  // open at 1200 bps, is closed."

  if (1200 == dwDTERate) {
    // We check DTR state to determine if host port is open (bit 0 of lineState).
    if (!b_enable) {

      // Set RST pin to go low for 65535 clock cycles on reset
      //  This helps restarting when firmware flash ends
      RSTC->RSTC_MR = 0xA5000F01;

      // Schedule delayed reset
      initiateReset(250);
    }
    else
      cancelReset();
  }
}

bool usb_task_cdc_dtr_active(void)             { return main_b_dtr_active; }

/// Microsoft WCID descriptor
typedef struct USB_MicrosoftCompatibleDescriptor_Interface {
  uint8_t bFirstInterfaceNumber;
  uint8_t reserved1;
  uint8_t compatibleID[8];
  uint8_t subCompatibleID[8];
  uint8_t reserved2[6];
} __attribute__((packed)) USB_MicrosoftCompatibleDescriptor_Interface;

typedef struct USB_MicrosoftCompatibleDescriptor {
  uint32_t dwLength;
  uint16_t bcdVersion;
  uint16_t wIndex;
  uint8_t bCount;
  uint8_t reserved[7];
  USB_MicrosoftCompatibleDescriptor_Interface interfaces[];
} __attribute__((packed)) USB_MicrosoftCompatibleDescriptor;

// 3D Printer compatible descriptor
static USB_MicrosoftCompatibleDescriptor microsoft_compatible_id_descriptor = {
  .dwLength = sizeof(USB_MicrosoftCompatibleDescriptor) +
              1*sizeof(USB_MicrosoftCompatibleDescriptor_Interface),
  .bcdVersion = 0x0100,
  .wIndex = 0x0004,
  .bCount = 1,
  .reserved = {0, 0, 0, 0, 0, 0, 0},
  .interfaces = {
    {
      .bFirstInterfaceNumber = 0,
      .reserved1 = 1,
      .compatibleID = "3DPRINT",
      .subCompatibleID = {0, 0, 0, 0, 0, 0, 0, 0},
      .reserved2 = {0, 0, 0, 0, 0, 0},
    }
  }
};

#define xstr(s) str(s)
#define str(s) #s

#define MS3DPRINT_CONFIG      u"MS3DPrintConfig"
#define MS3DPRINT_CONFIG_DATA \
  u"Base=SD\0"\
  u"Job3DOutputAreaWidth=" xstr(X_BED_SIZE) "000\0"\
  u"Job3DOutputAreaDepth=" xstr(Y_BED_SIZE) "000\0"\
  u"Job3DOutputAreaHeight=" xstr(Z_MAX_POS) "000\0"\
  u"filamentdiameter=1750\0"

typedef struct USB_MicrosoftExtendedPropertiesDescriptor {
  uint32_t  dwLength;
  uint16_t  bcdVersion;
  uint16_t  wIndex;
  uint16_t  bCount;
  uint32_t  dwPropertySize;
  uint32_t  dwPropertyDataType;
  uint16_t  wPropertyNameLength;
  uint16_t  PropertyName[sizeof(MS3DPRINT_CONFIG)/sizeof(uint16_t)];
  uint32_t  dwPropertyDataLength;
  uint16_t  PropertyData[sizeof(MS3DPRINT_CONFIG_DATA)/sizeof(uint16_t)];
} __attribute__((packed)) USB_MicrosoftExtendedPropertiesDescriptor;

static USB_MicrosoftExtendedPropertiesDescriptor microsoft_extended_properties_descriptor = {
  .dwLength = sizeof(USB_MicrosoftExtendedPropertiesDescriptor),
  .bcdVersion = 0x0100,
  .wIndex = 0x0005,
  .bCount = 1,

  .dwPropertySize = 4 + 4 + 2 + 4 + sizeof(MS3DPRINT_CONFIG) + sizeof(MS3DPRINT_CONFIG_DATA),
  .dwPropertyDataType = 7, // (1=REG_SZ, 4=REG_DWORD, 7=REG_MULTI_SZ)
  .wPropertyNameLength = sizeof(MS3DPRINT_CONFIG),
  .PropertyName = MS3DPRINT_CONFIG,
  .dwPropertyDataLength = sizeof(MS3DPRINT_CONFIG_DATA),
  .PropertyData = MS3DPRINT_CONFIG_DATA
};

/**************************************************************************************************
** WCID configuration information
** Hooked into UDC via UDC_GET_EXTRA_STRING #define.
*/
bool usb_task_extra_string(void) {
  static uint8_t udi_msft_magic[] = "MSFT100\xEE";
  static uint8_t udi_cdc_name[] = "CDC interface";
  #if HAS_MEDIA
    static uint8_t udi_msc_name[] = "MSC interface";
  #endif

  struct extra_strings_desc_t {
    usb_str_desc_t header;
    #if HAS_MEDIA
      le16_t string[Max(Max(sizeof(udi_cdc_name) - 1, sizeof(udi_msc_name) - 1), sizeof(udi_msft_magic) - 1)];
    #else
      le16_t string[Max(sizeof(udi_cdc_name) - 1, sizeof(udi_msft_magic) - 1)];
    #endif
  };
  static UDC_DESC_STORAGE struct extra_strings_desc_t extra_strings_desc = {
    .header.bDescriptorType = USB_DT_STRING
  };

  uint8_t *str;
  uint8_t str_lgt = 0;

  // Link payload pointer to the string corresponding at request
  switch (udd_g_ctrlreq.req.wValue & 0xFF) {
  case UDI_CDC_IAD_STRING_ID:
    str_lgt = sizeof(udi_cdc_name) - 1;
    str = udi_cdc_name;
    break;
  #if HAS_MEDIA
    case UDI_MSC_STRING_ID:
      str_lgt = sizeof(udi_msc_name) - 1;
      str = udi_msc_name;
      break;
  #endif
  case 0xEE:
    str_lgt = sizeof(udi_msft_magic) - 1;
    str = udi_msft_magic;
    break;
  default:
    return false;
  }

  for (uint_fast8_t i = 0; i < str_lgt; i++)
    extra_strings_desc.string[i] = cpu_to_le16((le16_t)str[i]);

  extra_strings_desc.header.bLength = 2 + str_lgt * 2;
  udd_g_ctrlreq.payload_size = extra_strings_desc.header.bLength;
  udd_g_ctrlreq.payload = (uint8_t*)&extra_strings_desc;

  // if the string is larger than request length, then cut it
  if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength) {
    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
  }

  return true;
}

/**************************************************************************************************
** Handle device requests that the ASF stack doesn't
*/
bool usb_task_other_requests(void) {
  uint8_t *ptr = 0;
  uint16_t size = 0;

  if (Udd_setup_type() == USB_REQ_TYPE_VENDOR) {
    //if (udd_g_ctrlreq.req.bRequest == 0x30)
    if (1) {
      if (udd_g_ctrlreq.req.wIndex == 0x04) {
        ptr = (uint8_t*)&microsoft_compatible_id_descriptor;
        size = (udd_g_ctrlreq.req.wLength);
        if (size > microsoft_compatible_id_descriptor.dwLength)
          size = microsoft_compatible_id_descriptor.dwLength;
      }
      else if (udd_g_ctrlreq.req.wIndex == 0x05) {
        ptr = (uint8_t*)&microsoft_extended_properties_descriptor;
        size = (udd_g_ctrlreq.req.wLength);
        if (size > microsoft_extended_properties_descriptor.dwLength)
          size = microsoft_extended_properties_descriptor.dwLength;
      }
      else
        return false;
    }
  }

  udd_g_ctrlreq.payload_size = size;
  if (size == 0) {
    udd_g_ctrlreq.callback = 0;
    udd_g_ctrlreq.over_under_run = 0;
  }
  else
    udd_g_ctrlreq.payload = ptr;

  return true;
}

void usb_task_init(void) {

  uint16_t *ptr;

  // Disable USB peripheral so we start clean and avoid lockups
  otg_disable();
  udd_disable();

  // Set the USB interrupt to our stack
  UDD_SetStack(&USBD_ISR);

  // Start USB stack to authorize VBus monitoring
  udc_start();

  // Patch in filament diameter - Be careful: String is in UNICODE (2bytes per char)
  ptr = &microsoft_extended_properties_descriptor.PropertyData[0];
  while (ptr[0] || ptr[1]) { // Double 0 flags end of resource

    // Found the filamentdiameter= unicode string
    if (ptr[0] == 'r' && ptr[1] == '=') {
      char diam[16];
      char *sptr;

      // Patch in the filament diameter
      itoa((int)((DEFAULT_NOMINAL_FILAMENT_DIA) * 1000), diam, 10);

      // And copy it to the proper place, expanding it to unicode
      sptr = &diam[0];
      ptr += 2;
      while (*sptr) *ptr++ = *sptr++;

      // Done!
      break;
    }

    // Go to the next character
    ptr++;
  }
}

#endif // ARDUINO_ARCH_SAM
