/**
 * \file
 *
 * \brief USB configuration file
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
 * Support and FAQ: visit <a href="https://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef _CONF_USB_H_
#define _CONF_USB_H_

#undef UNUSED                           /* To avoid a macro clash as macros.h already defines it */
#include "../../../inc/MarlinConfigPre.h"
#include "compiler.h"

/**
 * USB Device Configuration
 * @{
 */

//! Device definition (mandatory)
#define  USB_DEVICE_MAJOR_VERSION         1
#define  USB_DEVICE_MINOR_VERSION         0
#define  USB_DEVICE_POWER                 100 // Consumption on Vbus line (mA)
#define  USB_DEVICE_ATTR                  \
  (USB_CONFIG_ATTR_SELF_POWERED)
// (USB_CONFIG_ATTR_BUS_POWERED)
//  (USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_SELF_POWERED)
//  (USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_BUS_POWERED)

/**
 * Device speeds support
 * Low speed not supported by CDC and MSC
 * @{
 */

//! To define a Low speed device
//#define  USB_DEVICE_LOW_SPEED

//! To define a Full speed device
//#define USB_DEVICE_FULL_SPEED

//! To authorize the High speed
#ifndef USB_DEVICE_FULL_SPEED
  #if (UC3A3||UC3A4)
    #define  USB_DEVICE_HS_SUPPORT
  #elif (SAM3XA||SAM3U)
    #define  USB_DEVICE_HS_SUPPORT
  #endif
#endif
//@}

/**
 * USB Device Callbacks definitions (Optional)
 * @{
 */
#define  UDC_VBUS_EVENT(b_vbus_high)
#define  UDC_SOF_EVENT()
#define  UDC_SUSPEND_EVENT()
#define  UDC_RESUME_EVENT()
#define  UDC_GET_EXTRA_STRING()         usb_task_extra_string()
#define  USB_DEVICE_SPECIFIC_REQUEST()  usb_task_other_requests()
//@}

#if HAS_MEDIA
  /**
   * USB Device low level configuration
   * When only one interface is used, these configurations are defined by the class module.
   * For composite device, these configuration must be defined here
   * @{
   */
  //! Control endpoint size
  #define  USB_DEVICE_EP_CTRL_SIZE       64

  //! Two interfaces for this device (CDC COM + CDC DATA + MSC)
  #define  USB_DEVICE_NB_INTERFACE       3

  //! 5 endpoints used by CDC and MSC interfaces
  #if SAM3U
  // (3 | USB_EP_DIR_IN)  // CDC Notify endpoint
  // (6 | USB_EP_DIR_IN)  // CDC TX
  // (5 | USB_EP_DIR_OUT) // CDC RX
  // (1 | USB_EP_DIR_IN)  // MSC IN
  // (2 | USB_EP_DIR_OUT) // MSC OUT
  #  define  USB_DEVICE_MAX_EP           6
  #  if defined(USB_DEVICE_HS_SUPPORT)
  // In HS mode, size of bulk endpoints are 512
  // If CDC and MSC endpoints all uses 2 banks, DPRAM is not enough: 4 bulk
  // endpoints requires 4K bytes. So reduce the number of banks of CDC bulk
  // endpoints to use less DPRAM. Keep MSC setting to keep MSC performance.
  #     define  UDD_BULK_NB_BANK(ep) ((ep == 5 || ep== 6) ? 1 : 2)
  #endif
  #else
  // (3 | USB_EP_DIR_IN)  // CDC Notify endpoint
  // (4 | USB_EP_DIR_IN)  // CDC TX
  // (5 | USB_EP_DIR_OUT) // CDC RX
  // (1 | USB_EP_DIR_IN)  // MSC IN
  // (2 | USB_EP_DIR_OUT) // MSC OUT
  #  define  USB_DEVICE_MAX_EP           5
  #  if SAM3XA && defined(USB_DEVICE_HS_SUPPORT)
  // In HS mode, size of bulk endpoints are 512
  // If CDC and MSC endpoints all uses 2 banks, DPRAM is not enough: 4 bulk
  // endpoints requires 4K bytes. So reduce the number of banks of CDC bulk
  // endpoints to use less DPRAM. Keep MSC setting to keep MSC performance.
  #     define  UDD_BULK_NB_BANK(ep) ((ep == 4 || ep== 5) ? 1 : 2)
  #  endif
  #endif
#endif

//@}

//@}

/**
 * USB Interface Configuration
 * @{
 */
/**
 * Configuration of CDC interface
 * @{
 */

//! Define one USB communication ports
#define  UDI_CDC_PORT_NB 1

//! Interface callback definition
#define  UDI_CDC_ENABLE_EXT(port)         usb_task_cdc_enable(port)
#define  UDI_CDC_DISABLE_EXT(port)        usb_task_cdc_disable(port)
#define  UDI_CDC_RX_NOTIFY(port)          usb_task_cdc_rx_notify(port)
#define  UDI_CDC_TX_EMPTY_NOTIFY(port)
#define  UDI_CDC_SET_CODING_EXT(port,cfg) usb_task_cdc_config(port,cfg)
#define  UDI_CDC_SET_DTR_EXT(port,set)    usb_task_cdc_set_dtr(port,set)
#define  UDI_CDC_SET_RTS_EXT(port,set)

//! Define it when the transfer CDC Device to Host is a low rate (<512000 bauds)
//! to reduce CDC buffers size
//#define  UDI_CDC_LOW_RATE

//! Default configuration of communication port
#define  UDI_CDC_DEFAULT_RATE             115200
#define  UDI_CDC_DEFAULT_STOPBITS         CDC_STOP_BITS_1
#define  UDI_CDC_DEFAULT_PARITY           CDC_PAR_NONE
#define  UDI_CDC_DEFAULT_DATABITS         8

//! Enable id string of interface to add an extra USB string
#define  UDI_CDC_IAD_STRING_ID            4

#if HAS_MEDIA
  /**
   * USB CDC low level configuration
   * In standalone these configurations are defined by the CDC module.
   * For composite device, these configuration must be defined here
   * @{
   */
  //! Endpoint numbers definition
  #if SAM3U
  #  define  UDI_CDC_COMM_EP_0             (3 | USB_EP_DIR_IN) // Notify endpoint
  #  define  UDI_CDC_DATA_EP_IN_0          (6 | USB_EP_DIR_IN) // TX
  #  define  UDI_CDC_DATA_EP_OUT_0         (5 | USB_EP_DIR_OUT)// RX
  #else
  #  define  UDI_CDC_COMM_EP_0             (3 | USB_EP_DIR_IN) // Notify endpoint
  #  define  UDI_CDC_DATA_EP_IN_0          (4 | USB_EP_DIR_IN) // TX
  #  define  UDI_CDC_DATA_EP_OUT_0         (5 | USB_EP_DIR_OUT)// RX
  #endif

  //! Interface numbers
  #define  UDI_CDC_COMM_IFACE_NUMBER_0   0
  #define  UDI_CDC_DATA_IFACE_NUMBER_0   1

  //@}
  //@}

  /**
   * Configuration of MSC interface
   * @{
   */
  //! Vendor name and Product version of MSC interface
  #define UDI_MSC_GLOBAL_VENDOR_ID            \
     'M', 'A', 'R', 'L', 'I', 'N', '3', 'D'
  #define UDI_MSC_GLOBAL_PRODUCT_VERSION            \
     '1', '.', '0', '0'

  //! Interface callback definition
  #define  UDI_MSC_ENABLE_EXT()          usb_task_msc_enable()
  #define  UDI_MSC_DISABLE_EXT()         usb_task_msc_disable()

  //! Enable id string of interface to add an extra USB string
  #define  UDI_MSC_STRING_ID             5

  /**
   * USB MSC low level configuration
   * In standalone these configurations are defined by the MSC module.
   * For composite device, these configuration must be defined here
   * @{
   */
  //! Endpoint numbers definition
  #define  UDI_MSC_EP_IN                 (1 | USB_EP_DIR_IN)
  #define  UDI_MSC_EP_OUT                (2 | USB_EP_DIR_OUT)

  //! Interface number
  #define  UDI_MSC_IFACE_NUMBER          2
  //@}
  //@}

  //@}

  /**
   * Description of Composite Device
   * @{
   */
  //! USB Interfaces descriptor structure
  #define UDI_COMPOSITE_DESC_T \
    usb_iad_desc_t       udi_cdc_iad; \
    udi_cdc_comm_desc_t  udi_cdc_comm; \
    udi_cdc_data_desc_t  udi_cdc_data; \
    udi_msc_desc_t       udi_msc

  //! USB Interfaces descriptor value for Full Speed
  #define UDI_COMPOSITE_DESC_FS \
    .udi_cdc_iad   = UDI_CDC_IAD_DESC_0, \
    .udi_cdc_comm  = UDI_CDC_COMM_DESC_0, \
    .udi_cdc_data  = UDI_CDC_DATA_DESC_0_FS, \
    .udi_msc       = UDI_MSC_DESC_FS

  //! USB Interfaces descriptor value for High Speed
  #define UDI_COMPOSITE_DESC_HS \
    .udi_cdc_iad   = UDI_CDC_IAD_DESC_0, \
    .udi_cdc_comm  = UDI_CDC_COMM_DESC_0, \
    .udi_cdc_data  = UDI_CDC_DATA_DESC_0_HS, \
    .udi_msc       = UDI_MSC_DESC_HS

  //! USB Interface APIs
  #define UDI_COMPOSITE_API \
    &udi_api_cdc_comm, \
    &udi_api_cdc_data, \
    &udi_api_msc
  //@}

  /**
   * USB Device Driver Configuration
   * @{
   */
  //@}

  //! The includes of classes and other headers must be done at the end of this file to avoid compile error
  #include "udi_cdc.h"
  #include "udi_msc.h"
#else
  #include "udi_cdc_conf.h"
#endif

#include "usb_task.h"

#endif // _CONF_USB_H_
