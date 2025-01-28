/**
 * \file
 *
 * \brief Declaration of main function used by Composite example 4
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

#ifndef _USB_TASK_H_
#define _USB_TASK_H_

#include "usb_protocol_cdc.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Called by MSC interface
 * Callback running when USB Host enable MSC interface
 *
 * \retval true if MSC startup is ok
 */
bool usb_task_msc_enable(void);

/*! \brief Called by MSC interface
 * Callback running when USB Host disable MSC interface
 */
void usb_task_msc_disable(void);

/*! \brief Opens the communication port
 * This is called by CDC interface when USB Host enable it.
 *
 * \retval true if cdc startup is successfully done
 */
bool usb_task_cdc_enable(const uint8_t port);

/*! \brief Closes the communication port
 * This is called by CDC interface when USB Host disable it.
 */
void usb_task_cdc_disable(const uint8_t port);

/*! \brief Save new DTR state to change led behavior.
 * The DTR notify that the terminal have open or close the communication port.
 */
void usb_task_cdc_set_dtr(const uint8_t port, const bool b_enable);

/*! \brief Check if MSC is enumerated and configured on the PC side
 */
bool usb_task_msc_isenabled(void);

/*! \brief Check if CDC is enumerated and configured on the PC side
 */
bool usb_task_cdc_isenabled(void);

/*! \brief Check if CDC is actually OPEN by an application on the PC side
 *  assuming DTR signal means a program is listening to messages
 */
bool usb_task_cdc_dtr_active(void);

/*! \brief Called by UDC when USB Host request a extra string different
 * of this specified in USB device descriptor
 */
bool usb_task_extra_string(void);

/*! \brief Called by UDC when USB Host performs unknown requests
 */
bool usb_task_other_requests(void);

/*! \brief Called by CDC interface
 * Callback running when CDC device have received data
 */
void usb_task_cdc_rx_notify(const uint8_t port);

/*! \brief Configures communication line
 *
 * \param cfg      line configuration
 */
void usb_task_cdc_config(const uint8_t port, usb_cdc_line_coding_t *cfg);

/*! \brief The USB device interrupt
 */
void USBD_ISR(void);

/*! \brief USB task init
 */
void usb_task_init(void);

/*! \brief USB task idle
 */
void usb_task_idle(void);

#ifdef __cplusplus
}
#endif

#endif // _USB_TASK_H_
