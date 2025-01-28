/**
 * \file
 *
 * \brief USB OTG Driver for UOTGHS.
 *
 * Copyright (c) 2012-2015 Atmel Corporation. All rights reserved.
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

#ifndef UOTGHS_OTG_H_INCLUDED
#define UOTGHS_OTG_H_INCLUDED

#include "compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

//! \ingroup usb_group
//! \defgroup otg_group UOTGHS OTG Driver
//! UOTGHS low-level driver for OTG features
//!
//! @{

/**
 * \brief Initialize the dual role
 * This function is implemented in uotghs_host.c file.
 *
 * \return \c true if the ID pin management has been started, otherwise \c false.
 */
bool otg_dual_enable(void);

/**
 * \brief Uninitialize the dual role
 * This function is implemented in uotghs_host.c file.
 */
void otg_dual_disable(void);

//! @name UOTGHS OTG ID pin management
//! The ID pin come from the USB OTG connector (A and B receptable) and
//! allows to select the USB mode host or device.
//! The UOTGHS hardware can manage it automatically. This feature is optional.
//! When USB_ID_GPIO is defined (in board.h), this feature is enabled.
//!
//! @{
   //! Enable external OTG_ID pin (listened to by USB)
#define otg_enable_id_pin()                 (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_UIDE))
   //! Disable external OTG_ID pin (ignored by USB)
#define otg_disable_id_pin()                (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_UIDE))
   //! Test if external OTG_ID pin enabled (listened to by USB)
#define Is_otg_id_pin_enabled()             (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_UIDE))
   //! Disable external OTG_ID pin and force device mode
#define otg_force_device_mode()             (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_UIMOD), otg_disable_id_pin())
   //! Test if device mode is forced
#define Is_otg_device_mode_forced()         (!Is_otg_id_pin_enabled() && Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_UIMOD))
   //! Disable external OTG_ID pin and force host mode
#define otg_force_host_mode()               (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_UIMOD), otg_disable_id_pin())
   //! Test if host mode is forced
#define Is_otg_host_mode_forced()           (!Is_otg_id_pin_enabled() && !Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_UIMOD))

//! @name UOTGHS OTG ID pin interrupt management
//! These macros manage the ID pin interrupt
//! @{
#define otg_enable_id_interrupt()           (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_IDTE))
#define otg_disable_id_interrupt()          (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_IDTE))
#define Is_otg_id_interrupt_enabled()       (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_IDTE))
#define Is_otg_id_device()                  (Tst_bits(UOTGHS->UOTGHS_SR, UOTGHS_SR_ID))
#define Is_otg_id_host()                    (!Is_otg_id_device())
#define otg_ack_id_transition()             (UOTGHS->UOTGHS_SCR = UOTGHS_SCR_IDTIC)
#define otg_raise_id_transition()           (UOTGHS->UOTGHS_SFR = UOTGHS_SFR_IDTIS)
#define Is_otg_id_transition()              (Tst_bits(UOTGHS->UOTGHS_SR, UOTGHS_SR_IDTI))
//! @}
//! @}

//! @name OTG Vbus management
//! @{
#define otg_enable_vbus_interrupt()         (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_VBUSTE))
#define otg_disable_vbus_interrupt()        (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_VBUSTE))
#define Is_otg_vbus_interrupt_enabled()     (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_VBUSTE))
#define Is_otg_vbus_high()                  (Tst_bits(UOTGHS->UOTGHS_SR, UOTGHS_SR_VBUS))
#define Is_otg_vbus_low()                   (!Is_otg_vbus_high())
#define otg_ack_vbus_transition()           (UOTGHS->UOTGHS_SCR = UOTGHS_SCR_VBUSTIC)
#define otg_raise_vbus_transition()         (UOTGHS->UOTGHS_SFR = UOTGHS_SFR_VBUSTIS)
#define Is_otg_vbus_transition()            (Tst_bits(UOTGHS->UOTGHS_SR, UOTGHS_SR_VBUSTI))
//! @}

//! @name UOTGHS OTG main management
//! These macros allows to enable/disable pad and UOTGHS hardware
//! @{
  //! Reset USB macro
#define otg_reset()                      \
  do {                                   \
    UOTGHS->UOTGHS_CTRL = 0;             \
    while( UOTGHS->UOTGHS_SR & 0x3FFF) { \
      UOTGHS->UOTGHS_SCR = 0xFFFFFFFF;   \
    }                                    \
  } while (0)
  //! Enable USB macro
#define otg_enable()                        (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_USBE))
  //! Disable USB macro
#define otg_disable()                       (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_USBE))
#define Is_otg_enabled()                    (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_USBE))

  //! Enable OTG pad
#define otg_enable_pad()                    (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_OTGPADE))
  //! Disable OTG pad
#define otg_disable_pad()                   (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_OTGPADE))
#define Is_otg_pad_enabled()                (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_OTGPADE))

  //! Check Clock Usable
  //! For parts with HS feature, this one corresponding at UTMI clock
#define Is_otg_clock_usable()               (Tst_bits(UOTGHS->UOTGHS_SR, UOTGHS_SR_CLKUSABLE))

  //! Stop (freeze) internal USB clock
#define otg_freeze_clock()                  (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_FRZCLK))
#define otg_unfreeze_clock()                (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_FRZCLK))
#define Is_otg_clock_frozen()               (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_FRZCLK))

  //! Configure time-out of specified OTG timer
#define otg_configure_timeout(timer, timeout) (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_UNLOCK),\
    Wr_bitfield(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_TIMPAGE_Msk, timer),\
    Wr_bitfield(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_TIMVALUE_Msk, timeout),\
    Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_UNLOCK))
  //! Get configured time-out of specified OTG timer
#define otg_get_timeout(timer)              (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_UNLOCK),\
    Wr_bitfield(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_TIMPAGE_Msk, timer),\
    Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_UNLOCK),\
    Rd_bitfield(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_TIMVALUE_Msk))

  //! Get the dual-role device state of the internal USB finite state machine of the UOTGHS controller
#define otg_get_fsm_drd_state()             (Rd_bitfield(UOTGHS->UOTGHS_FSM, UOTGHS_FSM_DRDSTATE_Msk))
#define Is_otg_a_suspend()                  (4==otg_get_fsm_drd_state())
#define Is_otg_a_wait_vrise()               (1==otg_get_fsm_drd_state())
//! @}

//! @name UOTGHS OTG hardware protocol
//! These macros manages the hardware OTG protocol
//! @{
  //! Initiates a Host negotiation Protocol
#define otg_device_initiate_hnp()             (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_HNPREQ))
  //! Accepts a Host negotiation Protocol
#define otg_host_accept_hnp()                 (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_HNPREQ))
  //! Rejects a Host negotiation Protocol
#define otg_host_reject_hnp()                 (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_HNPREQ))
  //! initiates a Session Request Protocol
#define otg_device_initiate_srp()             (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_SRPREQ))
  //! Selects VBus as SRP method
#define otg_select_vbus_srp_method()          (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_SRPSEL))
#define Is_otg_vbus_srp_method_selected()     (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_SRPSEL))
  //! Selects data line as SRP method
#define otg_select_data_srp_method()          (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_SRPSEL))
#define Is_otg_data_srp_method_selected()     (!Is_otg_vbus_srp_method_selected())
  //! Tests if a HNP occurs
#define Is_otg_hnp()                          (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_HNPREQ))
  //! Tests if a SRP from device occurs
#define Is_otg_device_srp()                   (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_SRPREQ))

  //! Enables HNP error interrupt
#define otg_enable_hnp_error_interrupt()      (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_HNPERRE))
  //! Disables HNP error interrupt
#define otg_disable_hnp_error_interrupt()     (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_HNPERRE))
#define Is_otg_hnp_error_interrupt_enabled()  (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_HNPERRE))
  //! ACKs HNP error interrupt
#define otg_ack_hnp_error_interrupt()         (UOTGHS->UOTGHS_SCR = UOTGHS_SCR_HNPERRIC)
  //! Raises HNP error interrupt
#define otg_raise_hnp_error_interrupt()       (UOTGHS->UOTGHS_SFR = UOTGHS_SFR_HNPERRIS)
  //! Tests if a HNP error occurs
#define Is_otg_hnp_error_interrupt()          (Tst_bits(UOTGHS->UOTGHS_SR, UOTGHS_SR_HNPERRI))

  //! Enables role exchange interrupt
#define otg_enable_role_exchange_interrupt()      (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_ROLEEXE))
  //! Disables role exchange interrupt
#define otg_disable_role_exchange_interrupt()     (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_ROLEEXE))
#define Is_otg_role_exchange_interrupt_enabled()  (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_ROLEEXE))
  //! ACKs role exchange interrupt
#define otg_ack_role_exchange_interrupt()         (UOTGHS->UOTGHS_SCR = UOTGHS_SCR_ROLEEXIC)
  //! Raises role exchange interrupt
#define otg_raise_role_exchange_interrupt()       (UOTGHS->UOTGHS_SFR = UOTGHS_SFR_ROLEEXIS)
  //! Tests if a role exchange occurs
#define Is_otg_role_exchange_interrupt()          (Tst_bits(UOTGHS->UOTGHS_SR, UOTGHS_SR_ROLEEXI))

  //! Enables SRP interrupt
#define otg_enable_srp_interrupt()          (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_SRPE))
  //! Disables SRP interrupt
#define otg_disable_srp_interrupt()         (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_SRPE))
#define Is_otg_srp_interrupt_enabled()      (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_SRPE))
  //! ACKs SRP interrupt
#define otg_ack_srp_interrupt()             (UOTGHS->UOTGHS_SCR = UOTGHS_SCR_SRPIC)
  //! Raises SRP interrupt
#define otg_raise_srp_interrupt()           (UOTGHS->UOTGHS_SFR = UOTGHS_SFR_SRPIS)
  //! Tests if a SRP occurs
#define Is_otg_srp_interrupt()              (Tst_bits(UOTGHS->UOTGHS_SR, UOTGHS_SR_SRPI))
//! @}

//! @}

#ifdef __cplusplus
}
#endif

#endif // UOTGHS_OTG_H_INCLUDED
