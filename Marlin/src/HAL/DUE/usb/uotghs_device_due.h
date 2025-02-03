/**
 * \file
 *
 * \brief USB Device Driver for UOTGHS. Compliant with common UDD driver.
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
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

#ifndef UOTGHS_DEVICE_DUE_H_INCLUDED
#define UOTGHS_DEVICE_DUE_H_INCLUDED

//#include "compiler.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

//! \ingroup udd_group
//! \defgroup udd_udphs_group USB On-The-Go High-Speed Port for device mode (UOTGHS)
//! UOTGHS low-level driver for USB device mode
//!
//! @{

#ifndef UOTGHS_DEVEPTCFG_EPDIR_Pos
// Bit pos is not defined in SAM header file but we need it.
# define UOTGHS_DEVEPTCFG_EPDIR_Pos 8
#endif

//! @name UOTGHS Device IP properties
//! These macros give access to IP properties
//! @{
  //! Get maximal number of endpoints
#define udd_get_endpoint_max_nbr()             (9)
#define UDD_MAX_PEP_NB                         (udd_get_endpoint_max_nbr() + 1)
  //! Get maximal number of banks of endpoints
#define udd_get_endpoint_bank_max_nbr(ep)      ((ep == 0) ? 1 : (( ep <= 2) ? 3 : 2))
  //! Get maximal size of endpoint (3X, 1024/64)
#define udd_get_endpoint_size_max(ep)          (((ep) == 0) ? 64 : 1024)
  //! Get DMA support of endpoints
#define Is_udd_endpoint_dma_supported(ep)      ((((ep) >= 1) && ((ep) <= 6)) ? true : false)
  //! Get High Band Width support of endpoints
#define Is_udd_endpoint_high_bw_supported(ep)  (((ep) >= 2) ? true : false)
//! @}

//! @name UOTGHS Device speeds management
//! @{
  //! Enable/disable device low-speed mode
#define udd_low_speed_enable()               (Set_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_LS))
#define udd_low_speed_disable()              (Clr_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_LS))
  //! Test if device low-speed mode is forced
#define Is_udd_low_speed_enable()            (Tst_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_LS))

#ifdef UOTGHS_DEVCTRL_SPDCONF_HIGH_SPEED
  //! Enable high speed mode
# define udd_high_speed_enable()          (Wr_bitfield(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_SPDCONF_Msk, 0))
  //! Disable high speed mode
# define udd_high_speed_disable()         (Wr_bitfield(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_SPDCONF_Msk, 3))
  //! Test if controller is in full speed mode
# define Is_udd_full_speed_mode()         (Rd_bitfield(UOTGHS->UOTGHS_SR, UOTGHS_SR_SPEED_Msk) == UOTGHS_SR_SPEED_FULL_SPEED)
#else
# define udd_high_speed_enable()          do { } while (0)
# define udd_high_speed_disable()         do { } while (0)
# define Is_udd_full_speed_mode()         true
#endif
//! @}

//! @name UOTGHS Device HS test mode management
//! @{
#ifdef UOTGHS_DEVCTRL_SPDCONF_HIGH_SPEED
  //! Enable high speed test mode
# define udd_enable_hs_test_mode()        (Wr_bitfield(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_SPDCONF_Msk, 2))
# define udd_enable_hs_test_mode_j()      (Set_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_TSTJ))
# define udd_enable_hs_test_mode_k()      (Set_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_TSTK))
# define udd_enable_hs_test_mode_packet() (Set_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_TSTPCKT))
#endif
//! @}

//! @name UOTGHS Device vbus management
//! @{
#define udd_enable_vbus_interrupt()       (Set_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_VBUSTE))
#define udd_disable_vbus_interrupt()      (Clr_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_VBUSTE))
#define Is_udd_vbus_interrupt_enabled()   (Tst_bits(UOTGHS->UOTGHS_CTRL, UOTGHS_CTRL_VBUSTE))
#define Is_udd_vbus_high()                (Tst_bits(UOTGHS->UOTGHS_SR, UOTGHS_SR_VBUS))
#define Is_udd_vbus_low()                 (!Is_udd_vbus_high())
#define udd_ack_vbus_transition()         (UOTGHS->UOTGHS_SCR = UOTGHS_SCR_VBUSTIC)
#define udd_raise_vbus_transition()       (UOTGHS->UOTGHS_SFR = UOTGHS_SFR_VBUSTIS)
#define Is_udd_vbus_transition()          (Tst_bits(UOTGHS->UOTGHS_SR, UOTGHS_SR_VBUSTI))
//! @}

//! @name UOTGHS device attach control
//! These macros manage the UOTGHS Device attach.
//! @{
  //! Detaches from USB bus
#define udd_detach_device()               (Set_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_DETACH))
  //! Attaches to USB bus
#define udd_attach_device()               (Clr_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_DETACH))
  //! Test if the device is detached
#define Is_udd_detached()                 (Tst_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_DETACH))
//! @}

//! @name UOTGHS device bus events control
//! These macros manage the UOTGHS Device bus events.
//! @{

//! Initiates a remote wake-up event
//! @{
#define udd_initiate_remote_wake_up()     (Set_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_RMWKUP))
#define Is_udd_pending_remote_wake_up()   (Tst_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_RMWKUP))
//! @}

//! Manage upstream resume event (=remote wakeup)
//! The USB driver sends a resume signal called "Upstream Resume"
//! @{
#define udd_enable_remote_wake_up_interrupt()     (UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_UPRSMES)
#define udd_disable_remote_wake_up_interrupt()    (UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_UPRSMEC)
#define Is_udd_remote_wake_up_interrupt_enabled() (Tst_bits(UOTGHS->UOTGHS_DEVIMR, UOTGHS_DEVIMR_UPRSME))
#define udd_ack_remote_wake_up_start()            (UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVICR_UPRSMC)
#define udd_raise_remote_wake_up_start()          (UOTGHS->UOTGHS_DEVIFR = UOTGHS_DEVIFR_UPRSMS)
#define Is_udd_remote_wake_up_start()             (Tst_bits(UOTGHS->UOTGHS_DEVISR, UOTGHS_DEVISR_UPRSM))
//! @}

//! Manage downstream resume event (=remote wakeup from host)
//! The USB controller detects a valid "End of Resume" signal initiated by the host
//! @{
#define udd_enable_resume_interrupt()             (UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_EORSMES)
#define udd_disable_resume_interrupt()            (UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_EORSMEC)
#define Is_udd_resume_interrupt_enabled()         (Tst_bits(UOTGHS->UOTGHS_DEVIMR, UOTGHS_DEVIMR_EORSME))
#define udd_ack_resume()                          (UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVICR_EORSMC)
#define udd_raise_resume()                        (UOTGHS->UOTGHS_DEVIFR = UOTGHS_DEVIFR_EORSMS)
#define Is_udd_resume()                           (Tst_bits(UOTGHS->UOTGHS_DEVISR, UOTGHS_DEVISR_EORSM))
//! @}

//! Manage wake-up event (=usb line activity)
//! The USB controller is reactivated by a filtered non-idle signal from the lines
//! @{
#define udd_enable_wake_up_interrupt()            (UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_WAKEUPES)
#define udd_disable_wake_up_interrupt()           (UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_WAKEUPEC)
#define Is_udd_wake_up_interrupt_enabled()        (Tst_bits(UOTGHS->UOTGHS_DEVIMR, UOTGHS_DEVIMR_WAKEUPE))
#define udd_ack_wake_up()                         (UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVICR_WAKEUPC)
#define udd_raise_wake_up()                       (UOTGHS->UOTGHS_DEVIFR = UOTGHS_DEVIFR_WAKEUPS)
#define Is_udd_wake_up()                          (Tst_bits(UOTGHS->UOTGHS_DEVISR, UOTGHS_DEVISR_WAKEUP))
//! @}

//! Manage reset event
//! Set when a USB "End of Reset" has been detected
//! @{
#define udd_enable_reset_interrupt()              (UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_EORSTES)
#define udd_disable_reset_interrupt()             (UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_EORSTEC)
#define Is_udd_reset_interrupt_enabled()          (Tst_bits(UOTGHS->UOTGHS_DEVIMR, UOTGHS_DEVIMR_EORSTE))
#define udd_ack_reset()                           (UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVICR_EORSTC)
#define udd_raise_reset()                         (UOTGHS->UOTGHS_DEVIFR = UOTGHS_DEVIFR_EORSTS)
#define Is_udd_reset()                            (Tst_bits(UOTGHS->UOTGHS_DEVISR, UOTGHS_DEVISR_EORST))
//! @}

//! Manage start of frame event
//! @{
#define udd_enable_sof_interrupt()                (UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_SOFES)
#define udd_disable_sof_interrupt()               (UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_SOFEC)
#define Is_udd_sof_interrupt_enabled()            (Tst_bits(UOTGHS->UOTGHS_DEVIMR, UOTGHS_DEVIMR_SOFE))
#define udd_ack_sof()                             (UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVICR_SOFC)
#define udd_raise_sof()                           (UOTGHS->UOTGHS_DEVIFR = UOTGHS_DEVIFR_SOFS)
#define Is_udd_sof()                              (Tst_bits(UOTGHS->UOTGHS_DEVISR, UOTGHS_DEVISR_SOF))
#define udd_frame_number()                        (Rd_bitfield(UOTGHS->UOTGHS_DEVFNUM, UOTGHS_DEVFNUM_FNUM_Msk))
#define Is_udd_frame_number_crc_error()           (Tst_bits(UOTGHS->UOTGHS_DEVFNUM, UOTGHS_DEVFNUM_FNCERR))
//! @}

//! Manage Micro start of frame event (High Speed Only)
//! @{
#define udd_enable_msof_interrupt()               (UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_MSOFES)
#define udd_disable_msof_interrupt()              (UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_MSOFEC)
#define Is_udd_msof_interrupt_enabled()           (Tst_bits(UOTGHS->UOTGHS_DEVIMR, UOTGHS_DEVIMR_MSOFE))
#define udd_ack_msof()                            (UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVIMR_MSOFE)
#define udd_raise_msof()                          (UOTGHS->UOTGHS_DEVIFR = UOTGHS_DEVIFR_MSOFS)
#define Is_udd_msof()                             (Tst_bits(UOTGHS->UOTGHS_DEVISR, UOTGHS_DEVISR_MSOF))
#define udd_micro_frame_number()                  \
  (Rd_bitfield(UOTGHS->UOTGHS_DEVFNUM, (UOTGHS_DEVFNUM_FNUM_Msk|UOTGHS_DEVFNUM_MFNUM_Msk)))
//! @}

//! Manage suspend event
//! @{
#define udd_enable_suspend_interrupt()            (UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_SUSPES)
#define udd_disable_suspend_interrupt()           (UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_SUSPEC)
#define Is_udd_suspend_interrupt_enabled()        (Tst_bits(UOTGHS->UOTGHS_DEVIMR, UOTGHS_DEVIMR_SUSPE))
#define udd_ack_suspend()                         (UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVICR_SUSPC)
#define udd_raise_suspend()                       (UOTGHS->UOTGHS_DEVIFR = UOTGHS_DEVIFR_SUSPS)
#define Is_udd_suspend()                          (Tst_bits(UOTGHS->UOTGHS_DEVISR, UOTGHS_DEVISR_SUSP))
//! @}

//! @}

//! @name UOTGHS device address control
//! These macros manage the UOTGHS Device address.
//! @{
  //! enables USB device address
#define udd_enable_address()                      (Set_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_ADDEN))
  //! disables USB device address
#define udd_disable_address()                     (Clr_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_ADDEN))
#define Is_udd_address_enabled()                  (Tst_bits(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_ADDEN))
  //! configures the USB device address
#define udd_configure_address(addr)               (Wr_bitfield(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_UADD_Msk, addr))
  //! gets the currently configured USB device address
#define udd_get_configured_address()              (Rd_bitfield(UOTGHS->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_UADD_Msk))
//! @}

//! @name UOTGHS Device endpoint drivers
//! These macros manage the common features of the endpoints.
//! @{

//! Generic macro for UOTGHS registers that can be arrayed
//! @{
#define UOTGHS_ARRAY(reg,index)                   ((&(UOTGHS->reg))[(index)])
//! @}

//! @name UOTGHS Device endpoint configuration
//! @{
  //! enables the selected endpoint
#define udd_enable_endpoint(ep)                   (Set_bits(UOTGHS->UOTGHS_DEVEPT, UOTGHS_DEVEPT_EPEN0 << (ep)))
  //! disables the selected endpoint
#define udd_disable_endpoint(ep)                  (Clr_bits(UOTGHS->UOTGHS_DEVEPT, UOTGHS_DEVEPT_EPEN0 << (ep)))
  //! tests if the selected endpoint is enabled
#define Is_udd_endpoint_enabled(ep)               (Tst_bits(UOTGHS->UOTGHS_DEVEPT, UOTGHS_DEVEPT_EPEN0 << (ep)))
  //! resets the selected endpoint
#define udd_reset_endpoint(ep)                                         \
  do {                                                               \
    Set_bits(UOTGHS->UOTGHS_DEVEPT, UOTGHS_DEVEPT_EPRST0 << (ep)); \
    Clr_bits(UOTGHS->UOTGHS_DEVEPT, UOTGHS_DEVEPT_EPRST0 << (ep)); \
  } while (0)
  //! Tests if the selected endpoint is being reset
#define Is_udd_resetting_endpoint(ep)             (Tst_bits(UOTGHS->UOTGHS_DEVEPT, UOTGHS_DEVEPT_EPRST0 << (ep)))

  //! Configures the selected endpoint type
#define udd_configure_endpoint_type(ep, type)     (Wr_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPTYPE_Msk, type))
  //! Gets the configured selected endpoint type
#define udd_get_endpoint_type(ep)                 (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPTYPE_Msk))
  //! Enables the bank autoswitch for the selected endpoint
#define udd_enable_endpoint_bank_autoswitch(ep)   (Set_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_AUTOSW))
  //! Disables the bank autoswitch for the selected endpoint
#define udd_disable_endpoint_bank_autoswitch(ep)    (Clr_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_AUTOSW))
#define Is_udd_endpoint_bank_autoswitch_enabled(ep) (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_AUTOSW))
  //! Configures the selected endpoint direction
#define udd_configure_endpoint_direction(ep, dir) (Wr_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPDIR, dir))
  //! Gets the configured selected endpoint direction
#define udd_get_endpoint_direction(ep)            (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPDIR))
#define Is_udd_endpoint_in(ep)                    (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPDIR))
  //! Bounds given integer size to allowed range and rounds it up to the nearest
  //! available greater size, then applies register format of UOTGHS controller
  //! for endpoint size bit-field.
#undef udd_format_endpoint_size
#define udd_format_endpoint_size(size)            (32 - clz(((uint32_t)min(max(size, 8), 1024) << 1) - 1) - 1 - 3)
  //! Configures the selected endpoint size
#define udd_configure_endpoint_size(ep, size)     (Wr_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPSIZE_Msk, udd_format_endpoint_size(size)))
  //! Gets the configured selected endpoint size
#define udd_get_endpoint_size(ep)                 (8 << Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPSIZE_Msk))
  //! Configures the selected endpoint number of banks
#define udd_configure_endpoint_bank(ep, bank)     (Wr_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPBK_Msk, bank))
  //! Gets the configured selected endpoint number of banks
#define udd_get_endpoint_bank(ep)                 (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPBK_Msk)+1)
  //! Allocates the configuration selected endpoint in DPRAM memory
#define udd_allocate_memory(ep)                   (Set_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_ALLOC))
  //! un-allocates the configuration selected endpoint in DPRAM memory
#define udd_unallocate_memory(ep)                 (Clr_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_ALLOC))
#define Is_udd_memory_allocated(ep)               (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_ALLOC))

  //! Configures selected endpoint in one step
#define udd_configure_endpoint(ep, type, dir, size, bank) (\
  Wr_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPTYPE_Msk |\
      UOTGHS_DEVEPTCFG_EPDIR  |\
      UOTGHS_DEVEPTCFG_EPSIZE_Msk |\
      UOTGHS_DEVEPTCFG_EPBK_Msk ,   \
      (((uint32_t)(type) << UOTGHS_DEVEPTCFG_EPTYPE_Pos) & UOTGHS_DEVEPTCFG_EPTYPE_Msk) |\
      (((uint32_t)(dir ) << UOTGHS_DEVEPTCFG_EPDIR_Pos ) & UOTGHS_DEVEPTCFG_EPDIR) |\
      ( (uint32_t)udd_format_endpoint_size(size) << UOTGHS_DEVEPTCFG_EPSIZE_Pos) |\
      (((uint32_t)(bank) << UOTGHS_DEVEPTCFG_EPBK_Pos) & UOTGHS_DEVEPTCFG_EPBK_Msk))\
)
  //! Tests if current endpoint is configured
#define Is_udd_endpoint_configured(ep)            (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_CFGOK))
  //! Returns the control direction
#define udd_control_direction()                   (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], EP_CONTROL), UOTGHS_DEVEPTISR_CTRLDIR))

  //! Resets the data toggle sequence
#define udd_reset_data_toggle(ep)                 (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_RSTDTS)
  //! Tests if the data toggle sequence is being reset
#define Is_udd_data_toggle_reset(ep)              (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_RSTDT))
  //! Returns data toggle
#define udd_data_toggle(ep)                       (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_DTSEQ_Msk))
//! @}

//! @name UOTGHS Device control endpoint
//! These macros control the endpoints.
//! @{

//! @name UOTGHS Device control endpoint interrupts
//! These macros control the endpoints interrupts.
//! @{
  //! Enables the selected endpoint interrupt
#define udd_enable_endpoint_interrupt(ep)         (UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_PEP_0 << (ep))
  //! Disables the selected endpoint interrupt
#define udd_disable_endpoint_interrupt(ep)        (UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_PEP_0 << (ep))
  //! Tests if the selected endpoint interrupt is enabled
#define Is_udd_endpoint_interrupt_enabled(ep)     (Tst_bits(UOTGHS->UOTGHS_DEVIMR, UOTGHS_DEVIMR_PEP_0 << (ep)))
  //! Tests if an interrupt is triggered by the selected endpoint
#define Is_udd_endpoint_interrupt(ep)             (Tst_bits(UOTGHS->UOTGHS_DEVISR, UOTGHS_DEVISR_PEP_0 << (ep)))
  //! Returns the lowest endpoint number generating an endpoint interrupt or MAX_PEP_NB if none
#define udd_get_interrupt_endpoint_number()       (ctz(((UOTGHS->UOTGHS_DEVISR >> UOTGHS_DEVISR_PEP_Pos) & \
                                                   (UOTGHS->UOTGHS_DEVIMR >> UOTGHS_DEVIMR_PEP_Pos)) |     \
                                                   (1 << MAX_PEP_NB)))
#define UOTGHS_DEVISR_PEP_Pos   12
#define UOTGHS_DEVIMR_PEP_Pos   12
//! @}

//! @name UOTGHS Device control endpoint errors
//! These macros control the endpoint errors.
//! @{
  //! Enables the STALL handshake
#define udd_enable_stall_handshake(ep)            (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_STALLRQS)
  //! Disables the STALL handshake
#define udd_disable_stall_handshake(ep)           (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_STALLRQC)
  //! Tests if STALL handshake request is running
#define Is_udd_endpoint_stall_requested(ep)       (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_STALLRQ))
  //! Tests if STALL sent
#define Is_udd_stall(ep)                          (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_STALLEDI))
  //! ACKs STALL sent
#define udd_ack_stall(ep)                         (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_STALLEDIC)
  //! Raises STALL sent
#define udd_raise_stall(ep)                       (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_STALLEDIS)
  //! Enables STALL sent interrupt
#define udd_enable_stall_interrupt(ep)            (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_STALLEDES)
  //! Disables STALL sent interrupt
#define udd_disable_stall_interrupt(ep)           (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_STALLEDEC)
  //! Tests if STALL sent interrupt is enabled
#define Is_udd_stall_interrupt_enabled(ep)        (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_STALLEDE))

  //! Tests if NAK OUT received
#define Is_udd_nak_out(ep)                        (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_NAKOUTI))
  //! ACKs NAK OUT received
#define udd_ack_nak_out(ep)                       (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_NAKOUTIC)
  //! Raises NAK OUT received
#define udd_raise_nak_out(ep)                     (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_NAKOUTIS)
  //! Enables NAK OUT interrupt
#define udd_enable_nak_out_interrupt(ep)          (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_NAKOUTES)
  //! Disables NAK OUT interrupt
#define udd_disable_nak_out_interrupt(ep)         (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_NAKOUTEC)
  //! Tests if NAK OUT interrupt is enabled
#define Is_udd_nak_out_interrupt_enabled(ep)      (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_NAKOUTE))

  //! Tests if NAK IN received
#define Is_udd_nak_in(ep)                         (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_NAKINI))
  //! ACKs NAK IN received
#define udd_ack_nak_in(ep)                        (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_NAKINIC)
  //! Raises NAK IN received
#define udd_raise_nak_in(ep)                      (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_NAKINIS)
  //! Enables NAK IN interrupt
#define udd_enable_nak_in_interrupt(ep)           (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_NAKINES)
  //! Disables NAK IN interrupt
#define udd_disable_nak_in_interrupt(ep)          (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_NAKINEC)
  //! Tests if NAK IN interrupt is enabled
#define Is_udd_nak_in_interrupt_enabled(ep)       (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_NAKINE))

  //! ACKs endpoint isochronous overflow interrupt
#define udd_ack_overflow_interrupt(ep)            (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_OVERFIC)
  //! Raises endpoint isochronous overflow interrupt
#define udd_raise_overflow_interrupt(ep)          (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_OVERFIS)
  //! Tests if an overflow occurs
#define Is_udd_overflow(ep)                       (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_OVERFI))
  //! Enables overflow interrupt
#define udd_enable_overflow_interrupt(ep)         (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_OVERFES)
  //! Disables overflow interrupt
#define udd_disable_overflow_interrupt(ep)        (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_OVERFEC)
  //! Tests if overflow interrupt is enabled
#define Is_udd_overflow_interrupt_enabled(ep)     (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_OVERFE))

  //! ACKs endpoint isochronous underflow interrupt
#define udd_ack_underflow_interrupt(ep)           (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_UNDERFIC)
  //! Raises endpoint isochronous underflow interrupt
#define udd_raise_underflow_interrupt(ep)         (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_UNDERFIS)
  //! Tests if an underflow occurs
#define Is_udd_underflow(ep)                      (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_UNDERFI))
  //! Enables underflow interrupt
#define udd_enable_underflow_interrupt(ep)        (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_UNDERFES)
  //! Disables underflow interrupt
#define udd_disable_underflow_interrupt(ep)       (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_UNDERFEC)
  //! Tests if underflow interrupt is enabled
#define Is_udd_underflow_interrupt_enabled(ep)    (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_UNDERFE))

  //! Tests if CRC ERROR ISO OUT detected
#define Is_udd_crc_error(ep)                      (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_CRCERRI))
  //! ACKs CRC ERROR ISO OUT detected
#define udd_ack_crc_error(ep)                     (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_CRCERRIC)
  //! Raises CRC ERROR ISO OUT detected
#define udd_raise_crc_error(ep)                   (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_CRCERRIS)
  //! Enables CRC ERROR ISO OUT detected interrupt
#define udd_enable_crc_error_interrupt(ep)        (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_CRCERRES)
  //! Disables CRC ERROR ISO OUT detected interrupt
#define udd_disable_crc_error_interrupt(ep)       (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_CRCERREC)
  //! Tests if CRC ERROR ISO OUT detected interrupt is enabled
#define Is_udd_crc_error_interrupt_enabled(ep)    (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_CRCERRE))
//! @}

//! @name UOTGHS Device control endpoint transfer
//! These macros control the endpoint transfer.
//! @{

  //! Tests if endpoint read allowed
#define Is_udd_read_enabled(ep)                   (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_RWALL))
  //! Tests if endpoint write allowed
#define Is_udd_write_enabled(ep)                  (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_RWALL))

  //! Returns the byte count
#define udd_byte_count(ep)                        (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_BYCT_Msk))
  //! Clears FIFOCON bit
#define udd_ack_fifocon(ep)                       (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_FIFOCONC)
  //! Tests if FIFOCON bit set
#define Is_udd_fifocon(ep)                        (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_FIFOCON))

  //! Returns the number of busy banks
#define udd_nb_busy_bank(ep)                      (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_NBUSYBK_Msk))
  //! Returns the number of the current bank
#define udd_current_bank(ep)                      (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_CURRBK_Msk))
  //! Kills last bank
#define udd_kill_last_in_bank(ep)                 (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_KILLBKS)
#define Is_udd_kill_last(ep)                      (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_KILLBK))
  //! Tests if last bank killed
#define Is_udd_last_in_bank_killed(ep)            (!Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_KILLBK))
  //! Forces all banks full (OUT) or free (IN) interrupt
#define udd_force_bank_interrupt(ep)              (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_NBUSYBKS)
  //! Unforces all banks full (OUT) or free (IN) interrupt
#define udd_unforce_bank_interrupt(ep)            (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_NBUSYBKS)
  //! Enables all banks full (OUT) or free (IN) interrupt
#define udd_enable_bank_interrupt(ep)             (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_NBUSYBKES)
  //! Disables all banks full (OUT) or free (IN) interrupt
#define udd_disable_bank_interrupt(ep)            (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_NBUSYBKEC)
  //! Tests if all banks full (OUT) or free (IN) interrupt enabled
#define Is_udd_bank_interrupt_enabled(ep)         (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_NBUSYBKE))

  //! Tests if SHORT PACKET received
#define Is_udd_short_packet(ep)                   (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_SHORTPACKET))
  //! ACKs SHORT PACKET received
#define udd_ack_short_packet(ep)                  (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_SHORTPACKETC)
  //! Raises SHORT PACKET received
#define udd_raise_short_packet(ep)                (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_SHORTPACKETS)
  //! Enables SHORT PACKET received interrupt
#define udd_enable_short_packet_interrupt(ep)     (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_SHORTPACKETES)
  //! Disables SHORT PACKET received interrupt
#define udd_disable_short_packet_interrupt(ep)    (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_SHORTPACKETEC)
  //! Tests if SHORT PACKET received interrupt is enabled
#define Is_udd_short_packet_interrupt_enabled(ep) (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_SHORTPACKETE))

  //! Tests if SETUP received
#define Is_udd_setup_received(ep)                    (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_RXSTPI))
  //! ACKs SETUP received
#define udd_ack_setup_received(ep)                   (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_RXSTPIC)
  //! Raises SETUP received
#define udd_raise_setup_received(ep)                 (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_RXSTPIS)
  //! Enables SETUP received interrupt
#define udd_enable_setup_received_interrupt(ep)      (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_RXSTPES)
  //! Disables SETUP received interrupt
#define udd_disable_setup_received_interrupt(ep)     (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_RXSTPEC)
  //! Tests if SETUP received interrupt is enabled
#define Is_udd_setup_received_interrupt_enabled(ep)  (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_RXSTPE))

  //! Tests if OUT received
#define Is_udd_out_received(ep)                   (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_RXOUTI))
  //! ACKs OUT received
#define udd_ack_out_received(ep)                  (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_RXOUTIC)
  //! Raises OUT received
#define udd_raise_out_received(ep)                (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_RXOUTIS)
  //! Enables OUT received interrupt
#define udd_enable_out_received_interrupt(ep)     (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_RXOUTES)
  //! Disables OUT received interrupt
#define udd_disable_out_received_interrupt(ep)    (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_RXOUTEC)
  //! Tests if OUT received interrupt is enabled
#define Is_udd_out_received_interrupt_enabled(ep) (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_RXOUTE))

  //! Tests if IN sending
#define Is_udd_in_send(ep)                        (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_TXINI))
  //! ACKs IN sending
#define udd_ack_in_send(ep)                       (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_TXINIC)
  //! Raises IN sending
#define udd_raise_in_send(ep)                     (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_TXINIS)
  //! Enables IN sending interrupt
#define udd_enable_in_send_interrupt(ep)          (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_TXINES)
  //! Disables IN sending interrupt
#define udd_disable_in_send_interrupt(ep)         (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_TXINEC)
  //! Tests if IN sending interrupt is enabled
#define Is_udd_in_send_interrupt_enabled(ep)      (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_TXINE))

  //! Get 64-, 32-, 16- or 8-bit access to FIFO data register of selected endpoint.
  //! @param ep Endpoint of which to access FIFO data register
  //! @param scale Data scale in bits: 64, 32, 16 or 8
  //! @return Volatile 64-, 32-, 16- or 8-bit data pointer to FIFO data register
  //! @warning It is up to the user of this macro to make sure that all accesses
  //! are aligned with their natural boundaries except 64-bit accesses which
  //! require only 32-bit alignment.
  //! @warning It is up to the user of this macro to make sure that used HSB
  //! addresses are identical to the DPRAM internal pointer modulo 32 bits.
#define udd_get_endpoint_fifo_access(ep, scale) \
    (((volatile TPASTE2(U, scale) (*)[0x8000 / ((scale) / 8)])UOTGHS_RAM_ADDR)[(ep)])

//! @name UOTGHS endpoint DMA drivers
//! These macros manage the common features of the endpoint DMA channels.
//! @{

  //! Maximum transfer size on USB DMA
#define UDD_ENDPOINT_MAX_TRANS 0x10000
  //! Enables the disabling of HDMA requests by endpoint interrupts
#define udd_enable_endpoint_int_dis_hdma_req(ep)     (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0](ep) = UOTGHS_DEVEPTIER_EPDISHDMAS)
  //! Disables the disabling of HDMA requests by endpoint interrupts
#define udd_disable_endpoint_int_dis_hdma_req(ep)    (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0](ep) = UOTGHS_DEVEPTIDR_EPDISHDMAC)
  //! Tests if the disabling of HDMA requests by endpoint interrupts is enabled
#define Is_udd_endpoint_int_dis_hdma_req_enabled(ep) (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0](ep), UOTGHS_DEVEPTIMR_EPDISHDMA))

  //! Raises the selected endpoint DMA channel interrupt
#define udd_raise_endpoint_dma_interrupt(ep)         (UOTGHS->UOTGHS_DEVIFR = UOTGHS_DEVIFR_DMA_1 << ((ep) - 1))
  //! Raises the selected endpoint DMA channel interrupt
#define udd_clear_endpoint_dma_interrupt(ep)         (UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVISR_DMA_1 << ((ep) - 1))
  //! Tests if an interrupt is triggered by the selected endpoint DMA channel
#define Is_udd_endpoint_dma_interrupt(ep)            (Tst_bits(UOTGHS->UOTGHS_DEVISR, UOTGHS_DEVISR_DMA_1 << ((ep) - 1)))
  //! Enables the selected endpoint DMA channel interrupt
#define udd_enable_endpoint_dma_interrupt(ep)        (UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_DMA_1 << ((ep) - 1))
  //! Disables the selected endpoint DMA channel interrupt
#define udd_disable_endpoint_dma_interrupt(ep)       (UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_DMA_1 << ((ep) - 1))
  //! Tests if the selected endpoint DMA channel interrupt is enabled
#define Is_udd_endpoint_dma_interrupt_enabled(ep)    (Tst_bits(UOTGHS->UOTGHS_DEVIMR, UOTGHS_DEVIMR_DMA_1 << ((ep) - 1)))

  //! Access points to the UOTGHS device DMA memory map with arrayed registers
  //! @{
      //! Structure for DMA next descriptor register
typedef struct {
  uint32_t *NXT_DSC_ADD;
} uotghs_dma_nextdesc_t;
      //! Structure for DMA control register
typedef struct {
  uint32_t CHANN_ENB:1,
    LDNXT_DSC:1,
    END_TR_EN:1,
    END_B_EN:1,
    END_TR_IT:1,
    END_BUFFIT:1,
    DESC_LD_IT:1,
    BUST_LCK:1,
    reserved:8,
    BUFF_LENGTH:16;
} uotghs_dma_control_t;
      //! Structure for DMA status register
typedef struct {
  uint32_t CHANN_ENB:1,
    CHANN_ACT:1,
    reserved0:2,
    END_TR_ST:1,
    END_BF_ST:1,
    DESC_LDST:1,
    reserved1:9,
    BUFF_COUNT:16;
} uotghs_dma_status_t;
      //! Structure for DMA descriptor
typedef struct {
  union {
    uint32_t nextdesc;
    uotghs_dma_nextdesc_t NEXTDESC;
  };
  uint32_t addr;
  union {
    uint32_t control;
    uotghs_dma_control_t CONTROL;
  };
  uint32_t reserved;
} sam_uotghs_dmadesc_t, uotghs_dmadesc_t;
      //! Structure for DMA registers in a channel
typedef struct {
  union {
    uint32_t nextdesc;
    uotghs_dma_nextdesc_t NEXTDESC;
  };
  uint32_t addr;
  union {
    uint32_t control;
    uotghs_dma_control_t CONTROL;
  };
  union {
    unsigned long status;
    uotghs_dma_status_t STATUS;
  };
} sam_uotghs_dmach_t, uotghs_dmach_t;
      //! DMA channel control command
#define UDD_ENDPOINT_DMA_STOP_NOW                 (0)
#define UDD_ENDPOINT_DMA_RUN_AND_STOP             (UOTGHS_DEVDMACONTROL_CHANN_ENB)
#define UDD_ENDPOINT_DMA_LOAD_NEXT_DESC           (UOTGHS_DEVDMACONTROL_LDNXT_DSC)
#define UDD_ENDPOINT_DMA_RUN_AND_LINK             (UOTGHS_DEVDMACONTROL_CHANN_ENB|UOTGHS_DEVDMACONTROL_LDNXT_DSC)
      //! Structure for DMA registers
#define UOTGHS_UDDMA_ARRAY(ep)                    (((volatile uotghs_dmach_t *)UOTGHS->UOTGHS_DEVDMA)[(ep) - 1])

      //! Set control desc to selected endpoint DMA channel
#define udd_endpoint_dma_set_control(ep,desc)     (UOTGHS_UDDMA_ARRAY(ep).control = desc)
      //! Get control desc to selected endpoint DMA channel
#define udd_endpoint_dma_get_control(ep)          (UOTGHS_UDDMA_ARRAY(ep).control)
      //! Set RAM address to selected endpoint DMA channel
#define udd_endpoint_dma_set_addr(ep,add)         (UOTGHS_UDDMA_ARRAY(ep).addr = add)
      //! Get status to selected endpoint DMA channel
#define udd_endpoint_dma_get_status(ep)           (UOTGHS_UDDMA_ARRAY(ep).status)
   //! @}
//! @}

//! @}
//! @}
//! @}
//! @}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif // UOTGHS_DEVICE_H_INCLUDED
