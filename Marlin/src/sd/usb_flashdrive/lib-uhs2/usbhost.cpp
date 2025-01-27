/****************
 * usb_host.cpp *
 ****************/

/****************************************************************************
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

/* What follows is a modified version of the MAX3421e originally defined in
 * lib/usbhost.c". This has been rewritten to use SPI routines from the
 * Marlin HAL */

#include "../../../inc/MarlinConfigPre.h"

#if HAS_USB_FLASH_DRIVE && DISABLED(USE_UHS3_USB)

#if !PINS_EXIST(USB_CS, USB_INTR)
  #error "USB_FLASH_DRIVE_SUPPORT requires USB_CS_PIN and USB_INTR_PIN (or USE_UHS3_USB) to be defined."
#endif

#include "Usb.h"
#include "usbhost.h"

uint8_t MAX3421e::vbusState = 0;

// constructor
void MAX3421e::cs()  { WRITE(USB_CS_PIN, LOW); }
void MAX3421e::ncs() { WRITE(USB_CS_PIN, HIGH); }

// write single byte into MAX3421 register
void MAX3421e::regWr(uint8_t reg, uint8_t data) {
  cs();
  spiSend(reg | 0x02);
  spiSend(data);
  ncs();
}

// multiple-byte write
// return a pointer to memory position after last written
uint8_t* MAX3421e::bytesWr(uint8_t reg, uint8_t nbytes, uint8_t *data_p) {
  cs();
  spiSend(reg | 0x02);
  while (nbytes--) spiSend(*data_p++);
  ncs();
  return data_p;
}

// GPIO write
// GPIO byte is split between 2 registers, so two writes are needed to write one byte

// GPOUT bits are in the low nybble. 0-3 in IOPINS1, 4-7 in IOPINS2
void MAX3421e::gpioWr(uint8_t data) {
  regWr(rIOPINS1, data);
  regWr(rIOPINS2, data >> 4);
}

// single host register read
uint8_t MAX3421e::regRd(uint8_t reg) {
  cs();
  spiSend(reg);
  uint8_t rv = spiRec();
  ncs();
  return rv;
}

// multiple-byte register read
// return a pointer to a memory position after last read
uint8_t* MAX3421e::bytesRd(uint8_t reg, uint8_t nbytes, uint8_t *data_p) {
  cs();
  spiSend(reg);
  while (nbytes--) *data_p++ = spiRec();
  ncs();
  return data_p;
}

// GPIO read. See gpioWr for explanation
// GPIN pins are in high nybbles of IOPINS1, IOPINS2
uint8_t MAX3421e::gpioRd() {
  return (regRd(rIOPINS2) & 0xF0) | // pins 4-7, clean lower nybble
         (regRd(rIOPINS1)   >> 4);  // shift low bits and OR with upper from previous operation.
}

// reset MAX3421e. Returns false if PLL failed to stabilize 1 second after reset
bool MAX3421e::reset() {
  regWr(rUSBCTL, bmCHIPRES);
  regWr(rUSBCTL, 0x00);
  for (uint8_t i = 100; i--;) {
    if (regRd(rUSBIRQ) & bmOSCOKIRQ) return true;
    delay(10);
  }
  return false;
}

// initialize MAX3421e. Set Host mode, pullups, and stuff. Returns 0 if success, -1 if not
bool MAX3421e::start() {
  // Initialize pins and SPI bus

  SET_OUTPUT(USB_CS_PIN);
  SET_INPUT_PULLUP(USB_INTR_PIN);
  ncs();
  spiBegin();

  spiInit(SD_SPI_SPEED);

  // MAX3421e - full-duplex, level interrupt, vbus off.
  regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL | GPX_VBDET));

  const uint8_t revision = regRd(rREVISION);
  if (revision == 0x00 || revision == 0xFF) {
    SERIAL_ECHOLNPGM("Revision register appears incorrect on MAX3421e initialization. Got ", revision);
    return false;
  }

  if (!reset()) {
    SERIAL_ECHOLNPGM("OSCOKIRQ hasn't asserted in time");
    return false;
  }

  // Delay a minimum of 1 second to ensure any capacitors are drained.
  // 1 second is required to make sure we do not smoke a Microdrive!

  delay(1000);

  regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // set pull-downs, Host
  regWr(rHIEN, bmCONDETIE | bmFRAMEIE); // connection detection

  // check if device is connected
  regWr(rHCTL, bmSAMPLEBUS); // sample USB bus
  while (!(regRd(rHCTL) & bmSAMPLEBUS)) delay(10); // wait for sample operation to finish

  busprobe(); // check if anything is connected

  regWr(rHIRQ, bmCONDETIRQ); // clear connection detect interrupt
  regWr(rCPUCTL, 0x01);      // enable interrupt pin

  // GPX pin on. This is done here so that busprobe will fail if we have a switch connected.
  regWr(rPINCTL, bmFDUPSPI | bmINTLEVEL);

  return true;
}

// Probe bus to determine device presence and speed. Switch host to this speed.
void MAX3421e::busprobe() {
  // Switch on just the J & K bits
  switch (regRd(rHRSL) & (bmJSTATUS | bmKSTATUS)) {
    case bmJSTATUS:
      if ((regRd(rMODE) & bmLOWSPEED) == 0) {
        regWr(rMODE, MODE_FS_HOST); // start full-speed host
        vbusState = FSHOST;
      }
      else {
        regWr(rMODE, MODE_LS_HOST); // start low-speed host
        vbusState = LSHOST;
      }
      break;
    case bmKSTATUS:
      if ((regRd(rMODE) & bmLOWSPEED) == 0) {
        regWr(rMODE, MODE_LS_HOST); // start low-speed host
        vbusState = LSHOST;
      }
      else {
        regWr(rMODE, MODE_FS_HOST); // start full-speed host
        vbusState = FSHOST;
      }
      break;
    case bmSE1: // illegal state
      vbusState = SE1;
      break;
    case bmSE0: // disconnected state
      regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST | bmSEPIRQ);
      vbusState = SE0;
      break;
  }
}

// MAX3421 state change task and interrupt handler
uint8_t MAX3421e::Task() {
  return READ(USB_INTR_PIN) ? 0 : IntHandler();
}

uint8_t MAX3421e::IntHandler() {
  uint8_t HIRQ = regRd(rHIRQ), // determine interrupt source
          HIRQ_sendback = 0x00;
  if (HIRQ & bmCONDETIRQ) {
    busprobe();
    HIRQ_sendback |= bmCONDETIRQ;
  }
  // End HIRQ interrupts handling, clear serviced IRQs
  regWr(rHIRQ, HIRQ_sendback);
  return HIRQ_sendback;
}

#endif // HAS_USB_FLASH_DRIVE && !USE_UHS3_USB
