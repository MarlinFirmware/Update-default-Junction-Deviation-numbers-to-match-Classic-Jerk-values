/**
 * Copyright (C) 2015-2016 Andrew J. Kroll
 * and
 * Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.
 *
 * This software may be distributed and modified under the terms of the GNU
 * General Public License version 2 (GPL2) as publishe7d by the Free Software
 * Foundation and appearing in the file GPL2.TXT included in the packaging of
 * this file. Please note that GPL2 Section 2[b] requires that all works based
 * on this software must also be made publicly available under the terms of
 * the GPL2 ("Copyleft").
 *
 * Contact information
 * -------------------
 *
 * Circuits At Home, LTD
 * Web      :  https://www.circuitsathome.com
 * e-mail   :  support@circuitsathome.com
 */

#if defined(USB_HOST_SHIELD_H) && !defined(USB_HOST_SHIELD_LOADED)
#define USB_HOST_SHIELD_LOADED

#include <Arduino.h>

#ifndef digitalPinToInterrupt
    #error digitalPinToInterrupt not defined, complain to your board maintainer.
#endif

#if USB_HOST_SHIELD_USE_ISR

  // allow two slots. this makes the maximum allowed shield count TWO
  // for AVRs this is limited to pins 2 and 3 ONLY
  // for all other boards, one odd and one even pin number is allowed.
  static MAX3421E_HOST *ISReven;
  static MAX3421E_HOST *ISRodd;

  static void UHS_NI call_ISReven() {
    ISReven->ISRTask();
  }

  static void UHS_NI call_ISRodd() {
    UHS_PIN_WRITE(LED_BUILTIN, HIGH);
    ISRodd->ISRTask();
  }
#endif

void UHS_NI MAX3421E_HOST::resume_host() {
  // Used on MCU that lack control of IRQ priority (AVR).
  // Resumes ISRs.
  // NOTE: you must track the state yourself!
  #ifdef __AVR__
    noInterrupts();
    if (irq_pin & 1) {
    ISRodd = this;
    attachInterrupt(UHS_GET_DPI(irq_pin), call_ISRodd, IRQ_SENSE);
    } else {
    ISReven = this;
    attachInterrupt(UHS_GET_DPI(irq_pin), call_ISReven, IRQ_SENSE);
    }
    interrupts();
  #endif
}

/* write single byte into MAX3421e register */
void UHS_NI MAX3421E_HOST::regWr(uint8_t reg, uint8_t data) {
  SPIclass.beginTransaction(MAX3421E_SPI_Settings);
  MARLIN_UHS_WRITE_SS(LOW);
  SPIclass.transfer(reg | 0x02);
  SPIclass.transfer(data);
  MARLIN_UHS_WRITE_SS(HIGH);
  SPIclass.endTransaction();
}

/* multiple-byte write                            */

/* returns a pointer to memory position after last written */
uint8_t* UHS_NI MAX3421E_HOST::bytesWr(uint8_t reg, uint8_t nbytes, uint8_t *data_p) {
  SPIclass.beginTransaction(MAX3421E_SPI_Settings);
  MARLIN_UHS_WRITE_SS(LOW);
  SPIclass.transfer(reg | 0x02);
  //printf("%2.2x :", reg);

  while (nbytes) {
    SPIclass.transfer(*data_p);
    //printf("%2.2x ", *data_p);
    nbytes--;
    data_p++; // advance data pointer
  }
  MARLIN_UHS_WRITE_SS(HIGH);
  SPIclass.endTransaction();
  //printf("\r\n");
  return (data_p);
}

/* GPIO write                                           */
/*GPIO byte is split between 2 registers, so two writes are needed to write one byte */

/* GPOUT bits are in the low nybble. 0-3 in IOPINS1, 4-7 in IOPINS2 */
void UHS_NI MAX3421E_HOST::gpioWr(uint8_t data) {
  regWr(rIOPINS1, data);
  data >>= 4;
  regWr(rIOPINS2, data);
  return;
}

/* single host register read    */
uint8_t UHS_NI MAX3421E_HOST::regRd(uint8_t reg) {
  SPIclass.beginTransaction(MAX3421E_SPI_Settings);
  MARLIN_UHS_WRITE_SS(LOW);
  SPIclass.transfer(reg);
  uint8_t rv = SPIclass.transfer(0);
  MARLIN_UHS_WRITE_SS(HIGH);
  SPIclass.endTransaction();
  return (rv);
}
/* multiple-byte register read  */

/* returns a pointer to a memory position after last read   */
uint8_t* UHS_NI MAX3421E_HOST::bytesRd(uint8_t reg, uint8_t nbytes, uint8_t *data_p) {
  SPIclass.beginTransaction(MAX3421E_SPI_Settings);
  MARLIN_UHS_WRITE_SS(LOW);
  SPIclass.transfer(reg);
  while (nbytes) {
    *data_p++ = SPIclass.transfer(0);
    nbytes--;
  }
  MARLIN_UHS_WRITE_SS(HIGH);
  SPIclass.endTransaction();
  return ( data_p);
}

/* GPIO read. See gpioWr for explanation */

/* GPIN pins are in high nybbles of IOPINS1, IOPINS2    */
uint8_t UHS_NI MAX3421E_HOST::gpioRd() {
  uint8_t gpin = 0;
  gpin = regRd(rIOPINS2); // pins 4-7
  gpin &= 0xF0; // clean lower nybble
  gpin |= (regRd(rIOPINS1) >> 4); // shift low bits and OR with upper from previous operation.
  return (gpin);
}

/* reset MAX3421E. Returns number of microseconds it took for PLL to stabilize after reset
or zero if PLL haven't stabilized in 65535 cycles */
uint16_t UHS_NI MAX3421E_HOST::reset() {
  uint16_t i = 0;

  // Initiate chip reset
  regWr(rUSBCTL, bmCHIPRES);
  regWr(rUSBCTL, 0x00);

  int32_t now;
  uint32_t expires = micros() + 65535;

  // Enable full-duplex SPI so we can read rUSBIRQ
  regWr(rPINCTL, bmFDUPSPI);
  while ((int32_t)(micros() - expires) < 0L) {
    if ((regRd(rUSBIRQ) & bmOSCOKIRQ)) {
      break;
    }
  }
  now = (int32_t)(micros() - expires);
  if (now < 0L) {
    i = 65535 + now; // Note this subtracts, as now is negative
  }
  return (i);
}

void UHS_NI MAX3421E_HOST::VBUS_changed() {
  /* modify USB task state because Vbus changed or unknown */
  uint8_t speed = 1;
  //printf("\r\n\r\n\r\n\r\nSTATE %2.2x -> ", usb_task_state);
  switch (vbusState) {
    case LSHOST: // Low speed
    speed = 0;
    // Intentional fall-through
    case FSHOST: // Full speed
      // Start device initialization if we are not initializing
      // Resets to the device cause an IRQ
      // usb_task_state == UHS_USB_HOST_STATE_RESET_NOT_COMPLETE;
      //if ((usb_task_state & UHS_USB_HOST_STATE_MASK) != UHS_USB_HOST_STATE_DETACHED) {
      ReleaseChildren();
      if (!doingreset) {
        if (usb_task_state == UHS_USB_HOST_STATE_RESET_NOT_COMPLETE) {
          usb_task_state = UHS_USB_HOST_STATE_WAIT_BUS_READY;
        } else if (usb_task_state != UHS_USB_HOST_STATE_WAIT_BUS_READY) {
          usb_task_state = UHS_USB_HOST_STATE_DEBOUNCE;
        }
      }
      sof_countdown = 0;
    break;
    case SE1: // illegal state
      sof_countdown = 0;
      doingreset = false;
      ReleaseChildren();
      usb_task_state = UHS_USB_HOST_STATE_ILLEGAL;
      break;
    case SE0: // disconnected
    default:
      sof_countdown = 0;
      doingreset = false;
      ReleaseChildren();
      usb_task_state = UHS_USB_HOST_STATE_IDLE;
      break;
  }
  usb_host_speed = speed;
  //printf("0x%2.2x\r\n\r\n\r\n\r\n", usb_task_state);
  return;
}

/**
 * Probe bus to determine device presence and speed,
 * then switch host to detected speed.
 */
void UHS_NI MAX3421E_HOST::busprobe() {
  uint8_t bus_sample;
  uint8_t tmpdata;
  bus_sample = regRd(rHRSL); // Get J,K status
  bus_sample &= (bmJSTATUS | bmKSTATUS); // zero the rest of the byte
  switch (bus_sample) { // start full-speed or low-speed host
    case bmJSTATUS:
      // Serial.println("J");
      if ((regRd(rMODE) & bmLOWSPEED) == 0) {
      regWr(rMODE, MODE_FS_HOST); // start full-speed host
      vbusState = FSHOST;
      } else {
      regWr(rMODE, MODE_LS_HOST); // start low-speed host
      vbusState = LSHOST;
      }
      #ifdef USB_HOST_MANUAL_POLL
          enable_frame_irq(true);
      #endif
      tmpdata = regRd(rMODE) | bmSOFKAENAB; // start SOF generation
      regWr(rHIRQ, bmFRAMEIRQ); // see data sheet.
      regWr(rMODE, tmpdata);
      break;
    case bmKSTATUS:
      // Serial.println("K");
      if ((regRd(rMODE) & bmLOWSPEED) == 0) {
      regWr(rMODE, MODE_LS_HOST); // start low-speed host
      vbusState = LSHOST;
      } else {
      regWr(rMODE, MODE_FS_HOST); // start full-speed host
      vbusState = FSHOST;
      }
      #ifdef USB_HOST_MANUAL_POLL
        enable_frame_irq(true);
      #endif
      tmpdata = regRd(rMODE) | bmSOFKAENAB; // start SOF generation
      regWr(rHIRQ, bmFRAMEIRQ); // see data sheet.
      regWr(rMODE, tmpdata);
      break;
    case bmSE1: // illegal state
      // Serial.println("I");
      regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);
      vbusState = SE1;
      // sofevent = false;
      break;
    case bmSE0: // disconnected state
      // Serial.println("D");
      regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);
      vbusState = SE0;
      // sofevent = false;
      break;
  } // end switch ( bus_sample )
}

/**
 * Initialize USB hardware, turn on VBUS
 *
 * @param mseconds Delay energizing VBUS after mseconds, A value of INT16_MIN means no delay.
 * @return 0 on success, -1 on error
 */
int16_t UHS_NI MAX3421E_HOST::Init(int16_t mseconds) {
  usb_task_state = UHS_USB_HOST_STATE_INITIALIZE; //set up state machine
  //Serial.print("MAX3421E 'this' USB Host @ 0x");
  //Serial.println((uint32_t)this, HEX);
  //Serial.print("MAX3421E 'this' USB Host Address Pool @ 0x");
  //Serial.println((uint32_t)GetAddressPool(), HEX);
  Init_dyn_SWI();
  UHS_printf_HELPER_init();
  noInterrupts();
  #ifdef ARDUINO_AVR_ADK
    // For Mega ADK, which has a Max3421e on-board,
    // set MAX_RESET to output mode, and then set it to HIGH
    // PORTJ bit 2
    if (irq_pin == 54) {
      DDRJ |= 0x04; // output
      PORTJ |= 0x04; // HIGH
    }
  #endif
  SPIclass.begin();
  #ifdef ARDUINO_AVR_ADK
    if (irq_pin == 54) {
      DDRE &= ~0x20; // input
      PORTE |= 0x20; // pullup
    } else
  #endif
  pinMode(irq_pin, INPUT_PULLUP);
  //UHS_PIN_WRITE(irq_pin, HIGH);
  pinMode(ss_pin, OUTPUT);
  MARLIN_UHS_WRITE_SS(HIGH);

  #ifdef USB_HOST_SHIELD_TIMING_PIN
    pinMode(USB_HOST_SHIELD_TIMING_PIN, OUTPUT);
    // My counter/timer can't work on an inverted gate signal
    // so we gate using a high pulse -- AJK
    UHS_PIN_WRITE(USB_HOST_SHIELD_TIMING_PIN, LOW);
  #endif
  interrupts();

  #if USB_HOST_SHIELD_USE_ISR
    int intr = digitalPinToInterrupt(irq_pin);
    if (intr == NOT_AN_INTERRUPT) {
      #ifdef ARDUINO_AVR_ADK
        if (irq_pin == 54)
          intr = 6;
        else
      #endif
      return (-2);
    }
    SPIclass.usingInterrupt(intr);
  #else
    SPIclass.usingInterrupt(255);
  #endif
  #ifndef NO_AUTO_SPEED
    // test to get to reset acceptance.
    uint32_t spd = UHS_MAX3421E_SPD;
    again:
      MAX3421E_SPI_Settings = SPISettings(spd, MSBFIRST, SPI_MODE0);
    if (reset() == 0) {
      MAX_HOST_DEBUG(PSTR("Fail SPI speed %lu\r\n"), spd);
      if (spd > 1999999) {
        spd -= 1000000;
        goto again;
      }
      return (-1);
    } else {
      // reset passes, does 64k?
      uint8_t sample_wr = 0;
      uint8_t sample_rd = 0;
      uint8_t gpinpol_copy = regRd(rGPINPOL);
      for (uint16_t j = 0; j < 65535; j++) {
        regWr(rGPINPOL, sample_wr);
        sample_rd = regRd(rGPINPOL);
        if (sample_rd != sample_wr) {
          MAX_HOST_DEBUG(PSTR("Fail SPI speed %lu\r\n"), spd);
          if (spd > 1999999) {
            spd -= 1000000;
            goto again;
          }
          return (-1);
        }
        sample_wr++;
      }
      regWr(rGPINPOL, gpinpol_copy);
    }

    MAX_HOST_DEBUG(PSTR("Pass SPI speed %lu\r\n"), spd);
  #endif

  if (reset() == 0) { // OSCOKIRQ hasn't asserted in time
    MAX_HOST_DEBUG(PSTR("OSCOKIRQ hasn't asserted in time"));
    return ( -1);
  }

  /* MAX3421E - full-duplex SPI, interrupt kind, vbus off */
  regWr(rPINCTL, (bmFDUPSPI | bmIRQ_SENSE | GPX_VBDET));

  // Delay a minimum of 1 second to ensure any capacitors are drained.
  // 1 second is required to make sure we do not smoke a Microdrive!
  if (mseconds != INT16_MIN) {
    if (mseconds < 1000) mseconds = 1000;
    delay(mseconds); // We can't depend on SOF timer here.
  }

  regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // set pull-downs, Host

  // Enable interrupts on the MAX3421e
  regWr(rHIEN, IRQ_CHECK_MASK);
  // Enable interrupt pin on the MAX3421e, set pulse width for edge
  regWr(rCPUCTL, (bmIE | bmPULSEWIDTH));

  /* check if device is connected */
  regWr(rHCTL, bmSAMPLEBUS); // sample USB bus
  while (!(regRd(rHCTL) & bmSAMPLEBUS)); // wait for sample operation to finish

  busprobe(); // check if anything is connected
  VBUS_changed();

  // GPX pin on. This is done here so that a change is detected if we have a switch connected.
  /* MAX3421E - full-duplex SPI, interrupt kind, vbus on */
  regWr(rPINCTL, (bmFDUPSPI | bmIRQ_SENSE));
  regWr(rHIRQ, bmBUSEVENTIRQ); // see data sheet.
  regWr(rHCTL, bmBUSRST); // issue bus reset to force generate yet another possible IRQ

  #if USB_HOST_SHIELD_USE_ISR
    // Attach ISR to service IRQ from MAX3421e
    noInterrupts();
    if (irq_pin & 1) {
      ISRodd = this;
      attachInterrupt(UHS_GET_DPI(irq_pin), call_ISRodd, IRQ_SENSE);
    } else {
      ISReven = this;
      attachInterrupt(UHS_GET_DPI(irq_pin), call_ISReven, IRQ_SENSE);
    }
    interrupts();
  #endif
  //printf("\r\nrPINCTL 0x%2.2X\r\n", rPINCTL);
  //printf("rCPUCTL 0x%2.2X\r\n", rCPUCTL);
  //printf("rHIEN 0x%2.2X\r\n", rHIEN);
  //printf("irq_pin %i\r\n", irq_pin);
  return 0;
}

/**
 * Setup UHS_EpInfo structure
 *
 * @param addr USB device address
 * @param ep Endpoint
 * @param ppep pointer to the pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @return 0 on success
 */
uint8_t UHS_NI MAX3421E_HOST::SetAddress(uint8_t addr, uint8_t ep, UHS_EpInfo **ppep, uint16_t &nak_limit) {
  UHS_Device *p = addrPool.GetUsbDevicePtr(addr);

  if (!p)
    return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;

  if (!p->epinfo)
    return UHS_HOST_ERROR_NULL_EPINFO;

  *ppep = getEpInfoEntry(addr, ep);

  if (!*ppep)
    return UHS_HOST_ERROR_NO_ENDPOINT_IN_TABLE;

  nak_limit = (0x0001UL << (((*ppep)->bmNakPower > UHS_USB_NAK_MAX_POWER) ? UHS_USB_NAK_MAX_POWER : (*ppep)->bmNakPower));
  nak_limit--;
  /*
  USBTRACE2("\r\nAddress: ", addr);
  USBTRACE2(" EP: ", ep);
  USBTRACE2(" NAK Power: ",(*ppep)->bmNakPower);
  USBTRACE2(" NAK Limit: ", nak_limit);
  USBTRACE("\r\n");
  */
  regWr(rPERADDR, addr); // set peripheral address

  uint8_t mode = regRd(rMODE);

  //Serial.print("\r\nMode: ");
  //Serial.println( mode, HEX);
  //Serial.print("\r\nLS: ");
  //Serial.println(p->speed, HEX);

  // Set bmLOWSPEED and bmHUBPRE in case of low-speed device, reset them otherwise
  regWr(rMODE, (p->speed) ? mode & ~(bmHUBPRE | bmLOWSPEED) : mode | bmLOWSPEED | hub_present);

  return 0;
}

/**
 * Receive a packet
 *
 * @param pep pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @param nbytesptr pointer to maximum number of bytes of data to receive
 * @param data pointer to data buffer
 * @return 0 on success
 */
uint8_t UHS_NI MAX3421E_HOST::InTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t *nbytesptr, uint8_t *data) {
  uint8_t rcode = 0;
  uint8_t pktsize;

  uint16_t nbytes = *nbytesptr;
  MAX_HOST_DEBUG(PSTR("Requesting %i bytes "), nbytes);
  uint8_t maxpktsize = pep->maxPktSize;

  *nbytesptr = 0;
  regWr(rHCTL, (pep->bmRcvToggle) ? bmRCVTOG1 : bmRCVTOG0); // set toggle value

  // use a 'break' to exit this loop
  while (1) {
    rcode = dispatchPkt(MAX3421E_tokIN, pep->epAddr, nak_limit); // IN packet to EP-'endpoint'. Function takes care of NAKS.
    #if 0
      // This issue should be resolved now.
      if (rcode == UHS_HOST_ERROR_TOGERR) {
      //MAX_HOST_DEBUG(PSTR("toggle wrong\r\n"));
      // yes, we flip it wrong here so that next time it is actually correct!
      pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
      regWr(rHCTL, (pep->bmRcvToggle) ? bmRCVTOG1 : bmRCVTOG0); // set toggle value
      continue;
      }
    #endif
    if (rcode) {
      //MAX_HOST_DEBUG(PSTR(">>>>>>>> Problem! dispatchPkt %2.2x\r\n"), rcode);
      break; // should be 0, indicating ACK. Else return error code.
    }
    /* check for RCVDAVIRQ and generate error if not present */
    /* the only case when absence of RCVDAVIRQ makes sense is when toggle error occurred. Need to add handling for that */
    if ((regRd(rHIRQ) & bmRCVDAVIRQ) == 0) {
      //MAX_HOST_DEBUG(PSTR(">>>>>>>> Problem! NO RCVDAVIRQ!\r\n"));
      rcode = 0xF0; // receive error
      break;
    }
    pktsize = regRd(rRCVBC); // number of received bytes
    MAX_HOST_DEBUG(PSTR("Got %i bytes \r\n"), pktsize);

    if (pktsize > nbytes) { // certain devices send more than asked
      //MAX_HOST_DEBUG(PSTR(">>>>>>>> Warning: wanted %i bytes but got %i.\r\n"), nbytes, pktsize);
      pktsize = nbytes;
    }

    int16_t mem_left = (int16_t)nbytes - *((int16_t*)nbytesptr);

    if (mem_left < 0)
      mem_left = 0;

    data = bytesRd(rRCVFIFO, ((pktsize > mem_left) ? mem_left : pktsize), data);

    regWr(rHIRQ, bmRCVDAVIRQ); // Clear the IRQ & free the buffer
    *nbytesptr += pktsize; // add this packet's byte count to total transfer length

    /* The transfer is complete under two conditions:           */
    /* 1. The device sent a short packet (L.T. maxPacketSize)   */
    /* 2. 'nbytes' have been transferred.                       */
    if ((pktsize < maxpktsize) || (*nbytesptr >= nbytes)) { // have we transferred 'nbytes' bytes?
      // Save toggle value
      pep->bmRcvToggle = ((regRd(rHRSL) & bmRCVTOGRD)) ? 1 : 0;
      //MAX_HOST_DEBUG(PSTR("\r\n"));
      rcode = 0;
      break;
    } // if
  } // while( 1 )
  return (rcode);
}

/**
 * Transmit a packet
 *
 * @param pep pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @param nbytes number of bytes of data to send
 * @param data pointer to data buffer
 * @return 0 on success
 */
uint8_t UHS_NI MAX3421E_HOST::OutTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t nbytes, uint8_t *data) {
  uint8_t rcode = UHS_HOST_ERROR_NONE;
  uint8_t retry_count;
  uint8_t *data_p = data; // local copy of the data pointer
  uint16_t bytes_tosend;
  uint16_t nak_count;
  uint16_t bytes_left = nbytes;

  uint8_t maxpktsize = pep->maxPktSize;

  if (maxpktsize < 1 || maxpktsize > 64)
  return UHS_HOST_ERROR_BAD_MAX_PACKET_SIZE;

  unsigned long timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;

  regWr(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0); // set toggle value

  while (bytes_left) {
    SYSTEM_OR_SPECIAL_YIELD();
    retry_count = 0;
    nak_count = 0;
    bytes_tosend = (bytes_left >= maxpktsize) ? maxpktsize : bytes_left;
    bytesWr(rSNDFIFO, bytes_tosend, data_p); // filling output FIFO
    regWr(rSNDBC, bytes_tosend); // set number of bytes
    regWr(rHXFR, (MAX3421E_tokOUT | pep->epAddr)); // dispatch packet
    while (!(regRd(rHIRQ) & bmHXFRDNIRQ)); // wait for the completion IRQ
    regWr(rHIRQ, bmHXFRDNIRQ); // clear IRQ
    rcode = (regRd(rHRSL) & 0x0F);

    while (rcode && ((long)(millis() - timeout) < 0L)) {
      switch (rcode) {
        case UHS_HOST_ERROR_NAK:
          nak_count++;
          if (nak_limit && (nak_count == nak_limit))
            goto breakout;
          break;
        case UHS_HOST_ERROR_TIMEOUT:
          retry_count++;
          if (retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM)
            goto breakout;
          break;
        case UHS_HOST_ERROR_TOGERR:
          // yes, we flip it wrong here so that next time it is actually correct!
          pep->bmSndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
          regWr(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0); // set toggle value
          break;
        default:
          goto breakout;
      } // switch (rcode

      /* process NAK according to Host out NAK bug */
      regWr(rSNDBC, 0);
      regWr(rSNDFIFO, *data_p);
      regWr(rSNDBC, bytes_tosend);
      regWr(rHXFR, (MAX3421E_tokOUT | pep->epAddr)); // dispatch packet
      while (!(regRd(rHIRQ) & bmHXFRDNIRQ)); // wait for the completion IRQ
      regWr(rHIRQ, bmHXFRDNIRQ); // clear IRQ
      rcode = (regRd(rHRSL) & 0x0F);
      SYSTEM_OR_SPECIAL_YIELD();
    } // while (rcode && ....
    bytes_left -= bytes_tosend;
    data_p += bytes_tosend;
  } // while (bytes_left...
  breakout:

  pep->bmSndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 1 : 0; // bmSNDTOG1 : bmSNDTOG0;  // update toggle
  return (rcode); // should be 0 in all cases
}

/**
 * Send the actual packet.
 *
 * @param token
 * @param ep Endpoint
 * @param nak_limit how many NAKs before aborting, 0 == exit after timeout
 * @return 0 on success, 0xFF indicates NAK timeout. @see
 */
/* Assumes peripheral address is set and relevant buffer is loaded/empty       */
/* If NAK, tries to re-send up to nak_limit times                                                   */
/* If nak_limit == 0, do not count NAKs, exit after timeout                                         */
/* If bus timeout, re-sends up to USB_RETRY_LIMIT times                                             */

/* return codes 0x00-0x0F are HRSLT( 0x00 being success ), 0xFF means timeout                       */
uint8_t UHS_NI MAX3421E_HOST::dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit) {
  unsigned long timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;
  uint8_t tmpdata;
  uint8_t rcode = UHS_HOST_ERROR_NONE;
  uint8_t retry_count = 0;
  uint16_t nak_count = 0;

  for (;;) {
    regWr(rHXFR, (token | ep)); // launch the transfer
    while (long(millis() - timeout) < 0L) { // wait for transfer completion
      SYSTEM_OR_SPECIAL_YIELD();
      tmpdata = regRd(rHIRQ);

      if (tmpdata & bmHXFRDNIRQ) {
        regWr(rHIRQ, bmHXFRDNIRQ); // clear the interrupt
        //rcode = 0x00;
        break;
      } // if (tmpdata & bmHXFRDNIRQ

    } // while (millis() < timeout

    rcode = (regRd(rHRSL) & 0x0F); // analyze transfer result

    switch (rcode) {
      case UHS_HOST_ERROR_NAK:
        nak_count++;
        if (nak_limit && (nak_count == nak_limit))
          return (rcode);
        delayMicroseconds(200);
        break;
      case UHS_HOST_ERROR_TIMEOUT:
        retry_count++;
        if (retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM)
          return (rcode);
        break;
      default:
        return (rcode);
    } // switch (rcode)
  }
}

//
// NULL is error, we don't need to know the reason.
//

UHS_EpInfo * UHS_NI MAX3421E_HOST::ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t *dataptr) {
  uint8_t rcode;
  UHS_EpInfo *pep = NULL;
  uint16_t nak_limit = 0;
  rcode = SetAddress(addr, 0, &pep, nak_limit);

  if (!rcode) {

    bytesWr(rSUDFIFO, 8, (uint8_t*)(&Request)); // transfer to setup packet FIFO

    rcode = dispatchPkt(MAX3421E_tokSETUP, 0, nak_limit); // dispatch packet
    if (!rcode) {
      if (dataptr != NULL) {
        if (((Request)/* bmReqType*/ & 0x80) == 0x80) {
          pep->bmRcvToggle = 1; //bmRCVTOG1;
        } else {
          pep->bmSndToggle = 1; //bmSNDTOG1;
        }
      }
    } else {
      pep = NULL;
    }
  }
  return pep;
}

uint8_t UHS_NI MAX3421E_HOST::ctrlReqRead(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint16_t nbytes, uint8_t *dataptr) {
  *read = 0;
  uint16_t nak_limit = 0;
  MAX_HOST_DEBUG(PSTR("ctrlReqRead left: %i\r\n"), *left);
  if (*left) {
    again:
    *read = nbytes;
    uint8_t rcode = InTransfer(pep, nak_limit, read, dataptr);
    if (rcode == UHS_HOST_ERROR_TOGERR) {
      // yes, we flip it wrong here so that next time it is actually correct!
      pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
      goto again;
    }

    if (rcode) {
      MAX_HOST_DEBUG(PSTR("ctrlReqRead ERROR: %2.2x, left: %i, read %i\r\n"), rcode, *left, *read);
      return rcode;
    }
    *left -= *read;
    MAX_HOST_DEBUG(PSTR("ctrlReqRead left: %i, read %i\r\n"), *left, *read);
  }
  return 0;
}

uint8_t UHS_NI MAX3421E_HOST::ctrlReqClose(UHS_EpInfo *pep, uint8_t bmReqType, uint16_t left, uint16_t nbytes, uint8_t *dataptr) {
  uint8_t rcode = 0;

  //MAX_HOST_DEBUG(PSTR("Closing"));
  if (((bmReqType & 0x80) == 0x80) && pep && left && dataptr) {
    MAX_HOST_DEBUG(PSTR("ctrlReqRead Sinking %i\r\n"), left);
    // If reading, sink the rest of the data.
    while (left) {
      uint16_t read = nbytes;
      rcode = InTransfer(pep, 0, &read, dataptr);
      if (rcode == UHS_HOST_ERROR_TOGERR) {
        // yes, we flip it wrong here so that next time it is actually correct!
        pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
        continue;
      }
      if (rcode) break;
      left -= read;
      if (read < nbytes) break;
    }
  }
  if (!rcode) {
    //Serial.println("Dispatching");
    rcode = dispatchPkt(((bmReqType & 0x80) == 0x80) ? MAX3421E_tokOUTHS : MAX3421E_tokINHS, 0, 0); //GET if direction
  //} else {
    //Serial.println("Bypassed Dispatch");
  }
  return rcode;
}

/**
 * Bottom half of the ISR task
 */
void UHS_NI MAX3421E_HOST::ISRbottom() {
  uint8_t x;
  //        Serial.print("Enter ");
  //        Serial.print((uint32_t)this,HEX);
  //        Serial.print(" ");
  //        Serial.println(usb_task_state, HEX);

  DDSB();
  if (condet) {
    VBUS_changed();
    #if USB_HOST_SHIELD_USE_ISR
      noInterrupts();
    #endif
    condet = false;
    #if USB_HOST_SHIELD_USE_ISR
      interrupts();
    #endif
  }
  switch (usb_task_state) {
    case UHS_USB_HOST_STATE_INITIALIZE:
      // should never happen...
      MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_INITIALIZE\r\n"));
      busprobe();
      VBUS_changed();
      break;
    case UHS_USB_HOST_STATE_DEBOUNCE:
      MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_DEBOUNCE\r\n"));
      // This seems to not be needed. The host controller has debounce built in.
      sof_countdown = UHS_HOST_DEBOUNCE_DELAY_MS;
      usb_task_state = UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE;
      break;
    case UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE:
      MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE\r\n"));
      if (!sof_countdown) usb_task_state = UHS_USB_HOST_STATE_RESET_DEVICE;
      break;
    case UHS_USB_HOST_STATE_RESET_DEVICE:
      MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_RESET_DEVICE\r\n"));
      busevent = true;
      usb_task_state = UHS_USB_HOST_STATE_RESET_NOT_COMPLETE;
      regWr(rHIRQ, bmBUSEVENTIRQ); // see data sheet.
      regWr(rHCTL, bmBUSRST); // issue bus reset
      break;
    case UHS_USB_HOST_STATE_RESET_NOT_COMPLETE:
      MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_RESET_NOT_COMPLETE\r\n"));
      if (!busevent) usb_task_state = UHS_USB_HOST_STATE_WAIT_BUS_READY;
      break;
    case UHS_USB_HOST_STATE_WAIT_BUS_READY:
      MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_WAIT_BUS_READY\r\n"));
      usb_task_state = UHS_USB_HOST_STATE_CONFIGURING;
      break; // don't fall through

    case UHS_USB_HOST_STATE_CONFIGURING:
      usb_task_state = UHS_USB_HOST_STATE_CHECK;
      x = Configuring(0, 1, usb_host_speed);
      usb_error = x;
      if (usb_task_state == UHS_USB_HOST_STATE_CHECK) {
        if (x) {
          MAX_HOST_DEBUG(PSTR("Error 0x%2.2x"), x);
          if (x == UHS_HOST_ERROR_JERR) {
            usb_task_state = UHS_USB_HOST_STATE_IDLE;
          } else if (x != UHS_HOST_ERROR_DEVICE_INIT_INCOMPLETE) {
            usb_error = x;
            usb_task_state = UHS_USB_HOST_STATE_ERROR;
          }
        } else
          usb_task_state = UHS_USB_HOST_STATE_CONFIGURING_DONE;
      }
      break;

    case UHS_USB_HOST_STATE_CHECK:
      // Serial.println((uint32_t)__builtin_return_address(0), HEX);
      break;
    case UHS_USB_HOST_STATE_CONFIGURING_DONE:
      usb_task_state = UHS_USB_HOST_STATE_RUNNING;
      break;

    #ifdef USB_HOST_MANUAL_POLL
      case UHS_USB_HOST_STATE_RUNNING:
      case UHS_USB_HOST_STATE_ERROR:
      case UHS_USB_HOST_STATE_IDLE:
      case UHS_USB_HOST_STATE_ILLEGAL:
        enable_frame_irq(false);
        break;
    #else
      case UHS_USB_HOST_STATE_RUNNING:
        Poll_Others();
        for (x = 0; (usb_task_state == UHS_USB_HOST_STATE_RUNNING) && (x < UHS_HOST_MAX_INTERFACE_DRIVERS); x++) {
          if (devConfig[x]) {
            if (devConfig[x]->bPollEnable) devConfig[x]->Poll();
          }
        }
        // fall thru
    #endif
    default:
      // Do nothing
      break;
  } // switch ( usb_task_state )
  DDSB();
  #if USB_HOST_SHIELD_USE_ISR
    if (condet) {
      VBUS_changed();
      noInterrupts();
      condet = false;
      interrupts();
    }
  #endif
  #ifdef USB_HOST_SHIELD_TIMING_PIN
    // My counter/timer can't work on an inverted gate signal
    // so we gate using a high pulse -- AJK
    UHS_PIN_WRITE(USB_HOST_SHIELD_TIMING_PIN, LOW);
  #endif
  //usb_task_polling_disabled--;
  EnablePoll();
  DDSB();
}

/* USB main task. Services the MAX3421e */
#if !USB_HOST_SHIELD_USE_ISR
  void UHS_NI MAX3421E_HOST::ISRTask() {}
  void UHS_NI MAX3421E_HOST::Task()
#else
  void UHS_NI MAX3421E_HOST::Task() {
    #ifdef USB_HOST_MANUAL_POLL
      if (usb_task_state == UHS_USB_HOST_STATE_RUNNING) {
        noInterrupts();
        for (uint8_t x = 0; x < UHS_HOST_MAX_INTERFACE_DRIVERS; x++)
          if (devConfig[x] && devConfig[x]->bPollEnable)
            devConfig[x]->Poll();
        interrupts();
      }
    #endif
  }

  void UHS_NI MAX3421E_HOST::ISRTask()
#endif
{
  DDSB();

  #ifndef SWI_IRQ_NUM
    suspend_host();
    #if USB_HOST_SHIELD_USE_ISR
      // Enable interrupts
      interrupts();
    #endif
  #endif

  counted = false;
  if (!MARLIN_UHS_READ_IRQ()) {
    uint8_t HIRQALL = regRd(rHIRQ); // determine interrupt source
    uint8_t HIRQ = HIRQALL & IRQ_CHECK_MASK;
    uint8_t HIRQ_sendback = 0x00;

    if ((HIRQ & bmCONDETIRQ) || (HIRQ & bmBUSEVENTIRQ)) {
      MAX_HOST_DEBUG
      (PSTR("\r\nBEFORE CDIRQ %s BEIRQ %s resetting %s state 0x%2.2x\r\n"),
        (HIRQ & bmCONDETIRQ) ? "T" : "F",
        (HIRQ & bmBUSEVENTIRQ) ? "T" : "F",
        doingreset ? "T" : "F",
        usb_task_state
      );
    }
    // ALWAYS happens BEFORE or WITH CONDETIRQ
    if (HIRQ & bmBUSEVENTIRQ) {
      HIRQ_sendback |= bmBUSEVENTIRQ;
      if (!doingreset) condet = true;
      busprobe();
      busevent = false;
    }

    if (HIRQ & bmCONDETIRQ) {
      HIRQ_sendback |= bmCONDETIRQ;
      if (!doingreset) condet = true;
      busprobe();
    }

    #if 1
      if ((HIRQ & bmCONDETIRQ) || (HIRQ & bmBUSEVENTIRQ)) {
        MAX_HOST_DEBUG
        (PSTR("\r\nAFTER CDIRQ %s BEIRQ %s resetting %s state 0x%2.2x\r\n"),
          (HIRQ & bmCONDETIRQ) ? "T" : "F",
          (HIRQ & bmBUSEVENTIRQ) ? "T" : "F",
          doingreset ? "T" : "F",
          usb_task_state
        );
      }
    #endif

    if (HIRQ & bmFRAMEIRQ) {
      HIRQ_sendback |= bmFRAMEIRQ;
      if (sof_countdown) {
        sof_countdown--;
        counted = true;
      }
      sofevent = false;
    }

    //MAX_HOST_DEBUG(PSTR("\r\n%s%s%s\r\n"),
    //        sof_countdown ? "T" : "F",
    //        counted ? "T" : "F",
    //        usb_task_polling_disabled? "T" : "F");
    DDSB();
    regWr(rHIRQ, HIRQ_sendback);
    #ifndef SWI_IRQ_NUM
      resume_host();
      #if USB_HOST_SHIELD_USE_ISR
        // Disable interrupts
        noInterrupts();
      #endif
    #endif
    if (!sof_countdown && !counted && !usb_task_polling_disabled) {
      DisablePoll();
      //usb_task_polling_disabled++;
      #ifdef USB_HOST_SHIELD_TIMING_PIN
        // My counter/timer can't work on an inverted gate signal
        // so we gate using a high pulse -- AJK
        UHS_PIN_WRITE(USB_HOST_SHIELD_TIMING_PIN, HIGH);
      #endif

      #ifdef SWI_IRQ_NUM
        //MAX_HOST_DEBUG(PSTR("--------------- Doing SWI ----------------"));
        exec_SWI(this);
      #else
        #if USB_HOST_SHIELD_USE_ISR
          // Enable interrupts
          interrupts();
        #endif
        ISRbottom();
      #endif // SWI_IRQ_NUM
    }
  }
}

#if 0
  DDSB();
#endif

#else
  #error "Never include USB_HOST_SHIELD_INLINE.h, include UHS_host.h instead"
#endif // USB_HOST_SHIELD_LOADED
