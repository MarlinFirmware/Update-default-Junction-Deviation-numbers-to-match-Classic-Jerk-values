/* Copyright (C) 2015-2016 Andrew J. Kroll
   and
Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

Circuits At Home, LTD
Web      :  https://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */
#if !defined(USB_HOST_SHIELD_H) || defined(_max3421e_h_)
#error "Never include UHS_max3421e.h directly; include USB_HOST_SHIELD.h instead"
#else

#define _max3421e_h_

// MAX3421E register/bit names and bitmasks
#define SE0     0
#define SE1     1
#define FSHOST  2
#define LSHOST  3

/**
 * MAX3421E command byte format: rrrrr0wa where 'r' is register number
 */

// MAX3421E Registers in HOST mode.
#define        rRCVFIFO 0x08            // Receive FIFO Register
#define        rSNDFIFO 0x10            // Send FIFO Register
#define        rSUDFIFO 0x20            // Set Up Data FIFO Register
#define          rRCVBC 0x30            // Receive FIFO Byte Count Register
#define          rSNDBC 0x38            // Send FIFO Byte Count Register

// USB Interrupt Request Status (USBIRQ)
#define         rUSBIRQ 0x68            // USB Interrupt Request Register
#define       bmVBUSIRQ 0x40            // Vbus Present Interrupt Request
#define     bmNOVBUSIRQ 0x20            // Vbus Absent Interrupt Request
#define      bmOSCOKIRQ 0x01            // Oscillator OK Interrupt Request

// USB Interrupt Request Control (USBIEN)
#define         rUSBIEN 0x70            // USB Interrupt Request Enable Register
#define        bmVBUSIE bmVBUSIRQ       // Vbus Present Interrupt Request Enable
#define      bmNOVBUSIE bmNOVBUSIRQ     // Vbus Absent Interrupt Request Enable
#define       bmOSCOKIE bmOSCOKIRQ      // Oscillator OK Interrupt Request Enable

// (USBCTL)
#define         rUSBCTL 0x78            //15<<3
#define       bmCHIPRES 0x20            //b5
#define       bmPWRDOWN 0x10            //b4

// (CPUCTL)
#define         rCPUCTL 0x80            //16<<3
#define     bmPULSEWID1 0x80            //b7
#define     bmPULSEWID0 0x40            //b6
#define            bmIE 0x01            //b0

// bmPULSEWID1 bmPULSEWID0 Pulse width
// 0           0           10.6uS
// 0           1            5.3uS
// 1           0            2.6uS
// 1           1            1.3uS
#define  PULSEWIDTH10_6 (0)
#define   PULSEWIDTH5_3 (bmPULSEWID0)
#define   PULSEWIDTH2_6 (bmPULSEWID1)
#define   PULSEWIDTH1_3 (bmPULSEWID0 | bmPULSEWID1)

// (PINCTL)
#define         rPINCTL 0x88            //17<<3
#define       bmFDUPSPI 0x10            //b4
#define      bmINTLEVEL 0x08            //b3
#define        bmPOSINT 0x04            //b2
#define          bmGPXB 0x02            //b1
#define          bmGPXA 0x01            //b0

// GPX pin selections
#define     GPX_OPERATE 0x00            //
#define       GPX_VBDET 0x01            //
#define      GPX_BUSACT 0x02            //
#define         GPX_SOF 0x03            //

#define       rREVISION 0x90            //18<<3

// (IOPINS1)
#define        rIOPINS1 0xA0            //20<<3
#define        bmGPOUT0 0x01            //
#define        bmGPOUT1 0x02            //
#define        bmGPOUT2 0x04            //
#define        bmGPOUT3 0x08            //
#define         bmGPIN0 0x10            //
#define         bmGPIN1 0x20            //
#define         bmGPIN2 0x40            //
#define         bmGPIN3 0x80            //

// (IOPINS2)
#define        rIOPINS2 0xA8            //21<<3
#define        bmGPOUT4 0x01            //
#define        bmGPOUT5 0x02            //
#define        bmGPOUT6 0x04            //
#define        bmGPOUT7 0x08            //
#define         bmGPIN4 0x10            //
#define         bmGPIN5 0x20            //
#define         bmGPIN6 0x40            //
#define         bmGPIN7 0x80            //

// (GPINIRQ)
#define        rGPINIRQ 0xB0            //22<<3
#define      bmGPINIRQ0 0x01            //
#define      bmGPINIRQ1 0x02            //
#define      bmGPINIRQ2 0x04            //
#define      bmGPINIRQ3 0x08            //
#define      bmGPINIRQ4 0x10            //
#define      bmGPINIRQ5 0x20            //
#define      bmGPINIRQ6 0x40            //
#define      bmGPINIRQ7 0x80            //

// (GPINIEN)
#define        rGPINIEN 0xB8            //23<<3
#define      bmGPINIEN0 0x01            //
#define      bmGPINIEN1 0x02            //
#define      bmGPINIEN2 0x04            //
#define      bmGPINIEN3 0x08            //
#define      bmGPINIEN4 0x10            //
#define      bmGPINIEN5 0x20            //
#define      bmGPINIEN6 0x40            //
#define      bmGPINIEN7 0x80            //

// (GPINPOL)
#define        rGPINPOL 0xC0            //24<<3
#define      bmGPINPOL0 0x01            //
#define      bmGPINPOL1 0x02            //
#define      bmGPINPOL2 0x04            //
#define      bmGPINPOL3 0x08            //
#define      bmGPINPOL4 0x10            //
#define      bmGPINPOL5 0x20            //
#define      bmGPINPOL6 0x40            //
#define      bmGPINPOL7 0x80            //

/**
 * If any data transfer errors occur, the HXFRDNIRQ asserts, while the RCVDAVIRQ does not.
 *
 * The CPU clears the SNDBAVIRQ by writing the SNDBC register.
 * The CPU should never directly clear the SNDBAVIRQ bit.
 */

// Host Interrupt Request Status (HIRQ)
#define           rHIRQ 0xC8            // Host Interrupt Request Register
#define   bmBUSEVENTIRQ 0x01            // BUS Reset Done or BUS Resume Interrupt Request
#define        bmRWUIRQ 0x02            // Remote Wakeup Interrupt Request
#define     bmRCVDAVIRQ 0x04            // Receive FIFO Data Available Interrupt Request
#define     bmSNDBAVIRQ 0x08            // Send Buffer Available Interrupt Request
#define      bmSUSDNIRQ 0x10            // Suspend operation Done Interrupt Request
#define     bmCONDETIRQ 0x20            // Peripheral Connect/Disconnect Interrupt Request
#define      bmFRAMEIRQ 0x40            // Frame Generator Interrupt Request
#define     bmHXFRDNIRQ 0x80            // Host Transfer Done Interrupt Request

// IRQs that are OK for the CPU to clear
#define     ICLRALLBITS (bmBUSEVENTIRQ | bmRWUIRQ | bmRCVDAVIRQ | bmSUSDNIRQ | bmCONDETIRQ | bmFRAMEIRQ | bmHXFRDNIRQ)

// Host Interrupt Request Control (HIEN)
#define           rHIEN 0xD0            //
#define    bmBUSEVENTIE bmBUSEVENTIRQ   // BUS Reset Done or BUS Resume Interrupt Request Enable
#define         bmRWUIE bmRWUIRQ        // Remote Wakeup Interrupt Request Enable
#define      bmRCVDAVIE bmRCVDAVIRQ     // Receive FIFO Data Available Interrupt Request Enable
#define      bmSNDBAVIE bmSNDBAVIRQ     // Send Buffer Available Interrupt Request Enable
#define       bmSUSDNIE bmSUSDNIRQ      // Suspend operation Done Interrupt Request Enable
#define      bmCONDETIE bmCONDETIRQ     // Peripheral Connect/Disconnect Interrupt Request Enable
#define       bmFRAMEIE bmFRAMEIRQ      // Frame Generator Interrupt Request Enable
#define      bmHXFRDNIE bmHXFRDNIRQ     // Host Transfer Done Interrupt Request Enable

// (MODE))
#define           rMODE 0xD8            //27<<3
#define          bmHOST 0x01            //
#define      bmLOWSPEED 0x02            //
#define        bmHUBPRE 0x04            //
#define     bmSOFKAENAB 0x08            //
#define        bmSEPIRQ 0x10            //
#define      bmDELAYISO 0x20            //
#define      bmDMPULLDN 0x40            //
#define      bmDPPULLDN 0x80            //

#define        rPERADDR 0xE0            //28<<3

// (HCTL)
#define           rHCTL 0xE8            //29<<3
#define        bmBUSRST 0x01            //
#define        bmFRMRST 0x02            //
#define     bmSAMPLEBUS 0x04            //
#define        bmSIGRSM 0x08            //
#define       bmRCVTOG0 0x10            //
#define       bmRCVTOG1 0x20            //
#define       bmSNDTOG0 0x40            //
#define       bmSNDTOG1 0x80            //

// Host transfer (HXFR)
#define           rHXFR 0xF0            //30<<3

// Host transfer token values for writing the HXFR register (R30)
// OR this bit field with the endpoint number in bits 3:0
#define       MAX3421E_tokSETUP 0x10    // HS=0, ISO=0, OUTNIN=0, SETUP=1
#define          MAX3421E_tokIN 0x00    // HS=0, ISO=0, OUTNIN=0, SETUP=0
#define         MAX3421E_tokOUT 0x20    // HS=0, ISO=0, OUTNIN=1, SETUP=0
#define        MAX3421E_tokINHS 0x80    // HS=1, ISO=0, OUTNIN=0, SETUP=0
#define       MAX3421E_tokOUTHS 0xA0    // HS=1, ISO=0, OUTNIN=1, SETUP=0
#define       MAX3421E_tokISOIN 0x40    // HS=0, ISO=1, OUTNIN=0, SETUP=0
#define      MAX3421E_tokISOOUT 0x60    // HS=0, ISO=1, OUTNIN=1, SETUP=0

// (HRSL)
#define           rHRSL 0xF8            //31<<3
#define      bmRCVTOGRD 0x10            //
#define      bmSNDTOGRD 0x20            //
#define       bmKSTATUS 0x40            //
#define       bmJSTATUS 0x80            //
#define           bmSE0 0x00            //SE0 - disconnect state
#define           bmSE1 0xC0            //SE1 - illegal state

#define    MODE_FS_HOST (bmDPPULLDN|bmDMPULLDN|bmHOST|bmSOFKAENAB)
#define    MODE_LS_HOST (bmDPPULLDN|bmDMPULLDN|bmHOST|bmLOWSPEED|bmSOFKAENAB)

#endif // _max3421e_h_
