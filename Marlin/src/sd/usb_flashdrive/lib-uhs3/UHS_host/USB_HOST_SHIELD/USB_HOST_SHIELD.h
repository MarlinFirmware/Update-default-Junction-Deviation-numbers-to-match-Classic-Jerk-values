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

#ifndef USB_HOST_SHIELD_H
#define USB_HOST_SHIELD_H

// uncomment to get 'printf' console debugging. NOT FOR UNO!
//#define DEBUG_PRINTF_EXTRA_HUGE_USB_HOST_SHIELD

#ifdef LOAD_USB_HOST_SHIELD
#include "UHS_max3421e.h"
#include <SPI.h>

#ifndef SPI_HAS_TRANSACTION
#error "Your SPI library installation is too old."
#else
#ifndef SPI_ATOMIC_VERSION
#warning "Your SPI library installation lacks 'SPI_ATOMIC_VERSION'. Please complain to the maintainer."
#elif SPI_ATOMIC_VERSION < 1
#error "Your SPI library installation is too old."
#endif

#endif
#if DEBUG_PRINTF_EXTRA_HUGE
#ifdef DEBUG_PRINTF_EXTRA_HUGE_USB_HOST_SHIELD
#define MAX_HOST_DEBUG(...) printf_P(__VA_ARGS__)
#else
#define MAX_HOST_DEBUG(...) VOID0
#endif
#else
#define MAX_HOST_DEBUG(...) VOID0
#endif

#ifndef USB_HOST_SHIELD_USE_ISR
#ifdef USE_MULTIPLE_APP_API
#define USB_HOST_SHIELD_USE_ISR 0
#else
#define USB_HOST_SHIELD_USE_ISR 1
#endif
#else
#define USB_HOST_SHIELD_USE_ISR 1
#endif

#if !USB_HOST_SHIELD_USE_ISR
#error NOISR Polled mode _NOT SUPPORTED YET_

//
// Polled defaults
//
#ifdef BOARD_BLACK_WIDDOW
#define UHS_MAX3421E_SS_ 6
#define UHS_MAX3421E_INT_ 3
#elif defined(CORE_TEENSY) && (defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__))
#if EXT_RAM
// Teensy++ 2.0 with XMEM2
#define UHS_MAX3421E_SS_ 20
#define UHS_MAX3421E_INT_ 7
#else
#define UHS_MAX3421E_SS_ 9
#define UHS_MAX3421E_INT_ 8
#endif
#define UHS_MAX3421E_SPD
#elif defined(ARDUINO_AVR_ADK)
#define UHS_MAX3421E_SS_ 53
#define UHS_MAX3421E_INT_ 54
#elif defined(ARDUINO_AVR_BALANDUINO)
#define UHS_MAX3421E_SS_ 20
#define UHS_MAX3421E_INT_ 19
#else
#define UHS_MAX3421E_SS_ 10
#define UHS_MAX3421E_INT_ 9
#endif

#else
#ifdef ARDUINO_ARCH_PIC32
// PIC32 only allows edge interrupts, isn't that lovely? We'll emulate it...
#if CHANGE < 2
#error core too old.
#endif

#define IRQ_IS_EDGE 0
#ifndef digitalPinToInterrupt
// great, this isn't implemented.
#warning digitalPinToInterrupt is not defined, complain here https://github.com/chipKIT32/chipKIT-core/issues/114
#if defined(_BOARD_UNO_) || defined(_BOARD_UC32_)
#define digitalPinToInterrupt(p) ((p) == 2 ? 1 : ((p) == 7 ? 2 : ((p) == 8 ? 3 : ((p) == 35 ? 4 : ((p) == 38 ? 0 : NOT_AN_INTERRUPT)))))
#warning digitalPinToInterrupt is now defined until this is taken care of.
#else
#error digitalPinToInterrupt not defined for your board, complain here https://github.com/chipKIT32/chipKIT-core/issues/114
#endif
#endif
#else
#define IRQ_IS_EDGE 0
#endif

// More stupidity from our friends @ Sony...
#ifdef ARDUINO_spresense_ast
#ifndef NOT_AN_INTERRUPT
#define NOT_AN_INTERRUPT -1
#endif
#endif

// SAMD uses an enum for this instead of a define. Isn't that just dandy?
#if !defined(NOT_AN_INTERRUPT) && !defined(ARDUINO_ARCH_SAMD)
#warning NOT_AN_INTERRUPT not defined, possible problems ahead.
#warning If NOT_AN_INTERRUPT is an enum or something else, complain to UHS30 developers on github.
#warning Otherwise complain to your board core developer/maintainer.
#define NOT_AN_INTERRUPT -1
#endif

//
// Interrupt defaults. Int0 or Int1
//
#ifdef BOARD_BLACK_WIDDOW
#error "HELP! Please send us an email, I don't know the values for Int0 and Int1 on the Black Widow board!"
#elif defined(ARDUINO_AVR_ADK)
#define UHS_MAX3421E_SS_ 53
#define UHS_MAX3421E_INT_ 54
#elif defined(ARDUINO_spresense_ast)
#define UHS_MAX3421E_SS_ 21
#define UHS_MAX3421E_INT_ 20
#define SPIclass SPI5
//#define UHS_MAX3421E_SPD 100000
#elif defined(CORE_TEENSY) && (defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__))

// TODO:

#if EXT_RAM
// Teensy++ 2.0 with XMEM2
#define UHS_MAX3421E_SS_ 20
#define UHS_MAX3421E_INT_ 7
#else
#define UHS_MAX3421E_SS_ 9
#define UHS_MAX3421E_INT_ 8
#endif

#elif defined(ARDUINO_AVR_BALANDUINO)
#error "ISR mode is currently not supported on the Balanduino. Please set USB_HOST_SHIELD_USE_ISR to 0."
#else
#define UHS_MAX3421E_SS_ 10
#ifdef __AVR__
#ifdef __AVR_ATmega32U4__
#define INT_FOR_PIN2 1
#define INT_FOR_PIN3 0
#else
// Everybody else???
#define INT_FOR_PIN2 0
#define INT_FOR_PIN3 1
#endif
#define UHS_MAX3421E_INT_ 3
#else
// Non-avr
#ifdef ARDUINO_ARCH_PIC32
// UNO32 External Interrupts:
// Pin 38 (INT0), Pin 2 (INT1), Pin 7 (INT2), Pin 8 (INT3), Pin 35 (INT4)
#define UHS_MAX3421E_INT_ 7
#else
#define UHS_MAX3421E_INT_ 9
#endif
#endif
#endif
#endif

#ifdef NO_AUTO_SPEED
// Ugly details section...
// MAX3421E characteristics
// SPI Serial - Clock Input. An external SPI master supplies SCLK with frequencies up to 26MHz. The
// logic level is referenced to the voltage on VL. Data is clocked into the SPI slave inter face on the
// rising edge of SCLK. Data is clocked out of the SPI slave interface on the falling edge of SCLK.
// Serial Clock (SCLK) Period 38.4ns minimum. 17ns minimum pulse width. VL >2.5V
// SCLK Fall to MISO Propagation Delay 14.2ns
// SCLK Fall to MOSI Propagation Delay 14.2ns
// SCLK Fall to MOSI Drive 3.5ns
// Theoretical deadline for reply 17.7ns
// 26MHz 38.4615ns period <-- MAX3421E theoretical maximum

#ifndef UHS_MAX3421E_SPD
#ifdef ARDUINO_SAMD_ZERO
// Zero violates spec early, needs a long setup time, or doesn't like high latency.
#define UHS_MAX3421E_SPD 10000000
#elif defined(ARDUINO_ARCH_PIC32)
// PIC MX 5/6/7 characteristics
// 25MHZ 40ns period <-- PIC MX  5/6/7 theoretical maximum
// pulse width minimum Tsclk/2ns
// Trise/fall 10ns maximum. 5ns is typical but not guaranteed.
// Tsetup minimum for MISO 10ns.
// We are in violation by 7.7ns @ 25MHz due to latency alone.
// Even reading at end of data cycle, we only have a 2.3ns window.
// This is too narrow to to compensate for capacitance, trace lengths, and noise.

// 17.7ns + 10ns = 27.7ns
// 18MHz fits and has enough slack time to compensate for capacitance, trace lengths, and noise.
// For high speeds the SMP bit is recommended too, which samples at the end instead of the middle.
// 20Mhz seems to work.

#define UHS_MAX3421E_SPD 20000000
#else
#define UHS_MAX3421E_SPD 25000000
#endif
#endif
#else
// We start at 25MHz, and back down until hardware can take it.
// Of course, SPI library can adjust this for us too.
// Why not 26MHz? Because I have not found any MCU board that
// can actually go that fast without problems.
// Could be a shield limitation too.
#ifndef UHS_MAX3421E_SPD
#define UHS_MAX3421E_SPD 25000000
#endif
#endif

#ifndef UHS_MAX3421E_INT
#define UHS_MAX3421E_INT UHS_MAX3421E_INT_
#endif

#ifndef UHS_MAX3421E_SS
#define UHS_MAX3421E_SS UHS_MAX3421E_SS_
#endif

// NOTE: On the max3421e the irq enable and irq bits are in the same position.

// IRQs used if CPU polls
#define   ENIBITSPOLLED (bmCONDETIE | bmBUSEVENTIE  | bmFRAMEIE)
// IRQs used if CPU is interrupted
#define      ENIBITSISR (bmCONDETIE | bmBUSEVENTIE | bmFRAMEIE /* | bmRCVDAVIRQ | bmSNDBAVIRQ | bmHXFRDNIRQ */ )

#if !USB_HOST_SHIELD_USE_ISR
#define IRQ_CHECK_MASK (ENIBITSPOLLED & ICLRALLBITS)
#define IRQ_IS_EDGE 0
#else
#define IRQ_CHECK_MASK (ENIBITSISR & ICLRALLBITS)
#endif

#if IRQ_IS_EDGE
// Note: UNO32 Interrupts can only be RISING, or FALLING.
// This poses an interesting problem, since we want to use a LOW level.
// The MAX3421E provides for pulse width control for an IRQ.
// We do need to watch the timing on this, as a second IRQ could cause
// a missed IRQ, since we read the level of the line to check if the IRQ
// is actually for this chip. The only other alternative is to add a capacitor
// and an NPN transistor, and use two lines. We can try this first, though.
// Worse case, we can ignore reading the pin for verification on UNO32.
// Too bad there is no minimum low width setting.
//
//   Single    Clear     First  Second   Clear first      Clear last
//   IRQ       Single    IRQ    IRQ      Second active    pending IRQ
//      |      |         |      |        |                |
//      V      V         V      V        V                V
// _____        _________        _        _                _______
//      |______|         |______| |______| |______________|
//
#define IRQ_SENSE FALLING
#ifdef ARDUINO_ARCH_PIC32
//#define bmPULSEWIDTH PULSEWIDTH10_6
#define bmPULSEWIDTH 0
#define bmIRQ_SENSE 0
#else
#define bmPULSEWIDTH PULSEWIDTH1_3
#define bmIRQ_SENSE 0
#endif
#else
#ifndef IRQ_SENSE
#define IRQ_SENSE LOW
#endif
#ifndef bmPULSEWIDTH
#define bmPULSEWIDTH 0
#endif
#ifndef bmIRQ_SENSE
#define bmIRQ_SENSE bmINTLEVEL
#endif
#endif

class MAX3421E_HOST :
public UHS_USB_HOST_BASE
#ifdef SWI_IRQ_NUM
, public dyn_SWI
#endif
{
        // TODO: move these into the parent class.
        volatile uint8_t vbusState;
        volatile uint16_t sof_countdown;

        // TODO: pack into a struct/union and use one byte
        volatile bool busevent;
        volatile bool sofevent;
        volatile bool counted;
        volatile bool condet;
        volatile bool doingreset;

        #ifdef USB_HOST_MANUAL_POLL
                volatile bool frame_irq_enabled = false;

                bool enable_frame_irq(bool enable) {
                        const bool prev_state = frame_irq_enabled;
                        if(prev_state != enable) {
                                if(enable)
                                        regWr(rHIEN, regRd(rHIEN) |  bmFRAMEIE);
                                else
                                        regWr(rHIEN, regRd(rHIEN) & ~bmFRAMEIE);
                                frame_irq_enabled = enable;
                        }
                        return prev_state;
                }
        #endif

public:
        SPISettings MAX3421E_SPI_Settings;
        uint8_t ss_pin;
        uint8_t irq_pin;
        // Will use the defaults UHS_MAX3421E_SS, UHS_MAX3421E_INT and speed

        UHS_NI MAX3421E_HOST() {
                sof_countdown = 0;
                busevent = false;
                doingreset = false;
                sofevent = false;
                condet = false;
                ss_pin = UHS_MAX3421E_SS;
                irq_pin = UHS_MAX3421E_INT;
                MAX3421E_SPI_Settings = SPISettings(UHS_MAX3421E_SPD, MSBFIRST, SPI_MODE0);
                hub_present = 0;
        };

        // Will use user supplied pins, and UHS_MAX3421E_SPD

        UHS_NI MAX3421E_HOST(uint8_t pss, uint8_t pirq) {
                sof_countdown = 0;
                busevent = false;
                doingreset = false;
                sofevent = false;
                condet = false;
                ss_pin = pss;
                irq_pin = pirq;
                MAX3421E_SPI_Settings = SPISettings(UHS_MAX3421E_SPD, MSBFIRST, SPI_MODE0);
                hub_present = 0;
        };

        // Will use user supplied pins, and speed

        UHS_NI MAX3421E_HOST(uint8_t pss, uint8_t pirq, uint32_t pspd) {
                sof_countdown = 0;
                doingreset = false;
                busevent = false;
                sofevent = false;
                condet = false;
                ss_pin = pss;
                irq_pin = pirq;
                MAX3421E_SPI_Settings = SPISettings(pspd, MSBFIRST, SPI_MODE0);
                hub_present = 0;
        };

        virtual bool UHS_NI sof_delay(uint16_t x) {
#ifdef USB_HOST_MANUAL_POLL
                const bool saved_irq_state = enable_frame_irq(true);
#endif
                sof_countdown = x;
                while((sof_countdown != 0) && !condet) {
                        SYSTEM_OR_SPECIAL_YIELD();
#if !USB_HOST_SHIELD_USE_ISR
                        Task();
#endif
                }
#ifdef USB_HOST_MANUAL_POLL
                enable_frame_irq(saved_irq_state);
#endif
                //                Serial.println("...Wake");
                return (!condet);
        };

        virtual UHS_EpInfo *ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t *dataptr);

        virtual void UHS_NI vbusPower(VBUS_t state) {
                regWr(rPINCTL, (bmFDUPSPI | bmIRQ_SENSE) | (uint8_t)(state));
        };

        void UHS_NI Task();

        virtual uint8_t SetAddress(uint8_t addr, uint8_t ep, UHS_EpInfo **ppep, uint16_t &nak_limit);
        virtual uint8_t OutTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t nbytes, uint8_t *data);
        virtual uint8_t InTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t *nbytesptr, uint8_t *data);
        virtual uint8_t ctrlReqClose(UHS_EpInfo *pep, uint8_t bmReqType, uint16_t left, uint16_t nbytes, uint8_t *dataptr);
        virtual uint8_t ctrlReqRead(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint16_t nbytes, uint8_t *dataptr);
        virtual uint8_t dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit);

        void UHS_NI ReleaseChildren() {
                for(uint8_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++)
                        if(devConfig[i])
                                devConfig[i]->Release();
                hub_present = 0;
        };

        virtual bool IsHub(uint8_t klass) {
                if(klass == UHS_USB_CLASS_HUB) {
                        hub_present = bmHUBPRE;
                        return true;
                }
                return false;
        };

        virtual void VBUS_changed();

        virtual void UHS_NI doHostReset() {
#if USB_HOST_SHIELD_USE_ISR
                // Enable interrupts
                noInterrupts();
#endif
                doingreset = true;
                busevent = true;
                regWr(rHIRQ, bmBUSEVENTIRQ); // see data sheet.
                regWr(rHCTL, bmBUSRST); //issue bus reset
#if USB_HOST_SHIELD_USE_ISR
                DDSB();
                // Enable interrupts
                interrupts();
#endif
                while(busevent) {
                        DDSB();
                        SYSTEM_OR_SPECIAL_YIELD();
                }
#endif
#if USB_HOST_SHIELD_USE_ISR
                // Enable interrupts
                noInterrupts();
#endif
                #ifdef USB_HOST_MANUAL_POLL
                        enable_frame_irq(true);
                #endif
                sofevent = true;
#if USB_HOST_SHIELD_USE_ISR
                DDSB();
                // Enable interrupts
                interrupts();
#endif
                // Wait for SOF
                while(sofevent) {
                }
#if USB_HOST_SHIELD_USE_ISR
                // Enable interrupts
                noInterrupts();
#endif
                doingreset = false;
#if USB_HOST_SHIELD_USE_ISR
                DDSB();
                // Enable interrupts
                interrupts();
        };

        int16_t UHS_NI Init(int16_t mseconds);

        int16_t UHS_NI Init() {
                return Init(INT16_MIN);
        };

        void ISRTask();
        void ISRbottom();
        void busprobe();
        uint16_t reset();

        // MAX3421e specific
        void regWr(uint8_t reg, uint8_t data);
        void gpioWr(uint8_t data);
        uint8_t regRd(uint8_t reg);
        uint8_t gpioRd();
        uint8_t* bytesWr(uint8_t reg, uint8_t nbytes, uint8_t *data_p);
        uint8_t* bytesRd(uint8_t reg, uint8_t nbytes, uint8_t *data_p);

        // ARM/NVIC specific, used to emulate reentrant ISR.
#ifdef SWI_IRQ_NUM

        void dyn_SWISR() {
                ISRbottom();
        };
#endif

        virtual void UHS_NI suspend_host() {
                // Used on MCU that lack control of IRQ priority (AVR).
                // Suspends ISRs, for critical code. IRQ will be serviced after it is resumed.
                // NOTE: you must track the state yourself!
#ifdef __AVR__
                noInterrupts();
                detachInterrupt(UHS_GET_DPI(irq_pin));
                interrupts();
#endif
        };

        virtual void UHS_NI resume_host();
};
#ifndef SPIclass
#define SPIclass SPI
#endif
#ifndef USB_HOST_SHIELD_LOADED
#include "USB_HOST_SHIELD_INLINE.h"
#endif
#else
#error "define LOAD_USB_HOST_SHIELD in your sketch, never include USB_HOST_SHIELD.h in a driver."
#endif
#endif // USB_HOST_SHIELD_H
