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

#ifndef _UHS_host_h_
#error "Never include UHS_usbhost.h directly; include UHS_host.h instead"
#else
#ifndef _USBHOST_H_
#define _USBHOST_H_

// Very early prototypes
#ifdef UHS_LOAD_BT
void UHS_BT_SetUSBInterface(UHS_USB_HOST_BASE *host, ENUMERATION_INFO *ei);
void UHS_BT_ScanUninitialized(UHS_USB_HOST_BASE *host);
void UHS_BT_Poll(UHS_USB_HOST_BASE *host);
#endif
#ifdef UHS_LOAD_HID
void UHS_HID_SetUSBInterface(UHS_USB_HOST_BASE *host, ENUMERATION_INFO *ei);
void UHS_HID_ScanUninitialized(UHS_USB_HOST_BASE *host);
void UHS_HID_Poll(UHS_USB_HOST_BASE *host);
#endif

//#if defined(LOAD_UHS_CDC_ACM) || defined(LOAD_UHS_CDC_ACM_FTDI) || defined(LOAD_UHS_CDC_ACM_PROLIFIC) || defined(LOAD_UHS_CDC_ACM_XR21B1411)
//void UHS_CDC_ACM_SetUSBInterface(UHS_USB_HOST_BASE *host, ENUMERATION_INFO *ei);
//void UHS_CDC_ACM_ScanUninitialized(UHS_USB_HOST_BASE *host);
//void UHS_CDC_ACM_Poll(UHS_USB_HOST_BASE *host);
//#endif

class UHS_USBInterface; // forward class declaration

// enumerator to turn the VBUS on/off

typedef enum {
        vbus_on = 0,
        vbus_off = 1
} VBUS_t;

// All host SEI use this base class

class UHS_USB_HOST_BASE {
public:
        AddressPool addrPool;
        UHS_USBInterface* devConfig[UHS_HOST_MAX_INTERFACE_DRIVERS];
        volatile uint8_t usb_error;
        volatile uint8_t usb_task_state;
        volatile uint8_t usb_task_polling_disabled;
        volatile uint8_t usb_host_speed;
        volatile uint8_t hub_present;

        UHS_USB_HOST_BASE() {
                for(uint16_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++) {
                        devConfig[i] = NULL;
                }
                usb_task_polling_disabled = 0;
                usb_task_state = UHS_USB_HOST_STATE_INITIALIZE; //set up state machine
                usb_host_speed = 0;
                usb_error = 0;
        };

        /////////////////////////////////////////////
        //
        // Virtual methods that interface to the SIE
        // Overriding each is mandatory.
        //
        /////////////////////////////////////////////

        /**
         * Delay for x milliseconds
         * Override if your controller provides an SOF IRQ, which may involve
         * some sort of reentrant ISR or workaround with interrupts enabled.
         *
         * @param x how many milliseconds to delay
         * @return true if delay completed without a state change, false if delay aborted
         */
        virtual bool UHS_NI sof_delay(uint16_t x) {
                if(!(usb_task_state & UHS_USB_HOST_STATE_MASK)) return false;
                uint8_t current_state = usb_task_state;
                while(current_state == usb_task_state && x--) {
                        delay(1);
                }
                return (current_state == usb_task_state);
        };

        virtual UHS_EpInfo * UHS_NI ctrlReqOpen(NOTUSED(uint8_t addr), NOTUSED(uint64_t Request), NOTUSED(uint8_t *dataptr)) {
                return NULL;
        };

        virtual void UHS_NI vbusPower(NOTUSED(VBUS_t state)) {
        };

        virtual void UHS_NI Task() {
        };

        virtual uint8_t UHS_NI SetAddress(NOTUSED(uint8_t addr), NOTUSED(uint8_t ep), NOTUSED(UHS_EpInfo **ppep), NOTUSED(uint16_t &nak_limit)) {
                return UHS_HOST_ERROR_NOT_IMPLEMENTED;
        };

        virtual uint8_t UHS_NI OutTransfer(NOTUSED(UHS_EpInfo *pep), NOTUSED(uint16_t nak_limit), NOTUSED(uint16_t nbytes), NOTUSED(uint8_t *data)) {
                return UHS_HOST_ERROR_NOT_IMPLEMENTED;
        };

        virtual uint8_t UHS_NI InTransfer(NOTUSED(UHS_EpInfo *pep), NOTUSED(uint16_t nak_limit), NOTUSED(uint16_t *nbytesptr), NOTUSED(uint8_t *data)) {
                return UHS_HOST_ERROR_NOT_IMPLEMENTED;
        };

        virtual uint8_t UHS_NI ctrlReqClose(NOTUSED(UHS_EpInfo *pep), NOTUSED(uint8_t bmReqType), NOTUSED(uint16_t left), NOTUSED(uint16_t nbytes), NOTUSED(uint8_t *dataptr)) {
                return UHS_HOST_ERROR_NOT_IMPLEMENTED;
        };

        virtual uint8_t UHS_NI ctrlReqRead(NOTUSED(UHS_EpInfo *pep), NOTUSED(uint16_t *left), NOTUSED(uint16_t *read), NOTUSED(uint16_t nbytes), NOTUSED(uint8_t *dataptr)) {
                return UHS_HOST_ERROR_NOT_IMPLEMENTED;
        };

        virtual uint8_t UHS_NI dispatchPkt(NOTUSED(uint8_t token), NOTUSED(uint8_t ep), NOTUSED(uint16_t nak_limit)) {
                return UHS_HOST_ERROR_NOT_IMPLEMENTED;
        };

        virtual uint8_t UHS_NI init() {
                return 0;
        };

        virtual void UHS_NI doHostReset() {
        };

        virtual int16_t UHS_NI Init(NOTUSED(int16_t mseconds)) {
                return -1;
        };

        virtual int16_t UHS_NI Init() {
                return Init(INT16_MIN);
        };

        virtual uint8_t hwlPowerUp() {
                /* This is for machine specific support to enable/power up the USB HW to operate*/
                return UHS_HOST_ERROR_NOT_IMPLEMENTED;
        };

        virtual uint8_t hwPowerDown() {
                /* This is for machine specific support to disable/powerdown the USB Hw */
                return UHS_HOST_ERROR_NOT_IMPLEMENTED;
        };

        virtual bool IsHub(uint8_t klass) {
                return (klass == UHS_USB_CLASS_HUB);
        };

        virtual void UHS_NI suspend_host() {
                // Used on MCU that lack control of IRQ priority (AVR).
                // Suspends ISRs, for critical code. IRQ will be serviced after it is resumed.
                // NOTE: you must track the state yourself!
        };

        virtual void UHS_NI resume_host() {
                // Used on MCU that lack control of IRQ priority (AVR).
                // Resumes ISRs.
                // NOTE: you must track the state yourself!
        };

        /////////////////////////////////////////////
        //
        // Built-ins, No need to override
        //
        /////////////////////////////////////////////
        // these two probably will go away, and won't be used, TBD
        inline void Poll_Others() {
#ifdef UHS_LOAD_BT
                UHS_BT_Poll(this);
#endif
#ifdef UHS_LOAD_HID
                UHS_HID_Poll(this);
#endif
        }

        inline void DisablePoll() {
                noInterrupts();
                usb_task_polling_disabled++;
                DDSB();
                interrupts();
        }

        inline void EnablePoll() {
                noInterrupts();
                usb_task_polling_disabled--;
                DDSB();
                interrupts();
        }

        uint8_t UHS_NI seekInterface(ENUMERATION_INFO *ei, uint16_t inf, USB_FD_CONFIGURATION_DESCRIPTOR *ucd);

        uint8_t UHS_NI setEpInfoEntry(uint8_t addr, uint8_t iface, uint8_t epcount, volatile UHS_EpInfo* eprecord_ptr);

        uint8_t UHS_NI EPClearHalt(uint8_t addr, uint8_t ep);

        uint8_t UHS_NI ctrlReq(uint8_t addr, uint64_t Request, uint16_t nbytes, uint8_t *dataptr);

        uint8_t UHS_NI getDevDescr(uint8_t addr, uint16_t nbytes, uint8_t *dataptr);

        uint8_t UHS_NI getConfDescr(uint8_t addr, uint16_t nbytes, uint8_t conf, uint8_t *dataptr);

        uint8_t UHS_NI setAddr(uint8_t oldaddr, uint8_t newaddr);

        uint8_t UHS_NI setConf(uint8_t addr, uint8_t conf_value);

        uint8_t UHS_NI getStrDescr(uint8_t addr, uint16_t nbytes, uint8_t index, uint16_t langid, uint8_t *dataptr);

        void UHS_NI ReleaseDevice(uint8_t addr);

        uint8_t UHS_NI Configuring(uint8_t parent, uint8_t port, uint8_t speed);

        void UHS_NI DeviceDefaults(uint8_t maxep, UHS_USBInterface *device);

        UHS_EpInfo* UHS_NI getEpInfoEntry(uint8_t addr, uint8_t ep);

        inline uint8_t getUsbTaskState() {
                return ( usb_task_state);
        };

        inline AddressPool* GetAddressPool() {
                return &addrPool;
        };

        int UHS_NI RegisterDeviceClass(UHS_USBInterface *pdev) {
                for(uint8_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++) {
                        if(!devConfig[i]) {
                                devConfig[i] = pdev;
                                return i;
                        }
                }
                //return UHS_HOST_ERROR_CANT_REGISTER_DEVICE_CLASS;
                return -1;
        };
#if 0

        inline void ForEachUsbDevice(UsbDeviceHandleFunc pfunc) {
                addrPool.ForEachUsbDevice(pfunc);
        };
#endif

        uint8_t TestInterface(ENUMERATION_INFO *ei);
        uint8_t enumerateInterface(ENUMERATION_INFO *ei);
        uint8_t getNextInterface(ENUMERATION_INFO *ei, UHS_EpInfo *pep, uint8_t data[], uint16_t *left, uint16_t *read, uint8_t *offset);
        uint8_t initDescrStream(ENUMERATION_INFO *ei, USB_FD_CONFIGURATION_DESCRIPTOR *ucd, UHS_EpInfo *pep, uint8_t *data, uint16_t *left, uint16_t *read, uint8_t *offset);
        uint8_t outTransfer(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t *data);
        uint8_t inTransfer(uint8_t addr, uint8_t ep, uint16_t *nbytesptr, uint8_t *data);
        uint8_t doSoftReset(uint8_t parent, uint8_t port, uint8_t address);
        uint8_t getone(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint8_t *dataptr, uint8_t *offset);
        uint8_t eat(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint8_t *dataptr, uint8_t *offset, uint16_t *yum);

};

// All device interface drivers use this subclass

class UHS_USBInterface {
public:

        UHS_USB_HOST_BASE *pUsb; // Parent USB host
        volatile uint8_t bNumEP; // total number of EP in this interface
        volatile UHS_EpInfo epInfo[16]; // This is a stub, override in the driver.

        volatile uint8_t bAddress; // address of the device
        volatile uint8_t bConfNum; // configuration number
        volatile uint8_t bIface; // interface value
        volatile bool bPollEnable; // poll enable flag, operating status
        volatile uint32_t qNextPollTime; // next poll time

        /**
         * Resets interface driver to unused state. You should override this in
         * your driver if it requires extra class variable cleanup.
         */
        virtual void DriverDefaults() {
                printf("Default driver defaults.\r\n");
                pUsb->DeviceDefaults(bNumEP, this);
        };

        /**
         * Checks if this interface is supported.
         * Executed called when new devices are connected.
         *
         * @param ei
         * @return true if the interface is supported
         */
        virtual bool OKtoEnumerate(NOTUSED(ENUMERATION_INFO *ei)) {
                return false;
        };

        /**
         * Configures any needed endpoint information for an interface.
         * You must provide this in your driver.
         * Executed when new devices are connected and OKtoEnumerate()
         * returned true.
         *
         * @param ei
         * @return zero on success
         */
        virtual uint8_t SetInterface(NOTUSED(ENUMERATION_INFO *ei)) {
                return UHS_HOST_ERROR_NOT_IMPLEMENTED;
        };

        /**
         * Interface specific additional setup and enumeration that
         * can't occur when the descriptor stream is open.
         * Also used for collection of unclaimed interfaces, to link to the master.
         *
         * @return zero on success
         */
        virtual uint8_t Finalize() {
                return 0;
        };

        /**
         *  Executed after interface is finalized but, before polling has started.
         *
         * @return 0 on success
         */
        virtual uint8_t OnStart() {
                return 0;
        };

        /**
         * Start interface polling
         * @return
         */
        virtual uint8_t Start() {
                uint8_t rcode = OnStart();
                if(!rcode) bPollEnable = true;
                return rcode;
        };

        /**
         * Executed before anything else in Release().
         */
        virtual void OnRelease() {
                return;
        };

        /**
         * Release resources when device is disconnected.
         * Normally this does not need to be overridden.
         */
        virtual void Release() {
                OnRelease();
                DriverDefaults();
                return;
        };

        /**
         * Executed After driver polls.
         * Can be used when there is an important change detected during polling
         * and you want to handle it elsewhere.
         * Examples:
         * Media status change for bulk, e.g. ready, not-ready, media changed, door opened.
         * Button state/joystick position/etc changes on a HID device.
         * Flow control status change on a communication device, e.g. CTS on serial
         */
        virtual void OnPoll() {
                return;
        };

        /**
         * Poll interface driver. You should override this in your driver if you
         * require polling faster or slower than every 100 milliseconds, or your
         * driver requires special housekeeping.
         */
        virtual void Poll() {
                OnPoll();
                qNextPollTime = millis() + 100;
        };

        virtual bool UHS_NI Polling() {
                return bPollEnable;
        }

        /**
         * This is only for a hub.
         * @param port
         */
        virtual void ResetHubPort(NOTUSED(uint8_t port)) {
                return;
        };

#if 0
        /**
         * @return true if this interface is Vendor Specific.
         */
        virtual bool IsVSI() {
                return false;
        }
#endif
};

#if 0
/**
 * Vendor Specific interface class.
 * This is used by a partner interface.
 * It can also be used to force-enumerate an interface that
 * can use this interface directly.
 * You can also add an instance of this class within the interface constructor
 * if you expect the interface.
 *
 * If this is not needed, it may be removed. Nothing I have written needs this.
 * Let me know if it is not required, then IsVSI method can also be shit-canned.
 * -- AJK
 */

class UHS_VSI : public UHS_USBInterface {
public:
        volatile UHS_EpInfo epInfo[1];
        volatile ENUMERATION_INFO eInfo;
        UHS_VSI(UHS_USB_HOST_BASE *p);
        bool OKtoEnumerate(ENUMERATION_INFO *ei);
        uint8_t SetInterface(ENUMERATION_INFO *ei);
        virtual void DriverDefaults();
        virtual void Release();

        uint8_t GetAddress() {
                return bAddress;
        };

        virtual bool IsVSI() {
                return true;
        }

};
#endif

#endif // _USBHOST_H_
#endif // !_UHS_host_h_
