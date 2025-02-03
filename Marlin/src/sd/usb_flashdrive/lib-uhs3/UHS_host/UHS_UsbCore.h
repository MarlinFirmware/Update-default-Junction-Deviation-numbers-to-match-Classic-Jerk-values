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

#if !defined(_UHS_host_h_) || defined(USBCORE_H)
#error "Never include UHS_UsbCore.h directly; include UHS_Host.h instead"
#else
#define USBCORE_H

#ifndef UHS_HOST_MAX_INTERFACE_DRIVERS
#define                UHS_HOST_MAX_INTERFACE_DRIVERS 0x10U // Default maximum number of USB interface drivers
#endif

#ifndef SYSTEM_OR_SPECIAL_YIELD
#define SYSTEM_OR_SPECIAL_YIELD(...) VOID0
#endif

#ifndef SYSTEM_OR_SPECIAL_YIELD_FROM_ISR
#define SYSTEM_OR_SPECIAL_YIELD_FROM_ISR(...) SYSTEM_OR_SPECIAL_YIELD
#endif

// As we make extensions to a target interface add to UHS_HOST_MAX_INTERFACE_DRIVERS
// This offset gets calculated for supporting wide subclasses, such as HID, BT, etc.
#define                                 UHS_HID_INDEX (UHS_HOST_MAX_INTERFACE_DRIVERS + 1)

/* Common setup data constant combinations  */
//get descriptor request type
#define                           UHS_bmREQ_GET_DESCR (USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE)

//set request type for all but 'set feature' and 'set interface'
#define                                 UHS_bmREQ_SET (USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE)

//get interface request type
#define                         UHS_bmREQ_CL_GET_INTF (USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE)

// D7           data transfer direction (0 - host-to-device, 1 - device-to-host)
// D6-5         Type (0- standard, 1 - class, 2 - vendor, 3 - reserved)
// D4-0         Recipient (0 - device, 1 - interface, 2 - endpoint, 3 - other, 4..31 - reserved)

// TODO: Use the python script to generate these.
// TODO: Add _all_ subclasses here.
// USB Device Classes, Subclasses and Protocols
////////////////////////////////////////////////////////////////////////////////
// Use Class Info in the Interface Descriptors
#define                  UHS_USB_CLASS_USE_CLASS_INFO 0x00U

////////////////////////////////////////////////////////////////////////////////
// Audio
#define                           UHS_USB_CLASS_AUDIO 0x01U
// Subclasses
#define                 UHS_USB_SUBCLASS_AUDIOCONTROL 0x01U
#define               UHS_USB_SUBCLASS_AUDIOSTREAMING 0x02U
#define                UHS_USB_SUBCLASS_MIDISTREAMING 0x03U

////////////////////////////////////////////////////////////////////////////////
// Communications and CDC Control
#define                UHS_USB_CLASS_COM_AND_CDC_CTRL 0x02U

////////////////////////////////////////////////////////////////////////////////
// HID
#define                             UHS_USB_CLASS_HID 0x03U
// Subclasses
#define                         UHS_HID_BOOT_SUBCLASS 0x01U
// Protocols
#define             UHS_HID_PROTOCOL_HIDBOOT_KEYBOARD 0x01U
#define                UHS_HID_PROTOCOL_HIDBOOT_MOUSE 0x02U
////////////////////////////////////////////////////////////////////////////////
// Physical
#define                        UHS_USB_CLASS_PHYSICAL 0x05U

////////////////////////////////////////////////////////////////////////////////
// Image
#define                           UHS_USB_CLASS_IMAGE 0x06U

////////////////////////////////////////////////////////////////////////////////
// Printer
#define                         UHS_USB_CLASS_PRINTER 0x07U

////////////////////////////////////////////////////////////////////////////////
// Mass Storage
#define                    UHS_USB_CLASS_MASS_STORAGE 0x08
// Subclasses
#define           UHS_BULK_SUBCLASS_SCSI_NOT_REPORTED 0x00U   // De facto use
#define                         UHS_BULK_SUBCLASS_RBC 0x01U
#define                       UHS_BULK_SUBCLASS_ATAPI 0x02U   // MMC-5 (ATAPI)
#define                   UHS_BULK_SUBCLASS_OBSOLETE1 0x03U   // Was QIC-157
#define                         UHS_BULK_SUBCLASS_UFI 0x04U   // Specifies how to interface Floppy Disk Drives to USB
#define                   UHS_BULK_SUBCLASS_OBSOLETE2 0x05U   // Was SFF-8070i
#define                        UHS_BULK_SUBCLASS_SCSI 0x06U   // SCSI Transparent Command Set
#define                       UHS_BULK_SUBCLASS_LSDFS 0x07U   // Specifies how host has to negotiate access before trying SCSI
#define                    UHS_BULK_SUBCLASS_IEEE1667 0x08U
// Protocols
#define                            UHS_STOR_PROTO_CBI 0x00U   // CBI (with command completion interrupt)
#define                     UHS_STOR_PROTO_CBI_NO_INT 0x01U   // CBI (without command completion interrupt)
#define                       UHS_STOR_PROTO_OBSOLETE 0x02U
#define                            UHS_STOR_PROTO_BBB 0x50U   // Bulk Only Transport
#define                            UHS_STOR_PROTO_UAS 0x62U

////////////////////////////////////////////////////////////////////////////////
// Hub
#define                             UHS_USB_CLASS_HUB 0x09U

////////////////////////////////////////////////////////////////////////////////
// CDC-Data
#define                        UHS_USB_CLASS_CDC_DATA 0x0AU

////////////////////////////////////////////////////////////////////////////////
// Smart-Card
#define                      UHS_USB_CLASS_SMART_CARD 0x0BU

////////////////////////////////////////////////////////////////////////////////
// Content Security
#define                UHS_USB_CLASS_CONTENT_SECURITY 0x0DU

////////////////////////////////////////////////////////////////////////////////
// Video
#define                           UHS_USB_CLASS_VIDEO 0x0EU

////////////////////////////////////////////////////////////////////////////////
// Personal Healthcare
#define                 UHS_USB_CLASS_PERSONAL_HEALTH 0x0FU

////////////////////////////////////////////////////////////////////////////////
// Diagnostic Device
#define               UHS_USB_CLASS_DIAGNOSTIC_DEVICE 0xDCU

////////////////////////////////////////////////////////////////////////////////
// Wireless Controller
#define                   UHS_USB_CLASS_WIRELESS_CTRL 0xE0U

////////////////////////////////////////////////////////////////////////////////
// Miscellaneous
#define                            UHS_USB_CLASS_MISC 0xEFU

////////////////////////////////////////////////////////////////////////////////
// Application Specific
#define                    UHS_USB_CLASS_APP_SPECIFIC 0xFEU

////////////////////////////////////////////////////////////////////////////////
// Vendor Specific
#define                 UHS_USB_CLASS_VENDOR_SPECIFIC 0xFFU

////////////////////////////////////////////////////////////////////////////////

/* USB state machine states */
#define                       UHS_USB_HOST_STATE_MASK 0xF0U

// Configure states, MSN == 0 --------------------------V
#define                   UHS_USB_HOST_STATE_DETACHED 0x00U
#define                   UHS_USB_HOST_STATE_DEBOUNCE 0x01U
#define      UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE 0x02U
#define         UHS_USB_HOST_STATE_RESET_NOT_COMPLETE 0x03U
#define                   UHS_USB_HOST_STATE_WAIT_SOF 0x04U
#define             UHS_USB_HOST_STATE_WAIT_BUS_READY 0x05U
#define               UHS_USB_HOST_STATE_RESET_DEVICE 0x0AU
#define                UHS_USB_HOST_STATE_CONFIGURING 0x0CU // Looks like "CO"nfig (backwards)
#define           UHS_USB_HOST_STATE_CONFIGURING_DONE 0x0DU // Looks like "DO"one (backwards)
#define                      UHS_USB_HOST_STATE_CHECK 0x0EU
#define                    UHS_USB_HOST_STATE_ILLEGAL 0x0FU // Foo

// Run states, MSN != 0 --------------------------------V
#define                    UHS_USB_HOST_STATE_RUNNING 0x60U // Looks like "GO"
#define                       UHS_USB_HOST_STATE_IDLE 0x1DU // Looks like "ID"le
#define                      UHS_USB_HOST_STATE_ERROR 0xF0U // Looks like "FO"o
#define                 UHS_USB_HOST_STATE_INITIALIZE 0x10U // Looks like "I"nit

// Host SE result codes.
// Common SE results are stored in the low nybble, all interface drivers understand these plus 0x1F.
// Extended SE results are 0x10-0x1E. SE code only understands these internal to the hardware.
// Values > 0x1F are driver or other internal error conditions.
// Return these result codes from your host controller driver to match the error condition
// ALL Non-zero values are errors.
// Values not listed in this table are not handled in the base class, or any host driver.

#define                           UHS_HOST_ERROR_NONE 0x00U // No error
#define                           UHS_HOST_ERROR_BUSY 0x01U // transfer pending
#define                         UHS_HOST_ERROR_BADREQ 0x02U // Transfer Launch Request was bad
#define                            UHS_HOST_ERROR_DMA 0x03U // DMA was too short, or too long
#define                            UHS_HOST_ERROR_NAK 0x04U // Peripheral returned NAK
#define                          UHS_HOST_ERROR_STALL 0x05U // Peripheral returned STALL
#define                         UHS_HOST_ERROR_TOGERR 0x06U // Toggle error/ISO over-underrun
#define                       UHS_HOST_ERROR_WRONGPID 0x07U // Received wrong Packet ID
#define                          UHS_HOST_ERROR_BADBC 0x08U // Byte count is bad
#define                         UHS_HOST_ERROR_PIDERR 0x09U // Received Packet ID is corrupted
#define                          UHS_HOST_ERROR_BADRQ 0x0AU // Packet error. Increase max packet.
#define                            UHS_HOST_ERROR_CRC 0x0BU // USB CRC was incorrect
#define                           UHS_HOST_ERROR_KERR 0x0CU // K-state instead of response, usually indicates wrong speed
#define                           UHS_HOST_ERROR_JERR 0x0DU // J-state instead of response, usually indicates wrong speed
#define                        UHS_HOST_ERROR_TIMEOUT 0x0EU // Device did not respond in time
#define                         UHS_HOST_ERROR_BABBLE 0x0FU // Line noise/unexpected data
#define                        UHS_HOST_ERROR_MEM_LAT 0x10U // Error caused by memory latency.
#define                           UHS_HOST_ERROR_NYET 0x11U // OUT transfer accepted with NYET

// Addressing error codes
#define                      ADDR_ERROR_INVALID_INDEX 0xA0U
#define                    ADDR_ERROR_INVALID_ADDRESS 0xA1U

// Common Interface Driver error codes
#define           UHS_HOST_ERROR_DEVICE_NOT_SUPPORTED 0xD1U // Driver doesn't support the device or interfaces
#define         UHS_HOST_ERROR_DEVICE_INIT_INCOMPLETE 0xD2U // Init partially finished, but died.
#define     UHS_HOST_ERROR_CANT_REGISTER_DEVICE_CLASS 0xD3U // There was no driver for the interface requested.
#define              UHS_HOST_ERROR_ADDRESS_POOL_FULL 0xD4U // No addresses left in the address pool.
#define           UHS_HOST_ERROR_HUB_ADDRESS_OVERFLOW 0xD5U // No hub addresses left. The maximum is 7.
#define             UHS_HOST_ERROR_NO_ADDRESS_IN_POOL 0xD6U // Address was not allocated in the pool, thus not found.
#define                    UHS_HOST_ERROR_NULL_EPINFO 0xD7U // The supplied endpoint was NULL, indicates a bug or other problem.
#define                   UHS_HOST_ERROR_BAD_ARGUMENT 0xD8U // Indicates a range violation bug.
#define             UHS_HOST_ERROR_DEVICE_DRIVER_BUSY 0xD9U // The interface driver is busy or out buffer is full, try again later.
#define            UHS_HOST_ERROR_BAD_MAX_PACKET_SIZE 0xDAU // The maximum packet size was exceeded. Try again with smaller size.
#define           UHS_HOST_ERROR_NO_ENDPOINT_IN_TABLE 0xDBU // The endpoint could not be found in the endpoint table.
#define                      UHS_HOST_ERROR_UNPLUGGED 0xDEU // Someone removed the USB device, or Vbus was turned off.
#define                          UHS_HOST_ERROR_NOMEM 0xDFU // Out Of Memory.

// Control request stream errors
#define                UHS_HOST_ERROR_FailGetDevDescr 0xE1U
#define             UHS_HOST_ERROR_FailSetDevTblEntry 0xE2U
#define               UHS_HOST_ERROR_FailGetConfDescr 0xE3U
#define                  UHS_HOST_ERROR_END_OF_STREAM 0xEFU

// Host base class specific Error codes
#define                UHS_HOST_ERROR_NOT_IMPLEMENTED 0xFEU
#define               UHS_HOST_ERROR_TRANSFER_TIMEOUT 0xFFU

// SEI interaction defaults
#define                      UHS_HOST_TRANSFER_MAX_MS 10000 // USB transfer timeout in ms, per section 9.2.6.1 of USB 2.0 spec
#define               UHS_HOST_TRANSFER_RETRY_MAXIMUM 3     // 3 retry limit for a transfer
#define                    UHS_HOST_DEBOUNCE_DELAY_MS 500   // settle delay in milliseconds
#define                        UHS_HUB_RESET_DELAY_MS 20    // hub port reset delay, 10ms recommended, but can be up to 20ms

//
// We only provide the minimum needed information for enumeration.
// Interface drivers should be able to set up what is needed with nothing more.
// A driver needs to know the following information:
// 1: address on the USB network, parent and port (aka UsbDeviceAddress)
// 2: endpoints
// 3: vid:pid, class, subclass, protocol
//

struct ENDPOINT_INFO {
        uint8_t bEndpointAddress;       // Endpoint address. Bit 7 indicates direction (0=OUT, 1=IN).
        uint8_t bmAttributes;           // Endpoint transfer type.
        uint16_t wMaxPacketSize;        // Maximum packet size.
        uint8_t bInterval;              // Polling interval in frames.
} __attribute__((packed));

struct INTERFACE_INFO {
        uint8_t bInterfaceNumber;
        uint8_t bAlternateSetting;
        uint8_t numep;
        uint8_t klass;
        uint8_t subklass;
        uint8_t protocol;
        ENDPOINT_INFO epInfo[16];
} __attribute__((packed));

struct ENUMERATION_INFO {
        uint16_t vid;
        uint16_t pid;
        uint16_t bcdDevice;
        uint8_t klass;
        uint8_t subklass;
        uint8_t protocol;
        uint8_t bMaxPacketSize0;
        uint8_t currentconfig;
        uint8_t parent;
        uint8_t port;
        uint8_t address;
        INTERFACE_INFO interface;
} __attribute__((packed));

/* USB Setup Packet Structure   */
typedef struct {
        // offset   description
        //   0      Bit-map of request type
         union {
                uint8_t bmRequestType;

                struct {
                        uint8_t recipient : 5;  // Recipient of the request
                        uint8_t type : 2;       // Type of request
                        uint8_t direction : 1;  // Direction of data transfer
                } __attribute__((packed));
        } ReqType_u;

        //   1      Request
        uint8_t bRequest;

        //   2      Depends on bRequest
        union {
                uint16_t wValue;

                struct {
                        uint8_t wValueLo;
                        uint8_t wValueHi;
                } __attribute__((packed));
        } wVal_u;
        //   4      Depends on bRequest
        uint16_t wIndex;
        //   6      Depends on bRequest
        uint16_t wLength;
        // 8 bytes total
} __attribute__((packed)) SETUP_PKT, *PSETUP_PKT;

// little endian :-)                                                                             8                                8                          8                         8                          16                      16
#define mkSETUP_PKT8(bmReqType, bRequest, wValLo, wValHi, wInd, total) ((uint64_t)(((uint64_t)(bmReqType)))|(((uint64_t)(bRequest))<<8)|(((uint64_t)(wValLo))<<16)|(((uint64_t)(wValHi))<<24)|(((uint64_t)(wInd))<<32)|(((uint64_t)(total)<<48)))
#define mkSETUP_PKT16(bmReqType, bRequest, wVal, wInd, total)          ((uint64_t)(((uint64_t)(bmReqType)))|(((uint64_t)(bRequest))<<8)|(((uint64_t)(wVal  ))<<16)                           |(((uint64_t)(wInd))<<32)|(((uint64_t)(total)<<48)))

// Big endian -- but we aren't able to use this :-/
//#define mkSETUP_PKT8(bmReqType, bRequest, wValLo, wValHi, wInd, total) ((uint64_t)(((uint64_t)(bmReqType))<<56)|(((uint64_t)(bRequest))<<48)|(((uint64_t)(wValLo))<<40)|(((uint64_t)(wValHi))<<32)|(((uint64_t)(wInd))<<16)|((uint64_t)(total)))
//#define mkSETUP_PKT16(bmReqType, bRequest, wVal, wInd, total)          ((uint64_t)(((uint64_t)(bmReqType))<<56)|(((uint64_t)(bRequest))<<48)                           |(((uint64_t)(wVal))<<32)  |(((uint64_t)(wInd))<<16)|((uint64_t)(total)))

#endif // USBCORE_H
