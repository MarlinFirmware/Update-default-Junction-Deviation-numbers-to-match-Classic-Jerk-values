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

#if !defined(_UHS_host_h_) || defined(_UHS_ch9_h_)
#error "Never include UHS_usb_ch9.h directly; include UHS_Usb.h instead"
#else

// USB chapter 9 structures
#define _UHS_ch9_h_

// Misc.USB constants
#define                           DEV_DESCR_LEN 18      //device descriptor length
#define                          CONF_DESCR_LEN 9       //configuration descriptor length
#define                          INTR_DESCR_LEN 9       //interface descriptor length
#define                            EP_DESCR_LEN 7       //endpoint descriptor length

// Standard Device Requests
#define                  USB_REQUEST_GET_STATUS 0       // Standard Device Request - GET STATUS
#define               USB_REQUEST_CLEAR_FEATURE 1       // Standard Device Request - CLEAR FEATURE
#define                 USB_REQUEST_SET_FEATURE 3       // Standard Device Request - SET FEATURE
#define                 USB_REQUEST_SET_ADDRESS 5       // Standard Device Request - SET ADDRESS
#define              USB_REQUEST_GET_DESCRIPTOR 6       // Standard Device Request - GET DESCRIPTOR
#define              USB_REQUEST_SET_DESCRIPTOR 7       // Standard Device Request - SET DESCRIPTOR
#define           USB_REQUEST_GET_CONFIGURATION 8       // Standard Device Request - GET CONFIGURATION
#define           USB_REQUEST_SET_CONFIGURATION 9       // Standard Device Request - SET CONFIGURATION
#define               USB_REQUEST_GET_INTERFACE 10      // Standard Device Request - GET INTERFACE
#define               USB_REQUEST_SET_INTERFACE 11      // Standard Device Request - SET INTERFACE
#define                 USB_REQUEST_SYNCH_FRAME 12      // Standard Device Request - SYNCH FRAME

// Wireless USB Device Requests
#define                  USB_REQ_SET_ENCRYPTION 0x0D
#define                  USB_REQ_GET_ENCRYPTION 0x0E
#define                     USB_REQ_RPIPE_ABORT 0x0E
#define                   USB_REQ_SET_HANDSHAKE 0x0F
#define                     USB_REQ_RPIPE_RESET 0x0F
#define                   USB_REQ_GET_HANDSHAKE 0x10
#define                  USB_REQ_SET_CONNECTION 0x11
#define               USB_REQ_SET_SECURITY_DATA 0x12
#define               USB_REQ_GET_SECURITY_DATA 0x13
#define                   USB_REQ_SET_WUSB_DATA 0x14
#define             USB_REQ_LOOPBACK_DATA_WRITE 0x15
#define              USB_REQ_LOOPBACK_DATA_READ 0x16
#define                USB_REQ_SET_INTERFACE_DS 0x17

// USB feature flags
#define                 USB_DEVICE_SELF_POWERED 0   // (read only)
#define                USB_DEVICE_REMOTE_WAKEUP 1   // dev may initiate wakeup
#define                    USB_DEVICE_TEST_MODE 2   // (wired high speed only)
#define                      USB_DEVICE_BATTERY 2   // (wireless)
#define                 USB_DEVICE_B_HNP_ENABLE 3   // (otg) dev may initiate HNP
#define                  USB_DEVICE_WUSB_DEVICE 3   // (wireless)
#define                USB_DEVICE_A_HNP_SUPPORT 4   // (otg) RH port supports HNP
#define            USB_DEVICE_A_ALT_HNP_SUPPORT 5   // (otg) other RH port does
#define                   USB_DEVICE_DEBUG_MODE 6   // (special devices only)

#define               USB_FEATURE_ENDPOINT_HALT 0       // CLEAR/SET FEATURE - Endpoint Halt
#define        USB_FEATURE_DEVICE_REMOTE_WAKEUP 1       // CLEAR/SET FEATURE - Device remote wake-up
#define                   USB_FEATURE_TEST_MODE 2       // CLEAR/SET FEATURE - Test mode
// OTG SET FEATURE Constants
#define               OTG_FEATURE_B_HNP_ENABLE  3       // SET FEATURE OTG - Enable B device to perform HNP
#define               OTG_FEATURE_A_HNP_SUPPORT 4       // SET FEATURE OTG - A device supports HNP
#define           OTG_FEATURE_A_ALT_HNP_SUPPORT 5       // SET FEATURE OTG - Another port on the A device supports HNP

// Setup Data Constants
#define                USB_SETUP_HOST_TO_DEVICE 0x00    // Device Request bmRequestType transfer direction - host to device transfer
#define                USB_SETUP_DEVICE_TO_HOST 0x80    // Device Request bmRequestType transfer direction - device to host transfer
#define                 USB_SETUP_TYPE_STANDARD 0x00    // Device Request bmRequestType type - standard
#define                    USB_SETUP_TYPE_CLASS 0x20    // Device Request bmRequestType type - class
#define                   USB_SETUP_TYPE_VENDOR 0x40    // Device Request bmRequestType type - vendor
#define              USB_SETUP_RECIPIENT_DEVICE 0x00    // Device Request bmRequestType recipient - device
#define           USB_SETUP_RECIPIENT_INTERFACE 0x01    // Device Request bmRequestType recipient - interface
#define            USB_SETUP_RECIPIENT_ENDPOINT 0x02    // Device Request bmRequestType recipient - endpoint
#define               USB_SETUP_RECIPIENT_OTHER 0x03    // Device Request bmRequestType recipient - other
#define                USB_SETUP_RECIPIENT_PORT 0x04    // Wireless USB 1.0
#define               USB_SETUP_RECIPIENT_RPIPE 0x05    // Wireless USB 1.0

// USB descriptors
#define                   USB_DESCRIPTOR_DEVICE 0x01    // bDescriptorType for a Device Descriptor.
#define            USB_DESCRIPTOR_CONFIGURATION 0x02    // bDescriptorType for a Configuration Descriptor.
#define                   USB_DESCRIPTOR_STRING 0x03    // bDescriptorType for a String Descriptor.
#define                USB_DESCRIPTOR_INTERFACE 0x04    // bDescriptorType for an Interface Descriptor.
#define                 USB_DESCRIPTOR_ENDPOINT 0x05    // bDescriptorType for an Endpoint Descriptor.
#define         USB_DESCRIPTOR_DEVICE_QUALIFIER 0x06    // bDescriptorType for a Device Qualifier.
#define              USB_DESCRIPTOR_OTHER_SPEED 0x07    // bDescriptorType for a Other Speed Configuration.
#define          USB_DESCRIPTOR_INTERFACE_POWER 0x08    // bDescriptorType for Interface Power.
#define                      USB_DESCRIPTOR_OTG 0x09    // bDescriptorType for an OTG Descriptor.
#define                    USB_DESCRIPTOR_DEBUG 0x0A
#define    USB_DESCRIPTOR_INTERFACE_ASSOCIATION 0x0B
#define                 USB_DESCRIPTOR_SECURITY 0x0C
#define                      USB_DESCRIPTOR_KEY 0x0D
#define          USB_DESCRIPTOR_ENCRYPTION_TYPE 0x0E
#define                      USB_DESCRIPTOR_BOS 0x0F
#define        USB_DESCRIPTOR_DEVICE_CAPABILITY 0x10
#define   USB_DESCRIPTOR_WIRELESS_ENDPOINT_COMP 0x11
#define             USB_DESCRIPTOR_WIRE_ADAPTER 0x21
#define                    USB_DESCRIPTOR_RPIPE 0x22
#define         USB_DESCRIPTOR_CS_RADIO_CONTROL 0x23
#define         USB_DESCRIPTOR_SS_ENDPOINT_COMP 0x30

#define                      USB_HID_DESCRIPTOR 0x21

// Conventional codes for class-specific descriptors. "Common Class" Spec (3.11)
#define                USB_DESCRIPTOR_CS_DEVICE 0x21
#define                USB_DESCRIPTOR_CS_CONFIG 0x22
#define                USB_DESCRIPTOR_CS_STRING 0x23
#define             USB_DESCRIPTOR_CS_INTERFACE 0x24
#define              USB_DESCRIPTOR_CS_ENDPOINT 0x25

// USB Endpoint Transfer Types
#define               USB_TRANSFER_TYPE_CONTROL 0x00    // Endpoint is a control endpoint.
#define           USB_TRANSFER_TYPE_ISOCHRONOUS 0x01    // Endpoint is an isochronous endpoint.
#define                  USB_TRANSFER_TYPE_BULK 0x02    // Endpoint is a bulk endpoint.
#define             USB_TRANSFER_TYPE_INTERRUPT 0x03    // Endpoint is an interrupt endpoint.
#define                     bmUSB_TRANSFER_TYPE 0x03    // bit mask to separate transfer type from ISO attributes
#define               USB_TRANSFER_DIRECTION_IN 0x80    // Indicate direction is IN

// Standard Feature Selectors for CLEAR_FEATURE Requests
#define              USB_FEATURE_ENDPOINT_STALL 0       // Endpoint recipient
#define        USB_FEATURE_DEVICE_REMOTE_WAKEUP 1       // Device recipient
#define                   USB_FEATURE_TEST_MODE 2       // Device recipient

// descriptor data structures

// Device descriptor structure
typedef struct {
        uint8_t bLength; // Length of this descriptor.
        uint8_t bDescriptorType; // DEVICE descriptor type (USB_DESCRIPTOR_DEVICE).
        uint16_t bcdUSB; // USB Spec Release Number (BCD).
        uint8_t bDeviceClass; // Class code (assigned by the USB-IF). 0xFF-Vendor specific.
        uint8_t bDeviceSubClass; // Subclass code (assigned by the USB-IF).
        uint8_t bDeviceProtocol; // Protocol code (assigned by the USB-IF). 0xFF-Vendor specific.
        uint8_t bMaxPacketSize0; // Maximum packet size for endpoint 0.
        uint16_t idVendor; // Vendor ID (assigned by the USB-IF).
        uint16_t idProduct; // Product ID (assigned by the manufacturer).
        uint16_t bcdDevice; // Device release number (BCD).
        uint8_t iManufacturer; // Index of String Descriptor describing the manufacturer.
        uint8_t iProduct; // Index of String Descriptor describing the product.
        uint8_t iSerialNumber; // Index of String Descriptor with the device's serial number.
        uint8_t bNumConfigurations; // Number of possible configurations.
} __attribute__((packed)) USB_FD_DEVICE_DESCRIPTOR;

// Configuration descriptor structure
typedef struct {
        uint8_t bLength; // Length of this descriptor.
        uint8_t bDescriptorType; // CONFIGURATION descriptor type (USB_DESCRIPTOR_CONFIGURATION).
        uint16_t wTotalLength; // Total length of all descriptors for this configuration.
        uint8_t bNumInterfaces; // Number of interfaces in this configuration.
        uint8_t bConfigurationValue; // Value of this configuration (1 based).
        uint8_t iConfiguration; // Index of String Descriptor describing the configuration.
        uint8_t bmAttributes; // Configuration characteristics.
        uint8_t bMaxPower; // Maximum power consumed by this configuration.
} __attribute__((packed)) USB_FD_CONFIGURATION_DESCRIPTOR;

// Interface descriptor structure
typedef struct {
        uint8_t bLength; // Length of this descriptor.
        uint8_t bDescriptorType; // INTERFACE descriptor type (USB_DESCRIPTOR_INTERFACE).
        uint8_t bInterfaceNumber; // Number of this interface (0 based).
        uint8_t bAlternateSetting; // Value of this alternate interface setting.
        uint8_t bNumEndpoints; // Number of endpoints in this interface.
        uint8_t bInterfaceClass; // Class code (assigned by the USB-IF).  0xFF-Vendor specific.
        uint8_t bInterfaceSubClass; // Subclass code (assigned by the USB-IF).
        uint8_t bInterfaceProtocol; // Protocol code (assigned by the USB-IF).  0xFF-Vendor specific.
        uint8_t iInterface; // Index of String Descriptor describing the interface.
} __attribute__((packed)) USB_FD_INTERFACE_DESCRIPTOR;

// Endpoint descriptor structure
typedef struct {
        uint8_t bLength; // Length of this descriptor.
        uint8_t bDescriptorType; // ENDPOINT descriptor type (USB_DESCRIPTOR_ENDPOINT).
        uint8_t bEndpointAddress; // Endpoint address. Bit 7 indicates direction (0=OUT, 1=IN).
        uint8_t bmAttributes; // Endpoint transfer type.
        uint16_t wMaxPacketSize; // Maximum packet size.
        uint8_t bInterval; // Polling interval in frames.
} __attribute__((packed)) USB_FD_ENDPOINT_DESCRIPTOR;

// HID descriptor
/*
typedef struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint16_t bcdHID; // HID class specification release
        uint8_t bCountryCode;
        uint8_t bNumDescriptors; // Number of additional class specific descriptors
        uint8_t bDescrType; // Type of class descriptor
        uint16_t wDescriptorLength; // Total size of the Report descriptor
} __attribute__((packed)) USB_HID_DESCRIPTOR;
*/

typedef struct {
        uint8_t bDescrType; // Type of class descriptor
        uint16_t wDescriptorLength; // Total size of the Report descriptor
} __attribute__((packed)) HID_CLASS_DESCRIPTOR_LEN_AND_TYPE;

#endif // _ch9_h_
