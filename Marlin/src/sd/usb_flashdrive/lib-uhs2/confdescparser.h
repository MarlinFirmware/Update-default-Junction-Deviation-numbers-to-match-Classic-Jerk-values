/**
 * Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Contact information
 * -------------------
 *
 * Circuits At Home, LTD
 * Web      :  https://www.circuitsathome.com
 * e-mail   :  support@circuitsathome.com
 */
#pragma once

#ifndef _usb_h_
  #error "Never include confdescparser.h directly; include Usb.h instead"
#endif

class UsbConfigXtracter {
public:
  //virtual void ConfigXtract(const USB_FD_CONFIGURATION_DESCRIPTOR *conf) = 0;
  //virtual void InterfaceXtract(uint8_t conf, const USB_FD_INTERFACE_DESCRIPTOR *iface) = 0;

  virtual void EndpointXtract(uint8_t conf __attribute__((unused)), uint8_t iface __attribute__((unused)), uint8_t alt __attribute__((unused)), uint8_t proto __attribute__((unused)), const USB_FD_ENDPOINT_DESCRIPTOR *ep __attribute__((unused))) {
  }
};

#define CP_MASK_COMPARE_CLASS     1
#define CP_MASK_COMPARE_SUBCLASS  2
#define CP_MASK_COMPARE_PROTOCOL  4
#define CP_MASK_COMPARE_ALL       7

// Configuration Descriptor Parser Class Template

template <const uint8_t CLASS_ID, const uint8_t SUBCLASS_ID, const uint8_t PROTOCOL_ID, const uint8_t MASK>
class ConfigDescParser : public USBReadParser {
  UsbConfigXtracter *theXtractor;
  MultiValueBuffer theBuffer;
  MultiByteValueParser valParser;
  ByteSkipper theSkipper;
  uint8_t varBuffer[16 /*sizeof(USB_FD_CONFIGURATION_DESCRIPTOR)*/];

  uint8_t stateParseDescr; // ParseDescriptor state

  uint8_t dscrLen; // Descriptor length
  uint8_t dscrType; // Descriptor type

  bool isGoodInterface; // Appropriate interface flag
  uint8_t confValue; // Configuration value
  uint8_t protoValue; // Protocol value
  uint8_t ifaceNumber; // Interface number
  uint8_t ifaceAltSet; // Interface alternate settings

  bool UseOr;
  bool ParseDescriptor(uint8_t **pp, uint16_t *pcntdn);
  void PrintHidDescriptor(const USB_HID_DESCRIPTOR *pDesc);

public:

  void SetOR() { UseOr = true; }
  ConfigDescParser(UsbConfigXtracter *xtractor);
  void Parse(const uint16_t len, const uint8_t *pbuf, const uint16_t &offset);
};

template <const uint8_t CLASS_ID, const uint8_t SUBCLASS_ID, const uint8_t PROTOCOL_ID, const uint8_t MASK>
ConfigDescParser<CLASS_ID, SUBCLASS_ID, PROTOCOL_ID, MASK>::ConfigDescParser(UsbConfigXtracter *xtractor) :
  theXtractor(xtractor),
  stateParseDescr(0),
  dscrLen(0),
  dscrType(0),
  UseOr(false) {
    theBuffer.pValue = varBuffer;
    valParser.Initialize(&theBuffer);
    theSkipper.Initialize(&theBuffer);
  };

template <const uint8_t CLASS_ID, const uint8_t SUBCLASS_ID, const uint8_t PROTOCOL_ID, const uint8_t MASK>
void ConfigDescParser<CLASS_ID, SUBCLASS_ID, PROTOCOL_ID, MASK>::Parse(const uint16_t len, const uint8_t *pbuf, const uint16_t &offset __attribute__((unused))) {
  uint16_t cntdn = (uint16_t)len;
  uint8_t *p = (uint8_t*)pbuf;
  while (cntdn) if (!ParseDescriptor(&p, &cntdn)) return;
}

/* Parser for the configuration descriptor. Takes values for class, subclass, protocol fields in interface descriptor and
  compare masks for them. When the match is found, calls EndpointXtract passing buffer containing endpoint descriptor */
template <const uint8_t CLASS_ID, const uint8_t SUBCLASS_ID, const uint8_t PROTOCOL_ID, const uint8_t MASK>
bool ConfigDescParser<CLASS_ID, SUBCLASS_ID, PROTOCOL_ID, MASK>::ParseDescriptor(uint8_t **pp, uint16_t *pcntdn) {
  USB_FD_CONFIGURATION_DESCRIPTOR* ucd = reinterpret_cast<USB_FD_CONFIGURATION_DESCRIPTOR*>(varBuffer);
  USB_FD_INTERFACE_DESCRIPTOR* uid = reinterpret_cast<USB_FD_INTERFACE_DESCRIPTOR*>(varBuffer);
  switch (stateParseDescr) {
    case 0:
      theBuffer.valueSize = 2;
      valParser.Initialize(&theBuffer);
      stateParseDescr = 1;
    case 1:
      if (!valParser.Parse(pp, pcntdn)) return false;
      dscrLen = *((uint8_t*)theBuffer.pValue);
      dscrType = *((uint8_t*)theBuffer.pValue + 1);
      stateParseDescr = 2;
    case 2:
      // This is a sort of hack. Assuming that two bytes are all ready in the buffer
      //      the pointer is positioned two bytes ahead in order for the rest of descriptor
      //      to be read right after the size and the type fields.
      // This should be used carefully. varBuffer should be used directly to handle data
      //      in the buffer.
      theBuffer.pValue = varBuffer + 2;
      stateParseDescr = 3;
    case 3:
      switch (dscrType) {
        case USB_DESCRIPTOR_INTERFACE:
          isGoodInterface = false;
          break;
        case USB_DESCRIPTOR_CONFIGURATION:
        case USB_DESCRIPTOR_ENDPOINT:
        case HID_DESCRIPTOR_HID:
          break;
      }
      theBuffer.valueSize = dscrLen - 2;
      valParser.Initialize(&theBuffer);
      stateParseDescr = 4;
    case 4:
      switch (dscrType) {
        case USB_DESCRIPTOR_CONFIGURATION:
          if (!valParser.Parse(pp, pcntdn)) return false;
          confValue = ucd->bConfigurationValue;
          break;
        case USB_DESCRIPTOR_INTERFACE:
          if (!valParser.Parse(pp, pcntdn)) return false;
          if ((MASK & CP_MASK_COMPARE_CLASS) && uid->bInterfaceClass != CLASS_ID)
            break;
          if ((MASK & CP_MASK_COMPARE_SUBCLASS) && uid->bInterfaceSubClass != SUBCLASS_ID)
            break;
          if (UseOr) {
            if ((!((MASK & CP_MASK_COMPARE_PROTOCOL) && uid->bInterfaceProtocol))) break;
          }
          else if ((MASK & CP_MASK_COMPARE_PROTOCOL) && uid->bInterfaceProtocol != PROTOCOL_ID)
            break;
          isGoodInterface = true;
          ifaceNumber = uid->bInterfaceNumber;
          ifaceAltSet = uid->bAlternateSetting;
          protoValue = uid->bInterfaceProtocol;
          break;
        case USB_DESCRIPTOR_ENDPOINT:
          if (!valParser.Parse(pp, pcntdn)) return false;
          if (isGoodInterface && theXtractor)
            theXtractor->EndpointXtract(confValue, ifaceNumber, ifaceAltSet, protoValue, (USB_FD_ENDPOINT_DESCRIPTOR*)varBuffer);
          break;
          //case HID_DESCRIPTOR_HID:
          //  if (!valParser.Parse(pp, pcntdn)) return false;
          //  PrintHidDescriptor((const USB_HID_DESCRIPTOR*)varBuffer);
          //  break;
        default:
          if (!theSkipper.Skip(pp, pcntdn, dscrLen - 2)) return false;
      }
      theBuffer.pValue = varBuffer;
      stateParseDescr = 0;
  }
  return true;
}

template <const uint8_t CLASS_ID, const uint8_t SUBCLASS_ID, const uint8_t PROTOCOL_ID, const uint8_t MASK>
void ConfigDescParser<CLASS_ID, SUBCLASS_ID, PROTOCOL_ID, MASK>::PrintHidDescriptor(const USB_HID_DESCRIPTOR *pDesc) {
  Notify(PSTR("\r\n\r\nHID Descriptor:\r\n"), 0x80);
  Notify(PSTR("bDescLength:\t\t"), 0x80);
  PrintHex<uint8_t > (pDesc->bLength, 0x80);

  Notify(PSTR("\r\nbDescriptorType:\t"), 0x80);
  PrintHex<uint8_t > (pDesc->bDescriptorType, 0x80);

  Notify(PSTR("\r\nbcdHID:\t\t\t"), 0x80);
  PrintHex<uint16_t > (pDesc->bcdHID, 0x80);

  Notify(PSTR("\r\nbCountryCode:\t\t"), 0x80);
  PrintHex<uint8_t > (pDesc->bCountryCode, 0x80);

  Notify(PSTR("\r\nbNumDescriptors:\t"), 0x80);
  PrintHex<uint8_t > (pDesc->bNumDescriptors, 0x80);

  for (uint_fast8_t i = 0; i < pDesc->bNumDescriptors; i++) {
    HID_CLASS_DESCRIPTOR_LEN_AND_TYPE *pLT = (HID_CLASS_DESCRIPTOR_LEN_AND_TYPE*)&(pDesc->bDescrType);

    Notify(PSTR("\r\nbDescrType:\t\t"), 0x80);
    PrintHex<uint8_t > (pLT[i].bDescrType, 0x80);

    Notify(PSTR("\r\nwDescriptorLength:\t"), 0x80);
    PrintHex<uint16_t > (pLT[i].wDescriptorLength, 0x80);
  }
  Notify(PSTR("\r\n"), 0x80);
}
