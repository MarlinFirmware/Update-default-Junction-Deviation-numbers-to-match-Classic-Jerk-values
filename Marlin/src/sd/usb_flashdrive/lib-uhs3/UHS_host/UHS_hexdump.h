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

#if !defined(_usb_h_) || defined(__HEXDUMP_H__)
#error "Never include UHS_hexdump.h directly; include UHS_Usb.h instead"
#else
#define __HEXDUMP_H__

extern int UsbDEBUGlvl;

template <class BASE_CLASS, class LEN_TYPE, class OFFSET_TYPE>
class HexDumper : public BASE_CLASS {
        uint8_t byteCount;
        OFFSET_TYPE byteTotal;

public:

        HexDumper() : byteCount(0), byteTotal(0) {
        };

        void Initialize() {
                byteCount = 0;
                byteTotal = 0;
        };

        virtual void Parse(const LEN_TYPE len, const uint8_t *pbuf, const OFFSET_TYPE &offset);
};

template <class BASE_CLASS, class LEN_TYPE, class OFFSET_TYPE>
void HexDumper<BASE_CLASS, LEN_TYPE, OFFSET_TYPE>::Parse(const LEN_TYPE len, const uint8_t *pbuf, const OFFSET_TYPE &offset) {
        if(UsbDEBUGlvl >= 0x80) { // Fully bypass this block of code if we do not debug.
                for(LEN_TYPE j = 0; j < len; j++, byteCount++, byteTotal++) {
                        if(!byteCount) {
                                PrintHex<OFFSET_TYPE > (byteTotal, 0x80);
                                E_Notify(PSTR(": "), 0x80);
                        }
                        PrintHex<uint8_t > (pbuf[j], 0x80);
                        E_Notify(PSTR(" "), 0x80);

                        if(byteCount == 15) {
                                E_Notify(PSTR("\r\n"), 0x80);
                                byteCount = 0xFF;
                        }
                }
        }
}

#endif // __HEXDUMP_H__
