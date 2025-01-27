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

#include "../../../inc/MarlinConfigPre.h"

#if HAS_USB_FLASH_DRIVE && DISABLED(USE_UHS3_USB)

#include "Usb.h"

bool MultiByteValueParser::Parse(uint8_t **pp, uint16_t *pcntdn) {
  if (!pBuf) {
    Notify(PSTR("Buffer pointer is NULL!\r\n"), 0x80);
    return false;
  }
  for (; countDown && (*pcntdn); countDown--, (*pcntdn)--, (*pp)++)
    pBuf[valueSize - countDown] = (**pp);

  if (countDown) return false;

  countDown = valueSize;
  return true;
}

bool PTPListParser::Parse(uint8_t **pp, uint16_t *pcntdn, PTP_ARRAY_EL_FUNC pf, const void *me) {
  switch (nStage) {
    case 0:
      pBuf->valueSize = lenSize;
      theParser.Initialize(pBuf);
      nStage = 1;

    case 1:
      if (!theParser.Parse(pp, pcntdn)) return false;

      arLen = 0;
      arLen = (pBuf->valueSize >= 4) ? *((uint32_t*)pBuf->pValue) : (uint32_t)(*((uint16_t*)pBuf->pValue));
      arLenCntdn = arLen;
      nStage = 2;

    case 2:
      pBuf->valueSize = valSize;
      theParser.Initialize(pBuf);
      nStage = 3;

    case 3:
      for (; arLenCntdn; arLenCntdn--) {
        if (!theParser.Parse(pp, pcntdn)) return false;
        if (pf) pf(pBuf, (arLen - arLenCntdn), me);
      }

      nStage = 0;
  }
  return true;
}

#endif // HAS_USB_FLASH_DRIVE && !USE_UHS3_USB
