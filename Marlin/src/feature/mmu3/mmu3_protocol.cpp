/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * mmu2_protocol.cpp
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU3

#include "mmu3_protocol.h"

// protocol definition
// command: Q0
// meaning: query operation status
// Query/command: query
// Expected reply from the MMU:
//  any of the running operation statuses: OID: [T|L|U|E|C|W|K][0-4]
//  <OID> P[0-9]      : command being processed i.e. operation running, may contain a state number
//  <OID> E[0-9][0-9] : error 1-9 while doing a tool change
//  <OID> F[0-9]      : operation finished - will be repeated to "Q" messages until a new command is issued

namespace modules {
namespace protocol {

  // decoding automaton
  // states:  input -> transition into state
  // Code      QTLMUXPSBEWK -> msgcode
  //           \n ->start
  //           * ->error
  // error     \n ->start
  //           * ->error
  // msgcode   0-9 ->msgvalue
  //           * ->error
  // msgvalue  0-9 ->msgvalue
  //           \n ->start successfully accepted command

  DecodeStatus Protocol::DecodeRequest(uint8_t c) {
    switch (rqState) {
      case RequestStates::Code:
        switch (c) {
          case 'Q':
          case 'T':
          case 'L':
          case 'M':
          case 'U':
          case 'X':
          case 'P':
          case 'S':
          case 'B':
          case 'E':
          case 'W': // write is gonna be a special one
          case 'K':
          case 'F':
          case 'f':
          case 'H':
          case 'R':
            requestMsg.code = (RequestMsgCodes)c;
            requestMsg.value = 0;
            requestMsg.value2 = 0;
            requestMsg.crc8 = 0;
            rqState = (c == 'W') ? RequestStates::Address : RequestStates::Value; // prepare special automaton path for Write commands
            return DecodeStatus::NeedMoreData;
          default:
            requestMsg.code = RequestMsgCodes::unknown;
            rqState = RequestStates::Error;
            return DecodeStatus::Error;
        }
      case RequestStates::Value:
        if (IsHexDigit(c)) {
          requestMsg.value <<= 4U;
          requestMsg.value |= Char2Nibble(c);
          return DecodeStatus::NeedMoreData;
        }
        else if (IsCRCSeparator(c)) {
          rqState = RequestStates::CRC;
          return DecodeStatus::NeedMoreData;
        }
        else {
          requestMsg.code = RequestMsgCodes::unknown;
          rqState = RequestStates::Error;
          return DecodeStatus::Error;
        }
      case RequestStates::Address:
        if (IsHexDigit(c)) {
          requestMsg.value <<= 4U;
          requestMsg.value |= Char2Nibble(c);
          return DecodeStatus::NeedMoreData;
        }
        else if (c == ' ') { // end of address, value coming
          rqState = RequestStates::WriteValue;
          return DecodeStatus::NeedMoreData;
        }
        else {
          requestMsg.code = RequestMsgCodes::unknown;
          rqState = RequestStates::Error;
          return DecodeStatus::Error;
        }
      case RequestStates::WriteValue:
        if (IsHexDigit(c)) {
          requestMsg.value2 <<= 4U;
          requestMsg.value2 |= Char2Nibble(c);
          return DecodeStatus::NeedMoreData;
        }
        else if (IsCRCSeparator(c)) {
          rqState = RequestStates::CRC;
          return DecodeStatus::NeedMoreData;
        }
        else {
          requestMsg.code = RequestMsgCodes::unknown;
          rqState = RequestStates::Error;
          return DecodeStatus::Error;
        }
      case RequestStates::CRC:
        if (IsHexDigit(c)) {
          requestMsg.crc8 <<= 4U;
          requestMsg.crc8 |= Char2Nibble(c);
          return DecodeStatus::NeedMoreData;
        }
        else if (IsNewLine(c)) {
          // check CRC at this spot
          if (requestMsg.crc8 != requestMsg.ComputeCRC8()) {
            // CRC mismatch
            requestMsg.code = RequestMsgCodes::unknown;
            rqState = RequestStates::Error;
            return DecodeStatus::Error;
          }
          else {
            rqState = RequestStates::Code;
            return DecodeStatus::MessageCompleted;
          }
        }
        else {
          requestMsg.code = RequestMsgCodes::unknown;
          rqState = RequestStates::Error;
          return DecodeStatus::Error;
        }
      default: // case error:
        if (IsNewLine(c)) {
          rqState = RequestStates::Code;
          return DecodeStatus::MessageCompleted;
        }
        else {
          requestMsg.code = RequestMsgCodes::unknown;
          rqState = RequestStates::Error;
          return DecodeStatus::Error;
        }
    }
  }

  uint8_t Protocol::EncodeRequest(const RequestMsg &msg, uint8_t *txbuff) {
    txbuff[0] = (uint8_t)msg.code;
    uint8_t i = 1 + UInt8ToHex(msg.value, txbuff + 1);

    i += AppendCRC(msg.getCRC(), txbuff + i);

    txbuff[i] = '\n';
    ++i;
    return i;
    static_assert(7 <= MaxRequestSize(), "Request message length exceeded the maximum size, increase the magic constant in MaxRequestSize()");
  }

  uint8_t Protocol::EncodeWriteRequest(uint8_t address, uint16_t value, uint8_t *txbuff) {
    const RequestMsg msg(RequestMsgCodes::Write, address, value);
    uint8_t i = BeginEncodeRequest(msg, txbuff);
    // dump the value
    i += UInt16ToHex(value, txbuff + i);

    i += AppendCRC(msg.getCRC(), txbuff + i);

    txbuff[i] = '\n';
    ++i;
    return i;
  }

  DecodeStatus Protocol::DecodeResponse(uint8_t c) {
    switch (rspState) {
      case ResponseStates::RequestCode:
        switch (c) {
          case 'Q':
          case 'T':
          case 'L':
          case 'M':
          case 'U':
          case 'X':
          case 'P':
          case 'S':
          case 'B':
          case 'E':
          case 'W':
          case 'K':
          case 'F':
          case 'f':
          case 'H':
          case 'R':
            responseMsg.request.code = (RequestMsgCodes)c;
            responseMsg.request.value = 0;
            responseMsg.request.value2 = 0;
            responseMsg.request.crc8 = 0;
            rspState = ResponseStates::RequestValue;
            return DecodeStatus::NeedMoreData;
          case 0x0a:
          case 0x0d:
            // skip leading whitespace if any (makes integration with other SW easier/tolerant)
            return DecodeStatus::NeedMoreData;
          default:
            rspState = ResponseStates::Error;
            return DecodeStatus::Error;
        }
      case ResponseStates::RequestValue:
        if (IsHexDigit(c)) {
          responseMsg.request.value <<= 4U;
          responseMsg.request.value += Char2Nibble(c);
          return DecodeStatus::NeedMoreData;
        }
        else if (c == ' ') {
          rspState = ResponseStates::ParamCode;
          return DecodeStatus::NeedMoreData;
        }
        else {
          rspState = ResponseStates::Error;
          return DecodeStatus::Error;
        }
      case ResponseStates::ParamCode:
        switch (c) {
          case 'P':
          case 'E':
          case 'F':
          case 'A':
          case 'R':
          case 'B':
            rspState = ResponseStates::ParamValue;
            responseMsg.paramCode = (ResponseMsgParamCodes)c;
            responseMsg.paramValue = 0;
            return DecodeStatus::NeedMoreData;
          default:
            responseMsg.paramCode = ResponseMsgParamCodes::unknown;
            rspState = ResponseStates::Error;
            return DecodeStatus::Error;
        }
      case ResponseStates::ParamValue:
        if (IsHexDigit(c)) {
          responseMsg.paramValue <<= 4U;
          responseMsg.paramValue += Char2Nibble(c);
          return DecodeStatus::NeedMoreData;
        }
        else if (IsCRCSeparator(c)) {
          rspState = ResponseStates::CRC;
          return DecodeStatus::NeedMoreData;
        }
        else {
          responseMsg.paramCode = ResponseMsgParamCodes::unknown;
          rspState = ResponseStates::Error;
          return DecodeStatus::Error;
        }
      case ResponseStates::CRC:
        if (IsHexDigit(c)) {
          responseMsg.request.crc8 <<= 4U;
          responseMsg.request.crc8 += Char2Nibble(c);
          return DecodeStatus::NeedMoreData;
        }
        else if (IsNewLine(c)) {
          // check CRC at this spot
          if (responseMsg.request.crc8 != responseMsg.ComputeCRC8()) {
            // CRC mismatch
            responseMsg.paramCode = ResponseMsgParamCodes::unknown;
            rspState = ResponseStates::Error;
            return DecodeStatus::Error;
          }
          else {
            rspState = ResponseStates::RequestCode;
            return DecodeStatus::MessageCompleted;
          }
        }
        else {
          responseMsg.paramCode = ResponseMsgParamCodes::unknown;
          rspState = ResponseStates::Error;
          return DecodeStatus::Error;
        }
      default: // case error:
        if (IsNewLine(c)) {
          rspState = ResponseStates::RequestCode;
          return DecodeStatus::MessageCompleted;
        }
        else {
          responseMsg.paramCode = ResponseMsgParamCodes::unknown;
          return DecodeStatus::Error;
        }
    }
  }

  uint8_t Protocol::EncodeResponseCmdAR(const RequestMsg &msg, ResponseMsgParamCodes ar, uint8_t *txbuff) {
    // BEWARE:
    // ResponseMsg rsp(RequestMsg(msg.code, msg.value), ar, 0);
    // ... is NOT the same as:
    // ResponseMsg rsp(msg, ar, 0);
    // ... because of the usually unused parameter value2 (which only comes non-zero in write requests).
    // It took me a few hours to find out why the CRC from the MMU never matched all the other sides (unit tests and the MK3S)
    // It is because this was the only place where the original request kept its value2 non-zero.
    // In the response, we must make sure value2 is actually zero unless being sent along with it (which is not right now)
    const ResponseMsg rsp(RequestMsg(msg.code, msg.value), ar, 0); // this needs some cleanup @@TODO - check assembly how bad is it
    uint8_t i = BeginEncodeRequest(rsp.request, txbuff);
    txbuff[i] = (uint8_t)ar;
    ++i;
    i += AppendCRC(rsp.getCRC(), txbuff + i);
    txbuff[i] = '\n';
    ++i;
    return i;
  }

  uint8_t Protocol::EncodeResponseReadFINDA(const RequestMsg &msg, uint8_t findaValue, uint8_t *txbuff) {
    return EncodeResponseRead(msg, true, findaValue, txbuff);
  }

  uint8_t Protocol::EncodeResponseQueryOperation(const RequestMsg &msg, ResponseCommandStatus rcs, uint8_t *txbuff) {
    const ResponseMsg rsp(msg, rcs.code, rcs.value);
    uint8_t i = BeginEncodeRequest(msg, txbuff);
    txbuff[i] = (uint8_t)rsp.paramCode;
    ++i;
    i += UInt16ToHex(rsp.paramValue, txbuff + i);
    i += AppendCRC(rsp.getCRC(), txbuff + i);
    txbuff[i] = '\n';
    return i + 1;
  }

  uint8_t Protocol::EncodeResponseRead(const RequestMsg &msg, bool accepted, uint16_t value2, uint8_t *txbuff) {
    const ResponseMsg rsp(msg,
                          accepted ? ResponseMsgParamCodes::Accepted : ResponseMsgParamCodes::Rejected,
                          accepted ? value2 : 0 // be careful about this value for CRC computation - rejected status doesn't have any meaningful value which could be reconstructed from the textual form of the message
                          );
    uint8_t i = BeginEncodeRequest(msg, txbuff);
    txbuff[i] = (uint8_t)rsp.paramCode;
    ++i;
    if (accepted)
      // dump the value
      i += UInt16ToHex(value2, txbuff + i);
    i += AppendCRC(rsp.getCRC(), txbuff + i);
    txbuff[i] = '\n';
    return i + 1;
  }

  uint8_t Protocol::UInt8ToHex(uint8_t value, uint8_t *dst) {
    if (value == 0) {
      *dst = '0';
      return 1;
    }

    uint8_t v = value >> 4U;
    uint8_t charsOut = 1;
    if (v != 0) { // skip the first '0' if any
      *dst = Nibble2Char(v);
      ++dst;
      charsOut = 2;
    }
    v = value & 0xfU;
    *dst = Nibble2Char(v);
    return charsOut;
  }

  uint8_t Protocol::UInt16ToHex(uint16_t value, uint8_t *dst) {
    constexpr uint16_t topNibbleMask = 0xf000;
    if (value == 0) {
      *dst = '0';
      return 1;
    }
    // skip initial zeros
    uint8_t charsOut = 4;
    while ((value & topNibbleMask) == 0) {
      value <<= 4U;
      --charsOut;
    }
    for (uint8_t i = 0; i < charsOut; ++i) {
      uint8_t n = (value & topNibbleMask) >> (8U + 4U);
      value <<= 4U;
      *dst = Nibble2Char(n);
      ++dst;
    }
    return charsOut;
  }

  uint8_t Protocol::BeginEncodeRequest(const RequestMsg &msg, uint8_t *dst) {
    dst[0] = (uint8_t)msg.code;

    uint8_t i = 1 + UInt8ToHex(msg.value, dst + 1);

    dst[i] = ' ';
    return i + 1;
  }

  uint8_t Protocol::AppendCRC(uint8_t crc, uint8_t *dst) {
    dst[0] = '*'; // reprap-style separator of CRC
    return 1 + UInt8ToHex(crc, dst + 1);
  }

} // namespace protocol
} // namespace modules

#endif // HAS_PRUSA_MMU3
