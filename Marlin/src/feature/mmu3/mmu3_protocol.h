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
#pragma once

/**
 * mmu2_protocol.h
 */

#include "../../MarlinCore.h"

#include <stdint.h>
#include "mmu3_crc.h"

// prevent ARM HAL macros from breaking our code
#undef CRC

namespace modules {

// @brief The MMU communication protocol implementation and related stuff.
//
// See description of the new protocol in the MMU 2021 doc
namespace protocol {

  // Definition of request message codes
  enum class RequestMsgCodes : uint8_t {
    unknown        =   0,
    Query          = 'Q',
    Tool           = 'T',
    Load           = 'L',
    Mode           = 'M',
    Unload         = 'U',
    Reset          = 'X',
    Finda          = 'P',
    Version        = 'S',
    Button         = 'B',
    Eject          = 'E',
    Write          = 'W',
    Cut            = 'K',
    FilamentType   = 'F',
    FilamentSensor = 'f',
    Home           = 'H',
    Read           = 'R'
  };

  // Definition of response message parameter codes
  enum class ResponseMsgParamCodes : uint8_t {
    unknown    =   0,
    Processing = 'P',
    Error      = 'E',
    Finished   = 'F',
    Accepted   = 'A',
    Rejected   = 'R',
    Button     = 'B'// the MMU registered a button press and is sending it to the printer for processing
  };

  // A request message - requests are being sent by the printer into the MMU.
  struct RequestMsg {
    RequestMsgCodes code; //!< code of the request message
    uint8_t value; //!< value of the request message or address of variable to read/write
    uint16_t value2; //!< in case or write messages - value to be written into the register

    // CRC8 check - please note we abuse this byte for CRC of ResponseMsgs as well.
    // The crc8 byte itself is not added into the CRC computation (obviously ;) )
    // Beware - adding any members of this data structure may need changing the way CRC is being computed!
    uint8_t crc8;

    constexpr uint8_t ComputeCRC8() const {
      uint8_t crc = 0;
      crc = modules::crc::CRC8::CCITT_updateCX(0, (uint8_t)code);
      crc = modules::crc::CRC8::CCITT_updateCX(crc, value);
      crc = modules::crc::CRC8::CCITT_updateW(crc, value2);
      return crc;
    }

    // @param code of the request message
    // @param value of the request message
    inline constexpr RequestMsg(RequestMsgCodes code, uint8_t value)
      : code(code)
      , value(value)
      , value2(0)
      , crc8(ComputeCRC8()) {
    }

    // Intended for write requests
    // @param code of the request message ('W')
    // @param address of the register
    // @param value to write into the register
    inline constexpr RequestMsg(RequestMsgCodes code, uint8_t address, uint16_t value)
      : code(code)
      , value(address)
      , value2(value)
      , crc8(ComputeCRC8()) {}

    constexpr uint8_t getCRC() const { return crc8; }
  };

  // A response message - responses are being sent from the MMU into the printer as a response to a request message.
  struct ResponseMsg {
    RequestMsg request; //!< response is always preceeded by the request message
    ResponseMsgParamCodes paramCode; //!< code of the parameter
    uint16_t paramValue; //!< value of the parameter

    constexpr uint8_t ComputeCRC8() const {
      uint8_t crc = request.ComputeCRC8();
      crc = modules::crc::CRC8::CCITT_updateCX(crc, (uint8_t)paramCode);
      crc = modules::crc::CRC8::CCITT_updateW(crc, paramValue);
      return crc;
    }

    // @param request the source request message this response is a reply to
    // @param paramCode code of the parameter
    // @param paramValue value of the parameter
    inline constexpr ResponseMsg(RequestMsg request, ResponseMsgParamCodes paramCode, uint16_t paramValue)
      : request(request)
      , paramCode(paramCode)
      , paramValue(paramValue) {
      this->request.crc8 = ComputeCRC8();
    }

    constexpr uint8_t getCRC() const { return request.crc8; }
  };

  // Combined commandStatus and its value into one data structure (optimization purposes)
  struct ResponseCommandStatus {
    ResponseMsgParamCodes code;
    uint16_t value;
    inline constexpr ResponseCommandStatus(ResponseMsgParamCodes code, uint16_t value)
      : code(code)
      , value(value) {}
  };

  // Message decoding return values
  enum class DecodeStatus : uint_fast8_t {
    MessageCompleted, //!< message completed and successfully lexed
    NeedMoreData, //!< message incomplete yet, waiting for another byte to come
    Error, //!< input character broke message decoding
  };

  // Protocol class is responsible for creating/decoding messages in Rx/Tx buffer
  //
  // Beware - in the decoding more, it is meant to be a statefull instance which works through public methods
  // processing one input byte per call.
  class Protocol {
  public:
    Protocol()
      : rqState(RequestStates::Code)
      , requestMsg(RequestMsgCodes::unknown, 0)
      , rspState(ResponseStates::RequestCode)
      , responseMsg(RequestMsg(RequestMsgCodes::unknown, 0), ResponseMsgParamCodes::unknown, 0) {}

    // Takes the input byte c and steps one step through the state machine
    // @return state of the message being decoded
    DecodeStatus DecodeRequest(uint8_t c);

    // Decodes response message in rxbuff
    // @return decoded response message structure
    DecodeStatus DecodeResponse(uint8_t c);

    // Encodes request message msg into txbuff memory
    // It is expected the txbuff is large enough to fit the message
    // @return number of bytes written into txbuff
    static uint8_t EncodeRequest(const RequestMsg &msg, uint8_t *txbuff);

    // Encodes Write request message msg into txbuff memory
    // It is expected the txbuff is large enough to fit the message
    // @return number of bytes written into txbuff
    static uint8_t EncodeWriteRequest(uint8_t address, uint16_t value, uint8_t *txbuff);

    // @return the maximum byte length necessary to encode a request message
    // Beneficial in case of pre-allocating a buffer for enconding a RequestMsg.
    static constexpr uint8_t MaxRequestSize() { return 13; }

    // @return the maximum byte length necessary to encode a response message
    // Beneficial in case of pre-allocating a buffer for enconding a ResponseMsg.
    static constexpr uint8_t MaxResponseSize() { return 14; }

    // Encode generic response Command Accepted or Rejected
    // @param msg source request message for this response
    // @param ar code of response parameter
    // @param txbuff where to format the message
    // @return number of bytes written into txbuff
    static uint8_t EncodeResponseCmdAR(const RequestMsg &msg, ResponseMsgParamCodes ar, uint8_t *txbuff);

    // Encode response to Read FINDA query
    // @param msg source request message for this response
    // @param findaValue 1/0 (on/off) status of FINDA
    // @param txbuff where to format the message
    // @return number of bytes written into txbuff
    static uint8_t EncodeResponseReadFINDA(const RequestMsg &msg, uint8_t findaValue, uint8_t *txbuff);

    // Encode response to Version query
    // @param msg source request message for this response
    // @param value version number (0-255)
    // @param txbuff where to format the message
    // @return number of bytes written into txbuff
    static uint8_t EncodeResponseVersion(const RequestMsg &msg, uint16_t value, uint8_t *txbuff);

    // Encode response to Query operation status
    // @param msg source request message for this response
    // @param code status of operation (Processing, Error, Finished)
    // @param value related to status of operation(e.g. error code or progress)
    // @param txbuff where to format the message
    // @return number of bytes written into txbuff
    static uint8_t EncodeResponseQueryOperation(const RequestMsg &msg, ResponseCommandStatus rcs, uint8_t *txbuff);

    // Encode response to Read query
    // @param msg source request message for this response
    // @param accepted true if the read query was accepted
    // @param value2 variable value
    // @param txbuff where to format the message
    // @return number of bytes written into txbuff
    static uint8_t EncodeResponseRead(const RequestMsg &msg, bool accepted, uint16_t value2, uint8_t *txbuff);

    // @return the most recently lexed request message
    inline const RequestMsg GetRequestMsg() const { return requestMsg; }

    // @return the most recently lexed response message
    inline const ResponseMsg GetResponseMsg() const { return responseMsg; }

    // resets the internal request decoding state (typically after an error)
    void ResetRequestDecoder() {
      rqState = RequestStates::Code;
    }

    // resets the internal response decoding state (typically after an error)
    void ResetResponseDecoder() {
      rspState = ResponseStates::RequestCode;
    }

    #ifndef UNITTEST
      private:
    #endif

    enum class RequestStates : uint8_t {
      Code,       //!< starting state - expects message code
      Value,      //!< expecting code value
      Address,    //!< expecting address for Write command
      WriteValue, //!< value to be written (Write command)
      CRC,        //!< CRC
      Error       //!< automaton in error state
    };

    RequestStates rqState;
    RequestMsg requestMsg;

    enum class ResponseStates : uint8_t {
      RequestCode,  //!< starting state - expects message code
      RequestValue, //!< expecting code value
      ParamCode,    //!< expecting param code
      ParamValue,   //!< expecting param value
      CRC,          //!< expecting CRC value
      Error         //!< automaton in error state
    };

    ResponseStates rspState;
    ResponseMsg responseMsg;

    static constexpr bool IsNewLine(uint8_t c) {
      return c == '\n' || c == '\r';
    }
    static constexpr bool IsDigit(uint8_t c) {
      return c >= '0' && c <= '9';
    }
    static constexpr bool IsCRCSeparator(uint8_t c) {
      return c == '*';
    }
    static constexpr bool IsHexDigit(uint8_t c) {
      return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    }
    static constexpr uint8_t Char2Nibble(uint8_t c) {
      switch (c) {
        case '0' ... '9': return c - '0';
        case 'a' ... 'f': return c - 'a' + 10;
        default: return 0;
      }
    }

    static constexpr uint8_t Nibble2Char(uint8_t n) {
      switch (n) {
        case 0x0 ... 0x9: return n + '0';
        case 0xA ... 0xF: return n - 10 + 'a';
        default: return 0;
      }
    }

    // @return number of characters written
    static uint8_t UInt8ToHex(uint8_t value, uint8_t *dst);

    // @return number of characters written
    static uint8_t UInt16ToHex(uint16_t value, uint8_t *dst);

    static uint8_t BeginEncodeRequest(const RequestMsg &msg, uint8_t *dst);

    static uint8_t AppendCRC(uint8_t crc, uint8_t *dst);
  };

} // namespace protocol
} // namespace modules

