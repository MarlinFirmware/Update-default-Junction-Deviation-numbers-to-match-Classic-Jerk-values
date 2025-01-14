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
 * mmu2_protocol_logic.cpp
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU3

#include "mmu3_protocol_logic.h"
#include "mmu3_log.h"
#include "mmu3_fsensor.h"

  #ifdef __AVR__
    // on MK3/S/+ we shuffle the timers a bit, thus "_millis" may not equal "millis"
    // #include "system_timer.h"
    #define _millis millis
  #else
    // irrelevant on Buddy FW, just keep "_millis" as "millis"
    // #include <wiring_time.h>
    #define _millis millis
    #ifdef UNITTEST
      #define strncmp_P strncmp
    #else
      #include "../../core/serial.h"
    #endif
  #endif

  #include <string.h>
  #include "mmu3_supported_version.h"

namespace MMU3 {

  static constexpr uint8_t supportedMmuFWVersion[] PROGMEM = { mmuVersionMajor, mmuVersionMinor, mmuVersionPatch };

  const Register ProtocolLogic::regs8Addrs[ProtocolLogic::regs8Count] PROGMEM = {
    Register::FINDA_State,            // FINDA state
    Register::Set_Get_Selector_Slot,  // Selector slot
    Register::Set_Get_Idler_Slot,     // Idler slot
  };

  const Register ProtocolLogic::regs16Addrs[ProtocolLogic::regs16Count] PROGMEM = {
    Register::MMU_Errors,             // MMU errors - aka statistics
    Register::Get_Pulley_Position,    // Pulley position [mm]
  };

  const Register ProtocolLogic::initRegs8Addrs[ProtocolLogic::initRegs8Count] PROGMEM = {
    Register::Extra_Load_Distance,    // Extra load distance [mm]
    Register::Pulley_Slow_Feedrate,   // Pulley slow feedrate [mm/s]
  };

  void ProtocolLogic::CheckAndReportAsyncEvents() {
    // even when waiting for a query period, we need to report a change in filament sensor's state
    // - it is vital for a precise synchronization of moves of the printer and the MMU
    uint8_t fs = (uint8_t)WhereIsFilament();
    if (fs != lastFSensor)
      SendAndUpdateFilamentSensor();
  }

  void ProtocolLogic::SendQuery() {
    SendMsg(RequestMsg(RequestMsgCodes::Query, 0));
    scopeState = ScopeState::QuerySent;
  }

  void ProtocolLogic::StartReading8bitRegisters() {
    regIndex = 0;
    SendReadRegister(pgm_read_byte(regs8Addrs + regIndex), ScopeState::Reading8bitRegisters);
  }

  void ProtocolLogic::ProcessRead8bitRegister() {
    regs8[regIndex] = rsp.paramValue;
    ++regIndex;
    if (regIndex >= regs8Count)
      // proceed with reading 16bit registers
      StartReading16bitRegisters();
    else
      SendReadRegister(pgm_read_byte(regs8Addrs + regIndex), ScopeState::Reading8bitRegisters);
  }

  void ProtocolLogic::StartReading16bitRegisters() {
    regIndex = 0;
    SendReadRegister(pgm_read_byte(regs16Addrs + regIndex), ScopeState::Reading16bitRegisters);
  }

  ProtocolLogic::ScopeState __attribute__((noinline)) ProtocolLogic::ProcessRead16bitRegister(ProtocolLogic::ScopeState stateAtEnd) {
    regs16[regIndex] = rsp.paramValue;
    ++regIndex;
    if (regIndex >= regs16Count)
      return stateAtEnd;
    else
      SendReadRegister(pgm_read_byte(regs16Addrs + regIndex), ScopeState::Reading16bitRegisters);
    return ScopeState::Reading16bitRegisters;
  }

  void ProtocolLogic::StartWritingInitRegisters() {
    regIndex = 0;
    SendWriteRegister(pgm_read_byte(initRegs8Addrs + regIndex), initRegs8[regIndex], ScopeState::WritingInitRegisters);
  }

  bool __attribute__((noinline)) ProtocolLogic::ProcessWritingInitRegister() {
    ++regIndex;
    if (regIndex >= initRegs8Count)
      return true;
    else
      SendWriteRegister(pgm_read_byte(initRegs8Addrs + regIndex), initRegs8[regIndex], ScopeState::WritingInitRegisters);
    return false;
  }

  void ProtocolLogic::SendAndUpdateFilamentSensor() {
    SendMsg(RequestMsg(RequestMsgCodes::FilamentSensor, lastFSensor = (uint8_t)WhereIsFilament()));
    scopeState = ScopeState::FilamentSensorStateSent;
  }

  void ProtocolLogic::SendButton(uint8_t btn) {
    SendMsg(RequestMsg(RequestMsgCodes::Button, btn));
    scopeState = ScopeState::ButtonSent;
  }

  void ProtocolLogic::SendVersion(uint8_t stage) {
    SendMsg(RequestMsg(RequestMsgCodes::Version, stage));
    scopeState = (ScopeState)((uint_fast8_t)ScopeState::S0Sent + stage);
  }

  void ProtocolLogic::SendReadRegister(uint8_t index, ScopeState nextState) {
    SendMsg(RequestMsg(RequestMsgCodes::Read, index));
    scopeState = nextState;
  }

  void ProtocolLogic::SendWriteRegister(uint8_t index, uint16_t value, ScopeState nextState) {
    SendWriteMsg(RequestMsg(RequestMsgCodes::Write, index, value));
    scopeState = nextState;
  }

  // searches for "ok\n" in the incoming serial data (that's the usual response of the old MMU FW)
  struct OldMMUFWDetector {
    uint8_t ok;
    inline constexpr OldMMUFWDetector()
      : ok(0) {}

    enum class State : uint8_t {
      MatchingPart,
      SomethingElse,
      Matched
    };

    // @return true when "ok\n" gets detected
    State Detect(uint8_t c) {
      // consume old MMU FW's data if any -> avoid confusion of protocol decoder
      if (ok == 0 && c == 'o') {
        ++ok;
        return State::MatchingPart;
      }
      else if (ok == 1 && c == 'k') {
        ++ok;
        return State::Matched;
      }
      return State::SomethingElse;
    }
  };

  StepStatus ProtocolLogic::ExpectingMessage() {
    int bytesConsumed = 0;
    int c = -1;

    OldMMUFWDetector oldMMUh4x0r; // old MMU FW hacker ;)

    // try to consume as many rx bytes as possible (until a message has been completed)
    while ((c = MMU_SERIAL.read()) >= 0) {
      ++bytesConsumed;
      RecordReceivedByte(c);
      switch (protocol.DecodeResponse(c)) {
        case DecodeStatus::MessageCompleted:
          rsp = protocol.GetResponseMsg();
          LogResponse();
          // @@TODO reset direction of communication
          RecordUARTActivity(); // something has happened on the UART, update the timeout record
          return MessageReady;
        case DecodeStatus::NeedMoreData:
          break;
        case DecodeStatus::Error: {
          // consume old MMU FW's data if any -> avoid confusion of protocol decoder
          auto old = oldMMUh4x0r.Detect(c);
          if (old == OldMMUFWDetector::State::Matched)
            // Old MMU FW 1.0.6 detected. Firmwares are incompatible.
            return VersionMismatch;
          else if (old == OldMMUFWDetector::State::MatchingPart)
            break;
        }
        // [[fallthrough]];      // otherwise
        // fall through
        default:
          RecordUARTActivity(); // something has happened on the UART, update the timeout record
          return ProtocolError;
      }
    }
    if (bytesConsumed != 0) {
      RecordUARTActivity(); // something has happened on the UART, update the timeout record
      return Processing;  // consumed some bytes, but message still not ready
    }
    else if (Elapsed(linkLayerTimeout) && currentScope != Scope::Stopped) {
      return CommunicationTimeout;
    }
    return Processing;
  }

  void ProtocolLogic::SendMsg(RequestMsg rq) {
    #if defined(__AVR__) || defined(TARGET_LPC1768)
      // Buddy FW cannot use stack-allocated txbuff - DMA doesn't work with CCMRAM
      // No restrictions on MK3/S/+ though
      uint8_t txbuff[Protocol::MaxRequestSize()];
    #endif
    uint8_t len = Protocol::EncodeRequest(rq, txbuff);
    #if defined(__AVR__) || defined(TARGET_LPC1768)
      // TODO: I'm not sure if this is the correct approach with AVR
      for ( uint8_t i = 0; i < len; i++) {
        MMU_SERIAL.write(txbuff[i]);
      }
    #else
      MMU_SERIAL.write(txbuff, len);
    #endif
    LogRequestMsg(txbuff, len);
    RecordUARTActivity();
  }

  void ProtocolLogic::SendWriteMsg(RequestMsg rq) {
    #if defined(__AVR__) || defined(TARGET_LPC1768)
      // Buddy FW cannot use stack-allocated txbuff - DMA doesn't work with CCMRAM
      // No restrictions on MK3/S/+ though
      uint8_t txbuff[Protocol::MaxRequestSize()];
    #endif
    uint8_t len = Protocol::EncodeWriteRequest(rq.value, rq.value2, txbuff);

    #if defined(__AVR__) || defined(TARGET_LPC1768)
      // TODO: I'm not sure if this is the correct approach with AVR
      for ( uint8_t i = 0; i < len; i++) {
        MMU_SERIAL.write(txbuff[i]);
      }
    #else
      MMU_SERIAL.write(txbuff, len);
    #endif
    LogRequestMsg(txbuff, len);
    RecordUARTActivity();
  }

  void ProtocolLogic::StartSeqRestart() {
    retries = maxRetries;
    SendVersion(0);
  }

  void ProtocolLogic::DelayedRestartRestart() {
    scopeState = ScopeState::RecoveringProtocolError;
  }

  void ProtocolLogic::CommandRestart() {
    scopeState = ScopeState::CommandSent;
    SendMsg(rq);
  }

  void ProtocolLogic::IdleRestart() {
    scopeState = ScopeState::Ready;
  }

  StepStatus ProtocolLogic::ProcessVersionResponse(uint8_t stage) {
    if (rsp.request.code != RequestMsgCodes::Version || rsp.request.value != stage) {
      // got a response to something else - protocol corruption probably, repeat the query OR restart the comm by issuing S0?
      SendVersion(stage);
    }
    else {
      mmuFwVersion[stage] = rsp.paramValue;
      if (mmuFwVersion[stage] != pgm_read_byte(&supportedMmuFWVersion[stage])) {
        if (--retries == 0) return VersionMismatch;
        SendVersion(stage);
      }
      else {
        ResetCommunicationTimeoutAttempts(); // got a meaningful response from the MMU, stop data layer timeout tracking
        SendVersion(stage + 1);
      }
    }
    return Processing;
  }

  StepStatus ProtocolLogic::ScopeStep() {
    if (!ExpectsResponse()) {
      // we are waiting for something
      switch (currentScope) {
        case Scope::DelayedRestart:
          return DelayedRestartWait();
        case Scope::Idle:
          return IdleWait();
        case Scope::Command:
          return CommandWait();
        case Scope::Stopped:
          return StoppedStep();
        default:
          break;
      }
    }
    else {
      // we are expecting a message
      auto expmsg = ExpectingMessage();
      if (expmsg != MessageReady)
        return expmsg;

      // process message
      switch (currentScope) {
        case Scope::StartSeq:
          return StartSeqStep(); // ~270B
        case Scope::Idle:
          return IdleStep(); // ~300B
        case Scope::Command:
          return CommandStep(); // ~430B
        case Scope::Stopped:
          return StoppedStep();
        default:
          break;
      }
    }
    return Finished;
  }

  StepStatus ProtocolLogic::StartSeqStep() {
    // solve initial handshake
    switch (scopeState) {
      case ScopeState::S0Sent: // received response to S0 - major
      case ScopeState::S1Sent: // received response to S1 - minor
      case ScopeState::S2Sent: // received response to S2 - patch
        return ProcessVersionResponse((uint8_t)scopeState - (uint8_t)ScopeState::S0Sent);
      case ScopeState::S3Sent: // received response to S3 - revision
        if (rsp.request.code != RequestMsgCodes::Version || rsp.request.value != 3) {
          // got a response to something else - protocol corruption probably, repeat the query OR restart the comm by issuing S0?
          SendVersion(3);
        }
        else {
          mmuFwVersionBuild = rsp.paramValue; // just register the build number
          // Start General Interrogation after line up - initial parametrization is started
          StartWritingInitRegisters();
        }
        return Processing;
      case ScopeState::WritingInitRegisters:
        if (ProcessWritingInitRegister())
          SendAndUpdateFilamentSensor();
        return Processing;
      case ScopeState::FilamentSensorStateSent:
        SwitchFromStartToIdle();
        return Processing; // Returning Finished is not a good idea in case of a fast error recovery
      // - it tells the printer, that the command which experienced a protocol error and recovered successfully actually terminated.
      // In such a case we must return "Processing" in order to keep the MMU state machine running and prevent the printer from executing next G-codes.
      default:
        return VersionMismatch;
    }
  }

  StepStatus ProtocolLogic::DelayedRestartWait() {
    if (Elapsed(heartBeatPeriod)) { // this basically means, that we are waiting until there is some traffic on
      while (MMU_SERIAL.read() != -1); // clear the input buffer
      // switch to StartSeq
      start();
    }
    return Processing;
  }

  StepStatus ProtocolLogic::CommandWait() {
    if (Elapsed(heartBeatPeriod))
      SendQuery();
    else
      // even when waiting for a query period, we need to report a change in filament sensor's state
      // - it is vital for a precise synchronization of moves of the printer and the MMU
      CheckAndReportAsyncEvents();
    return Processing;
  }

  StepStatus ProtocolLogic::ProcessCommandQueryResponse() {
    switch (rsp.paramCode) {
      case ResponseMsgParamCodes::Processing:
        progressCode = static_cast<ProgressCode>(rsp.paramValue);
        errorCode = ErrorCode::OK;
        SendAndUpdateFilamentSensor(); // keep on reporting the state of fsensor regularly
        return Processing;
      case ResponseMsgParamCodes::Error:
        // in case of an error the progress code remains as it has been before
        progressCode = ProgressCode::ERRWaitingForUser;
        errorCode = static_cast<ErrorCode>(rsp.paramValue);
        // keep on reporting the state of fsensor regularly even in command error state
        // - the MMU checks FINDA and fsensor even while recovering from errors
        SendAndUpdateFilamentSensor();
        return CommandError;
      case ResponseMsgParamCodes::Button:
        // The user pushed a button on the MMU. Save it, do what we need to do
        // to prepare, then pass it back to the MMU so it can work its magic.
        buttonCode = static_cast<Buttons>(rsp.paramValue);
        SendAndUpdateFilamentSensor();
        return ButtonPushed;
      case ResponseMsgParamCodes::Finished:
        // We must check whether the "finished" is actually related to the command issued into the MMU
        // It can also be an X0 F which means MMU just successfully restarted.
        if (ReqMsg().code == rsp.request.code && ReqMsg().value == rsp.request.value) {
          progressCode = ProgressCode::OK;
          errorCode = ErrorCode::OK;
          scopeState = ScopeState::Ready;
          rq = RequestMsg(RequestMsgCodes::unknown, 0); // clear the successfully finished request
          return Finished;
        }
        else {
          // got response to some other command - the originally issued command was interrupted!
          return Interrupted;
        }
      default:
        return ProtocolError;
    }
  }

  StepStatus ProtocolLogic::CommandStep() {
    switch (scopeState) {
      case ScopeState::CommandSent: {
        switch (rsp.paramCode) { // the response should be either accepted or rejected
          case ResponseMsgParamCodes::Accepted:
            progressCode = ProgressCode::OK;
            errorCode = ErrorCode::RUNNING;
            scopeState = ScopeState::Wait;
            break;
          case ResponseMsgParamCodes::Rejected:
            // rejected - should normally not happen, but report the error up
            progressCode = ProgressCode::OK;
            errorCode = ErrorCode::PROTOCOL_ERROR;
            return CommandRejected;
          default:
            return ProtocolError;
        }
      }
      break;
      case ScopeState::QuerySent:
        return ProcessCommandQueryResponse();
      case ScopeState::FilamentSensorStateSent:
        StartReading8bitRegisters();
        return Processing;
      case ScopeState::Reading8bitRegisters:
        ProcessRead8bitRegister();
        return Processing;
      case ScopeState::Reading16bitRegisters:
        scopeState = ProcessRead16bitRegister(ScopeState::Wait);
        return Processing;
      case ScopeState::ButtonSent:
        if (rsp.paramCode == ResponseMsgParamCodes::Accepted)
          // Button was accepted, decrement the retry.
          DecrementRetryAttempts();
        SendAndUpdateFilamentSensor();
        break;
      default:
        return ProtocolError;
    }
    return Processing;
  }

  StepStatus ProtocolLogic::IdleWait() {
    if (scopeState == ScopeState::Ready) { // check timeout
      if (Elapsed(heartBeatPeriod)) {
        SendQuery();
        return Processing;
      }
    }
    return Finished;
  }

  StepStatus ProtocolLogic::IdleStep() {
    switch (scopeState) {
      case ScopeState::QuerySent: // check UART
        // If we are accidentally in Idle and we receive something like "T0 P1" - that means the communication dropped out while a command was in progress.
        // That causes no issues here, we just need to switch to Command processing and continue there from now on.
        // The usual response in this case should be some command and "F" - finished - that confirms we are in an Idle state even on the MMU side.
        switch (rsp.request.code) {
          case RequestMsgCodes::Cut:
          case RequestMsgCodes::Eject:
          case RequestMsgCodes::Load:
          case RequestMsgCodes::Mode:
          case RequestMsgCodes::Tool:
          case RequestMsgCodes::Unload:
            if (rsp.paramCode != ResponseMsgParamCodes::Finished)
              return SwitchFromIdleToCommand();
            break;
          case RequestMsgCodes::Reset:
            // this one is kind of special
            // we do not transfer to any "running" command (i.e. we stay in Idle),
            // but in case there is an error reported we must make sure it gets propagated
            switch (rsp.paramCode) {
              case ResponseMsgParamCodes::Button:
                // The user pushed a button on the MMU. Save it, do what we need to do
                // to prepare, then pass it back to the MMU so it can work its magic.
                buttonCode = static_cast<Buttons>(rsp.paramValue);
                StartReading8bitRegisters();
                return ButtonPushed;
              case ResponseMsgParamCodes::Finished:
                if (ReqMsg().code != RequestMsgCodes::unknown) {
                  // got reset while doing some other command - the originally issued command was interrupted!
                  // this must be solved by the upper layer, protocol logic doesn't have all the context (like unload before trying again)
                  IdleRestart();
                  return Interrupted;
                }
              // [[fallthrough]];
              // fall through
              case ResponseMsgParamCodes::Processing:
                // @@TODO we may actually use this branch to report progress of manual operation on the MMU
                // The MMU sends e.g. X0 P27 after its restart when the user presses an MMU button to move the Selector
                progressCode = static_cast<ProgressCode>(rsp.paramValue);
                errorCode = ErrorCode::OK;
                break;
              default:
                progressCode = ProgressCode::ERRWaitingForUser;
                errorCode = static_cast<ErrorCode>(rsp.paramValue);
                StartReading8bitRegisters(); // continue Idle state without restarting the communication
                return CommandError;
            }
            break;
          default:
            return ProtocolError;
        }
        StartReading8bitRegisters();
        return Processing;
      case ScopeState::Reading8bitRegisters:
        ProcessRead8bitRegister();
        return Processing;
      case ScopeState::Reading16bitRegisters:
        scopeState = ProcessRead16bitRegister(ScopeState::Ready);
        return scopeState == ScopeState::Ready ? Finished : Processing;
      case ScopeState::ButtonSent:
        if (rsp.paramCode == ResponseMsgParamCodes::Accepted)
          // Button was accepted, decrement the retry.
          DecrementRetryAttempts();
        StartReading8bitRegisters();
        return Processing;
      case ScopeState::ReadRegisterSent:
        if (rsp.paramCode == ResponseMsgParamCodes::Accepted) {
          // @@TODO just dump the value onto the serial
        }
        return Finished;
      case ScopeState::WriteRegisterSent:
        if (rsp.paramCode == ResponseMsgParamCodes::Accepted) {
          // @@TODO do something? Retry if not accepted?
        }
        return Finished;
      default:
        return ProtocolError;
    }

    // The "return Finished" in this state machine requires a bit of explanation:
    // The Idle state either did nothing (still waiting for the heartbeat timeout)
    // or just successfully received the answer to Q0, whatever that was.
    // In both cases, it is ready to hand over work to a command or something else,
    // therefore we are returning Finished (also to exit mmu_loop() and unblock Marlin's loop!).
    // If there is no work, we'll end up in the Idle state again
    // and we'll send the heartbeat message after the specified timeout.
    return Finished;
  }

  ProtocolLogic::ProtocolLogic(uint8_t extraLoadDistance, uint8_t pulleySlowFeedrate)
    : explicitPrinterError(ErrorCode::OK)
    , currentScope(Scope::Stopped)
    , scopeState(ScopeState::Ready)
    , plannedRq(RequestMsgCodes::unknown, 0)
    , lastUARTActivityMs(0)
    , dataTO()
    , rsp(RequestMsg(RequestMsgCodes::unknown, 0), ResponseMsgParamCodes::unknown, 0)
    , state(State::Stopped)
    , lrb(0)
    , errorCode(ErrorCode::OK)
    , progressCode(ProgressCode::OK)
    , buttonCode(Buttons::NoButton)
    , lastFSensor((uint8_t)WhereIsFilament())
    , regIndex(0)
    , retryAttempts(MMU3_MAX_RETRIES)
    , inAutoRetry(false) {
    // @@TODO currently, I don't see a way of writing the initialization better :(
    // I'd like to write something like: initRegs8 { extraLoadDistance, pulleySlowFeedrate }
    // avr-gcc seems to like such a syntax, ARM gcc doesn't
    initRegs8[0] = extraLoadDistance;
    initRegs8[1] = pulleySlowFeedrate;
  }

  void ProtocolLogic::start() {
    state = State::InitSequence;
    currentScope = Scope::StartSeq;
    protocol.ResetResponseDecoder(); // important - finished delayed restart relies on this
    StartSeqRestart();
  }

  void ProtocolLogic::stop() {
    state = State::Stopped;
    currentScope = Scope::Stopped;
  }

  void ProtocolLogic::ToolChange(uint8_t slot) {
    PlanGenericRequest(RequestMsg(RequestMsgCodes::Tool, slot));
  }

  void ProtocolLogic::Statistics() {
    PlanGenericRequest(RequestMsg(RequestMsgCodes::Version, 3));
  }

  void ProtocolLogic::UnloadFilament() {
    PlanGenericRequest(RequestMsg(RequestMsgCodes::Unload, 0));
  }

  void ProtocolLogic::LoadFilament(uint8_t slot) {
    PlanGenericRequest(RequestMsg(RequestMsgCodes::Load, slot));
  }

  void ProtocolLogic::EjectFilament(uint8_t slot) {
    PlanGenericRequest(RequestMsg(RequestMsgCodes::Eject, slot));
  }

  void ProtocolLogic::CutFilament(uint8_t slot) {
    PlanGenericRequest(RequestMsg(RequestMsgCodes::Cut, slot));
  }

  void ProtocolLogic::ResetMMU(uint8_t mode /* = 0 */) {
    PlanGenericRequest(RequestMsg(RequestMsgCodes::Reset, mode));
  }

  void ProtocolLogic::button(uint8_t index) {
    PlanGenericRequest(RequestMsg(RequestMsgCodes::Button, index));
  }

  void ProtocolLogic::home(uint8_t mode) {
    PlanGenericRequest(RequestMsg(RequestMsgCodes::Home, mode));
  }

  void ProtocolLogic::readRegister(uint8_t address) {
    PlanGenericRequest(RequestMsg(RequestMsgCodes::Read, address));
  }

  void ProtocolLogic::writeRegister(uint8_t address, uint16_t data) {
    PlanGenericRequest(RequestMsg(RequestMsgCodes::Write, address, data));
  }

  void ProtocolLogic::PlanGenericRequest(RequestMsg rq) {
    plannedRq = rq;
    if (!ExpectsResponse())
      ActivatePlannedRequest();
    // otherwise wait for an empty window to activate the request
  }

  bool ProtocolLogic::ActivatePlannedRequest() {
    switch (plannedRq.code) {
      case RequestMsgCodes::Button:
        // only issue the button to the MMU and do not restart the state machines
        SendButton(plannedRq.value);
        plannedRq = RequestMsg(RequestMsgCodes::unknown, 0);
        return true;
      case RequestMsgCodes::Read:
        SendReadRegister(plannedRq.value, ScopeState::ReadRegisterSent);
        plannedRq = RequestMsg(RequestMsgCodes::unknown, 0);
        return true;
      case RequestMsgCodes::Write:
        SendWriteRegister(plannedRq.value, plannedRq.value2, ScopeState::WriteRegisterSent);
        plannedRq = RequestMsg(RequestMsgCodes::unknown, 0);
        return true;
      case RequestMsgCodes::unknown:
        return false;
      default: // commands
        currentScope = Scope::Command;
        SetRequestMsg(plannedRq);
        plannedRq = RequestMsg(RequestMsgCodes::unknown, 0);
        CommandRestart();
        return true;
    }
  }

  StepStatus ProtocolLogic::SwitchFromIdleToCommand() {
    currentScope = Scope::Command;
    SetRequestMsg(rsp.request);
    // we are recovering from a communication drop out, the command is already running
    // and we have just received a response to a Q0 message about a command progress
    return ProcessCommandQueryResponse();
  }

  void ProtocolLogic::SwitchToIdle() {
    state = State::Running;
    currentScope = Scope::Idle;
    IdleRestart();
  }

  void ProtocolLogic::SwitchFromStartToIdle() {
    state = State::Running;
    currentScope = Scope::Idle;
    IdleRestart();
    SendQuery(); // force sending Q0 immediately
  }

  bool ProtocolLogic::Elapsed(uint32_t timeout) const {
    return _millis() >= (lastUARTActivityMs + timeout);
  }

  void ProtocolLogic::RecordUARTActivity() {
    lastUARTActivityMs = _millis();
  }

  void ProtocolLogic::RecordReceivedByte(uint8_t c) {
    lastReceivedBytes[lrb] = c;
    lrb = (lrb + 1) % lastReceivedBytes.size();
  }

  constexpr char NibbleToChar(uint8_t c) {
    switch (c) {
      case 0x0 ... 0x9: return c + '0';
      case 0xA ... 0xF: return (c - 10) + 'a';
      default: return 0;
    }
  }

  void ProtocolLogic::FormatLastReceivedBytes(char *dst) {
    for (uint8_t i = 0; i < lastReceivedBytes.size(); ++i) {
      uint8_t b = lastReceivedBytes[(lrb - i - 1) % lastReceivedBytes.size()];
      dst[i * 3] = NibbleToChar(b >> 4);
      dst[i * 3 + 1] = NibbleToChar(b & 0xf);
      dst[i * 3 + 2] = ' ';
    }
    dst[(lastReceivedBytes.size() - 1) * 3 + 2] = 0; // terminate properly
  }

  void ProtocolLogic::FormatLastResponseMsgAndClearLRB(char *dst) {
    *dst++ = '<';
    for (uint8_t i = 0; i < lrb; ++i) {
      uint8_t b = lastReceivedBytes[i];
      // Check for printable character, including space
      if (b < 32 || b > 127)
        b = '.';
      *dst++ = b;
    }
    *dst = 0; // terminate properly
    lrb = 0; // reset the input buffer index in case of a clean message
  }

  void ProtocolLogic::LogRequestMsg(const uint8_t *txbuff, uint8_t size) {
    constexpr uint_fast8_t rqs = modules::protocol::Protocol::MaxRequestSize() + 1;
    char tmp[rqs] = ">";
    static char lastMsg[rqs] = "";
    for (uint8_t i = 0; i < size; ++i) {
      uint8_t b = txbuff[i];
      // Check for printable character, including space
      if (b < 32 || b > 127)
        b = '.';
      tmp[i + 1] = b;
    }
    tmp[size + 1] = 0;
    if (!strncmp_P(tmp, PSTR(">S0*c6."), rqs) && !strncmp(lastMsg, tmp, rqs)) {
      // @@TODO we skip the repeated request msgs for now
      // to avoid spoiling the whole log just with ">S0" messages
      // especially when the MMU is not connected.
      // We'll lose the ability to see if the printer is actually
      // trying to find the MMU, but since it has been reliable in the past
      // we can live without it for now.
    }
    else {
      MMU2_ECHO_MSGLN(tmp);
    }
    strncpy(lastMsg, tmp, rqs);
  }

  void ProtocolLogic::LogError(const char *reason_P) {
    char lrb[lastReceivedBytes.size() * 3];
    FormatLastReceivedBytes(lrb);

    MMU2_ERROR_MSGRPGM(reason_P);
    SERIAL_ECHOPGM(", last bytes: ");
    SERIAL_ECHOLN(lrb);
  }

  void ProtocolLogic::LogResponse() {
    char lrb[lastReceivedBytes.size()];
    FormatLastResponseMsgAndClearLRB(lrb);
    MMU2_ECHO_MSGLN(lrb);
  }

  StepStatus ProtocolLogic::SuppressShortDropOuts(const char *msg_P, StepStatus ss) {
    if (dataTO.Record(ss)) {
      LogError(msg_P);
      ResetCommunicationTimeoutAttempts(); // prepare for another run of consecutive retries before firing an error
      return dataTO.InitialCause();
    }
    else {
      return Processing; // suppress short drop outs of communication
    }
  }

  StepStatus ProtocolLogic::HandleCommunicationTimeout() {
    MMU_SERIAL.flush(); // clear the output buffer
    protocol.ResetResponseDecoder();
    start();
    return SuppressShortDropOuts(PSTR("Communication timeout"), CommunicationTimeout);
  }

  StepStatus ProtocolLogic::HandleProtocolError() {
    MMU_SERIAL.flush(); // clear the output buffer
    state = State::InitSequence;
    currentScope = Scope::DelayedRestart;
    DelayedRestartRestart();
    return SuppressShortDropOuts(PSTR("Protocol Error"), ProtocolError);
  }

  StepStatus ProtocolLogic::Step() {
    if (!ExpectsResponse()) // if not waiting for a response, activate a planned request immediately
      ActivatePlannedRequest();
    auto currentStatus = ScopeStep();
    switch (currentStatus) {
      case Processing:
        // we are ok, the state machine continues correctly
        break;
      case Finished: {
        // We are ok, switching to Idle if there is no potential next request planned.
        // But the trouble is we must report a finished command if the previous command has just been finished
        // i.e. only try to find some planned command if we just finished the Idle cycle
        if (!ActivatePlannedRequest()) {                           // if nothing is planned, switch to Idle
          SwitchToIdle();
        }
        else if (ExpectsResponse()) {
          // if the previous cycle was Idle and now we have planned a new command -> avoid returning Finished
          currentStatus = Processing;
        }
      }
      break;
      case CommandRejected:
        // we have to repeat it - that's the only thing we can do
        // no change in state
        // @@TODO wait until Q0 returns command in progress finished, then we can send this one
        LogError(PSTR("Command rejected"));
        CommandRestart();
        break;
      case CommandError:
        LogError(PSTR("Command Error"));
        // we should probably transfer into the Idle state and await further instructions from the upper layer
        // Idle state may solve the problem of keeping up the heart beat running
        break;
      case VersionMismatch:
        LogError(PSTR("Version mismatch"));
        break;
      case ProtocolError:
        currentStatus = HandleProtocolError();
        break;
      case CommunicationTimeout:
        currentStatus = HandleCommunicationTimeout();
        break;
      default:
        break;
    }
    // special handling of explicit printer errors
    return IsPrinterError() ? StepStatus::PrinterError : currentStatus;
  }

  uint8_t ProtocolLogic::CommandInProgress() const {
    if (currentScope != Scope::Command) return 0;
    return (uint8_t)ReqMsg().code;
  }

  void ProtocolLogic::DecrementRetryAttempts() {
    if (inAutoRetry && retryAttempts) {
      SERIAL_ECHOLNPGM("DecrementRetryAttempts");
      retryAttempts--;
    }
  }

  void ProtocolLogic::ResetRetryAttempts() {
    SERIAL_ECHOLNPGM("ResetRetryAttempts");
    retryAttempts = MMU3_MAX_RETRIES;
  }

  void __attribute__((noinline)) ProtocolLogic::ResetCommunicationTimeoutAttempts() {
    SERIAL_ECHOLNPGM("RSTCommTimeout");
    dataTO.reset();
  }

  bool DropOutFilter::Record(StepStatus ss) {
    if (occurrences == maxOccurrences) cause = ss;
    --occurrences;
    return occurrences == 0;
  }

} // MMU3

#endif // HAS_PRUSA_MMU3
