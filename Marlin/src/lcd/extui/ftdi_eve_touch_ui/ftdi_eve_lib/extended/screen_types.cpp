/******************
 * screen_types.h *
 ******************/

/****************************************************************************
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#include "ftdi_extended.h"

#if ENABLED(FTDI_EXTENDED)

/********************** VIRTUAL DISPATCH DATA TYPE  ******************************/

uint8_t ScreenRef::lookupScreen(onRedraw_func_t onRedraw_ptr) {
  for (uint_fast8_t type = 0; type < tableSize(); type++) {
    if (GET_METHOD(type, onRedraw) == onRedraw_ptr) {
      return type;
    }
  }
  #if ENABLED(TOUCH_UI_DEBUG)
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM("Screen not found: ", (uintptr_t) onRedraw_ptr);
  #endif
  return 0xFF;
}

void ScreenRef::setScreen(onRedraw_func_t onRedraw_ptr) {
  uint8_t type = lookupScreen(onRedraw_ptr);
  if (type != 0xFF) {
    setType(type);
    #if ENABLED(TOUCH_UI_DEBUG)
      SERIAL_ECHO_MSG("New screen: ", type);
    #endif
  }
}

void ScreenRef::initializeAll() {
  for (uint_fast8_t type = 0; type < tableSize(); type++)
    GET_METHOD(type, onStartup)();
}

/********************** SCREEN STACK  ******************************/

void ScreenStack::start() {
  initializeAll();
  onEntry();
}

void ScreenStack::push(onRedraw_func_t onRedraw_ptr) {
  stack[3] = stack[2];
  stack[2] = stack[1];
  stack[1] = stack[0];
  stack[0] = lookupScreen(onRedraw_ptr);
}

void ScreenStack::push() {
  stack[3] = stack[2];
  stack[2] = stack[1];
  stack[1] = stack[0];
  stack[0] = getType();
}

void ScreenStack::pop() {
  setType(stack[0]);
  forget();
}

void ScreenStack::forget() {
  stack[0] = stack[1];
  stack[1] = stack[2];
  stack[2] = stack[3];
  stack[3] = 0;
}

void ScreenStack::goTo(onRedraw_func_t s) {
  push();
  onExit();
  setScreen(s);
  onEntry();
}

void ScreenStack::goBack() {
  onExit();
  pop();
  onEntry();
}

ScreenStack current_screen;

#endif // FTDI_EXTENDED
