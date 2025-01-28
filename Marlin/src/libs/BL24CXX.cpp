/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../inc/MarlinConfig.h"

#if ENABLED(IIC_BL24CXX_EEPROM)

/**
 * PersistentStore for Arduino-style EEPROM interface
 * with simple implementations supplied by Marlin.
 */

#include "BL24CXX.h"
#ifdef __STM32F1__
  #include <libmaple/gpio.h>
#else
  #include "../HAL/shared/Delay.h"
  #define delay_us(n) DELAY_US(n)
#endif

#ifndef EEPROM_WRITE_DELAY
  #define EEPROM_WRITE_DELAY    10
#endif
#ifndef EEPROM_DEVICE_ADDRESS
  #define EEPROM_DEVICE_ADDRESS (0x50 << 1)
#endif

// IO direction setting
#ifdef __STM32F1__
  #define SDA_IN()  do{ PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH &= 0XFFFF0FFF; PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH |= 8 << 12; }while(0)
  #define SDA_OUT() do{ PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH &= 0XFFFF0FFF; PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH |= 3 << 12; }while(0)
#elif defined(STM32F1) || defined(STM32F4) || defined(ARDUINO_ARCH_HC32)
  #define SDA_IN()  SET_INPUT(IIC_EEPROM_SDA)
  #define SDA_OUT() SET_OUTPUT(IIC_EEPROM_SDA)
#endif

// IO ops
#define IIC_SCL_0() WRITE(IIC_EEPROM_SCL, LOW)
#define IIC_SCL_1() WRITE(IIC_EEPROM_SCL, HIGH)
#define IIC_SDA_0() WRITE(IIC_EEPROM_SDA, LOW)
#define IIC_SDA_1() WRITE(IIC_EEPROM_SDA, HIGH)
#define READ_SDA()  READ(IIC_EEPROM_SDA)

//
// Simple IIC interface via libmaple
//

// Initialize IIC
void IIC::init() {
  SET_OUTPUT(IIC_EEPROM_SDA);
  SET_OUTPUT(IIC_EEPROM_SCL);
  IIC_SCL_1();
  IIC_SDA_1();
}

// Generate IIC start signal
void IIC::start() {
  SDA_OUT();    // SDA line output
  IIC_SDA_1();
  IIC_SCL_1();
  delay_us(4);
  IIC_SDA_0();  // START:when CLK is high, DATA change from high to low
  delay_us(4);
  IIC_SCL_0();  // Clamp the I2C bus, ready to send or receive data
}

// Generate IIC stop signal
void IIC::stop() {
  SDA_OUT();    // SDA line output
  IIC_SCL_0();
  IIC_SDA_0();  // STOP:when CLK is high DATA change from low to high
  delay_us(4);
  IIC_SCL_1();
  IIC_SDA_1();  // Send I2C bus end signal
  delay_us(4);
}

// Wait for the response signal to arrive
// 1 = failed to receive response
// 0 = response received
uint8_t IIC::wait_ack() {
  uint8_t ucErrTime = 0;
  SDA_IN();      // SDA is set as input
  IIC_SDA_1(); delay_us(1);
  IIC_SCL_1(); delay_us(1);
  while (READ_SDA()) {
    if (++ucErrTime > 250) {
      stop();
      return 1;
    }
  }
  IIC_SCL_0(); // Clock output 0
  return 0;
}

// Generate ACK response
void IIC::ack() {
  IIC_SCL_0();
  SDA_OUT();
  IIC_SDA_0();
  delay_us(2);
  IIC_SCL_1();
  delay_us(2);
  IIC_SCL_0();
}

// No ACK response
void IIC::nAck() {
  IIC_SCL_0();
  SDA_OUT();
  IIC_SDA_1();
  delay_us(2);
  IIC_SCL_1();
  delay_us(2);
  IIC_SCL_0();
}

// Send one IIC byte
// Return whether the slave responds
// 1 = there is a response
// 0 = no response
void IIC::send_byte(uint8_t txd) {
  SDA_OUT();
  IIC_SCL_0(); // Pull down the clock to start data transmission
  for (uint_fast8_t t = 0; t < 8; ++t) {
    // IIC_SDA = (txd & 0x80) >> 7;
    if (txd & 0x80) IIC_SDA_1(); else IIC_SDA_0();
    txd <<= 1;
    delay_us(2);   // All three delays are necessary for TEA5767
    IIC_SCL_1();
    delay_us(2);
    IIC_SCL_0();
    delay_us(2);
  }
}

// Read 1 byte, when ack=1, send ACK, ack=0, send nACK
uint8_t IIC::read_byte(unsigned char ack_chr) {
  unsigned char receive = 0;
  SDA_IN(); // SDA is set as input
  for (uint_fast8_t i = 0; i < 8; ++i) {
    IIC_SCL_0();
    delay_us(2);
    IIC_SCL_1();
    receive <<= 1;
    if (READ_SDA()) receive++;
    delay_us(1);
  }
  ack_chr ? ack() : nAck(); // Send ACK / send nACK
  return receive;
}

/******************** EEPROM ********************/

// Initialize the IIC interface
void BL24CXX::init() { IIC::init(); }

// Read a byte at the specified address
// ReadAddr: the address to start reading
// Return: the byte read
uint8_t BL24CXX::readOneByte(uint16_t ReadAddr) {
  uint8_t temp = 0;
  IIC::start();
  if (EE_TYPE > BL24C16) {
    IIC::send_byte(EEPROM_DEVICE_ADDRESS);        // Send write command
    IIC::wait_ack();
    IIC::send_byte(ReadAddr >> 8);                // Send high address
    IIC::wait_ack();
  }
  else
    IIC::send_byte(EEPROM_DEVICE_ADDRESS + ((ReadAddr >> 8) << 1)); // Send device address 0xA0, write data

  IIC::wait_ack();
  IIC::send_byte(ReadAddr & 0xFF);                // Send low address
  IIC::wait_ack();
  IIC::start();
  IIC::send_byte(EEPROM_DEVICE_ADDRESS | 0x01);   // Send byte
  IIC::wait_ack();
  temp = IIC::read_byte(0);
  IIC::stop();                                    // Generate a stop condition
  return temp;
}

// Write a data at the address specified by BL24CXX
// WriteAddr: The destination address for writing data
// DataToWrite: the data to be written
void BL24CXX::writeOneByte(uint16_t WriteAddr, uint8_t DataToWrite) {
  IIC::start();
  if (EE_TYPE > BL24C16) {
    IIC::send_byte(EEPROM_DEVICE_ADDRESS);        // Send write command
    IIC::wait_ack();
    IIC::send_byte(WriteAddr >> 8);               // Send high address
  }
  else
    IIC::send_byte(EEPROM_DEVICE_ADDRESS + ((WriteAddr >> 8) << 1)); // Send device address 0xA0, write data

  IIC::wait_ack();
  IIC::send_byte(WriteAddr & 0xFF);               // Send low address
  IIC::wait_ack();
  IIC::send_byte(DataToWrite);                    // Receiving mode
  IIC::wait_ack();
  IIC::stop();                                    // Generate a stop condition
  delay(10);
}

// Start writing data of length Len at the specified address in BL24CXX
// This function is used to write 16bit or 32bit data.
// WriteAddr: the address to start writing
// DataToWrite: the first address of the data array
// Len: The length of the data to be written 2, 4
void BL24CXX::writeLenByte(uint16_t WriteAddr, uint32_t DataToWrite, uint8_t Len) {
  for (uint_fast8_t t = 0; t < Len; ++t)
    writeOneByte(WriteAddr + t, (DataToWrite >> (8 * t)) & 0xFF);
}

// Start reading data of length Len from the specified address in BL24CXX
// This function is used to read 16bit or 32bit data.
// ReadAddr: the address to start reading
// Return value: data
// Len: The length of the data to be read 2,4
uint32_t BL24CXX::readLenByte(uint16_t ReadAddr, uint8_t Len) {
  uint32_t temp = 0;
  for (uint_fast8_t t = 0; t < Len; ++t) {
    temp <<= 8;
    temp += readOneByte(ReadAddr + Len - t - 1);
  }
  return temp;
}

// Check if BL24CXX is normal
// Return 1: Detection failed
// return 0: detection is successful
#define BL24CXX_TEST_ADDRESS 0x00
#define BL24CXX_TEST_VALUE   0x55

bool BL24CXX::_check() {
  return (readOneByte(BL24CXX_TEST_ADDRESS) != BL24CXX_TEST_VALUE); // false = success!
}

bool BL24CXX::check() {
  if (_check()) {                                           // Value was written? Good EEPROM!
    writeOneByte(BL24CXX_TEST_ADDRESS, BL24CXX_TEST_VALUE); // Write now and check.
    return _check();
  }
  return false; // success!
}

// Start reading the specified number of data at the specified address in BL24CXX
// ReadAddr: The address to start reading is 0~255 for 24c02
// pBuffer: the first address of the data array
// NumToRead: the number of data to be read
void BL24CXX::read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead) {
  for (; NumToRead; NumToRead--)
    *pBuffer++ = readOneByte(ReadAddr++);
}

// Start writing the specified number of data at the specified address in BL24CXX
// WriteAddr: the address to start writing, 0~255 for 24c02
// pBuffer: the first address of the data array
// NumToWrite: the number of data to be written
void BL24CXX::write(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite) {
  for (; NumToWrite; NumToWrite--, WriteAddr++)
    writeOneByte(WriteAddr, *pBuffer++);
}

#endif // IIC_BL24CXX_EEPROM
