/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief Wirish SPI implementation.
 */

#ifdef __STM32F1__

#include <SPI.h>

#include <libmaple/timer.h>
#include <libmaple/util.h>
#include <libmaple/rcc.h>

#include <boards.h>
#include <wirish.h>

#include "../../inc/MarlinConfig.h"
#include "spi_pins.h"

// Time in ms for DMA receive timeout
#define DMA_TIMEOUT 100

#if CYCLES_PER_MICROSECOND != 72
  #warning "Unexpected clock speed; SPI frequency calculation will be incorrect"
#endif

struct spi_pins { uint8_t nss, sck, miso, mosi; };

static const spi_pins* dev_to_spi_pins(spi_dev *dev);
static void configure_gpios(spi_dev *dev, bool as_master);
static spi_baud_rate determine_baud_rate(spi_dev *dev, uint32_t freq);

#if BOARD_NR_SPI >= 3 && !defined(STM32_HIGH_DENSITY)
  #error "The SPI library is misconfigured: 3 SPI ports only available on high density STM32 devices"
#endif

static const spi_pins board_spi_pins[] __FLASH__ = {
  #if BOARD_NR_SPI >= 1
    { BOARD_SPI1_NSS_PIN,
      BOARD_SPI1_SCK_PIN,
      BOARD_SPI1_MISO_PIN,
      BOARD_SPI1_MOSI_PIN },
  #endif
  #if BOARD_NR_SPI >= 2
    { BOARD_SPI2_NSS_PIN,
      BOARD_SPI2_SCK_PIN,
      BOARD_SPI2_MISO_PIN,
      BOARD_SPI2_MOSI_PIN },
  #endif
  #if BOARD_NR_SPI >= 3
    { BOARD_SPI3_NSS_PIN,
      BOARD_SPI3_SCK_PIN,
      BOARD_SPI3_MISO_PIN,
      BOARD_SPI3_MOSI_PIN },
  #endif
};

#if BOARD_NR_SPI >= 1
  static void *_spi1_this;
#endif
#if BOARD_NR_SPI >= 2
  static void *_spi2_this;
#endif
#if BOARD_NR_SPI >= 3
  static void *_spi3_this;
#endif

/**
 * @brief Wait until TXE (tx empty) flag is set and BSY (busy) flag unset.
 */
static inline void waitSpiTxEnd(spi_dev *spi_d) {
  while (spi_is_tx_empty(spi_d) == 0) { /* nada */ } // wait until TXE=1
  while (spi_is_busy(spi_d) != 0) { /* nada */ }     // wait until BSY=0
}

/**
 * Constructor
 */
SPIClass::SPIClass(uint32_t spi_num) {
  _currentSetting = &_settings[spi_num - 1];  // SPI channels are called 1 2 and 3 but the array is zero indexed

  switch (spi_num) {
    #if BOARD_NR_SPI >= 1
      case 1:
        _currentSetting->spi_d = SPI1;
        _spi1_this = (void*)this;
        break;
    #endif
    #if BOARD_NR_SPI >= 2
      case 2:
        _currentSetting->spi_d = SPI2;
        _spi2_this = (void*)this;
        break;
    #endif
    #if BOARD_NR_SPI >= 3
      case 3:
        _currentSetting->spi_d = SPI3;
        _spi3_this = (void*)this;
        break;
    #endif
    default: ASSERT(0);
  }

  // Init things specific to each SPI device
  // clock divider setup is a bit of hack, and needs to be improved at a later date.
  #if BOARD_NR_SPI >= 1
    _settings[0].spi_d = SPI1;
    _settings[0].clockDivider = determine_baud_rate(_settings[0].spi_d, _settings[0].clock);
    _settings[0].spiDmaDev = DMA1;
    _settings[0].spiTxDmaChannel = DMA_CH3;
    _settings[0].spiRxDmaChannel = DMA_CH2;
  #endif
  #if BOARD_NR_SPI >= 2
    _settings[1].spi_d = SPI2;
    _settings[1].clockDivider = determine_baud_rate(_settings[1].spi_d, _settings[1].clock);
    _settings[1].spiDmaDev = DMA1;
    _settings[1].spiTxDmaChannel = DMA_CH5;
    _settings[1].spiRxDmaChannel = DMA_CH4;
  #endif
  #if BOARD_NR_SPI >= 3
    _settings[2].spi_d = SPI3;
    _settings[2].clockDivider = determine_baud_rate(_settings[2].spi_d, _settings[2].clock);
    _settings[2].spiDmaDev = DMA2;
    _settings[2].spiTxDmaChannel = DMA_CH2;
    _settings[2].spiRxDmaChannel = DMA_CH1;
  #endif

  // Added for DMA callbacks.
  _currentSetting->state = SPI_STATE_IDLE;
}

SPIClass::SPIClass(int8_t mosi, int8_t miso, int8_t sclk, int8_t ssel) : SPIClass(1) {
  #if BOARD_NR_SPI >= 1
    if (mosi == BOARD_SPI1_MOSI_PIN) setModule(1);
  #endif
  #if BOARD_NR_SPI >= 2
    if (mosi == BOARD_SPI2_MOSI_PIN) setModule(2);
  #endif
  #if BOARD_NR_SPI >= 3
    if (mosi == BOARD_SPI3_MOSI_PIN) setModule(3);
  #endif
}

/**
 * Set up/tear down
 */
void SPIClass::updateSettings() {
  uint32_t flags = ((_currentSetting->bitOrder == MSBFIRST ? SPI_FRAME_MSB : SPI_FRAME_LSB) | _currentSetting->dataSize | SPI_SW_SLAVE | SPI_SOFT_SS);
  spi_master_enable(_currentSetting->spi_d, (spi_baud_rate)_currentSetting->clockDivider, (spi_mode)_currentSetting->dataMode, flags);
}

void SPIClass::begin() {
  spi_init(_currentSetting->spi_d);
  configure_gpios(_currentSetting->spi_d, 1);
  updateSettings();
  // Added for DMA callbacks.
  _currentSetting->state = SPI_STATE_READY;
}

void SPIClass::beginSlave() {
  spi_init(_currentSetting->spi_d);
  configure_gpios(_currentSetting->spi_d, 0);
  uint32_t flags = ((_currentSetting->bitOrder == MSBFIRST ? SPI_FRAME_MSB : SPI_FRAME_LSB) | _currentSetting->dataSize);
  spi_slave_enable(_currentSetting->spi_d, (spi_mode)_currentSetting->dataMode, flags);
  // Added for DMA callbacks.
  _currentSetting->state = SPI_STATE_READY;
}

void SPIClass::end() {
  if (!spi_is_enabled(_currentSetting->spi_d)) return;

  // Follows RM0008's sequence for disabling a SPI in master/slave
  // full duplex mode.
  while (spi_is_rx_nonempty(_currentSetting->spi_d)) {
    // FIXME [0.1.0] remove this once you have an interrupt based driver
    volatile uint16_t rx __attribute__((unused)) = spi_rx_reg(_currentSetting->spi_d);
  }
  waitSpiTxEnd(_currentSetting->spi_d);

  spi_peripheral_disable(_currentSetting->spi_d);
  // Added for DMA callbacks.
  // Need to add unsetting the callbacks for the DMA channels.
  _currentSetting->state = SPI_STATE_IDLE;
}

// Roger Clark added 3 functions
void SPIClass::setClockDivider(uint32_t clockDivider) {
  _currentSetting->clockDivider = clockDivider;
  uint32_t cr1 = _currentSetting->spi_d->regs->CR1 & ~(SPI_CR1_BR);
  _currentSetting->spi_d->regs->CR1 = cr1 | (clockDivider & SPI_CR1_BR);
}

void SPIClass::setBitOrder(BitOrder bitOrder) {
  _currentSetting->bitOrder = bitOrder;
  uint32_t cr1 = _currentSetting->spi_d->regs->CR1 & ~(SPI_CR1_LSBFIRST);
  if (bitOrder == LSBFIRST) cr1 |= SPI_CR1_LSBFIRST;
  _currentSetting->spi_d->regs->CR1 = cr1;
}

/**
 * Victor Perez. Added to test changing datasize from 8 to 16 bit modes on the fly.
 * Input parameter should be SPI_CR1_DFF set to 0 or 1 on a 32bit word.
 */
void SPIClass::setDataSize(uint32_t datasize) {
  _currentSetting->dataSize = datasize;
  uint32_t cr1 = _currentSetting->spi_d->regs->CR1 & ~(SPI_CR1_DFF);
  uint8_t en = spi_is_enabled(_currentSetting->spi_d);
  spi_peripheral_disable(_currentSetting->spi_d);
  _currentSetting->spi_d->regs->CR1 = cr1 | (datasize & SPI_CR1_DFF) | en;
}

void SPIClass::setDataMode(uint8_t dataMode) {
  /**
   * NOTES:
   * As far as we know the AVR numbers for dataMode match the numbers required by the STM32.
   * From the AVR doc https://www.atmel.com/images/doc2585.pdf section 2.4
   *
   * SPI Mode  CPOL  CPHA  Shift SCK-edge  Capture SCK-edge
   * 0       0     0     Falling     Rising
   * 1       0     1     Rising      Falling
   * 2       1     0     Rising      Falling
   * 3       1     1     Falling     Rising
   *
   * On the STM32 it appears to be
   *
   * bit 1 - CPOL : Clock polarity
   *   (This bit should not be changed when communication is ongoing)
   *   0 : CLK to 0 when idle
   *   1 : CLK to 1 when idle
   *
   * bit 0 - CPHA : Clock phase
   *   (This bit should not be changed when communication is ongoing)
   *   0 : The first clock transition is the first data capture edge
   *   1 : The second clock transition is the first data capture edge
   *
   * If someone finds this is not the case or sees a logic error with this let me know ;-)
   */
  _currentSetting->dataMode = dataMode;
  uint32_t cr1 = _currentSetting->spi_d->regs->CR1 & ~(SPI_CR1_CPOL|SPI_CR1_CPHA);
  _currentSetting->spi_d->regs->CR1 = cr1 | (dataMode & (SPI_CR1_CPOL|SPI_CR1_CPHA));
}

void SPIClass::beginTransaction(uint8_t pin, const SPISettings &settings) {
  setBitOrder(settings.bitOrder);
  setDataMode(settings.dataMode);
  setDataSize(settings.dataSize);
  setClockDivider(determine_baud_rate(_currentSetting->spi_d, settings.clock));
  begin();
}

void SPIClass::beginTransactionSlave(const SPISettings &settings) {
  setBitOrder(settings.bitOrder);
  setDataMode(settings.dataMode);
  setDataSize(settings.dataSize);
  beginSlave();
}

void SPIClass::endTransaction() { }

/**
 * I/O
 */

uint16_t SPIClass::read() {
  while (!spi_is_rx_nonempty(_currentSetting->spi_d)) { /* nada */ }
  return (uint16_t)spi_rx_reg(_currentSetting->spi_d);
}

void SPIClass::read(uint8_t *buf, uint32_t len) {
  if (len == 0) return;
  spi_rx_reg(_currentSetting->spi_d);   // clear the RX buffer in case a byte is waiting on it.
  spi_reg_map * regs = _currentSetting->spi_d->regs;
  // start sequence: write byte 0
  regs->DR = 0x00FF; // write the first byte
  // main loop
  while (--len) {
    while (!(regs->SR & SPI_SR_TXE)) { /* nada */ } // wait for TXE flag
    noInterrupts();    // go atomic level - avoid interrupts to surely get the previously received data
    regs->DR = 0x00FF; // write the next data item to be transmitted into the SPI_DR register. This clears the TXE flag.
    while (!(regs->SR & SPI_SR_RXNE)) { /* nada */ } // wait till data is available in the DR register
    *buf++ = (uint8)(regs->DR); // read and store the received byte. This clears the RXNE flag.
    interrupts();      // let systick do its job
  }
  // read remaining last byte
  while (!(regs->SR & SPI_SR_RXNE)) { /* nada */ } // wait till data is available in the Rx register
  *buf++ = (uint8)(regs->DR);  // read and store the received byte
}

void SPIClass::write(uint16_t data) {
  /**
   * Added for 16bit data Victor Perez. Roger Clark
   * Improved speed by just directly writing the single byte to the SPI data reg and wait for completion,
   * by taking the Tx code from transfer(byte)
   * This almost doubles the speed of this function.
   */
  spi_tx_reg(_currentSetting->spi_d, data); // write the data to be transmitted into the SPI_DR register (this clears the TXE flag)
  waitSpiTxEnd(_currentSetting->spi_d);
}

void SPIClass::write16(uint16_t data) {
  // Added by stevestrong: write two consecutive bytes in 8 bit mode (DFF=0)
  spi_tx_reg(_currentSetting->spi_d, data>>8); // write high byte
  while (!spi_is_tx_empty(_currentSetting->spi_d)) { /* nada */ } // Wait until TXE=1
  spi_tx_reg(_currentSetting->spi_d, data); // write low byte
  waitSpiTxEnd(_currentSetting->spi_d);
}

void SPIClass::write(uint16_t data, uint32_t n) {
  // Added by stevstrong: Repeatedly send same data by the specified number of times
  spi_reg_map * regs = _currentSetting->spi_d->regs;
  while (n--) {
    regs->DR = data; // write the data to be transmitted into the SPI_DR register (this clears the TXE flag)
    while (!(regs->SR & SPI_SR_TXE)) { /* nada */ } // wait till Tx empty
  }
  while (regs->SR & SPI_SR_BSY) { /* nada */ } // wait until BSY=0 before returning
}

void SPIClass::write(const void *data, uint32_t length) {
  spi_dev * spi_d = _currentSetting->spi_d;
  spi_tx(spi_d, data, length); // data can be array of bytes or words
  waitSpiTxEnd(spi_d);
}

uint8_t SPIClass::transfer(uint8_t byte) const {
  spi_dev * spi_d = _currentSetting->spi_d;
  spi_rx_reg(spi_d); // read any previous data
  spi_tx_reg(spi_d, byte); // Write the data item to be transmitted into the SPI_DR register
  waitSpiTxEnd(spi_d);
  return (uint8)spi_rx_reg(spi_d); // "... and read the last received data."
}

uint16_t SPIClass::transfer16(uint16_t data) const {
  // Modified by stevestrong: write & read two consecutive bytes in 8 bit mode (DFF=0)
  // This is more effective than two distinct byte transfers
  spi_dev * spi_d = _currentSetting->spi_d;
  spi_rx_reg(spi_d);                              // read any previous data
  spi_tx_reg(spi_d, data>>8);                     // write high byte
  waitSpiTxEnd(spi_d);                            // wait until TXE=1 and then wait until BSY=0
  uint16_t ret = spi_rx_reg(spi_d)<<8;            // read and shift high byte
  spi_tx_reg(spi_d, data);                        // write low byte
  waitSpiTxEnd(spi_d);                            // wait until TXE=1 and then wait until BSY=0
  ret += spi_rx_reg(spi_d);                       // read low byte
  return ret;
}

/**
 * Roger Clark and Victor Perez, 2015
 * Performs a DMA SPI transfer with at least a receive buffer.
 * If a TX buffer is not provided, FF is sent over and over for the length of the transfer.
 * On exit TX buffer is not modified, and RX buffer contains the received data.
 * Still in progress.
 */
void SPIClass::dmaTransferSet(const void *transmitBuf, void *receiveBuf) {
  dma_init(_currentSetting->spiDmaDev);
  //spi_rx_dma_enable(_currentSetting->spi_d);
  //spi_tx_dma_enable(_currentSetting->spi_d);
  dma_xfer_size dma_bit_size = (_currentSetting->dataSize==DATA_SIZE_16BIT) ? DMA_SIZE_16BITS : DMA_SIZE_8BITS;
  dma_setup_transfer(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel, &_currentSetting->spi_d->regs->DR,
      dma_bit_size, receiveBuf, dma_bit_size, (DMA_MINC_MODE | DMA_TRNS_CMPLT ));// receive buffer DMA
  if (!transmitBuf) {
    transmitBuf = &ff;
    dma_setup_transfer(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, &_currentSetting->spi_d->regs->DR,
        dma_bit_size, (volatile void*)transmitBuf, dma_bit_size, (DMA_FROM_MEM));// Transmit FF repeatedly
  }
  else {
    dma_setup_transfer(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, &_currentSetting->spi_d->regs->DR,
        dma_bit_size, (volatile void*)transmitBuf, dma_bit_size, (DMA_MINC_MODE |  DMA_FROM_MEM ));// Transmit buffer DMA
  }
  dma_set_priority(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, DMA_PRIORITY_LOW);
  dma_set_priority(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel, DMA_PRIORITY_VERY_HIGH);
}

uint8_t SPIClass::dmaTransferRepeat(uint16_t length) {
  if (length == 0) return 0;
  if (spi_is_rx_nonempty(_currentSetting->spi_d) == 1) spi_rx_reg(_currentSetting->spi_d);
  _currentSetting->state = SPI_STATE_TRANSFER;
  dma_set_num_transfers(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel, length);
  dma_set_num_transfers(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, length);
  dma_enable(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel);// enable receive
  dma_enable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);// enable transmit
  spi_rx_dma_enable(_currentSetting->spi_d);
  spi_tx_dma_enable(_currentSetting->spi_d);
  if (_currentSetting->receiveCallback)
    return 0;

  //uint32_t m = millis();
  uint8_t b = 0;
  uint32_t m = millis();
  while (!(dma_get_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel) & DMA_ISR_TCIF1)) {
    // Avoid interrupts and just loop waiting for the flag to be set.
    if ((millis() - m) > DMA_TIMEOUT) { b = 2; break; }
  }

  waitSpiTxEnd(_currentSetting->spi_d); // until TXE=1 and BSY=0
  spi_tx_dma_disable(_currentSetting->spi_d);
  spi_rx_dma_disable(_currentSetting->spi_d);
  dma_disable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
  dma_disable(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel);
  dma_clear_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel);
  dma_clear_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
  _currentSetting->state = SPI_STATE_READY;
  return b;
}

/**
 * Roger Clark and Victor Perez, 2015
 * Performs a DMA SPI transfer with at least a receive buffer.
 * If a TX buffer is not provided, FF is sent over and over for the length of the transfer.
 * On exit TX buffer is not modified, and RX buffer contains the received data.
 * Still in progress.
 */
uint8_t SPIClass::dmaTransfer(const void *transmitBuf, void *receiveBuf, uint16_t length) {
  dmaTransferSet(transmitBuf, receiveBuf);
  return dmaTransferRepeat(length);
}

/**
 * Roger Clark and Victor Perez, 2015
 * Performs a DMA SPI send using a TX buffer.
 * On exit TX buffer is not modified.
 * Still in progress.
 * 2016 - stevstrong - reworked to automatically detect bit size from SPI setting
 */
void SPIClass::dmaSendSet(const void * transmitBuf, bool minc) {
  uint32_t flags = ( (DMA_MINC_MODE*minc) | DMA_FROM_MEM | DMA_TRNS_CMPLT);
  dma_init(_currentSetting->spiDmaDev);
  dma_xfer_size dma_bit_size = (_currentSetting->dataSize==DATA_SIZE_16BIT) ? DMA_SIZE_16BITS : DMA_SIZE_8BITS;
  dma_setup_transfer(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, &_currentSetting->spi_d->regs->DR, dma_bit_size,
             (volatile void*)transmitBuf, dma_bit_size, flags);// Transmit buffer DMA
  dma_set_priority(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, DMA_PRIORITY_LOW);
}

uint8_t SPIClass::dmaSendRepeat(uint16_t length) {
  if (length == 0) return 0;

  dma_clear_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
  dma_set_num_transfers(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, length);
  _currentSetting->state = SPI_STATE_TRANSMIT;
  dma_enable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);   // enable transmit
  spi_tx_dma_enable(_currentSetting->spi_d);
  if (_currentSetting->transmitCallback) return 0;

  uint32_t m = millis();
  uint8_t b = 0;
  while (!(dma_get_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel) & DMA_ISR_TCIF1)) {
    // Avoid interrupts and just loop waiting for the flag to be set.
    if ((millis() - m) > DMA_TIMEOUT) { b = 2; break; }
  }
  waitSpiTxEnd(_currentSetting->spi_d); // until TXE=1 and BSY=0
  spi_tx_dma_disable(_currentSetting->spi_d);
  dma_disable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
  dma_clear_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
  _currentSetting->state = SPI_STATE_READY;
  return b;
}

uint8_t SPIClass::dmaSend(const void * transmitBuf, uint16_t length, bool minc) {
  dmaSendSet(transmitBuf, minc);
  return dmaSendRepeat(length);
}

uint8_t SPIClass::dmaSendAsync(const void * transmitBuf, uint16_t length, bool minc) {
  uint8_t b = 0;

  if (_currentSetting->state != SPI_STATE_READY) {
    uint32_t m = millis();
    while (!(dma_get_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel) & DMA_ISR_TCIF1)) {
      //Avoid interrupts and just loop waiting for the flag to be set.
      //delayMicroseconds(10);
      if ((millis() - m) > DMA_TIMEOUT) { b = 2; break; }
    }
    waitSpiTxEnd(_currentSetting->spi_d); // until TXE=1 and BSY=0
    spi_tx_dma_disable(_currentSetting->spi_d);
    dma_disable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
    _currentSetting->state = SPI_STATE_READY;
  }

  if (length == 0) return 0;
  uint32_t flags = ( (DMA_MINC_MODE*minc) | DMA_FROM_MEM | DMA_TRNS_CMPLT);

  dma_init(_currentSetting->spiDmaDev);
  // TX
  dma_xfer_size dma_bit_size = (_currentSetting->dataSize==DATA_SIZE_16BIT) ? DMA_SIZE_16BITS : DMA_SIZE_8BITS;
  dma_setup_transfer(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, &_currentSetting->spi_d->regs->DR,
      dma_bit_size, (volatile void*)transmitBuf, dma_bit_size, flags);// Transmit buffer DMA
  dma_set_num_transfers(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, length);
  dma_clear_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
  dma_enable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);// enable transmit
  spi_tx_dma_enable(_currentSetting->spi_d);

  _currentSetting->state = SPI_STATE_TRANSMIT;
  return b;
}

/**
 *  New functions added to manage callbacks.
 *  Victor Perez 2017
 */
void SPIClass::onReceive(void(*callback)()) {
  _currentSetting->receiveCallback = callback;
  if (callback) {
    switch (_currentSetting->spi_d->clk_id) {
      #if BOARD_NR_SPI >= 1
        case RCC_SPI1:
          dma_attach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel, &SPIClass::_spi1EventCallback);
          break;
      #endif
      #if BOARD_NR_SPI >= 2
        case RCC_SPI2:
          dma_attach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel, &SPIClass::_spi2EventCallback);
          break;
      #endif
      #if BOARD_NR_SPI >= 3
        case RCC_SPI3:
          dma_attach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel, &SPIClass::_spi3EventCallback);
          break;
      #endif
      default: ASSERT(0);
    }
  }
  else {
    dma_detach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel);
  }
}

void SPIClass::onTransmit(void(*callback)()) {
  _currentSetting->transmitCallback = callback;
  if (callback) {
    switch (_currentSetting->spi_d->clk_id) {
      #if BOARD_NR_SPI >= 1
        case RCC_SPI1:
          dma_attach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, &SPIClass::_spi1EventCallback);
          break;
      #endif
      #if BOARD_NR_SPI >= 2
         case RCC_SPI2:
          dma_attach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, &SPIClass::_spi2EventCallback);
          break;
      #endif
      #if BOARD_NR_SPI >= 3
        case RCC_SPI3:
          dma_attach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, &SPIClass::_spi3EventCallback);
          break;
      #endif
      default: ASSERT(0);
    }
  }
  else {
    dma_detach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
  }
}

/**
 * TODO: check if better to first call the customer code, next disable the DMA requests.
 * Also see if we need to check whether callbacks are set or not, may be better to be checked
 * during the initial setup and only set the callback to EventCallback if they are set.
 */
void SPIClass::EventCallback() {
  waitSpiTxEnd(_currentSetting->spi_d);
  switch (_currentSetting->state) {
  case SPI_STATE_TRANSFER:
    while (spi_is_rx_nonempty(_currentSetting->spi_d)) { /* nada */ }
    _currentSetting->state = SPI_STATE_READY;
    spi_tx_dma_disable(_currentSetting->spi_d);
    spi_rx_dma_disable(_currentSetting->spi_d);
    //dma_disable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
    //dma_disable(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel);
    if (_currentSetting->receiveCallback)
      _currentSetting->receiveCallback();
    break;
  case SPI_STATE_TRANSMIT:
    _currentSetting->state = SPI_STATE_READY;
    spi_tx_dma_disable(_currentSetting->spi_d);
    //dma_disable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
    if (_currentSetting->transmitCallback)
      _currentSetting->transmitCallback();
    break;
  default:
    break;
  }
}

void SPIClass::attachInterrupt() {
  // Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
  // Should be disableInterrupt()
}

/**
 * Pin accessors
 */

uint8_t SPIClass::misoPin() {
  return dev_to_spi_pins(_currentSetting->spi_d)->miso;
}

uint8_t SPIClass::mosiPin() {
  return dev_to_spi_pins(_currentSetting->spi_d)->mosi;
}

uint8_t SPIClass::sckPin() {
  return dev_to_spi_pins(_currentSetting->spi_d)->sck;
}

uint8_t SPIClass::nssPin() {
  return dev_to_spi_pins(_currentSetting->spi_d)->nss;
}

/**
 * Deprecated functions
 */
uint8_t SPIClass::send(uint8_t data)               { write(data); return 1; }
uint8_t SPIClass::send(uint8_t *buf, uint32_t len) { write(buf, len); return len; }
uint8_t SPIClass::recv()                           { return read(); }

/**
 * DMA call back functions, one per port.
 */
#if BOARD_NR_SPI >= 1
  void SPIClass::_spi1EventCallback() {
    reinterpret_cast<class SPIClass*>(_spi1_this)->EventCallback();
  }
#endif
#if BOARD_NR_SPI >= 2
  void SPIClass::_spi2EventCallback() {
    reinterpret_cast<class SPIClass*>(_spi2_this)->EventCallback();
  }
#endif
#if BOARD_NR_SPI >= 3
  void SPIClass::_spi3EventCallback() {
    reinterpret_cast<class SPIClass*>(_spi3_this)->EventCallback();
  }
#endif

/**
 * Auxiliary functions
 */
static const spi_pins* dev_to_spi_pins(spi_dev *dev) {
  switch (dev->clk_id) {
    #if BOARD_NR_SPI >= 1
      case RCC_SPI1: return board_spi_pins;
    #endif
    #if BOARD_NR_SPI >= 2
      case RCC_SPI2: return board_spi_pins + 1;
    #endif
    #if BOARD_NR_SPI >= 3
      case RCC_SPI3: return board_spi_pins + 2;
    #endif
    default: return nullptr;
  }
}

static void disable_pwm(const stm32_pin_info *i) {
  if (i->timer_device)
    timer_set_mode(i->timer_device, i->timer_channel, TIMER_DISABLED);
}

static void configure_gpios(spi_dev *dev, bool as_master) {
  const spi_pins *pins = dev_to_spi_pins(dev);
  if (!pins) return;

  const stm32_pin_info *nssi = &PIN_MAP[pins->nss],
                       *scki = &PIN_MAP[pins->sck],
                       *misoi = &PIN_MAP[pins->miso],
                       *mosii = &PIN_MAP[pins->mosi];

  disable_pwm(nssi);
  disable_pwm(scki);
  disable_pwm(misoi);
  disable_pwm(mosii);

  spi_config_gpios(dev, as_master, nssi->gpio_device, nssi->gpio_bit,
  scki->gpio_device, scki->gpio_bit, misoi->gpio_bit,
  mosii->gpio_bit);
}

static const spi_baud_rate baud_rates[8] __FLASH__ = {
  SPI_BAUD_PCLK_DIV_2,
  SPI_BAUD_PCLK_DIV_4,
  SPI_BAUD_PCLK_DIV_8,
  SPI_BAUD_PCLK_DIV_16,
  SPI_BAUD_PCLK_DIV_32,
  SPI_BAUD_PCLK_DIV_64,
  SPI_BAUD_PCLK_DIV_128,
  SPI_BAUD_PCLK_DIV_256,
};

/**
 * NOTE: This assumes you're on a LeafLabs-style board
 * (CYCLES_PER_MICROSECOND == 72, APB2 at 72MHz, APB1 at 36MHz).
 */
static spi_baud_rate determine_baud_rate(spi_dev *dev, uint32_t freq) {
  uint32_t clock = 0;
  switch (rcc_dev_clk(dev->clk_id)) {
    case RCC_AHB:
    case RCC_APB2: clock = STM32_PCLK2; break; // 72 Mhz
    case RCC_APB1: clock = STM32_PCLK1; break; // 36 Mhz
  }
  clock >>= 1;

  uint8_t i = 0;
  while (i < 7 && freq < clock) { clock >>= 1; i++; }
  return baud_rates[i];
}

SPIClass SPI(SPI_DEVICE);

#endif // __STM32F1__
