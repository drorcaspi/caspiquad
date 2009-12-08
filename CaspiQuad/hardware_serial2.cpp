/*
  HardwareSerial.cpp - Improved HardwareSerial library for Arduino
  Copyright (c) 2009 Kiril Zyapkov.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
*/

// Published in http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1242466935

#include <avr/io.h>
#include <avr/interrupt.h>

#include "hardware_serial2.h"

//=============================================================================
//
// Private Definitions
//
//=============================================================================

#define USART_RX_BUFFER_MASK (USART_RX_BUFFER_SIZE - 1)
#define USART_TX_BUFFER_MASK (USART_TX_BUFFER_SIZE - 1)

#if (USART_RX_BUFFER_SIZE & USART_RX_BUFFER_MASK)
#    error RX buffer size is not a power of 2
#endif
#if (USART_TX_BUFFER_SIZE & USART_TX_BUFFER_MASK)
#    error TX buffer size is not a power of 2
#endif

#ifndef BAUD_TOL
#   define BAUD_TOL 2
#endif


//=============================================================================
//
// Static Variables
//
//=============================================================================

// both buffers are static global vars in this file. interrupt handlers
// need to access them

static rx_ring_buffer rx_buffer;
static tx_ring_buffer tx_buffer;


//=============================================================================
//
// Public Functions
//
//=============================================================================

/**
 * Initialize the USART module with the BAUD rate predefined
 * in HardwareSerial.h
 *
 * This may implement passing addresses of registers, like the HardwareSerial
 * class from the Arduino library, but that's not necessary at this point.
 *
 */
HardwareSerial2::HardwareSerial2(rx_ring_buffer *rx_buffer_ptr,
                                 tx_ring_buffer *tx_buffer_ptr)
{
  _rx_buffer = rx_buffer_ptr;
  _tx_buffer = tx_buffer_ptr;
}

void HardwareSerial2::begin(unsigned long baud)
{
  uint8_t  use2x;
  uint16_t ubbr;


  // taken from <util/setbaud.h>
  
  use2x = 0;
  ubbr =  (F_CPU + 8UL * baud) / (16UL * baud) - 1UL;
  if ((100 * (F_CPU)) > (16 * (ubbr + 1) * (100 * ubbr + ubbr * BAUD_TOL)))
  {
    use2x = 1;
    ubbr = (F_CPU + 4UL * baud) / (8UL * baud) - 1UL;
  }

  UBRR0L = ubbr & 0xff;
  UBRR0H = ubbr >> 8;
  if (use2x)
    UCSR0A |= (1 << U2X0);
  else
    UCSR0A &= ~(1 << U2X0);

  // Flush buffers
  
  _tx_buffer->head = _tx_buffer->tail = 0;
  _rx_buffer->head = _rx_buffer->tail = 0;

  UCSR0B |= (1<<TXEN0);  // Enable Transmitter
  UCSR0B |= (1<<RXEN0);  // Enable Reciever
  UCSR0B |= (1<<RXCIE0); // Enable Rx Complete Interrupt
}

void HardwareSerial2::write(uint8_t data)
{
  uint8_t tmp_head;

  
  // Calculate new head position
  
  tmp_head = (_tx_buffer->head + (uint8_t)1) & (uint8_t)USART_TX_BUFFER_MASK;

  // Block until there's room in the buffer
  // XXX: this may block forever if someone externally disabled the transmitter
  //      or the DRE interrupt and there's data in the buffer. Careful!
  
  while (tmp_head == _tx_buffer->tail);

  // Advance the head, store the data
  
  _tx_buffer->buffer[tmp_head] = data;
  _tx_buffer->head = tmp_head;

  UCSR0B |= (1<<UDRIE0); // Enable Data Register Empty interrupt
}

void HardwareSerial2::flush(void)
{
  uint8_t oldSREG;
  
  // Not sure if disabling interrupts is needed here, but let's be on
  // the safe side
  // disable interrupts
  
  oldSREG = SREG;
  cli();
  _rx_buffer->head = _rx_buffer->tail;
  
  // Re-enable interrupts
  
  SREG = oldSREG;
}

/**
 * Returns the number of bytes in the RX buffer
 *
 */
uint8_t HardwareSerial2::available(void)
{
    return (_rx_buffer->head - _rx_buffer->tail) & (uint8_t)USART_RX_BUFFER_MASK;
}

/**
 * Returns a byte from the RX buffer, or NULL if none are available
 */
uint8_t HardwareSerial2::read(void)
{
  uint8_t tmp_tail;
  uint8_t tmp_data;


  // disable interrupts
  uint8_t oldSREG = SREG;
  
  cli();
  if (_rx_buffer->head == _rx_buffer->tail)
  {
    // Better that than block the code waiting for data. Users should call
    // available() first
    
    tmp_data = -1;
  }

  else
  {
    tmp_tail = (_rx_buffer->tail + (uint8_t)1) & (uint8_t)USART_RX_BUFFER_MASK;
    tmp_data = _rx_buffer->buffer[tmp_tail];
    _rx_buffer->tail = tmp_tail;
  }
  
  // Re-enable interrupts
  SREG = oldSREG;

  return tmp_data;
}

/**
 * Receive handler
 */
ISR(USART_RX_vect)
{
  uint8_t data;
  uint8_t tmp_head;

  
  data = UDR0;
  tmp_head = (rx_buffer.head + (uint8_t)1) & (uint8_t)USART_RX_BUFFER_MASK;

  if (tmp_head != rx_buffer.tail)
  {
    // No buffer overflow

    rx_buffer.buffer[tmp_head] = data;
    rx_buffer.head = tmp_head;
  }
}

/**
 * Data Register Empty Handler
 */
ISR(USART_UDRE_vect)
{
  if (tx_buffer.head == tx_buffer.tail)
  {
    // Buffer is empty, disable the interrupt

    UCSR0B &= ~(1 << UDRIE0);
  }

  else
  {
    tx_buffer.tail = (tx_buffer.tail + (uint8_t)1) & (uint8_t)USART_TX_BUFFER_MASK;
    UDR0 = tx_buffer.buffer[tx_buffer.tail];
  }
}

HardwareSerial2 Serial(&rx_buffer, &tx_buffer);
 

