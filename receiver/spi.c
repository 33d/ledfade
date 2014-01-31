/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.

    $Id$
*/

#include "spi.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#define PORT_SPI    PORTD
#define DDR_SPI     DDRD
#define DD_MISO     DDD0
#define DD_MOSI     DDD1
#define DD_SCK      DDD4
// SS not available on the UART; the NRF24L01 needs it controlled manually anyway

void spi_init()
// Initialize pins for spi communication
{
    DDR_SPI &= ~((1<<DD_MOSI)|(1<<DD_MISO)|(1<<DD_SCK));
    // Define the following pins as output
    DDR_SPI |= ((1<<DD_MOSI)|(1<<DD_SCK));
    
    UCSR0B = ((0<<RXCIE0)|  // RX interrupt enable
            (0<<TXCIE0)|    // TX interrupt enable
            (0<<UDRIE0)|     // Data register empty interrupt enable
            (1<<RXEN0)|     // RX enable
            (1<<TXEN0));    // TX enable
    UCSR0C = ((1<<UMSEL01)|(1<<UMSEL00)| // UART in Master SPI mode
            (0<<UDORD0)|            // Data Order (0:MSB first / 1:LSB first)
            (0<<UCPOL0)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
            (0<<UCPHA0));             // Clock Phase (0:leading / 1:trailing edge sampling)

    SPSR = (1<<SPI2X);              // Double Clock Rate
    UBRR0 = (F_CPU / (2*2000000)) - 1; // 2Mbaud
    
}

void spi_transfer_sync (const uint8_t * dataout, uint8_t * datain, uint8_t len)
// Shift full array through target device
{
       uint8_t i;      
       for (i = 0; i < len; i++) {
             UDR0 = dataout[i];
             while((UCSR0A & (1<<RXC0))==0);
             datain[i] = UDR0;
       }
}

void spi_transmit_sync (const uint8_t * dataout, uint8_t len)
// Shift full array to target device without receiving any byte
{
       uint8_t i;      
       for (i = 0; i < len; i++) {
             UDR0 = dataout[i];
             while((UCSR0A & (1<<RXC0))==0);
       }
}

uint8_t spi_fast_shift (uint8_t data)
// Clocks only one byte to target device and returns the received one
{
    UDR0 = data;
    while((UCSR0A & (1<<RXC0))==0);
    return UDR0;
}

