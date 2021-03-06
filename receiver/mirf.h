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

#ifndef _MIRF_H_
#define _MIRF_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include <avr/io.h>

// Mirf settings
#define mirf_CH         2
#define mirf_PAYLOAD    6
// W_TX_PAYLOAD or W_TX_PAYLOAD_NO_ACK
#define mirf_TX_CMD     W_TX_PAYLOAD_NO_ACK
#define mirf_CONFIG     ( (1<<EN_CRC) | (0<<CRCO) )

// Pin definitions for chip select and chip enabled of the MiRF module
#define CE  PB0
#define CSN PC2

// Definitions for selecting and enabling MiRF module
#define mirf_CSN_hi     PORTC |=  (1<<CSN);
#define mirf_CSN_lo     PORTC &= ~(1<<CSN);
#define mirf_CE_hi      PORTB |=  (1<<CE);
#define mirf_CE_lo      PORTB &= ~(1<<CE);

// Public standart functions
extern void mirf_init();
extern void mirf_config();
extern void mirf_poweroff();
extern void mirf_send(const uint8_t * value, uint8_t len);
extern void mirf_set_RADDR(uint8_t * adr);
extern void mirf_set_TADDR(uint8_t * adr);
extern uint8_t mirf_data_ready();
extern void mirf_get_data(uint8_t * data);
extern void mirf_handle_interrupt();
// implement this one yourself
extern void mirf_handle_rx(uint8_t data[mirf_PAYLOAD]);

// Public extended functions
extern void mirf_config_register(uint8_t reg, uint8_t value);
extern void mirf_read_register(uint8_t reg, uint8_t * value, uint8_t len);
extern void mirf_write_register(uint8_t reg, uint8_t * value, uint8_t len);

#if defined(__cplusplus)
} // extern
#endif

#endif /* _MIRF_H_ */
