#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "mirf.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define SERIAL_BAUD 115200
#define SERIAL_U2X 1
#include "serial.h"

typedef struct {
  unsigned int nrf_interrupt : 1;
} Events;
#define event(e) ((volatile Events*) &GPIOR0)->e
#define check_event(e, action) if (event(e)) { event(e) = 0; action; }
#define more_events (GPIOR0 != 0)

void mirf_handle_rx(uint8_t buf_read[mirf_PAYLOAD]) {
  printf("Read data ");
  for (const uint8_t* ch = buf_read; ch < buf_read + mirf_PAYLOAD; ch++)
    printf("%02X", *ch);
  putchar('\n');
}

ISR(INT0_vect) {
  event(nrf_interrupt) = 1;
}

int main(void) {

  serial_init();

  // nRF interrupt on INT0 aka D2 (INT1 is shared with a PWM output)
  DDRD &= ~_BV(PORTD2);
  EIMSK |= _BV(INT0); 

  // Initialize AVR for use with mirf
  mirf_init();
  // Wait for mirf to come up
  _delay_ms(50);
  // Configure mirf
  mirf_config();

  while(1) {
    if (!more_events) {
      sleep_enable();
      sei();
      sleep_cpu();
      sleep_disable();
      cli();
    }
    check_event(nrf_interrupt, mirf_handle_interrupt());
  }
}

