#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "mirf.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define SERIAL_BAUD 115200
#define SERIAL_U2X 1
#include "serial.h"

int main(void) {
  const uint8_t buf_write[] = { 0x01, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0e, 0x10 };
  uint8_t buf_read[mirf_PAYLOAD];

  serial_init();

  // Initialize AVR for use with mirf
  mirf_init();
  // Wait for mirf to come up
  _delay_ms(50);
  // Activate interrupts
  sei();
  // Configure mirf
  mirf_config();

  while(1) {
    // Test communication
    printf("Sending ");
    mirf_send(buf_write, sizeof(buf_write));
#if 0
    printf("Data ready: %d ", mirf_data_ready());
    mirf_get_data(buf_read);
    for (const uint8_t* ch = buf_read; ch < buf_read + sizeof(buf_read); ch++)
      printf("%02X", *ch);
#endif
    putchar('\n');
    _delay_ms(1000);
  }
}

