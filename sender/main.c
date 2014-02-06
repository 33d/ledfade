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

// nicked from
// http://forum.htsoft.com/all/showflat.php/Cat/0/Number/122342/page/0/fpart/1/vc/1
uint8_t rand255() {
    static uint8_t x = 1;
    if (x & 0x80)
        x = (x<<1) ^ 0x1d;
    else
        x = x << 1;
    return x; 
}

int main(void) {
  uint8_t buf_write[6];
  uint8_t sequence = 0;

  const uint8_t duration = 2*16; // Fade duration * 1/16 sec
  buf_write[4] = duration;
  buf_write[5] = 10 * 16; // Time to next - TODO

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
    buf_write[0] = sequence++;
    buf_write[1] = rand255();
    buf_write[2] = rand255();
    buf_write[3] = rand255();
    // Occasionally change the colour immediately
    buf_write[4] = (rand255() & 0x03) == 0x03 ? 0 : duration;
    printf("Sending ");
    // the sizeof is wrong but it won't matter here
    for (uint8_t* n = buf_write; n < buf_write + sizeof(buf_write); ++n)
      printf("%02X", *n);
    mirf_send(buf_write, sizeof(buf_write));
    putchar('\n');
    _delay_ms(6000);
  }
}

