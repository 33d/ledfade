#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "mirf.h"
#include "fader.h"
#include "serial.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

// timer prescaler
#define PRESCALER 256
#define PRESCALER_CS (_BV(CS02))

#define PORT_RED PORTD
#define DDR_RED  DDRD
#define PIN_RED  PIND
#define WIRE_RED PORTD6
#define OCR_RED   OCR0A
#define PORT_GREEN PORTD
#define DDR_GREEN  DDRD
#define PIN_GREEN  PIND
#define WIRE_GREEN PORTD5
#define OCR_GREEN  OCR0B
#define PORT_BLUE PORTB
#define DDR_BLUE  DDRB
#define PIN_BLUE  PINB
#define WIRE_BLUE PORTB3
#define OCR_BLUE   OCR2A

Fader fader;
// How long (in timer0 overflows) we're fading for
int16_t fade_duration;

typedef struct {
  unsigned int nrf_interrupt : 1;
  unsigned int timer_overflow : 1;
} Events;
#define event(e) ((volatile Events*) &GPIOR0)->e
#define check_event(e, action) if (event(e)) { event(e) = 0; action; }
#define more_events (GPIOR0 != 0)

/*
Packet format:
  Sequence  R    G    B  Duration  TimeToNext
Duration = duration(seconds) / 16
*/

void mirf_handle_rx(uint8_t buf_read[mirf_PAYLOAD]) {
  printf("Read data ");
  for (const uint8_t* ch = buf_read; ch < buf_read + mirf_PAYLOAD; ch++)
    printf("%02X", *ch);
  putchar('\n');

  fade_duration =  (
    (F_CPU / PRESCALER / 256) // overflows per second
    * ((int16_t) buf_read[4])
    / 16
  );
  fader.start(&fade_duration, buf_read + 1);
  // enable the timer0 overflow interrupt, which times the fading
  TIMSK0 |= _BV(TOIE0);
}

ISR(INT0_vect, ISR_NAKED) {
  event(nrf_interrupt) = 1;
  reti();
}

ISR(TIMER0_OVF_vect, ISR_NAKED) {
  event(timer_overflow) = 1;
  reti();
}

void update_values() {
    fader.fade();
    OCR_RED = fader.led[0];
    OCR_GREEN = fader.led[1];
    OCR_BLUE = fader.led[2];
}

static void handle_timer_overflow() {
  static int16_t counter = 0; // the current fade or delay overflow count

  PINB |= _BV(PORTB5);

  ++counter;
  if (counter > fade_duration) {
    puts("Stopping fading");
    // turn off the timer0 overflow interrupt, so this doesn't get called
    TIMSK0 &= ~_BV(TOIE0);
    counter = 0;
  } else
    update_values();
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

  // Initialize pins
  DDR_RED |= _BV(WIRE_RED);
  DDR_GREEN |= _BV(WIRE_GREEN);
  DDR_BLUE |= _BV(WIRE_BLUE);
  // All outputs on
  PORT_RED |= _BV(WIRE_RED);
  PORT_GREEN |= _BV(WIRE_GREEN);
  PORT_BLUE |= _BV(WIRE_BLUE);

  // Init timers
  TCCR0A = _BV(COM0A1)  // Clear OC0A on timer match
         | _BV(COM0B1)  // Clear OC0B on timer match
         | _BV(WGM01) | _BV(WGM00) // Fast PWM mode
         ;
  TCCR0B = 0;
  TIMSK0 = _BV(TOIE0); // Overflow interrupt
  TCCR2A = _BV(COM2A1)  // Clear OC2A on timer match
         | _BV(WGM01) | _BV(WGM00) // Fast PWM mode
         ;
  TCCR2B = 0;

  // Initialize the timer values to 1.  Probably not needed, but value 0
  // isn't supported.
  OCR0A = OCR0B = OCR2A = 1;
  for (uint8_t i = 0; i < 3; i++)
      fader.led[i] = 1;

  // Start timers
  TCCR0B |= PRESCALER_CS;
  TCCR2B |= PRESCALER_CS;

  update_values();

  while(1) {
    if (!more_events) {
      sleep_enable();
      sei();
      sleep_cpu();
      sleep_disable();
      cli();
    }
    check_event(nrf_interrupt, mirf_handle_interrupt());
    check_event(timer_overflow, handle_timer_overflow());
  }
}

