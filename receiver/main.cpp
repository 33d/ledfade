#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
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
uint16_t counter = 0; // the current fade or delay overflow count
// If we get no signal for this many overflows, shut down.  16s is the
//  longest delay.
#define shutdown_delay (17 * (F_CPU / 256 / PRESCALER))
uint16_t radio_poweron = 0;

typedef struct {
  unsigned int nrf_interrupt : 1;
  unsigned int timer_overflow : 1;
} Events;
#define event(e) ((volatile Events*) &GPIOR0)->e
#define check_event(e, action) if (event(e)) { event(e) = 0; action; }
#define more_events (GPIOR0 != 0)

static void power_off() {
  cli();
  puts("Power off");
  mirf_poweroff();
  wdt_disable();
  ADCSRA = (1<<ADEN);
  // Disable IRQ, pull it high
  EIMSK &= ~_BV(INT0);  // worth 200 uA!
  // All ports outputs, all values high
  DDRB = DDRC = DDRD = 1;
  PORTB = PORTC = PORTD = ~0;
  // except the LED pins, or it keeps glowing
  PORT_RED &= ~_BV(WIRE_RED);
  PORT_GREEN &= ~_BV(WIRE_GREEN);
  PORT_BLUE &= ~_BV(WIRE_BLUE);
  PORTB &= ~_BV(PORTB5);
  // Make the IRQ line an input, since the chip holds it at 3.3V
  DDRD &= ~_BV(DDD2);
  PORTD &= ~_BV(PORTD2);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  power_all_disable();
#if defined(BODS)
  // Turn off brownout detection
  MCUCR |= (1<<BODS) | (1<<BODSE);
  MCUCR |= (1<<BODS);
#endif
  sleep_cpu(); 
}

/*
Packet format:
  Sequence  R    G    B  Duration  TimeToNext
Duration = duration(seconds) / 16
*/

static void update_leds() {
 OCR_RED = fader.led[0];
 OCR_GREEN = fader.led[1];
 OCR_BLUE = fader.led[2];
}

void mirf_handle_rx(uint8_t buf_read[mirf_PAYLOAD]) {
  printf("Read data ");
  for (const uint8_t* ch = buf_read; ch < buf_read + mirf_PAYLOAD; ch++)
    printf("%02X", *ch);
  putchar('\n');

  if (buf_read[4] == 0) {
    // Change immediately
    counter = 0;
    for (uint8_t i = 0; i < 3; i++)
      fader.led[i] = buf_read[i+1];
    update_leds();
  } else {
    fade_duration =  (
      (F_CPU / PRESCALER / 256) // overflows per second
      * ((int16_t) buf_read[4])
      / 16
    );
    fader.start(&fade_duration, buf_read + 1);
    // enable the timer0 overflow interrupt, which times the fading
    //TIMSK0 |= _BV(TOIE0);
    counter = 0;
  }

  // Set when the radio should turn on again.  It needs about 5ms to get from
  // power down to RX mode.
  // Don't bother if it's less than 10 ms or 1 overflow
  if (buf_read[5] == 0 // 1 = 62.5ms
      || buf_read[5] < (uint8_t) (16.0 / (F_CPU / PRESCALER / 256) + 1))
    radio_poweron = 0;
  else {
    radio_poweron = (
      (uint16_t) (((float) F_CPU / PRESCALER / 256) // overflows per second
      / 16)
      * 
      buf_read[5]
    );
    PORTB &= ~_BV(PORTB5);
    printf("Waking up in %d overflows\n", radio_poweron);
    mirf_poweroff();
  }
}

ISR(INT0_vect, ISR_NAKED) {
  event(nrf_interrupt) = 1;
  reti();
}

ISR(TIMER0_OVF_vect, ISR_NAKED) {
  event(timer_overflow) = 1;
  reti();
}

static void handle_timer_overflow() {
  ++counter;
  if (counter < fade_duration) {
    fader.fade();
    update_leds();
  }

  if (counter > radio_poweron) {
    printf("Power on\n");
    PORTB |= _BV(PORTB5);
    mirf_config(); // probably overkill
    radio_poweron = INT16_MAX;
  }

  if (counter > shutdown_delay) {
    power_off();
  };
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

