#include <avr/io.h>
#include <util/atomic.h>
#include <stdio.h>

int pin_putchar(char c, FILE* stream) {
/* Assembler version of:
  uint8_t count = 8;
  PORTD |= _BV(PORTD0);
  wait()
  for (; count > 0; count++) {
    if (c & 1)
      PORTD &= ~_BV(PORTD0);
    else
      PORTD |= _BV(PORTD0);
    c >>= 1;
    wait()
  }
  PORTD &= _BV(PORTD0);
  wait()
*/
  uint8_t temp;
  uint8_t delay;
  uint8_t count;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    asm volatile("\t"
      "  cbi %[port], 0 \n\t"        /* 1 */
      "  nop \n\t"
      "  ldi %[count], 8 \n\t"
      "  ldi %[delay], 3 \n\t"       /* 1 */ /* wait 9 cycles */
      "delay_start_bit: \n\t" /* it takes 5 cycles before the data starts */
      "  dec %[delay] \n\t"
      "  brne delay_start_bit \n\t"
      "loop_data_bit: \n\t"
      "  lsr %[data] \n\t"           /* 1 */
      "  brcs handle_0 \n\t"         /* 1  2 */
      "  nop \n\t"                   /* 1 */
      "  cbi %[port], 0 \n\t"        /* 1 */
      "  rjmp next_data_bit \n\t"    /* 2 */
      "handle_0: \n\t"
      "  sbi %[port], 0 \n\t"        /*    1 */
      "  rjmp . \n\t"                /*    2 */
      "next_data_bit: \n\t"
      "  rjmp . \n\t"                /* 2 */
      "  rjmp . \n\t"                /* 2 */
      "  rjmp . \n\t"                /* 2 */
      "  nop \n\t"                   /* 1 */
      "  dec %[count] \n\t"          /* 1 */
      "  brne loop_data_bit \n\t"    /* 2 */
                                   /* = 16 cycles */
      /* Burn another 5 cycles before setting the stop bit */
      "  rjmp . \n\t"                /* 2 */
      "  rjmp . \n\t"                /* 2 */
      "  rjmp . \n\t"                /* 2 */
      "  sbi %[port], 0 \n\t"
      "  ldi %[delay], 5 \n\t"       /* 1 */ /* wait at least 15 cycles */
      "delay_stop_bit: \n\t"
      "  dec %[delay] \n\t"
      "  brne delay_stop_bit \n\t"
    : [delay] "=&r" (delay),
      [count] "=&r" (count)
    : [port] "I" (_SFR_IO_ADDR(PORTC)), 
      [data] "r" (c)
    );
  }
  return 0;
}

/* This line cannot be in a C++ file */
FILE stdout_uart = FDEV_SETUP_STREAM(pin_putchar, NULL,
                                         _FDEV_SETUP_WRITE);

