#include "fader.h"
#include <stdint.h>
#include <stdio.h>

/* 1=1x, 2=8x, 3=64x, 4=256x, 5=1024x */
#define PRESCALER 256
#define PRESCALER_CS 4

/* how many times the 8-bit timer must overflow during a fade */
#define FADE_OVF (FADETIME * F_CPU / PRESCALER / 256)
#define DELAY_OVF (FADEDELAY * F_CPU / PRESCALER / 256)
// increments on each timer overflow

//#if !defined(AVR)
#include <string.h>
//#define PROGMEM
//#else
//#include <avr/pgmspace.h>
//#endif

#define R 0
#define G 1
#define B 2

void Interpolator::init(const int16_t* steps, uint8_t min, uint8_t max) {
  this->steps = steps;
  this->min = min;
  this->max = max;
  if (max > min) {
    dir = 1;
    delta = max - min;
  } else {
    dir = -1;
    delta = min - max;
  }
  error = *(this->steps) / 2;
  curr = min;
}

uint8_t Interpolator::next() {
  error -= delta;
  if (error < 0) {
    curr += dir;
    error += *steps + 1;
  }
  return curr;
}

void Fader::start(const int16_t* steps, const uint8_t colors[3]) {
  memcpy(target, colors, sizeof(target));
  for (uint8_t i = 0; i < 3; i++)
    interpolators[i].init(steps, led[i], target[i]);
}

void Fader::fade() {
  for (uint8_t i = 0; i < 3; i++)
    led[i] = interpolators[i].next();
}

