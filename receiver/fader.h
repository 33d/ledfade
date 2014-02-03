#if !defined(FADER_H)
#define FADER_H

#include <inttypes.h>

class Interpolator {
private:
  const int16_t* steps;
  uint8_t min;
  uint8_t max;
  uint8_t curr;
  uint8_t delta;
  int16_t error;
  int8_t dir;
  
public:
  Interpolator() : steps(0) { }
  Interpolator(const int16_t* steps, uint8_t min, uint8_t max) {
    init(steps, min, max);
  }
  void init(const int16_t* steps, uint8_t min, uint8_t max);
  uint8_t next();
};

class Fader {
  public:
    Fader() {
      for (int i = 0; i < 3; i++)
        led[i] = 0;
    }
    void start(const int16_t* duration, const uint8_t colors[3]);
    void fade();
    uint8_t led[3];
    uint8_t target[3];
  private:
    Interpolator interpolators[3];
};

#endif

