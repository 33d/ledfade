#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include <stdint.h>
#include "fader.h"

#define CU_ASSERT_NEARLY(n, e) \
  CU_ASSERT_TRUE((e) >= ((n) == 0 ? 0 : (n) - 1)); \
  CU_ASSERT_TRUE((e) <= ((n) == 255 ? 255 : (n) + 1));

void testFaderSetColor() {
  int16_t count = 300;
  Fader f;
  static const uint8_t l1[] = { 0, 0, 0 };
  f.start(&count, l1);
  CU_ASSERT(f.led[0] == 0 && f.led[1] == 0 && f.led[2] == 0);
  CU_ASSERT(f.target[0] == 0 && f.target[1] == 0 && f.target[2] == 0);
  static const uint8_t l2[] = { 128, 255, 192 };
  f.start(&count, l2);
  CU_ASSERT(f.led[0] == 0 && f.led[1] == 0 && f.led[2] == 0);
  CU_ASSERT(f.target[0] == 128 && f.target[1] == 255 && f.target[2] == 192);

  // Try one advance.  The LED shouldn't change much.
  f.fade();
  CU_ASSERT_NEARLY(1, f.led[0]);
  CU_ASSERT_NEARLY(1, f.led[1]);
  CU_ASSERT_NEARLY(1, f.led[2]);

  {
    int i = 0;
    // Run the interpolation half way
    for (; i < count / 2; i++)
      f.fade();
    CU_ASSERT_NEARLY( 64, f.led[0]);
    CU_ASSERT_NEARLY(128, f.led[1]);
    CU_ASSERT_NEARLY( 96, f.led[2]);

    // Run the interpolation to the end
    for (; i < count; i++)
      f.fade();
    CU_ASSERT_NEARLY(128, f.led[0]);
    CU_ASSERT_NEARLY(255, f.led[1]);
    CU_ASSERT_NEARLY(192, f.led[2]);
  }

  // See if it goes back down again
  static const uint8_t l3[] = { 32, 32, 32 };
  f.start(&count, l3);
  f.fade();

  CU_ASSERT_NEARLY(127, f.led[0]);
  CU_ASSERT_NEARLY(254, f.led[1]);
  CU_ASSERT_NEARLY(191, f.led[2]);

  for (int i = 0; i < count - 2; i++)
    f.fade();

  CU_ASSERT_NEARLY(32, f.led[0]);
  CU_ASSERT_NEARLY(32, f.led[1]);
  CU_ASSERT_NEARLY(32, f.led[2]);
}
