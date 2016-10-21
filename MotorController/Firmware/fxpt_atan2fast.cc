#include <stdint.h>
//-----------------------------------------------
// Fast arctan2
const uint16_t maxAngle = 65535;
uint16_t fxpt_atan2fast(int16_t y, int16_t x)
{
  if (x == y) { // x/y or y/x would return -1 since 1 isn't representable
    if (y > 0) { // 1/8
        return maxAngle / 8;
    } else if (y < 0) { // 5/8
        return 5 * (maxAngle / 8);
    } else { // x = y = 0
        return 0;
    }
  }
  int16_t coeff_1 = maxAngle/8;
  int16_t coeff_2 = 3*coeff_1;
  int16_t abs_y = y > 0 ? y : -y;
  const int16_t frac = 1024;
  int16_t angle;
  if (x>=0)
  {
    int16_t r = ((x - abs_y) * (int32_t) frac) /  (x + abs_y);
    angle = coeff_1 - (coeff_1 * r) / frac;
  }
  else
  {
    int16_t r = ((x + abs_y) * (int32_t) frac) / (abs_y - x);
    angle = coeff_2 - (coeff_1 * r) / frac;
  }
  if (y <= 0)
    return(maxAngle-angle);     // negate if in quad III or IV
  return(angle);
}
