
#include <stdint.h>
#include <stdlib.h>
#include "Ravl/SysLog.hh"

//-----------------------------------------------
const uint16_t maxAngle = 65535;
// Fast arctan2
uint16_t atan2b(int16_t y, int16_t x)
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


int main(int nargs,char **argv)
{
  RavlDebug(" %d ",(int) atan2b(140,-540));
  RavlDebug(" %d ",(int) atan2b(-180,-340));
  RavlDebug(" %d ",(int) atan2b(-380,460));
  RavlDebug(" %d ",(int) atan2b(60,580));
  RavlDebug(" %d ",(int) atan2b(460,380));
  RavlDebug(" %d ",(int) atan2b(500,-380));

#if 0
  float maxErr = 0;
  for(int x = -1024;x < 1024; x += 1) {
    for(int y = -1024;y < 1024;y += 1) {
      float angle = atan2b(x,y);
      float actualAngle = atan2(x,y) * (float) (maxAngle/2) / 3.14159265359;
      if(actualAngle < 0)
        actualAngle += maxAngle;
      RavlN::StringC s;
      //s.form("%3f (%f) ",actualAngle, angle);
      float error = RavlN::Min(fabs(actualAngle  - (angle - (float) maxAngle)),fabs(actualAngle - angle));
      if(error > maxErr)
        maxErr = error;
      if(error > (maxAngle / 42)) {
        RavlError("Angle out of range. (%d,%d) diff=%f  Was:%f Target:%f ",x,y,error,angle,actualAngle);
        //exit(-1);
      }
      //std::cout << s ;
      //RavlDebug(" %d %d -> %d (%3.2f)",x,y,angle,actualAngle);
    }
  }
  RavlDebug("MaxError:%f ", maxErr / (float) maxAngle);
#endif
  return 0;
}
