#ifndef _APURIT_H_
#define _APURIT_H_

#define map_yaw(x) (x < -180 ? x + 360 : (x > 180 ? x - 360: x))

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif