#ifndef BEHAVE_H
#define BEHAVE_H
#include <math.h>
#include <vector>
#include <iostream>
#include <iterator>     // std::ostream_iterator

using namespace std;

typedef struct _plan_t {
  int lane;
  double speed;
} plan_t;


//converts form Miles Per Hour to Meters per Second
inline double mph2ms(double mph) {
  return mph/2.25;
}

inline int d2lane(double d) {
  return min(2, (int)(fabs(d)/4));
}

#define SPEED_LIMIT mph2ms(45)
#define ACCEL_LIMIT 9.9  //m/s^2

//#define MAX_ACCEL  5 //m/s^2


#endif
