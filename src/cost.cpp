#include <algorithm>
#include <cassert>
#include "cost.h"


//priority levels for costs
#define COLLISION   10*1000*1000
#define DANGER      10*1000*100
#define REACH_GOAL  10*1000*100
#define COMFORT     10*1000*10
#define EFFICIENCY  10*10

#define DESIRED_BUFFER  1.5 //timesteps
#define PLANNING_HORIZON  2


vector<double> calc_cost_vec(std::vector<double> v, double a_max, double v_max, double dt)
{
  double a_cost = 0;
  double v_cost = 0;
  double jerk_tot = 0;
  assert(v.size() > 2);
  for (int i=2; i<v.size(); i++) 
  {
    double d1 = v[i]-v[i-1];
    double d2 = v[i-1]-v[i-2];
    double v1 = d1/dt;
    double v2 = d2/dt;
    double a = (v1-v2)/dt;
    if (fabs(a) > a_max)
      a_cost += 1;
    if (fabs(v2) > v_max)
      v_cost += 1;

    if (i>=3) 
    {
      double d3 = v[i-2] - v[i-3];
      double v3 = d3/dt;
      double a2 = (v2-v3)/dt;
      double jerk = (a-a2)/dt;
      jerk_tot += (jerk*jerk);
    }
    //cout << "i, a = " << i << ", " << a << endl;
  }

  return {a_cost, v_cost, jerk_tot};
}
