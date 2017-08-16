#ifndef UTILS_H
#define UTILS_H
#include <vector>
#include <math.h>
#include "spline.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

void init(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
double polyeval(vector<double> coefs, double x);

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v);

extern tk::spline sx;
extern tk::spline sy;
extern tk::spline sh;


#endif
