#include <math.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "utils.h"

using namespace std;


tk::spline sx;
tk::spline sy;
tk::spline sh;

ofstream csvfile;


void init(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) 
{
  csvfile.open ("debug.csv");

  vector<double> maps_h;
  double prev_heading = 0;
  for (int i=0; i<maps_s.size(); i++) 
  {
    int next_i = (i+1)%maps_x.size();
    double dy = maps_y[next_i]-maps_y[i];
    double dx = maps_x[next_i]-maps_x[i];
    double heading = atan2(dy,dx);
    if (fabs(heading-prev_heading) > pi()/2) {
      heading += 2*pi();
    }
    prev_heading = heading;
    //csvfile << dx << "," << dy << "," << heading << endl;
    maps_h.push_back(heading);
  }

  sx.set_points(maps_s, maps_x);    // currently it is required that X is already sorted
  sy.set_points(maps_s, maps_y);    // currently it is required that X is already sorted
  sh.set_points(maps_s, maps_h);    // currently it is required that X is already sorted

/*
  //csvfile.close();
  //csvfile.open("debug2.csv");

  for (double s=3200; s<3350; s+=0.1) {
    double d = 2;
    vector<double> xy = getXY(s, d, maps_s, maps_x, maps_y);
    csvfile << s << "," << d << "," << xy[0] << "," << xy[1] << endl;
  }
  csvfile.flush();
*/
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}



// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  double heading = sh(s);

	double perp_heading = heading-pi()/2;


  double x = sx(s) + d*cos(perp_heading);
  double y = sy(s) + d*sin(perp_heading);
  
  csvfile << sx(s) << "," << sy(s) << "," << perp_heading << ",";
/*
  cout << "s = " << s;
  cout << ", d = " << d;
  cout << ", x = " << x;
  cout << ", y = " << y;
  cout << ", heading = " << heading << endl;
*/
	return {x,y};

}

double polyeval(vector<double> coefs, double x)
{
  return coefs[0] + 
         coefs[1]*x +
         coefs[2]*x*x +
         coefs[3]*x*x*x +
         coefs[4]*x*x*x*x +
         coefs[5]*x*x*x*x*x;
}


void print_path(vector<double> x_pts, std::vector<double> y_pts)
{
  cout << "x: ";
  for (int i=0; i<x_pts.size(); i++) {
    cout << x_pts[i] << ", ";
  }
  cout << endl;

  cout << "y: ";
  for (int i=0; i<y_pts.size(); i++) {
    cout << y_pts[i] << ", ";
  }
  cout << endl;
}

double rectify(double x, double x_min, double x_max)
{
  if (x <= x_min)
    return x_min;
  else if (x >= x_max)
    return x_max;
  else
    return x;
}

//
void guard(std::vector<double> &v, double a_max=8, double v_max=15, double dt=0.02)
{
  double deficit = 0;
  for (int i=2; i<v.size(); i++)
  {
    double d1 = v[i]-v[i-1];
    double d2 = v[i-1]-v[i-2];
    double v1 = d1/dt;
    double v2 = d2/dt;
    double a = (v1-v2)/dt;

    double v1_max = rectify(v2 + a_max*dt, 0, v_max);
    double v1_min = rectify(v2 - a_max*dt, 0, v_max);
    double d1_max = v1_max*dt;
    double d1_min = v1_min*dt;     
    double vi_new = rectify(v[i], v[i-1] + d1_min, v[i-1] + d1_max);
    deficit = (v[i] - vi_new);

    v[i] = vi_new;
  }
  cout << "deficit = " << deficit << endl;

}


template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}
