#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

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

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}


tk::spline sx;
tk::spline sy;
tk::spline sh;

void init(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) 
{
  vector<double> maps_h;
  for (int i=0; i<maps_s.size(); i++) 
  {
    int next_i = (i+1)%maps_x.size();
    double heading = atan2((maps_y[next_i]-maps_y[i]),(maps_x[next_i]-maps_x[i]));
    maps_h.push_back(heading);
  }

  sx.set_points(maps_s, maps_x);    // currently it is required that X is already sorted
  sy.set_points(maps_s, maps_y);    // currently it is required that X is already sorted
  sh.set_points(maps_s, maps_h);    // currently it is required that X is already sorted

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  double heading = sh(s);

	double perp_heading = heading-pi()/2;


  double x = sx(s) + d*cos(perp_heading);
  double y = sy(s) + d*sin(perp_heading);
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


vector<double> calc_cost_vec(std::vector<double> v, double a_max=9.9, double v_max=22, double dt=0.02)
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


vector<double> gen_trajectory(vector<double> poly, int len, double dt=0.02)
{
  std::vector<double> v;
  for(int i = 1; i <= len; i++)
  {
      v.push_back(polyeval(poly, i*dt));
  }

  return v;

}

vector<double> JMT(vector< double> start, vector <double> end, double T);



int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);
  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  cout << "Map array length: " << map_waypoints_x.size() << endl;

  int grid_size = 1000;
  std::vector<std::vector<double>> grid;

  init(map_waypoints_s, map_waypoints_x, map_waypoints_y);
  //for (int i=0; i<gr)

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            int path_size = previous_path_x.size();
            double pos_x;
            double pos_y;
            double angle;
            double last_speed;
            double last_acc;
            static double last_s = 0; //Better then using the s reported from telemetry because it creates jerk
            static vector<double> s_traj;

            cout << "x, y, speed = " << car_x << ", " << car_y << ", " << car_speed <<  endl;

            for(int i = 0; i < path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            double dt = 0.02; //20ms between simulation steps
            if(path_size == 0)
            {
                pos_x = car_x;
                pos_y = car_y;
                angle = deg2rad(car_yaw);
                last_speed = car_speed;
                last_s = car_s;
                last_acc = 0;
                cout << "path_size = 0" << endl;
            }
            else
            {
                pos_x = previous_path_x[path_size-1];
                pos_y = previous_path_y[path_size-1];

                cout << "previous_path last x,y, path_size: " << pos_x << ", " << pos_y << ", " << path_size << endl;

/*
                assert(path_size >= 3);
                double pos_x2 = previous_path_x[path_size-2];
                double pos_y2 = previous_path_y[path_size-2];

                double pos_x3 = previous_path_x[path_size-3];
                double pos_y3 = previous_path_y[path_size-3];
                angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
                double dist = distance(pos_x, pos_y, pos_x2, pos_y2);
                double dist2 = distance(pos_x3, pos_y3, pos_x2, pos_y2);
                last_speed = dist/dt;
                double speed2 = dist2/dt;
                last_acc = (last_speed - speed2)/dt;
*/
                double last = s_traj.size()-1;
                double d1 = s_traj[last] - s_traj[last-1];
                double d2 = s_traj[last-1] - s_traj[last-2];
                double v1 = d1/dt;
                double v2 = d2/dt;
                double a = (v1-v2)/dt;

                last_speed = v1;
                last_acc = a;

            }

            //cout << "previous path:" << endl;
            //print_path(previous_path_x, previous_path_y);


            double time = 1;
            vector<double> sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
            double s = sd[0];
            double d = sd[1];
            //50 mph = 50*1600/3600 = 22
            double T = 2; //sec
            double target_speed = 15;//fmin(10, last_speed+max_acc*T); //m/s
            double max_path_size = 100;

            cout << "speed @ end of path = " << last_speed << endl;
            double min_cost = 0xFFFFFFFF;
            vector<double> best_poly;
            vector<double> best_traj;
            vector<double> best_end;
            for (double dS=-5; dS<5; dS += 1) 
            {
              for (double dT=-0.3; dT<0.3; dT+=0.1)
              {
                for (double dV=-10; dV<=0; dV+=2)
                {
                  double s_target = last_s + 1.1*(target_speed + dV)*(T + dT);
                  vector<double> start = {last_s,        last_speed,        last_acc};
                  vector<double> end =   {s_target + dS, target_speed + dV, 0};
                  vector<double> poly = JMT(start, end, T+dT);
                  vector<double> traj = gen_trajectory(poly, max_path_size);
                  if (s_traj.size() > 3)
                    //insert few elements from the end of previous path
                    traj.insert(traj.begin(), s_traj.end() - 3, s_traj.end());
                  //vector<double> full_traj(s_traj.end() - path_size, s_traj.end());

                  //full_traj.insert( full_traj.end(), future_traj.begin(), future_traj.end() );
                  //assert(full_traj.size() == max_path_size);
                  vector<double> costs = calc_cost_vec(traj);
                  //cout << costs << endl;
                  //cout << costs[2] << endl;
                  double cost = 1000*costs[0] + 1000*costs[1] + costs[2] + 1000*dV*dV;
                  if (cost > 0 && cost < min_cost)
                  {
                    min_cost = cost;
                    best_poly = poly;
                    best_traj = traj;
                    best_end = end;
                  //cout << "start: " << start << endl;
                  //cout << "end: " << end << endl;
                  //cout << "poly: " << poly << endl;
                    //cout << "min_cost = " << min_cost << endl;
                    //cout << "costs: " << endl;
                    //cout << costs << endl;
                  }

                }
              }
            }

            cout << "best_end: " << best_end << endl;
            cout << "best_traj: " << best_traj << endl;
            cout << "best_poly = " << best_poly << endl;

            s = polyeval(best_poly, 0);
            //cout << "zero s: " << s << endl;
            for(int i = 1; i < max_path_size-path_size+1; i++)
            {
                s = polyeval(best_poly, i*dt);
                d = 6;
                //cout << "s_diff = " << s - last_s << endl;
                vector<double> xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                double x_next = xy[0];
                double y_next = xy[1];
                //cout << "s, x, y: " << s << ", " << x_next << ", " << y_next << endl;
                //int idx = ( map_idx + i ) % map_size;
                next_x_vals.push_back(x_next);
                next_y_vals.push_back(y_next);
                last_s = s;
                s_traj.push_back(s);
            }
    
            //guard(next_x_vals);
            //guard(next_y_vals);

            cout << "next_x_vals.size(): " << next_x_vals.size() << endl;
           	//print_path(next_x_vals, next_y_vals);
            /*
            //output path to file:
            ofstream myfile("debug.xls");
            int vsize = next_x_vals.size();
            for(int n; n<vsize; n++)
                myfile << next_x_vals[n] << ',';
            myfile << endl;
            myfile.close();
            */

            //ofstream myfilestream("debug.csv");
            //std::copy(next_x_vals.begin(), next_x_vals.end(), std::ostream_iterator<double>(myfilestream, (const char*)'\n'));

            //check distances:
            double prev_speed = 0;
            for (int i=1; i<next_x_vals.size(); i++) 
            {
              double dist = distance(next_x_vals[i], next_y_vals[i], next_x_vals[i-1], next_y_vals[i-1]);
              //50mph max = 50*1600/3600 = 22 m/sec = 
              double speed = dist/dt;
              double acc = (speed-prev_speed)/dt;
              //cout << "acc: " << acc << endl;
              if (speed > 20)
                cout << "v > v_max @ i: " << i << ", speed = " << speed << endl;
              if (i > 1 && fabs(acc) > 9)
                cout << "a > a_max @ i: " << i << ", acc = " << acc << endl;
              
              prev_speed = speed;
              //if (acc > 5)
              //  cout << "a > a_max @ i: " << i << ", ddist = " << ddist << endl;
              /*
              if (fabs(next_x_vals[i] - next_x_vals[i-1]) > 0.3)
                cout << "Jump x, i: " << i << endl;

              if (fabs(next_y_vals[i] - next_y_vals[i-1]) > 0.3)
                cout << "Jump y, i: " << i << endl;
              */
            }


            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































