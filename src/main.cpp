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
#include "behave.h"
#include "trajectory.h"
#include "utils.h"

using namespace std;

// for convenience
using json = nlohmann::json;


vector<double> s_traj;  //Record of all the s points generated
vector<double> d_traj;  //Record of all the d points generated


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



//Calculates the position, velocity and acceleration at the end of the given trajectory
vector<double> calc_eop_state(vector<double> traj, double dt) {
  assert(traj.size() >= 3);
  double last = traj.size()-1;
  double d1 = traj[last] - traj[last-1];
  double d2 = traj[last-1] - traj[last-2];
  double v1 = d1/dt;
  double v2 = d2/dt;
  double a = (v1-v2)/dt;
  
  return {traj[last], v1, a};
}    

plan_t behavioral_planner(vector<vector<double>> sensor_fusion, int lane_eop, double s_eop, double time_to_skip);

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  //vector<string> map_files = {"data/highway_map.csv", "../data/highway_map.csv", "highway_map.csv", "../highway_map.csv"};
  vector<string> map_files = {"data/highway_map_bosch1.csv", "../data/highway_map_bosch1.csv", "highway_map_bosch1.csv", "../highway_map_bosch1.csv"};
  //vector<string> map_files = {"data/highway_map_bosch1.csv"};
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_;
  for (int i=0; i<map_files.size(); i++) { 
	in_map_.open(map_files[i].c_str(), ifstream::in);
 	if (in_map_.is_open())
		break;
  } 
  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
    iss.ignore(1, ',');
  	iss >> y;
    iss.ignore(1, ',');
  	iss >> s;
    iss.ignore(1, ',');
  	iss >> d_x;
    iss.ignore(1, ',');
  	iss >> d_y;
    //eliminate duplicates
    //cout << line << endl;
    if (find(map_waypoints_s.begin(), map_waypoints_s.end(), s) != map_waypoints_s.end())
      continue;
    cout << s << endl;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  cout << "Map array length: " << map_waypoints_x.size() << endl;

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
          	//The data format for each car is: [ id, x, y, vx, vy, s, d]
            auto sensor_fusion = j[1]["sensor_fusion"]; 

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            int path_size = previous_path_x.size();

            cout << "x, y, speed = " << car_x << ", " << car_y << ", " << car_speed <<  endl;
            cout << "path_size = " << path_size << endl;

            vector<double> s_eop_state = {0, 0, 0}; //end of path: {s, sd, sdd}
            vector<double> d_eop_state = {0, 0, 0}; //end of path: {d, dd, ddd}
            for(int i = 0; i < path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            double dt = 0.02; //20ms between simulation steps
            if(path_size == 0) {
                s_eop_state = {car_s, car_speed, 0};
                d_eop_state = {car_d, 0, 0};
            } else {
                s_eop_state = calc_eop_state(s_traj, dt);
                d_eop_state = calc_eop_state(d_traj, dt);
            }

      			static double target_speed = mph2ms(50);
      			static int target_lane = 1; 
      			double max_path_size = 100;
            map_t map = {
              .map_waypoints_s = map_waypoints_s,
              .map_waypoints_x = map_waypoints_x,
              .map_waypoints_y = map_waypoints_y
            };
      			trajectory_t trajectory =   gnerate_optimal_trajectory(target_lane, 
                                                                   target_speed, 
                                                                   s_eop_state,
                                                                   d_eop_state,
                                                                   dt,
                                                                   max_path_size-path_size,
                                                                   max_path_size,
                                                                   map);

            //csvfile << s << "," << d << ",";
            //csvfile << x_next << "," << y_next << "," << endl;
            //csvfile.flush(); 

            //Append the new path to the existing path
            next_x_vals.insert(next_x_vals.end(), trajectory.x.begin(), trajectory.x.end());
            next_y_vals.insert(next_y_vals.end(), trajectory.y.begin(), trajectory.y.end());
            //In Frenet coordinates for calculating end of state in the next frame
            s_traj.insert(s_traj.end(), trajectory.s.begin(), trajectory.s.end());
            d_traj.insert(d_traj.end(), trajectory.d.begin(), trajectory.d.end());

            plan_t plan = behavioral_planner(sensor_fusion, target_lane, s_eop_state[0], dt*path_size);
            target_lane = plan.lane;
            target_speed = plan.speed;

            //path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
















































































