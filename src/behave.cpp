#include <iostream>
#include <fstream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <cassert>
#include "behave.h"
#include "utils.h"

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}


double preferred_buffer = 30;
int num_of_lanes = 3;
double max_speed = mph2ms(50); 

typedef struct _closest_cars_t {
  vector<int>  in_front;  //one car per lane in front 
  vector<int>  behind;    //one car per lane from behind
} closest_cars_t;

//Returns vector of indexes of cars that are closest in front of the car's s position
//assuming 3 lanes
closest_cars_t find_closest_cars(vector<vector<double>> sensor_fusion, int num_lanes, double s, double time_to_skip) {

  vector<double> s_min_front = {};
  vector<double> s_max_back = {};
  vector<int> leading = {};
  vector<int> trailing = {};
  for (int i=0; i<num_lanes; i++) {
    leading.push_back(-1);
    trailing.push_back(-1);
    s_min_front.push_back(0xFFFFFFFF);
    s_max_back.push_back(0);
  }
  //The data format for each car is: [ id, x, y, vx, vy, s, d]
  for (int i=0; i<sensor_fusion.size(); i++) {
    vector<double> car_data = sensor_fusion[i];
    int id = car_data[0];
    double d = car_data[6];
    int l_other = d2lane(d);
    double s_other = car_data[5];
    double vx = car_data[3];
    double vy = car_data[4];
    double v = sqrt(vx*vx + vy*vy);
    //Predict into the end of path
    double s_eop = s_other + v * time_to_skip;

    //cout << "s_eop: " << s_eop << ", d: " << d << ", lane: " << l_other << ", id: " << id << endl;
    if (s_eop > s &&
        s_eop < s_min_front[l_other] &&
        d > 0 ) {
      leading[l_other] = i;
      s_min_front[l_other] = s_eop;
    } else if (s_eop <= s &&
        s_eop > s_max_back[l_other] &&
        d > 0 ) {
      trailing[l_other] = i;
      s_max_back[l_other] = s_eop;
    }

  }
  
  closest_cars_t closest = {
    .in_front = leading,
    .behind = trailing
  };
  return closest; 

}

plan_t behavioral_planner(vector<vector<double>> sensor_fusion, int lane_eop, double s_eop, double time_to_skip) {

  closest_cars_t closest = find_closest_cars(sensor_fusion, num_of_lanes, s_eop, time_to_skip);

  vector<double> cost_v = {0, 0, 0}; //cost per lane
  vector<int> in_front = closest.in_front;
  vector<int> behind = closest.behind;

  //Cost calculation
  for (int i=0; i<num_of_lanes; i++) {
  
    int idx = in_front[i];
    if (idx >= 0) {
      vector<double> data = sensor_fusion[idx];
      double vx = data[3];
      double vy = data[4];
      double v = sqrt(vx*vx + vy*vy);
      double v_diff = max_speed - v;
      double s_now_other = data[5];
      double s_eop_other = s_now_other + v*time_to_skip;  //predict to the end of the existing path
      double s_diff = s_eop_other - s_eop; //between our car at the end of the path and predicted s of the other car at the same point of time
      cout << "v_diff: " << v_diff << ", s_diff: " << s_diff << endl;
      cost_v[i] += v_diff*v_diff; //cost of diff from fastest lane
      cost_v[i] += 1000*(1000/(s_diff*s_diff)); //cost of closeness to leading car
    }

    idx = behind[i];
    if (idx >= 0) {
      vector<double> data = sensor_fusion[idx];
      double vx = data[3];
      double vy = data[4];
      double v = sqrt(vx*vx + vy*vy);
      double s_now_other = data[5];
      double s_eop_other = s_now_other + v*time_to_skip;  //predict to the end of the existing path
      double s_diff = s_eop_other - s_eop; //between our car at the end of the path and predicted s of the other car at the same point of time
      cost_v[i] += 1000*(1000/(s_diff*s_diff)); //cost of closeness to trailing car
    }
    
    cost_v[i] += 1000*(i != lane_eop); //cost of lane change
    //cost of closest car from behind:


  }
  
  cout << "cost_v: " << cost_v << endl;
  //int best_lane = min_element(cost_v.begin(), cost_v.end()) - cost_v.begin();
  //Choose adjustent best lane:
  int best_lane = lane_eop;
  double min_lane_cost = 0xFFFFFFFF;
  for (int i=0; i<num_of_lanes; i++) {
    if (abs(i - lane_eop) <= 1 &&
        cost_v[i] < min_lane_cost) {
      best_lane = i;
      min_lane_cost = cost_v[i];
    }
  }
  cout << "best_lane: " << best_lane << endl; 

  //Extract speed from leading vehicle in chose lane:
  double target_speed = max_speed;
  int idx = in_front[best_lane];
  if (idx >= 0) {
    vector<double> leading_data = sensor_fusion[idx];
    double vx = leading_data[3];
    double vy = leading_data[4];
    double v_leading = sqrt(vx*vx + vy*vy);
    double s_leading = leading_data[5];
    double s_eop_leading = s_leading + v_leading*time_to_skip;
    double dist =  s_eop_leading - s_eop;
    if ( dist < preferred_buffer) {
      target_speed = v_leading*(dist/preferred_buffer);
    } 
    cout << "dist: " << dist << ", target_speed: " << target_speed << endl;
    cout << "s_eop: " << s_eop << ", s_eop_leading: " << s_eop_leading << endl;
  }
  plan_t plan = {
    .lane = best_lane,
    .speed = target_speed
  };
  
  return plan;
}