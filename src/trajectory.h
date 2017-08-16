#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <vector>

using namespace std;

typedef struct _trajectory_t {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> d;
} trajectory_t;

typedef struct _map_t {
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
} map_t;

trajectory_t gnerate_optimal_trajectory(int target_lane, 
                                        double target_speed, 
                                        vector<double> s_eop_state, 
                                        vector<double> d_eop_state, 
                                        double dt, 
                                        int len,
                                        int max_path_size,
                                        map_t map);

#endif
