#include <iostream>
#include <fstream>
#include <vector>
#include <iterator> 
#include "trajectory.h"
#include "utils.h"
#include "behave.h"
#include "cost.h"

using namespace std;

extern ofstream csvfile;       //csv file for debugging

vector<double> gen_trajectory(vector<double> poly, int len, double dt)
{
  std::vector<double> v;
  for(int i = 1; i <= len; i++)
  {
      v.push_back(polyeval(poly, i*dt));
  }

  return v;

}

vector<double> JMT(vector< double> start, vector <double> end, double T);


trajectory_t gnerate_optimal_trajectory(int target_lane, 
                                        double target_speed, 
                                        vector<double> s_eop_state, 
                                        vector<double> d_eop_state,
                                        double dt, 
                                        int len,
                                        int max_path_size,
                                        map_t map) {

	trajectory_t traj;
	//50 mph = 50*1600/3600 = 22
	double T = 2; //sec

	//cout << "speed @ end of path = " << last_speed << endl;
	double min_cost = 0xFFFFFFFF;
	vector<double> best_poly;
	vector<double> best_end;
	double best_T = 0;
	for (double dS=-10; dS<=10; dS += 2.5) 
	{
	  for (double dT=-0.5; dT<=0.5; dT+=0.25)
	  {
		  double target_T = T + dT;
		  double speed_eop = s_eop_state[1];
		  double s_eop = s_eop_state[0];
		  double accel = (target_speed - speed_eop)/target_T;
		  accel = fmax(-ACCEL_LIMIT/2, fmin(ACCEL_LIMIT/2, accel));
		  double cur_target_speed = speed_eop + accel*target_T;
		  cur_target_speed = fmin(SPEED_LIMIT, cur_target_speed);
		  
		  double avg_speed = fabs(cur_target_speed - speed_eop)/2;

		  double s_target = s_eop + speed_eop*target_T + 0.5*accel*target_T*target_T + dS;
      if (s_target < s_eop) {
        continue;
      }
		  vector<double> start = s_eop_state;
		  vector<double> end =	 {s_target, cur_target_speed, 0};
		  vector<double> poly = JMT(start, end, target_T);
		  vector<double> traj = gen_trajectory(poly, max_path_size, dt);

		  //cout << "start: " << start << endl;
		  //cout << "end: " << end << endl;
		  
//		  if (s_traj.size() > 3)
			//insert few elements from the end of previous path
//    			traj.insert(traj.begin(), s_traj.end() - 3, s_traj.end());

		  //costs vector format: {a_cost, v_cost, jerk_cost}
		  vector<double> costs = calc_cost_vec(traj, ACCEL_LIMIT, SPEED_LIMIT, dt);
		  //cout << "costs: " << costs << endl;
		  double cost = costs[2];
		  
		  if (costs[0] == 0 &&	//max acceleration 
  			  costs[1] == 0 &&	//max speed
  			  cost < min_cost) 
      {
    			min_cost = cost;
    			best_poly = poly;
    			best_end = end;
    			best_T = target_T; 
  		}
	  }
	}

	cout << "best_end: " << best_end << ", T: " << best_T << endl;

	//cout << "best_traj: " << best_traj << endl;
	cout << "best_poly = " << best_poly << endl;


	//lane change:
	vector<double> d_start = d_eop_state;
	double target_d =  (target_lane*4 + 2);
	vector<double> d_end =	 {target_d, 		 0, 	   0};
	double lane_change_time = 2; 
	//cout << "target_d: " << target_d << endl;
	//cout << "target_lane: " << target_lane << endl;
	//cout << "d_start: " << d_start << endl;
	//cout << "d_end: " << d_end << endl;
	vector<double> d_poly = JMT(d_start, d_end, lane_change_time);

  //Generate the s, d, x, and y values of the trajectory
	for(int i = 1; i < len+1; i++)
	{
		double s = polyeval(best_poly, i*dt);
		double d = polyeval(d_poly, i*dt);
		vector<double> xy = getXY(s, d, map.map_waypoints_s, map.map_waypoints_x, map.map_waypoints_y);
		double x_next = xy[0];
		double y_next = xy[1];
		traj.s.push_back(s);
		traj.d.push_back(d);
		traj.x.push_back(x_next);
		traj.y.push_back(y_next);
	}

	return traj;
}
