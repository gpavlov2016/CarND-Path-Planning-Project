# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

# Model Documentation
The project is divided into two main modules: trajectory and behavior. The trajectory module is responsible of jenerating smooth trajectory that doesn't violate maximum acceleration, speed and jerk given target speed and target lane. The behavior planning module is responsible to choosing the optimal lane and speed.

##Main Pipeline
The function `onMessage()` in `main.cpp` is called every time a new telemetry arrived from the simulator over the socket interface. The incoming data contains the current car position in cartesian and Frenet coordinates as well as yaw, speed and previous unconsumed path points.  
There are three main steps in the main pipeline:
* Calculating the state (position, speed, acceleration) using the last points of the unconsumed path in Frenet coordinates space. This is done in function `calc_eop_state()` that receives the path and time duration between each point and returns a vector describing the state at the end of the path.  
* Trajectory generation given target lane and speed. This function makes sure to generate trajectory that does not violate constraints and minimizes jerk. This is done by calling the `gnerate_optimal_trajectory()` function.  
* Behavior planner is responsible to figure out what is the optimal lane and target speed for the next trajectory generation. This is done by calling the function `behavioral_planner()` with sensor fusion data (containing all the cars on the same side of the road) and current state at the end of the path.  

##Trajectory Generation
The trajectory generation related functions are located in the file `trajectory.cpp`. The process begins with the function `gnerate_optimal_trajectory()` that receives a lot of parameters from the `onMessage()` function and returns a trajectory in both Frenet and cartesian coordinates in a custom defined type `trajectory_t`:
```
typedef struct _trajectory_t {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> d;
} trajectory_t;
```
The process of generating trajectory can be viewed as divided into three parts:optimal polynomial search, d trajectory and conversion to cartesian.
###Optimal Polynomial
The core of this step is the `JMT` function that implements the algorithm from here: http://mplab.ucsd.edu/tutorials/minimumJerk.pdf to find a six degree polynomial that minimizes the jerk. It does not, though, ensures that the trajectory doesn't violate maximum acceleration or speed. Therefore we generate several trajectories varying the target s position and time during which the planning takes place, then we calculate a cost function for each of the trajectories and choose the minimal one. This planning is done only for the s component of the trajectory. 
###D trajectory
Fortunately here the distances are much shorter and do not require the search in parameter space that was done for the s component of the trajectory. The `JMT()` function is still used but with the target d position equal to the center of the target lane. The resulting polynomial then represents a valid d trajectory.
###Conversion to Cartesian
In this last step the final trajectory is generated for both s and d components using the best polynomial for s and the only one for d. During this process the s,d Frenet coordinates are converted to cartesian using the function `getXY()` from the file `utils.cpp`. Finaly a structure containing all four s,d,x and y coordinate vectors is constructed and returned. For more information about the conversion see separate section below.

##Behavioral Planner
This module is located in the file `behave.cpp` and starts execution when the function `behavioral_planner()` is called. This function receives the following input from the caller:
* Sensor fusion data which is information about other cars on the road.  
* Lane at the end of path
* s position of the car at the end of path
* How much time will pass from current moment to the end of already planned path
The output is a structure called `plan_t` that contains the optimal lane and speed starting the end of the planned path which is used as an input to the trajectory generation module in the next iteration.
The planning process itself is done in three stages - finding closest cars, calculating cost, validating feasibility.
###Closest Cars
This task is handled by the function `find_closest_cars()` located in the file `behave.cpp`. It receives the same parameters as `behavioral_planner()` function above and returns a stuct containig two vectors:
```
typedef struct _closest_cars_t {
  vector<int>  in_front;  //one car per lane in front 
  vector<int>  behind;    //one car per lane from behind
} closest_cars_t;
```
each one of the vectors represents the closest car index to the `sensor_fusion` array for each lane. If there are no cars on a specific lane it's value in `in_front` or `behid` arrays will be -1.
###Cost per Lane
The cost function contains the following elements:
* Square of the difference from the maximum speed allowed. This is applied only for the cars in front.
* Inverse of the square of the distance from the car. This is calculated for both in front and behind cars.
* Constant value for each lane excepts the lane that the car will be at the end of the path.
###Validating Feasivility
In this part we want to make sure that it is indeed feasible to jump to the lane with the miminum cost therefore the target lane is chose only from adjustant lanes to prevent jump over two lanes for example. As soon as the lane is chosen the speed is determined as a ratio of the speed of the leading car in that lane or the maximum speed allowed if there are not cars in front of us on the target lane. Both target lane and the speed are then returned to the caller, which is the fucntion `onMessage()` from `main.cpp`.

##Frenet to Cartesian Conversion
The conversion from Frenet to Cartesian coordinates is done using a sparse map containing the entries that denote the same point in both s,d Frenet coordinates and x,y cartesian coordinates. Since the resolution used by the trajectory generator is much higher than the resolution of the map a smoothing technique was required. Here a minimalistic implementation of spline, fully containted in a single `spline.h` file is being used. When the server is first invoked it calculates three splines in the function `init()` located in the file `utils.cpp`. 
```
  sx.set_points(maps_s, maps_x);    
  sy.set_points(maps_s, maps_y);    
  sh.set_points(maps_s, maps_h);    

```
Two of the splines map the s coordinate to x and y separetaly and the third maps the direction of the next point to s value. Those three splines are later used in the function `getXY()` to conver from s,d coordinates to x,y as follows:
```
double heading = sh(s);
double perp_heading = heading-pi()/2;
double x = sx(s) + d*cos(perp_heading);
double y = sy(s) + d*sin(perp_heading);
```

##Future Work
Here is a partial list of possible improvements to the algorithm:
* Add collision detection to the cost function of the behavior planner 
* Handle s value wraparound due to cyclical nature of the track
* Prevent lane change threshing by preventing lane change immidieately after another chane was done
* Implement smarter other cars trajectory prediction algorithm by using Naive Bayes for example
* Improve run time by using heuristics to limit the search space of the trajetory 
