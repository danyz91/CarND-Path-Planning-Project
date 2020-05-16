[//]: # (Image References)

[image1]: ./img/bp2.png "Behavior Planner" 

# Path Planning Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This repository contains all the code for the final project for the Path Planning lessons in Udacity's Self-Driving Car Nanodegree.

## Overview

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 

The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.

The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Provided Input

There are provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. 

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the Frenet s value, distance along the road, goes from 0 to 6945.554.

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

## Installation & Run

This project involves the Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems : `install_ubuntu.sh` and `install_mac.sh`. 

For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

    mkdir build
    cd build
    cmake ..
    make
    ./path_planning
 
#### Notes

Details about simulation protocols are given in [this appendix file](./appendix_sim_details.md)

## Code Style

Code is compliant with [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

It is formatted following [clangformat specification](https://clang.llvm.org/docs/ClangFormat.html)


---

## Code & Algorithm

### Main Classes

The solution code is mainly divided in 4 classes:

1. **Prediction**: implemented in file `prediction.h`. 

  This class has the role of predicting future states of obstacles vehicle using previous computed path length as time horizon

2. **Behavior Planner**: implemented in files `ego.h` and `ego.cpp`

  This class has the role of selecting two elements that will be provided as input to path planning module: 

  * `target_speed`, the speed ego vehicle must reach
  * `target_lane`, destination lane for ego vehicle given current environment configuration

3. **Reference Path**: implemented in files `reference_path.h` and `reference_path.cpp`

  This class has the role of selecting points on the map basing on target lane selected by the behavior planner module. These points are called *anchor points* and specifies the line path planner needs to take into account when computing ego trajectory

4. **Path Planner**: implemented in files `path_planner.h` and `path_planner.cpp`

  This class, basing on reference path received from previous module and target speed selected by behavior module, needs to generate a trajectory in global reference frame to update ego vehicle position.


Such organization enhances isolation of functionalities for path planning problem in general.

For example, `ReferencePath` class defined as follows can be used also for generating path for obstacle vehicle. This could be useful for enhancing prediction algorithm.

### Main Algorithm 

The algorithm that these module follow in each simulation step is contained in **lines 99-157** of `main.cpp`.

Here it is possible to see how environment information flow.

1. Prediction module reads obstacles information and computed future vehicles position (**lines 112-134**).
2. Behavior Planner read updated ego info (**lines 107-109**) and invoke its `selectBehavior` method (**lines 136-138**). 
3. Reference Path digest previous path info read from the simulator (**lines 99-105**) and invoke its `computeReferencePath` routine (**lines 143-148**)
4. Path Planner computes next trajectory (**lines 150-151**)

### Vehicle Representation

Other three classes have been implemented to simplify vehicles information handling throughout the code. These classes are organized in a little class hierarchy.

1. `Vehicle`

  The parent class of hierarchy. Contains generic information about a vehicle (position, speed, lane association). This information is in common between obstacles and ego vehicle. This class is implemented in files **vehicle.h** and **vehicle.cpp**.

  The class also define a static method `associate_vehicle` that. This method classifies which of the three lanes the vehicle is basing on Frenet `d` value coming from the simulator.The method is defined at **lines 39-46**  of `vehicle.h`

2. `Ego`
  
  This class inherits from `Vehicle` and represents the ego vehicle state during the simulation. It specifies the behavior of the base class by implementing all the behavior planning routine. Since ego vehicle is also the only vehicle always present in the simulation it also define a `update` method to update vehicle info with latest data coming from the simulation. .The method is defined at **lines 29-44**  of `ego.h`

3. `Obstacle`

  This class inherits from `Vehicle` and represents an obstacle in the simulation. In the current implementation it is used to store position and velocity of the vehicle.

### Helpers

The last two files remaining in the repo contain helpers function used throughout the code. These files are `helpers.h` and `helpers.cpp`

These files contain some helper code provided during the course such as the Frenet coordinate system conversion from/to Cartesian coordinate system. (`getFrenet` and `getXY` respectively) and functions for map waypoint based localization (`ClosestWaypoint` and `NextWaypoint`).

This files also define the `Lane` enum used for lane identification

```cpp
enum Lane { LEFT_LANE = 0, CENTER_LANE = 1, RIGHT_LANE = 2, NUM_LANES = 3 };
```
and helper functions converting miles based quantities from/to meter based quantities.

### Prediction

Current prediction implementation updates longitudinal position of obstacles basing on their speed and specified time horizon.

The function that implements the prediction is contained in **lines 22-27** of `prediction.h` file.

During the prediction, the `s` attribute of `Obstacle` class is updated using the formula:

```cpp
obs.s += (time_horizon * SIMULATION_STEP_LENGTH * obs.speed)
```

where SIMULATION_STEP_LENGTH represent the fixed time step of the simulator, currently **20ms**

### Behavior Planning

Behavior Planner needs to generate correct maneuver basing on environment information taking into account some sort of optimization criteria.

General schema for behavior planner is shown below.

![alt text][image1]

The Behavior Planner implemented in `Ego` class is based on the same information as above, but without route info since we do not have a destination but we just have to drive endlessly on the highway:

- Map: represented by previous Reference Path
- Predictions: the `Obstacle` objects with their updated predicted position

The `selectBehavior` method in `ego.cpp`, that implements the behavior planning is based on some sub-routines:

- `get_vehicle_ahead` (**lines 69-82**): This function finds and returns the nearest vehicle ahead ego vehicle on a given lane. This is used to check if there is a slow running vehicle that is blocking ego vehicle, by using the method ``is_whitin_safety_gap`. If the nearest obstacle is within a certain range, then we can consider to change lane. 
- `get_vehicle_behind` (**lines 54-67**): the dual of the previous method. It searches for first vehicle behind ego vehicle on a given lane. 
- `get_adjacent_lanes` (**lines 50-52**): This function returns the vector of adjacent lanes for each possible lanes. It gives the "neighbors" of a given lane.
- `evaluateLaneChange` (**lines 84-100**): This function checks if it is safe to perform a lane change on a given lane. This function searches for if there is a vehicle ahead and behind, using ego current `s` attribute and the `d` attribute corresponding to the center of the given lane. Than if the vehicles are present, it is checked if there is at least a safety gap between them and ego vehicle. This safety gap is currently fixed at 30 m. 

Once target lane is chosen basing on the subroutine above, target speed needs to be set. This is done in **lines 140-144**. Here a defined acceleration of 5 m/s^2 is used to adjust vehicle speed if it is needed to slow down for a vehicle in front of ego or speed up if it is needed and it is safe to approach maximum speed of 50 MPH.


### Reference Path

Reference Path building routine is implemented in `computeReferencePath` method at **lines 32-114** of `reference_path.cpp` file.

Reference path is computed w.r.t. vehicle parameter that needs a path to the passed target lane. 

The output of this function is populating the `anchor_points_x` and `anchor_points_y` attributes of `ReferencePath` class. These points will be the reference point for path planning.

The algorithm here implemented is basically divided in three parts:

1. **Anchor Starting reference** : It is needed to start from vehicle position. This section if previous state is almost empty, uses the car as starting reference, otherwise use the previous path's and point. This section is implemented in **lines 48-80**
2. **Anchor extraction**: Once starting reference is computed, it is needed to take reference point at given distance to it. In this section, using `s` and `d` parameter of passed vehicle and `target lane`, there are selected three points, far 30, 60 and 90 m from starting reference. This section is implemented in **lines 82-100**
3. **Rotation to vehicle reference frame**: Once reference points are computed, it is needed to transform them to vehicle reference frame. Points are first translated to vehicle position and then rotated basing on its heading. This section is implemented in **lines 102-113**

### Path Planner

Path planner has the role of update vehicle trajectory.

Path planner needs to respect the reference line computed by Reference Path module generating a trajectory that respects `target_speed` selected by the Behavior Planner.

Path planning is based on [Spline Interpolation](https://en.wikipedia.org/wiki/Spline_interpolation).

For implementation, [spline library](https://kluge.in-chemnitz.de/opensource/spline/) is used. This library is particularly useful and it is also packed in just one header file contained in the repository.

Path planning main routine is contained in `plan` function at **lines 28-82** of `path_planning.h` file.

Here spline interpolation is firstly computed using passed anchor points.

Then it is determined how many points it is needed to generate given on previous path info.

The points to be generated are selected using geometric relations implemented at **lines 61-64**. 

Taking a reference point x,y point using spline interpolation, we compute its norm. Then step size on x coordinate is computed basing on desired speed and simulation step length. Once x coordinate is known for each point, spline library is used to compute corresponding y value.

Here it is important to note that anchor points are in vehicle reference frame. So in order to generate a trajectory with coordinates in global reference frame, another transformation is needed. Now it is needed to rotate and then translate points. This transformation is implemented in **lines 71-77**.


## Result

The submitted program is able to run on the highway path without hitting other cars and violating any comfort constraints

[![final](https://img.youtube.com/vi/AQ2UWnsamWw/0.jpg)](https://www.youtube.com/watch?v=AQ2UWnsamWw) 

An interesting situation in the video above is the one found at 00:35.

Here it is possible to see an obstacle car that is changing lane from the left to our lane. Here we can see that thanks to our prediction phase, that consider the vehicle future position, we can avoid the obstacle by anticipating the lane change maneuver.


## Open Points

There are some aspects that could be improved from the current solution

- **Prediction module**: Since prediction is now basically a simple kinematic prediction assuming constant speed, this step could be enhanced using some data-driven methods such as [Naive Bayes Classifiers](https://en.wikipedia.org/wiki/Naive_Bayes_classifier) to predict the future behavior of other vehicles
- **Behavior planning**: behavior control can be improved by defining some cost functions over different possible trajectories and select the action with minimum cost.
- Another enhancement of the behavior planner could be the usage of a dynamic safety distance, for example using [Mobileye Responsibility Sensitive Safety](https://www.mobileye.com/responsibility-sensitive-safety/) safety distance definition.


