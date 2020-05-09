/*
 * path_planner.cpp
 *
 *  Created on: May 9, 2020
 *      Author: danyz
 */

#include "path_planner.h"

PathPlanner::PathPlanner(Vehicle v, ReferencePath prev, ReferencePath curr) {
  vehicle = v;
  previous_reference_path.anchor_points_x = prev.anchor_points_x;
  previous_reference_path.anchor_points_y = prev.anchor_points_y;
  previous_reference_path.size = prev.size;

  reference_path.anchor_points_x = curr.anchor_points_x;
  reference_path.anchor_points_y = curr.anchor_points_y;
  reference_path.map_waypoints_x = curr.map_waypoints_x;
  reference_path.map_waypoints_y = curr.map_waypoints_y;
  reference_path.map_waypoints_s = curr.map_waypoints_s;
  reference_path.size = curr.size;
}

PathPlanner::~PathPlanner() {
  // TODO Auto-generated destructor stub
}

void PathPlanner::plan(double target_speed, double target_acceleration,
                       std::vector<double>& planning_x,
                       std::vector<double>& planning_y) {
  // Setting up the spline points
  s.set_points(reference_path.anchor_points_x, reference_path.anchor_points_y);

  // All points in previous path are the points the car
  // still needs to cover
  // So copy it in the planning points
  for (int i = 0; i < previous_reference_path.size; i++) {
    planning_x.push_back(previous_reference_path.anchor_points_x[i]);
    planning_y.push_back(previous_reference_path.anchor_points_y[i]);
  }

  // Calculate how to break up spline points so that
  // we will be travel at desired speed

  // First compute norm of (s,d) vector of anchor point
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_norm = sqrt((target_x * target_x) + (target_y * target_y));

  // Equation is N_points * .02 * desired_velocity = norm
  // where N is the number of points we have and .02 is
  // how often the simulator will run

  double x_add_on = 0.0;

  // Populate the rest of our path planner after
  // filling it with previous points
  // here we will always output 50 points

  for (int i = 1; i <= NUM_TRAJECTORY_POINTS - previous_reference_path.size;
       i++) {
    double N = (target_norm /
                (SIMULATION_STEP_LENGTH * target_speed / target_acceleration));
    double x_point = x_add_on + (target_x / N);
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after have rotated
    // it before in car ref frame
    x_point = (x_ref * cos(vehicle.yaw) - y_ref * sin(vehicle.yaw));
    y_point = (x_ref * sin(vehicle.yaw) + y_ref * cos(vehicle.yaw));

    x_point += vehicle.x;
    y_point += vehicle.y;

    planning_x.push_back(x_point);
    planning_y.push_back(y_point);
  }
}
