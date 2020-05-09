/*
 * referencepath.cpp
 *
 *  Created on: May 9, 2020
 *      Author: danyz
 */

#include "reference_path.h"

ReferencePath::ReferencePath() {}

ReferencePath::ReferencePath(std::vector<double> anchor_x,
                             std::vector<double> anchor_y) {
  anchor_points_x = anchor_x;
  anchor_points_y = anchor_y;
  size = anchor_x.size();
}

ReferencePath::ReferencePath(std::vector<double> map_wp_x,
                             std::vector<double> map_wp_y,
                             std::vector<double> map_wp_s) {
  map_waypoints_x = map_wp_x;
  map_waypoints_y = map_wp_y;
  map_waypoints_s = map_wp_s;
  size = 0;
}

ReferencePath::~ReferencePath() {
  // TODO Auto-generated destructor stub
}

void ReferencePath::computeReferencePath(ReferencePath previous_path,
                                         Vehicle& vehicle, int target_lane) {
  // ######################################################
  // PATH PLANNING - REFERENCE PATH BUILDING
  // ######################################################
  // Create a list of widely spaced (x,y) waypoints evenly spaced at 30m
  // LAter we will interpolate these waypoints with a spline and
  // Fill it with more points that control

  std::cout << "Ego: " << vehicle.to_string() << std::endl;

  // Reference x,y, yaw status
  double ref_x = vehicle.x;
  double ref_y = vehicle.y;
  double ref_yaw = vehicle.yaw;

  int prev_size = previous_path.size;

  // If previous state is almost empty, use the car as starting reference
  if (prev_size < 2) {
    // Use two points that make the path tangents to the car
    // Go backward in time using angle
    double prev_car_x = vehicle.x - cos(vehicle.yaw);
    double prev_car_y = vehicle.y - sin(vehicle.yaw);

    anchor_points_x.push_back(prev_car_x);
    anchor_points_y.push_back(prev_car_y);

    anchor_points_x.push_back(vehicle.x);
    anchor_points_y.push_back(vehicle.y);

  } else { /* Use the previous path's and point as reference*/

    // Redefine reference state as previous path end point
    vehicle.x = previous_path.anchor_points_x[prev_size - 1];
    vehicle.y = previous_path.anchor_points_y[prev_size - 1];

    double ref_x_prev = previous_path.anchor_points_x[prev_size - 2];
    double ref_y_prev = previous_path.anchor_points_y[prev_size - 2];

    vehicle.yaw = atan2(vehicle.y - ref_y_prev, vehicle.x - ref_x_prev);

    // Use two points to make the path tangent to
    // the previous path end point

    anchor_points_x.push_back(ref_x_prev);
    anchor_points_y.push_back(ref_y_prev);

    anchor_points_x.push_back(vehicle.x);
    anchor_points_y.push_back(vehicle.y);
  }

  // In Frenet add evenly 30m spaced points ahead of
  // the starting reference
  vector<double> next_wp_0 =
      getXY(vehicle.s + 30, (CENTER_LANE + LANE_WIDTH * target_lane),
            map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp_1 =
      getXY(vehicle.s + 60, (CENTER_LANE + LANE_WIDTH * target_lane),
            map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp_2 =
      getXY(vehicle.s + 90, (CENTER_LANE + LANE_WIDTH * target_lane),
            map_waypoints_s, map_waypoints_x, map_waypoints_y);

  anchor_points_x.push_back(next_wp_0[0]);
  anchor_points_x.push_back(next_wp_1[0]);
  anchor_points_x.push_back(next_wp_2[0]);

  anchor_points_y.push_back(next_wp_0[1]);
  anchor_points_y.push_back(next_wp_1[1]);
  anchor_points_y.push_back(next_wp_2[1]);

  // Transfrom anchor to car vehicle reference frame
  for (int i = 0; i < anchor_points_x.size(); i++) {
    // shift curr point to vehicle ref frame
    double shift_x = anchor_points_x[i] - vehicle.x;
    double shift_y = anchor_points_y[i] - vehicle.y;
    double shift_yaw = 0.0 - vehicle.yaw;

    anchor_points_x[i] = (shift_x * cos(shift_yaw) - shift_y * sin(shift_yaw));
    anchor_points_y[i] = (shift_x * sin(shift_yaw) + shift_y * cos(shift_yaw));
  }

  size = anchor_points_x.size();
}
