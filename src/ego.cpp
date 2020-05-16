#include "ego.h"

#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "obstacle.h"

EgoVehicle::EgoVehicle() {
  this->x = 0.0;
  this->y = 0.0;
  this->s = 0.0;
  this->d = 0.0;
  this->yaw = 0.0;
  this->speed = 0.0;
  this->vx = 0.0;
  this->vy = 0.0;
  this->lane = CENTER_LANE;
  previous_path_end_s = 0.0;
  previous_path_end_d = 0.0;

  target_speed = 0.0;
  max_speed = DEFAULT_MAX_SPEED_MPH;
  target_lane = lane;
}

void EgoVehicle::update(double x, double y, double s, double d, double yaw,
                        double speed, ReferencePath prev_ref_path,
                        double prev_end_s, double prev_end_d) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;
  this->vx = speed * cos(yaw);
  this->vy = speed * sin(yaw);
  this->lane = Vehicle::associate_vehicle(d);
  previous_reference_path = prev_ref_path;
  previous_path_end_s = prev_end_s;
  previous_path_end_d = prev_end_d;
}

EgoVehicle::~EgoVehicle() {}

bool EgoVehicle::is_ahead(Obstacle& obs) { return obs.s > s; }
bool EgoVehicle::is_behind(Obstacle& obs) { return obs.s < s; }
bool EgoVehicle::is_whitin_safety_gap(Obstacle& obs, double gap) {
  return fabs(obs.s - s) < gap;
}

bool EgoVehicle::get_vehicle_behind(std::vector<Obstacle>& obstacles, Lane lane,
                                    Obstacle& found_obstacle) {
  double min_s = 0.0;
  bool found_vehicle = false;
  for (Obstacle obs : obstacles) {
    if (obs.lane == lane && is_behind(obs) && obs.s > min_s) {
      min_s = obs.s;
      found_vehicle = true;
      found_obstacle = obs;
    }
  }

  return found_vehicle;
}

bool EgoVehicle::get_vehicle_ahead(std::vector<Obstacle>& obstacles, Lane lane,
                                   Obstacle& found_obstacle) {
  double min_s = std::numeric_limits<double>::max();
  bool found_vehicle = false;
  for (Obstacle obs : obstacles) {
    if (obs.lane == lane && is_ahead(obs) && obs.s < min_s) {
      min_s = obs.s;
      found_vehicle = true;
      found_obstacle = obs;
    }
  }

  return found_vehicle;
}

bool EgoVehicle::evaluateLaneChange(std::vector<Obstacle>& obstacles,
                                    Lane target_lane) {
  Obstacle side_front, side_back;
  bool front_lc_found = get_vehicle_ahead(obstacles, target_lane, side_front);
  bool rear_lc_found = get_vehicle_behind(obstacles, target_lane, side_back);

  if (!front_lc_found && !rear_lc_found) {
    return true;
  } else if (front_lc_found && !rear_lc_found) {
    return !is_whitin_safety_gap(side_front, DEFAULT_GAP_FRONT);
  } else if (!front_lc_found && rear_lc_found) {
    return !is_whitin_safety_gap(side_back, DEFAULT_GAP_REAR);
  } else if (front_lc_found && rear_lc_found) {
    return !is_whitin_safety_gap(side_front, DEFAULT_GAP_FRONT) &&
           !is_whitin_safety_gap(side_back, DEFAULT_GAP_REAR);
  }
}

std::vector<Lane> EgoVehicle::get_adjacent_lanes(Lane lane) {
  if (lane == CENTER_LANE) {
    return {LEFT_LANE, RIGHT_LANE};
  } else if (lane == RIGHT_LANE) {
    return {CENTER_LANE};
  } else if (lane == LEFT_LANE) {
    return {CENTER_LANE};
  }

  return {};
}

void EgoVehicle::selectBehavior(std::vector<Obstacle>& predicted_obstacles) {
  if (previous_reference_path.size > 0) {
    // Update car s with last point in path
    s = previous_path_end_s;
  }

  bool too_close = false;

  Obstacle leader;
  bool leader_found = get_vehicle_ahead(predicted_obstacles, lane, leader);
  too_close = leader_found && is_whitin_safety_gap(leader, DEFAULT_GAP_FRONT);

  // Lane handling
  if (too_close) {
    for (Lane adjacent_lane : get_adjacent_lanes(lane)) {
      if (evaluateLaneChange(predicted_obstacles, adjacent_lane)) {
        target_lane = adjacent_lane;
        break;
      }
    }

  } else {
    target_lane = lane;
  }

  if (too_close) {
    target_speed -= DELTA_MPH_FROM_MS2(DESIRED_ACCELERATION);  // 5m/s^2
  } else if (target_speed < max_speed) {
    target_speed += DELTA_MPH_FROM_MS2(DESIRED_ACCELERATION);
  }
}
