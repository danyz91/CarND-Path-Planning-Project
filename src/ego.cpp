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
  this->a = 0.0;
  this->state = "CS";
  this->max_acceleration = -1.0;
  previous_path_end_s = 0.0;
  previous_path_end_d = 0.0;

  target_speed = 0.0;
  max_speed = DEFAULT_MAX_SPEED;
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

// std::vector<string> EgoVehicle::successor_states() {
//  // Provides the possible next states given the current state for the FSM
//  //   discussed in the course, with the exception that lane changes happen
//  //   instantaneously, so LCL and LCR can only transition back to KL.
//  vector<string> states;
//  states.push_back("KL");
//  string state = this->state;
//  if (state.compare("KL") == 0) {
//    if (lane != LEFT_LANE) {
//      states.push_back("LCL");
//    }
//    if (lane != RIGHT_LANE) {
//      states.push_back("LCR");
//    }
//  }
//
//  // If state is "LCL" or "LCR", then just return "KL"
//  return states;
//}
//

bool EgoVehicle::is_ahead(Obstacle& obs) { return obs.s > s; }
bool EgoVehicle::is_behind(Obstacle& obs) { return obs.s < s; }
bool EgoVehicle::is_whitin_safety_gap(Obstacle& obs, double gap) {
  return fabs(obs.s - s) < gap;
}

bool EgoVehicle::get_vehicle_behind(std::vector<Obstacle>& obstacles, Lane lane,
                                    Obstacle& found_obstacle) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is
  //   found.

  double min_s = std::numeric_limits<double>::max();
  bool found_vehicle = false;
  for (Obstacle obs : obstacles) {
    if (obs.lane == lane && is_behind(obs) && obs.s < min_s) {
      min_s = obs.s;
      found_vehicle = true;
      found_obstacle = obs;
    }
  }

  return found_vehicle;
}

bool EgoVehicle::get_vehicle_ahead(std::vector<Obstacle>& obstacles, Lane lane,
                                   Obstacle& found_obstacle) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is
  //   found.

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

  std::cout << " Other lane -> front_lc_found:" << front_lc_found
            << " rear_lc_found: " << rear_lc_found << std::endl;

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

bool EgoVehicle::handleLeftLane(std::vector<Obstacle>& obstacles,
                                Lane& target_lane) {
  Obstacle leader;
  bool leader_found = get_vehicle_ahead(obstacles, LEFT_LANE, leader);
  bool too_close =
      leader_found && is_whitin_safety_gap(leader, DEFAULT_GAP_FRONT);

  target_lane = LEFT_LANE;
  if (too_close) {
    if (evaluateLaneChange(obstacles, CENTER_LANE)) {
      target_lane = CENTER_LANE;
    }
  } else {
    target_lane = LEFT_LANE;
  }

  return too_close;
}

bool EgoVehicle::handleRightLane(std::vector<Obstacle>& obstacles,
                                 Lane& target_lane) {
  Obstacle leader;
  bool leader_found = get_vehicle_ahead(obstacles, RIGHT_LANE, leader);
  bool too_close =
      leader_found && is_whitin_safety_gap(leader, DEFAULT_GAP_FRONT);

  target_lane = RIGHT_LANE;
  if (too_close) {
    if (evaluateLaneChange(obstacles, CENTER_LANE)) {
      target_lane = CENTER_LANE;
    }
  } else {
    target_lane = RIGHT_LANE;
  }

  return too_close;
}

bool EgoVehicle::handleCenterLane(std::vector<Obstacle>& obstacles,
                                  Lane& target_lane) {
  Obstacle leader;
  bool leader_found = get_vehicle_ahead(obstacles, CENTER_LANE, leader);

  bool too_close =
      leader_found && is_whitin_safety_gap(leader, DEFAULT_GAP_FRONT);

  target_lane = CENTER_LANE;
  bool change_already_selected = false;
  if (too_close) {
    if (evaluateLaneChange(obstacles, LEFT_LANE)) {
      target_lane = LEFT_LANE;
      change_already_selected = true;
    }
    if (!change_already_selected && evaluateLaneChange(obstacles, RIGHT_LANE)) {
      target_lane = RIGHT_LANE;
    }
  } else {
    target_lane = CENTER_LANE;
  }

  return too_close;
}

void EgoVehicle::selectBehavior(std::vector<Obstacle>& predicted_obstacles) {
  if (previous_reference_path.size > 0) {
    // Update car s with last point in path
    s = previous_path_end_s;
  }

  bool too_close = false;

  switch (lane) {
    case LEFT_LANE:
      std::cout << "I am in the left lane" << std::endl;
      too_close = handleLeftLane(predicted_obstacles, target_lane);
      std::cout << "too close: " << too_close
                << " target lane : " << target_lane << std::endl;
      break;
    case CENTER_LANE:
      std::cout << "I am in the center lane" << std::endl;

      too_close = handleCenterLane(predicted_obstacles, target_lane);
      std::cout << "too close: " << too_close
                << " target lane : " << target_lane << std::endl;
      break;
    case RIGHT_LANE:
      std::cout << "I am in the right lane" << std::endl;
      too_close = handleRightLane(predicted_obstacles, target_lane);
      std::cout << "too close: " << too_close
                << " target lane : " << target_lane << std::endl;
      break;
  }

  double safety_gap = 30.0;

  if (too_close) {
    target_speed -= .224;  // 5m/s^2
  } else if (target_speed < max_speed) {
    target_speed += .224;
  }
}
