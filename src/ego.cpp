#include "ego.h"

#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "cost.h"

EgoVehicle::EgoVehicle(double x, double y, double s, double d, double yaw,
                       double speed, int lane, float a, std::string state) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;
  this->vx = speed * cos(yaw);
  this->vy = speed * sin(yaw);
  this->lane = lane;
  this->a = a;
  this->state = state;
  this->max_acceleration = -1.0;
}

EgoVehicle::~EgoVehicle() {}

//
// vector<EgoVehicle> EgoVehicle::choose_next_state(
//    map<int, vector<EgoVehicle>> &predictions) {
//  /**
//   * Here you can implement the transition_function code from the Behavior
//   *   Planning Pseudocode classroom concept.
//   *
//   * @param A predictions map. This is a map of EgoVehicle id keys with
//   * predicted EgoVehicle trajectories as values. Trajectories are a vector of
//   * EgoVehicle objects representing the EgoVehicle at the current timestep
//   and
//   * one timestep in the future.
//   * @output The best (lowest cost) trajectory corresponding to the next ego
//   *   EgoVehicle state.
//   *
//   * Functions that will be useful:
//   * 1. successor_states - Uses the current state to return a vector of
//   possible
//   *    successor states for the finite state machine.
//   * 2. generate_trajectory - Returns a vector of EgoVehicle objects
//   * representing a EgoVehicle trajectory, given a state and predictions. Note
//   * that trajectory vectors might have size 0 if no possible trajectory
//   exists
//   *    for the state.
//   * 3. calculate_cost - Included from cost.cpp, computes the cost for a
//   * trajectory.
//   *
//   * TODO: Your solution here.
//   */
//  vector<string> states = successor_states();
//  float cost;
//  vector<float> costs;
//  vector<vector<EgoVehicle>> final_trajectories;
//
//  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
//  {
//    vector<EgoVehicle> trajectory = generate_trajectory(*it, predictions);
//    if (trajectory.size() != 0) {
//      cost = calculate_cost(*this, predictions, trajectory);
//      costs.push_back(cost);
//      final_trajectories.push_back(trajectory);
//    }
//  }
//
//  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
//  int best_idx = distance(begin(costs), best_cost);
//
//  /**
//   * TODO: Change return value here:
//   */
//  return final_trajectories[best_idx];
//}
//
// vector<string> EgoVehicle::successor_states() {
//  // Provides the possible next states given the current state for the FSM
//  //   discussed in the course, with the exception that lane changes happen
//  //   instantaneously, so LCL and LCR can only transition back to KL.
//  vector<string> states;
//  states.push_back("KL");
//  string state = this->state;
//  if (state.compare("KL") == 0) {
//    states.push_back("PLCL");
//    states.push_back("PLCR");
//  } else if (state.compare("PLCL") == 0) {
//    if (lane != lanes_available - 1) {
//      states.push_back("PLCL");
//      states.push_back("LCL");
//    }
//  } else if (state.compare("PLCR") == 0) {
//    if (lane != 0) {
//      states.push_back("PLCR");
//      states.push_back("LCR");
//    }
//  }
//
//  // If state is "LCL" or "LCR", then just return "KL"
//  return states;
//}
//
// vector<EgoVehicle> EgoVehicle::generate_trajectory(
//    string state, map<int, vector<EgoVehicle>> &predictions) {
//  // Given a possible next state, generate the appropriate trajectory to
//  realize
//  //   the next state.
//  vector<EgoVehicle> trajectory;
//  if (state.compare("CS") == 0) {
//    trajectory = constant_speed_trajectory();
//  } else if (state.compare("KL") == 0) {
//    trajectory = keep_lane_trajectory(predictions);
//  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
//    trajectory = lane_change_trajectory(state, predictions);
//  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
//    trajectory = prep_lane_change_trajectory(state, predictions);
//  }
//
//  return trajectory;
//}
//
// vector<float> EgoVehicle::get_kinematics(
//    map<int, vector<EgoVehicle>> &predictions, int lane) {
//  // Gets next timestep kinematics (position, velocity, acceleration)
//  //   for a given lane. Tries to choose the maximum velocity and
//  acceleration,
//  //   given other EgoVehicle positions and accel/velocity constraints.
//  float max_velocity_accel_limit = this->max_acceleration + this->v;
//  float new_position;
//  float new_velocity;
//  float new_accel;
//  EgoVehicle EgoVehicle_ahead;
//  EgoVehicle EgoVehicle_behind;
//
//  if (get_EgoVehicle_ahead(predictions, lane, EgoVehicle_ahead)) {
//    if (get_EgoVehicle_behind(predictions, lane, EgoVehicle_behind)) {
//      // must travel at the speed of traffic, regardless of preferred buffer
//      new_velocity = EgoVehicle_ahead.v;
//    } else {
//      float max_velocity_in_front =
//          (EgoVehicle_ahead.s - this->s - this->preferred_buffer) +
//          EgoVehicle_ahead.v - 0.5 * (this->a);
//      new_velocity =
//          std::min(std::min(max_velocity_in_front, max_velocity_accel_limit),
//                   this->target_speed);
//    }
//  } else {
//    new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
//  }
//
//  new_accel = new_velocity - this->v;  // Equation: (v_1 - v_0)/t =
//  acceleration new_position = this->s + new_velocity + new_accel / 2.0;
//
//  return {new_position, new_velocity, new_accel};
//}
//
// vector<EgoVehicle> EgoVehicle::constant_speed_trajectory() {
//  // Generate a constant speed trajectory.
//  float next_pos = position_at(1);
//  vector<EgoVehicle> trajectory = {
//      EgoVehicle(this->lane, this->s, this->v, this->a, this->state),
//      EgoVehicle(this->lane, next_pos, this->v, 0, this->state)};
//  return trajectory;
//}
//
// vector<EgoVehicle> EgoVehicle::keep_lane_trajectory(
//    map<int, vector<EgoVehicle>> &predictions) {
//  // Generate a keep lane trajectory.
//  vector<EgoVehicle> trajectory = {
//      EgoVehicle(lane, this->s, this->v, this->a, state)};
//  vector<float> kinematics = get_kinematics(predictions, this->lane);
//  float new_s = kinematics[0];
//  float new_v = kinematics[1];
//  float new_a = kinematics[2];
//  trajectory.push_back(EgoVehicle(this->lane, new_s, new_v, new_a, "KL"));
//
//  return trajectory;
//}
//
// vector<EgoVehicle> EgoVehicle::prep_lane_change_trajectory(
//    string state, map<int, vector<EgoVehicle>> &predictions) {
//  // Generate a trajectory preparing for a lane change.
//  float new_s;
//  float new_v;
//  float new_a;
//  EgoVehicle EgoVehicle_behind;
//  int new_lane = this->lane + lane_direction[state];
//  vector<EgoVehicle> trajectory = {
//      EgoVehicle(this->lane, this->s, this->v, this->a, this->state)};
//  vector<float> curr_lane_new_kinematics =
//      get_kinematics(predictions, this->lane);
//
//  if (get_EgoVehicle_behind(predictions, this->lane, EgoVehicle_behind)) {
//    // Keep speed of current lane so as not to collide with car behind.
//    new_s = curr_lane_new_kinematics[0];
//    new_v = curr_lane_new_kinematics[1];
//    new_a = curr_lane_new_kinematics[2];
//  } else {
//    vector<float> best_kinematics;
//    vector<float> next_lane_new_kinematics =
//        get_kinematics(predictions, new_lane);
//    // Choose kinematics with lowest velocity.
//    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
//      best_kinematics = next_lane_new_kinematics;
//    } else {
//      best_kinematics = curr_lane_new_kinematics;
//    }
//    new_s = best_kinematics[0];
//    new_v = best_kinematics[1];
//    new_a = best_kinematics[2];
//  }
//
//  trajectory.push_back(EgoVehicle(this->lane, new_s, new_v, new_a, state));
//
//  return trajectory;
//}
//
// vector<EgoVehicle> EgoVehicle::lane_change_trajectory(
//    string state, map<int, vector<EgoVehicle>> &predictions) {
//  // Generate a lane change trajectory.
//  int new_lane = this->lane + lane_direction[state];
//  vector<EgoVehicle> trajectory;
//  EgoVehicle next_lane_EgoVehicle;
//  // Check if a lane change is possible (check if another EgoVehicle occupies
//  //   that spot).
//  for (map<int, vector<EgoVehicle>>::iterator it = predictions.begin();
//       it != predictions.end(); ++it) {
//    next_lane_EgoVehicle = it->second[0];
//    if (next_lane_EgoVehicle.s == this->s &&
//        next_lane_EgoVehicle.lane == new_lane) {
//      // If lane change is not possible, return empty trajectory.
//      return trajectory;
//    }
//  }
//  trajectory.push_back(
//      EgoVehicle(this->lane, this->s, this->v, this->a, this->state));
//  vector<float> kinematics = get_kinematics(predictions, new_lane);
//  trajectory.push_back(
//      EgoVehicle(new_lane, kinematics[0], kinematics[1], kinematics[2],
//      state));
//  return trajectory;
//}
//
// void EgoVehicle::increment(int dt = 1) { this->s = position_at(dt); }
//
// float EgoVehicle::position_at(int t) {
//  return this->s + this->v * t + this->a * t * t / 2.0;
//}
//
// bool EgoVehicle::get_EgoVehicle_behind(
//    map<int, vector<EgoVehicle>> &predictions, int lane,
//    EgoVehicle &rEgoVehicle) {
//  // Returns a true if a EgoVehicle is found behind the current EgoVehicle,
//  // false
//  //   otherwise. The passed reference rEgoVehicle is updated if a EgoVehicle
//  is
//  //   found.
//  int max_s = -1;
//  bool found_EgoVehicle = false;
//  EgoVehicle temp_EgoVehicle;
//  for (map<int, vector<EgoVehicle>>::iterator it = predictions.begin();
//       it != predictions.end(); ++it) {
//    temp_EgoVehicle = it->second[0];
//    if (temp_EgoVehicle.lane == this->lane && temp_EgoVehicle.s < this->s &&
//        temp_EgoVehicle.s > max_s) {
//      max_s = temp_EgoVehicle.s;
//      rEgoVehicle = temp_EgoVehicle;
//      found_EgoVehicle = true;
//    }
//  }
//
//  return found_EgoVehicle;
//}
//
// bool EgoVehicle::get_EgoVehicle_ahead(map<int, vector<EgoVehicle>>
// &predictions,
//                                      int lane, EgoVehicle &rEgoVehicle) {
//  // Returns a true if a EgoVehicle is found ahead of the current EgoVehicle,
//  // false
//  //   otherwise. The passed reference rEgoVehicle is updated if a EgoVehicle
//  is
//  //   found.
//  int min_s = this->goal_s;
//  bool found_EgoVehicle = false;
//  EgoVehicle temp_EgoVehicle;
//  for (map<int, vector<EgoVehicle>>::iterator it = predictions.begin();
//       it != predictions.end(); ++it) {
//    temp_EgoVehicle = it->second[0];
//    if (temp_EgoVehicle.lane == this->lane && temp_EgoVehicle.s > this->s &&
//        temp_EgoVehicle.s < min_s) {
//      min_s = temp_EgoVehicle.s;
//      rEgoVehicle = temp_EgoVehicle;
//      found_EgoVehicle = true;
//    }
//  }
//
//  return found_EgoVehicle;
//}
//
// vector<EgoVehicle> EgoVehicle::generate_predictions(int horizon) {
//  // Generates predictions for non-ego EgoVehicles to be used in trajectory
//  //   generation for the ego EgoVehicle.
//  vector<EgoVehicle> predictions;
//  for (int i = 0; i < horizon; ++i) {
//    float next_s = position_at(i);
//    float next_v = 0;
//    if (i < horizon - 1) {
//      next_v = position_at(i + 1) - s;
//    }
//    predictions.push_back(EgoVehicle(this->lane, next_s, next_v, 0));
//  }
//
//  return predictions;
//}
//
// void EgoVehicle::realize_next_state(vector<EgoVehicle> &trajectory) {
//  // Sets state and kinematics for ego EgoVehicle using the last state of the
//  // trajectory.
//  EgoVehicle next_state = trajectory[1];
//  this->state = next_state.state;
//  this->lane = next_state.lane;
//  this->s = next_state.s;
//  this->v = next_state.v;
//  this->a = next_state.a;
//}
//
// void EgoVehicle::configure(vector<int> &road_data) {
//  // Called by simulator before simulation begins. Sets various parameters
//  which
//  //   will impact the ego EgoVehicle.
//  target_speed = road_data[0];
//  lanes_available = road_data[1];
//  goal_s = road_data[2];
//  goal_lane = road_data[3];
//  max_acceleration = road_data[4];
//}
