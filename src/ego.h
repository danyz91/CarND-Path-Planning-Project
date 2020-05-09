#ifndef EGO_H
#define EGO_H

#include <map>
#include <string>
#include <vector>

#include "vehicle.h"

class EgoVehicle : public Vehicle {
 public:
  // Constructors
  EgoVehicle();
  EgoVehicle(double x, double y, double s, double d, double yaw, double speed,
             int lane, float a, std::string state = "CS");
  // Destructor
  virtual ~EgoVehicle();

 private:
  int lane;

  double a, max_acceleration;

  std::string state;


  /*
    // EgoVehicle functions
    vector<EgoVehicle> choose_next_state(
        map<int, vector<EgoVehicle>> &predictions);

    vector<string> successor_states();

    vector<EgoVehicle> generate_trajectory(
        string state, map<int, vector<EgoVehicle>> &predictions);

    vector<float> get_kinematics(map<int, vector<EgoVehicle>> &predictions,
                                 int lane);

    vector<EgoVehicle> constant_speed_trajectory();

    vector<EgoVehicle> keep_lane_trajectory(
        map<int, vector<EgoVehicle>> &predictions);

    vector<EgoVehicle> lane_change_trajectory(
        string state, map<int, vector<EgoVehicle>> &predictions);

    vector<EgoVehicle> prep_lane_change_trajectory(
        string state, map<int, vector<EgoVehicle>> &predictions);

    void increment(int dt);

    float position_at(int t);

    bool get_EgoVehicle_behind(map<int, vector<EgoVehicle>> &predictions,
                               int lane, EgoVehicle &rEgoVehicle);

    bool get_EgoVehicle_ahead(map<int, vector<EgoVehicle>> &predictions, int
    lane, EgoVehicle &rEgoVehicle);

    vector<EgoVehicle> generate_predictions(int horizon = 2);

    void realize_next_state(vector<EgoVehicle> &trajectory);

    void configure(vector<int> &road_data);

    // public EgoVehicle variables
    struct collider {
      bool collision;  // is there a collision?
      int time;        // time collision happens
    };

    map<string, int> lane_direction = {
        {"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

    int L = 1;

    int preferred_buffer = 6;  // impacts "keep lane" behavior.

    int lane, s, goal_lane, goal_s, lanes_available;

    float v, target_speed, a, max_acceleration;

    string state;
  */
};

#endif  // EGO_H
