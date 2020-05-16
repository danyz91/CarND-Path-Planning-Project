#ifndef EGO_H
#define EGO_H

#include <map>
#include <string>
#include <vector>

#include "helpers.h"
#include "obstacle.h"
#include "reference_path.h"
#include "vehicle.h"

#define DEFAULT_MAX_SPEED_MPH 49.5  // mph
#define DEFAULT_GAP_FRONT 30.0      // m
#define DEFAULT_GAP_REAR 30.0       // m
#define DESIRED_ACCELERATION 5      // m/s^2

class EgoVehicle : public Vehicle {
 public:
  // Constructors
  EgoVehicle();
  // Destructor
  virtual ~EgoVehicle();
  void update(double x, double y, double s, double d, double yaw, double speed,
              ReferencePath prev_ref_path, double prev_end_s,
              double prev_end_d);

  /**
   * The behavior planner function. It set up target_speed and target_lane
   * attributes of the class besing on predicted obstacles state
   */
  void selectBehavior(std::vector<Obstacle>& predicted_obstacles);

  /**
   * Returns true if passed vehicle is ahead of ego vehicle
   */
  bool is_ahead(Obstacle& obs);

  /**
   * Returns true if passed vehicle is behind ego vehicle
   */
  bool is_behind(Obstacle& obs);

  /**
   *  Returns a true if a vehicle is found behind of the current vehicle, false
   *  otherwise. The passed reference Obstacle is updated if a vehicle is
   *  found.
   */
  bool get_vehicle_behind(std::vector<Obstacle>& obstacles, Lane lane,
                          Obstacle& found_obstacle);

  /**
   * Returns a true if a vehicle is found ahead of the current vehicle,
   * false otherwise. The passed reference Obstacle is updated if a vehicle is
   * found.
   */
  bool get_vehicle_ahead(std::vector<Obstacle>& obstacles, Lane lane,
                         Obstacle& found_obstacle);

  void get_gap_per_lane(std::vector<Obstacle>& obstacles, Lane lane,
                        std::vector<Obstacle>& relevant_obstacles);

  /**
   * Returns true if passed vehicle is inside safety gap
   * Signaling it is too close to start maneuver
   */
  bool is_whitin_safety_gap(Obstacle& obs, double gap);

  /**
   * Returns true if given obstacles state it is possible to perform
   * a lane change to target_lane lane
   */
  bool evaluateLaneChange(std::vector<Obstacle>& obstacles, Lane target_lane);

  std::vector<Lane> get_adjacent_lanes(Lane lane);

  ReferencePath previous_reference_path;
  double previous_path_end_s;
  double previous_path_end_d;

  double target_speed;
  double max_speed;

  Lane target_lane;
};

#endif  // EGO_H
