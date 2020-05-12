#ifndef EGO_H
#define EGO_H

#include <map>
#include <string>
#include <vector>

#include "helpers.h"
#include "obstacle.h"
#include "reference_path.h"
#include "vehicle.h"

#define DEFAULT_MAX_SPEED 49.5
#define DEFAULT_GAP_FRONT 30.0
#define DEFAULT_GAP_REAR 30.0

class EgoVehicle : public Vehicle {
 public:
  // Constructors
  EgoVehicle();
  // Destructor
  virtual ~EgoVehicle();
  void update(double x, double y, double s, double d, double yaw, double speed,
              ReferencePath prev_ref_path, double prev_end_s,
              double prev_end_d);
  void selectBehavior(std::vector<Obstacle>& predicted_obstacles);
  bool is_ahead(Obstacle& obs);
  bool is_behind(Obstacle& obs);

  bool get_vehicle_behind(std::vector<Obstacle>& obstacles, Lane lane,
                          Obstacle& found_obstacle);

  bool get_vehicle_ahead(std::vector<Obstacle>& obstacles, Lane lane,
                         Obstacle& found_obstacle);

  void get_gap_per_lane(std::vector<Obstacle>& obstacles, Lane lane,
                        std::vector<Obstacle>& relevant_obstacles);

  bool is_whitin_safety_gap(Obstacle& obs, double gap);

  bool evaluateLaneChange(std::vector<Obstacle>& obstacles, Lane target_lane);

  std::vector<Lane> get_adjacent_lanes(Lane lane);

  double rss_safety_distance(Vehicle rear, Vehicle front);

  double a, max_acceleration;

  std::string state;

  ReferencePath previous_reference_path;
  double previous_path_end_s;
  double previous_path_end_d;

  double target_speed;
  double max_speed;

  Lane target_lane;
};

#endif  // EGO_H
