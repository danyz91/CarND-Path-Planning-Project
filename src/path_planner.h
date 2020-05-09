/*
 * path_planner.h
 *
 *  Created on: May 9, 2020
 *      Author: danyz
 */

#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <math.h>

#include <vector>

#include "reference_path.h"
#include "spline.h"
#include "vehicle.h"

#define NUM_TRAJECTORY_POINTS 50

class PathPlanner {
 public:
  PathPlanner(Vehicle v, ReferencePath prev, ReferencePath curr);
  virtual ~PathPlanner();

  void plan(double target_speed, std::vector<double>& planning_x,
            std::vector<double>& planning_y);

  tk::spline s;
  ReferencePath previous_reference_path;
  ReferencePath reference_path;
  Vehicle vehicle;
};

#endif /* PATH_PLANNER_H_ */
