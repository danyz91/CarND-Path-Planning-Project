/*
 * referencepath.h
 *
 *  Created on: May 9, 2020
 *      Author: danyz
 */

#ifndef REFERENCE_PATH_H_
#define REFERENCE_PATH_H_

#include <iostream>
#include <vector>

#include "helpers.h"
#include "vehicle.h"

class ReferencePath {
 public:
  ReferencePath();
  ReferencePath(std::vector<double> map_wp_x, std::vector<double> map_wp_y,
                std::vector<double> map_wp_s);
  ReferencePath(std::vector<double> anchor_x, std::vector<double> anchor_y);
  void computeReferencePath(ReferencePath previous_reference_path,
                            Vehicle& vehicle, int target_lane);

  virtual ~ReferencePath();
  const std::vector<double>& get_anchor_points_x() const;
  const std::vector<double>& get_anchor_points_y() const;

  std::vector<double> anchor_points_x;
  std::vector<double> anchor_points_y;
  int size;

  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
};

#endif /* REFERENCE_PATH_H_ */
