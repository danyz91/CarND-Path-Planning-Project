/*
 * vehicle.h
 *
 *  Created on: May 9, 2020
 *      Author: danyz
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <math.h>

#include <string>

#include "helpers.h"

class Vehicle {
 public:
  Vehicle();  // pure virtual

  virtual ~Vehicle();

  double x;
  double y;
  double yaw;
  double s;
  double d;
  double speed;
  double vx;  // X axis component of velocity
  double vy;  // Y axis component of velocity
  Lane lane;

  std::string to_string();

  /**
   * The function associate current vehicle with one of possible lanes
   * basing on passed d value
   */
  static Lane associate_vehicle(double d) {
    for (int i = 0; i < NUM_LANES; i++) {
      if (is_on_lane(d, i)) {
        return static_cast<Lane>(i);
      }
    }
    return static_cast<Lane>(0);
  }
};

#endif /* VEHICLE_H_ */
