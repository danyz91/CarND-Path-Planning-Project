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
  double vx;
  double vy;

  std::string to_string();
};

#endif /* VEHICLE_H_ */
