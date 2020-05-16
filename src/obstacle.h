/*
 * obstacle.h
 *
 *  Created on: May 9, 2020
 *      Author: danyz
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include "vehicle.h"

class Obstacle : public Vehicle {
 public:
  Obstacle(int id, double x, double y, double s, double d, double vx,
           double vy);

  Obstacle();
  virtual ~Obstacle();

  int id;
};

#endif /* OBSTACLE_H_ */
