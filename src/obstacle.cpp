/*
 * obstacle.cpp
 *
 *  Created on: May 9, 2020
 *      Author: danyz
 */

#include "obstacle.h"

Obstacle::Obstacle() {
  // TODO Auto-generated constructor stub
}

Obstacle::Obstacle(int id, double x, double y, double s, double d, double vx,
                   double vy) {
  this->id = id;
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->vx = vx;
  this->vy = vy;
  this->speed = sqrt((vx * vx) + (vy * vy));
  this->yaw = atan2(y, x);
}

Obstacle::~Obstacle() {
  // TODO Auto-generated destructor stub
}
