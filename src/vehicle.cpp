/*
 * vehicle.cpp
 *
 *  Created on: May 9, 2020
 *      Author: danyz
 */

#include "vehicle.h"

Vehicle::Vehicle() {
  // TODO Auto-generated constructor stub
}
Vehicle::~Vehicle() {
  // TODO Auto-generated destructor stub
}

std::string Vehicle::to_string() {
  std::string res = "";

  res += "X: " + std::to_string(x) + "\n";
  res += "Y: " + std::to_string(y) + "\n";
  res += "S: " + std::to_string(s) + "\n";
  res += "D: " + std::to_string(d) + "\n";
  res += "Yaw: " + std::to_string(yaw) + "\n";
  res += "Speed: " + std::to_string(speed) + "\n";
  res += "Vx: " + std::to_string(vx) + "\n";
  res += "Vy: " + std::to_string(vy) + "\n";
  res += "Lane: " + std::to_string(lane) + "\n";

  return res;
}
