/*
 * prediction.h
 *
 *  Created on: May 10, 2020
 *      Author: danyz
 */

#ifndef PREDICTION_H_
#define PREDICTION_H_

#include <iostream>
#include <vector>

#include "helpers.h"
#include "obstacle.h"

class Predictor {
 public:
  Predictor() {}
  ~Predictor() {}

  void predict(std::vector<Obstacle>& obstacles, int previous_path_length) {
    for (Obstacle& obs : obstacles) {
      // Move forward obs according to path state
      obs.s +=
          ((double)previous_path_length * SIMULATION_STEP_LENGTH * obs.speed);
    }
  }
};

#endif /* PREDICTION_H_ */
