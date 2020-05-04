/*
 * utils.h
 *
 *  Created on: May 4, 2020
 *      Author: danyz
 */

#ifndef UTILS_H_
#define UTILS_H_


#define LANE_WIDTH 4.0

bool is_on_lane(double d, double test_lane){
  return ( d < (LANE_WIDTH/2.0 + LANE_WIDTH * test_lane + LANE_WIDTH/2.0) && d > (LANE_WIDTH/2.0 + LANE_WIDTH * test_lane - LANE_WIDTH/2.0));
}


#endif /* UTILS_H_ */
