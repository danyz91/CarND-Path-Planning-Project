/*
 * gnu_plot.h
 *
 *  Created on: May 9, 2020
 *      Author: danyz
 */

#ifndef SRC_PLOTTER_H_
#define SRC_PLOTTER_H_

#include <iostream>
#include <string>
#include <vector>

#include "gnuplot_i.hpp"
#include "helpers.h"

class Plotter {
 public:
  Plotter();
  virtual ~Plotter();

  void plotMap(const std::vector<double> &maps_s,
               const std::vector<double> &maps_x,
               const std::vector<double> &maps_y);

  void show();
  void reset();

  Gnuplot gplot;
};

Plotter::Plotter() {  // TODO Auto-generated constructor stub
  Gnuplot gplot;
  gplot.set_terminal_std("qt");

  gplot.set_title("Pplot");
}

Plotter::~Plotter() {
  // TODO Auto-generated destructor stub
}

void Plotter::plotMap(const std::vector<double> &maps_s,
                      const std::vector<double> &maps_x,
                      const std::vector<double> &maps_y) {
  int num_lanes = 3;

  std::vector<double> map_x;
  std::vector<double> map_y;

  for (int i = 0; i < maps_s.size(); i++) {
    for (int j = 0; j < num_lanes; j++) {
      std::vector<double> curr_point =
          getXY(maps_s[i], j, maps_s, maps_x, maps_y);
      map_x.push_back(curr_point[0]);
      map_y.push_back(curr_point[1]);
    }
  }

  gplot.set_style("lines");

  gplot.plot_xy(map_x, map_y);
}

// void Plotter::plotVehicle(Vehicle v) {
//  gplot.cmd("set obj 1 rect at " + v.x + "," + v.y + " size 3,1");
//}

void Plotter::reset() { gplot.reset_plot(); }

void Plotter::show() { gplot.showonscreen(); }

#endif /* SRC_PLOTTER_H_ */
