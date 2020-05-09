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

class Plotter {
 public:
  Plotter();
  virtual ~Plotter();

  void plotMap(std::vector<double> waypoints_x,
               std::vector<double> waypoints_y);

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

void Plotter::plotMap(std::vector<double> waypoints_x,
                      std::vector<double> waypoints_y) {
  gplot.set_style("lines");
  gplot.plot_xy(waypoints_x, waypoints_y);
}

// void Plotter::plotVehicle(Vehicle v) {
//  gplot.cmd("set obj 1 rect at " + v.x + "," + v.y + " size 3,1");
//}

void Plotter::reset() { gplot.reset_plot(); }

void Plotter::show() { gplot.showonscreen(); }

#endif /* SRC_PLOTTER_H_ */
