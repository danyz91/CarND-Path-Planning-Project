#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>

#include <string>
#include <vector>

#define LANE_WIDTH 4.0

#define CENTER_LINE_REF (LANE_WIDTH / 2.0)

#define MPH_TO_MS(X) (X / 2.237)

#define MS_TO_MPH(X) (X * 2.237)

#define SIMULATION_STEP_LENGTH .02

#define DELTA_MPH_FROM_MS2(X) (SIMULATION_STEP_LENGTH * MS_TO_MPH(X))  // mph

enum Lane { LEFT_LANE = 0, CENTER_LANE = 1, RIGHT_LANE = 2, NUM_LANES = 3 };

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//
// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y);

bool is_on_lane(double d, double test_lane);

#endif  // HELPERS_H
