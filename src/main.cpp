#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "utils.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

enum Lanes{
  RIGHT_LANE = 0,
  CENTER_LANE = 1,
  LEFT_LANE = 2
};

#define MAX_SPEED 49.5


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;


  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  double ref_vel = 0.0;  // near to 50mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;

          int prev_size = previous_path_x.size();


          // ######################################################
          // SENSOR FUSION
          // ######################################################

          if (prev_size>0){
            // Update car s with last point in path
            car_s = end_path_s;
          }

          bool too_close = false;

          // find ref_v of obstacle to use
          for (int i=0; i<sensor_fusion.size(); i++){
            // car is in my lane
            double curr_obs_d = sensor_fusion[i][6];
            if (is_on_lane(curr_obs_d, lane)) {
              double curr_obs_vx = sensor_fusion[i][3];
              double curr_obs_vy = sensor_fusion[i][4];

              double curr_obs_speed = sqrt(curr_obs_vx*curr_obs_vx+curr_obs_vy*curr_obs_vy);
              double curr_obs_s = sensor_fusion[i][5];

              // Move forward obs according to path state
              curr_obs_s += ((double)prev_size * .02 * curr_obs_speed);

              // If is in front of me and the gap is less than 30m
              if ( (curr_obs_s>car_s) && (curr_obs_s-car_s)<30 ){
                too_close = true;

                // Try to aggressively going on left
                if(lane>0){
                  lane=0;
                }


              }
            }
          }

          if(too_close){
            ref_vel -= .224; //5m/s^2
          } else if (ref_vel < MAX_SPEED){
            ref_vel += .224;
          }

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */



          // ######################################################
          // PATH PLANNING - REFERENCE PATH BUILDING
          // ######################################################

          // Create a list of widely spaced (x,y) waypoints evenly spaced at 30m
          // LAter we will interpolate these waypoints with a spline and
          // Fill it with more points thtat control
          vector<double> anchor_points_x;
          vector<double> anchor_points_y;

          // Reference x,y, yaw status
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If previous state is almost empty, use the car as starting reference
          if (prev_size < 2) {

            // Use two points that make the path tangents to the car

            // Go backward in time using angle
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            anchor_points_x.push_back(prev_car_x);
            anchor_points_y.push_back(prev_car_y);

            anchor_points_x.push_back(car_x);
            anchor_points_y.push_back(car_y);

          }
          // Use the previous path's and point as reference
          else {

            // Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];

            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // Use two points to make the path tangent to the previous path end point

            anchor_points_x.push_back(ref_x_prev);
            anchor_points_y.push_back(ref_y_prev);

            anchor_points_x.push_back(ref_x);
            anchor_points_y.push_back(ref_y);

          }


          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp_0 = getXY(car_s+30, (LANE_WIDTH/2.0 + LANE_WIDTH*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp_1 = getXY(car_s+60, (LANE_WIDTH/2.0 + LANE_WIDTH*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp_2 = getXY(car_s+90, (LANE_WIDTH/2.0 + LANE_WIDTH*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          anchor_points_x.push_back(next_wp_0[0]);
          anchor_points_x.push_back(next_wp_1[0]);
          anchor_points_x.push_back(next_wp_2[0]);

          anchor_points_y.push_back(next_wp_0[1]);
          anchor_points_y.push_back(next_wp_1[1]);
          anchor_points_y.push_back(next_wp_2[1]);


          // Transfrom anchor to car vehicle reference frame
          for (int i=0; i<anchor_points_x.size(); i++) {
            // shift curr point to vehicle ref frame
            double shift_x = anchor_points_x[i] - ref_x;
            double shift_y = anchor_points_y[i] - ref_y;
            double shift_yaw = 0.0-ref_yaw;

            anchor_points_x[i] = (shift_x*cos(shift_yaw) - shift_y*sin(shift_yaw));
            anchor_points_y[i] = (shift_x*sin(shift_yaw) + shift_y*cos(shift_yaw));
          }

          // Create the spline
          tk::spline s;
          // Setting up the spline points
          s.set_points(anchor_points_x, anchor_points_y);

          // Define the actual (x,y) points that will be used by the planner
          vector<double> planning_x;
          vector<double> planning_y;

          // All points in previous path are the points the car still needs to cover
          // So copy it in the planning points
          for(int i=0; i<previous_path_x.size(); i++){
            planning_x.push_back(previous_path_x[i]);
            planning_y.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we will be travel at desired speed

          // First compute norm of (s,d) vector of anchor point
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_norm = sqrt((target_x*target_x)+(target_y*target_y));

          // Equation is N_points * .02 * desired_velocity = norm
          // where N is the number of points we have and .02 is how often the simulator will run

          double x_add_on = 0.0;

          // Populate the rest of our path planner after filling it with previous points
          // here we will always output 50 points

          for (int i=1; i<=50-previous_path_x.size(); i++) {

            double N = (target_norm/(.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x/N);
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after have rotated it before in car ref frame
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            planning_x.push_back(x_point);
            planning_y.push_back(y_point);
          }

          for(int i=0;i<planning_x.size();i++){
            std::cout<<"("<<planning_x[i]<<", "<<planning_y[i]<<")"<<std::endl;
          }

          msgJson["next_x"] = planning_x;
          msgJson["next_y"] = planning_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
