#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "ego.h"
#include "helpers.h"
#include "json.hpp"
#include "obstacle.h"
#include "path_planner.h"
#include "prediction.h"
#include "reference_path.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  EgoVehicle ego;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy,
               &ego](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          // Sensor Fusion Data, a list of all other cars
          auto sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;

          // Previous reference path computation
          std::vector<double> prev_path_x, prev_path_y;
          for (int i = 0; i < previous_path_x.size(); i++) {
            prev_path_x.push_back(previous_path_x[i]);
            prev_path_y.push_back(previous_path_y[i]);
          }
          ReferencePath prev_ref_path(prev_path_x, prev_path_y);

          // Update ego with current information
          ego.update(car_x, car_y, car_s, car_d, deg2rad(car_yaw), car_speed,
                     prev_ref_path, end_path_s, end_path_d);

          // Obstacle population
          std::vector<Obstacle> obstacles;

          for (int i = 0; i < sensor_fusion.size(); i++) {
            double curr_obs_id = sensor_fusion[i][0];
            double curr_obs_x = sensor_fusion[i][1];
            double curr_obs_y = sensor_fusion[i][2];
            double curr_obs_s = sensor_fusion[i][5];
            double curr_obs_d = sensor_fusion[i][6];
            double curr_obs_vx = sensor_fusion[i][3];
            double curr_obs_vy = sensor_fusion[i][4];
            // Also association information is computed when an
            // obstacle is created
            Obstacle curr_obstacle(curr_obs_id, curr_obs_x, curr_obs_y,
                                   curr_obs_s, curr_obs_d, curr_obs_vx,
                                   curr_obs_vy);

            obstacles.push_back(curr_obstacle);
          }

          // Predict obstacle future location
          // Will update s attribute of obstacle
          Predictor predictor;
          predictor.predict(obstacles, prev_ref_path.size);

          // Behavior Planning
          // Compute new target speed and target lane
          ego.selectBehavior(obstacles);

          std::cout << "---- DECISION START ----" << std::endl;
          std::cout << "Decision taken!" << std::endl;
          std::cout << "Target speed: " << ego.target_speed << std::endl;
          std::cout << "Target lane: " << ego.target_lane << std::endl;
          std::cout << "---- DECISION END ----" << std::endl;

          // Creating and building ReferencePath
          ReferencePath reference_path(map_waypoints_x, map_waypoints_y,
                                       map_waypoints_s);

          reference_path.computeReferencePath(prev_ref_path, ego,
                                              ego.target_lane);

          // Create smooth trajectory
          PathPlanner path_planner(ego, prev_ref_path, reference_path);

          std::vector<double> ego_trajectory_x;
          std::vector<double> ego_trajectory_y;

          path_planner.plan(ego.target_speed, ego_trajectory_x,
                            ego_trajectory_y);

          msgJson["next_x"] = ego_trajectory_x;
          msgJson["next_y"] = ego_trajectory_y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

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
