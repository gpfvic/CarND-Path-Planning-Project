#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <math.h>

#include "json.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "cost.h"
#include "vehicle.h"
#include "constants.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }


int main()
{
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

  Vehicle ego_car = Vehicle();

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &ego_car](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
              {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2')
                {

                  auto s = hasData(data);

                  if (s != "")
                  {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry")
                    {
                      // j[1] is the data JSON object

                      // Main car's localization Data
                      double car_x = j[1]["x"];
                      double car_y = j[1]["y"];
                      double car_s = j[1]["s"];
                      double car_d = j[1]["d"];
                      double car_yaw = j[1]["yaw"];
                      double car_speed = j[1]["speed"];
                      car_speed *= 0.44704; // convert mph to m/s

                      cout<<"#DEBUG######################################"<<endl;
                      cout<<"0.   x="<<car_x<<", y="<<car_y<<", s="<<car_s<<", d="<<car_d<<", yaw="<<car_yaw<<endl;

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

                      vector<double> next_x_vals;
                      vector<double> next_y_vals;

                      /**
                       * TODO: define a path made up of (x,y) points that the car will visit
                       *   sequentially every .02 seconds
                       */

                      // ********************* CONSTRUCT INTERPOLATED WAYPOINTS OF NEARBY AREA **********************
                      int num_waypoints = map_waypoints_x.size();
                      int next_waypoint_index = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
                      vector<double> coarse_waypoints_s,
                          coarse_waypoints_x,
                          coarse_waypoints_y,
                          coarse_waypoints_dx,
                          coarse_waypoints_dy;

                      for (int i = -NUM_WAYPOINTS_BEHIND; i < NUM_WAYPOINTS_AHEAD; i++)
                      {
                        int idx = (next_waypoint_index + i) % num_waypoints;
                        if (idx < 0)
                        {
                          idx += num_waypoints;
                        }
                        double current_s = map_waypoints_s[idx];
                        double base_s = map_waypoints_s[next_waypoint_index];
                        if (i < 0 && current_s > base_s)
                        {
                          current_s = current_s - MAX_S;
                        }
                        if (i > 0 && current_s < base_s)
                        {
                          current_s = current_s + MAX_S;
                        }
                        coarse_waypoints_s.push_back(current_s);
                        coarse_waypoints_x.push_back(map_waypoints_x[idx]);
                        coarse_waypoints_y.push_back(map_waypoints_y[idx]);
                        coarse_waypoints_dx.push_back(map_waypoints_dx[idx]);
                        coarse_waypoints_dy.push_back(map_waypoints_dy[idx]);
                      }

                      // interpolation parameters
                      double dist_inc = 0.5; // meters
                      int num_interpolation_points = (coarse_waypoints_s[coarse_waypoints_s.size() - 1] - coarse_waypoints_s[0]) / dist_inc;
                      vector<double> interpolated_waypoints_s,
                          interpolated_waypoints_x,
                          interpolated_waypoints_y,
                          interpolated_waypoints_dx,
                          interpolated_waypoints_dy;

                      interpolated_waypoints_s.push_back(coarse_waypoints_s[0]);
                      for (int i = 1; i < num_interpolation_points; i++)
                      {
                        interpolated_waypoints_s.push_back(coarse_waypoints_s[0] + i * dist_inc);
                      }

                      interpolated_waypoints_x = interpolate_points(coarse_waypoints_s, coarse_waypoints_x, dist_inc, num_interpolation_points);
                      interpolated_waypoints_y = interpolate_points(coarse_waypoints_s, coarse_waypoints_y, dist_inc, num_interpolation_points);
                      interpolated_waypoints_dx = interpolate_points(coarse_waypoints_s, coarse_waypoints_dx, dist_inc, num_interpolation_points);
                      interpolated_waypoints_dy = interpolate_points(coarse_waypoints_s, coarse_waypoints_dy, dist_inc, num_interpolation_points);

                      // **************** DETERMINE EGO CAR PARAMETERS AND CONSTRUCT VEHICLE OBJECT ******************
                      // Vehicle class requires s,s_d,s_dd,d,d_d,d_dd - in that order
                      double pos_s, s_dot, s_ddot;
                      double pos_d, d_dot, d_ddot;
                      double pos_x, pos_y, pos_x2, pos_y2, angle, vel_x1, vel_y1,
                          pos_x3, pos_y3, vel_x2, vel_y2, acc_x, acc_y;

                      int subpath_size = min(PREVIOUS_PATH_POINTS_TO_KEEP, (int)previous_path_x.size());
                      cout<<"subpath_size = "<<subpath_size<<"  "<<previous_path_x.size()<<endl;

                      // use default values if not enough previous path points
                      if (subpath_size < 4)
                      {
                        pos_x = car_x;
                        pos_y = car_y;
                        angle = deg2rad(car_yaw);
                        pos_s = car_s;
                        pos_d = car_d;
                        s_dot = car_speed;
                        d_dot = 0;
                        s_ddot = 0;
                        d_ddot = 0;
                      }
                      else
                      {
                        // consider current position to be last point of previous path to be kept
                        pos_x = previous_path_x[subpath_size - 1];
												pos_y = previous_path_y[subpath_size - 1];
												pos_x2 = previous_path_x[subpath_size - 2];
												pos_y2 = previous_path_y[subpath_size - 2];
												angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
                        cout<<pos_x<<"\t"<<pos_y<<"\t"<<pos_x2<<"\t"<<pos_y2<<endl;
                        cout<<">4  angle = "<<angle<<endl;
												vector<double> frenet = getFrenet(pos_x, pos_y, angle, interpolated_waypoints_x, interpolated_waypoints_y, interpolated_waypoints_s);
                        pos_s = frenet[0];
                        pos_d = frenet[1];

                        // determine dx, dy vector from set of interpoated waypoints, with pos_x,pos_y as reference point;
                        // since interpolated waypoints are ~1m apart and path points tend to be <0.5m apart, these
                        // values can be reused for previous two points (and using the previous waypoint data may be
                        // more accurate) to calculate vel_s (s_dot), vel_d (d_dot), acc_s (s_ddot), and acc_d (d_ddot)
                        int next_interp_waypoint_index = NextWaypoint(pos_x, pos_y, angle, interpolated_waypoints_x,
                                                                      interpolated_waypoints_y);
                        double dx = interpolated_waypoints_dx[next_interp_waypoint_index - 1];
                        double dy = interpolated_waypoints_dy[next_interp_waypoint_index - 1];
                        // sx,sy vector is perpendicular to dx,dy
                        double sx = -dy;
                        double sy = dx;

                        // calculate s_dot & d_dot
                        vel_x1 = (pos_x - pos_x2) / PATH_DT;
                        vel_y1 = (pos_y - pos_y2) / PATH_DT;
                        // want projection of xy velocity vector (V) onto S (sx,sy) and D (dx,dy) vectors, and since S
                        // and D are unit vectors this is simply the dot products of V with S and V with D
                        s_dot = vel_x1 * sx + vel_y1 * sy;
                        d_dot = vel_x1 * dx + vel_y1 * dy;
                        // have to get another point to calculate s_ddot, d_ddot from xy acceleration
                        pos_x3 = previous_path_x[subpath_size - 3];
                        pos_y3 = previous_path_y[subpath_size - 3];
                        vel_x2 = (pos_x2 - pos_x3) / PATH_DT;
                        vel_y2 = (pos_y2 - pos_y3) / PATH_DT;
                        acc_x = (vel_x1 - vel_x2) / PATH_DT;
                        acc_y = (vel_y1 - vel_y2) / PATH_DT;
                        s_ddot = acc_x * sx + acc_y * sy;
                        d_ddot = acc_x * dx + acc_y * dy;
                      }

                      ego_car.s = pos_s;     // s position
                      ego_car.s_d = s_dot;   // s dot - velocity in s
                      ego_car.s_dd = s_ddot; // s dot-dot - acceleration in s
                      ego_car.d = pos_d;     // d position
                      ego_car.d_d = d_dot;   // d dot - velocity in d
                      ego_car.d_dd = d_ddot; // d dot-dot - acceleration in d

                      // ********************* GENERATE PREDICTIONS FROM SENSOR FUSION DATA **************************
                      // The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car. The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map. Finally s and d are the Frenet coordinates for that car.
                      double duration = N_SAMPLES * DT - subpath_size * PATH_DT;
                      vector<Vehicle> other_cars;
                      map<int, Vehicle> predictions;
                      for (auto sf : sensor_fusion)
                      {
                        double other_car_vel = sqrt(pow((double)sf[3], 2) + pow((double)sf[4], 2));
                        Vehicle other_car = Vehicle(sf[5], other_car_vel, 0, sf[6], 0, 0);
                        other_cars.push_back(other_car);
                      }

                      // Add a little ADAS-like warning system - if any other car is immediately to left or right, set a
                      // flag to be used for hard limiting available states (i.e. if there is a car to the left, prevent
                      // Lane Change Left as an available state)
                      bool car_to_left = false, car_to_right = false, car_just_ahead = false;
                      for (Vehicle other_car : other_cars)
                      {
                        double s_diff = fabs(other_car.s - car_s);
                        if (s_diff < FOLLOW_DISTANCE)
                        {
                          cout << "s diff: " << s_diff << endl;
                          double d_diff = other_car.d - car_d;
                          if (d_diff > 2 && d_diff < 6)
                          {
                            car_to_right = true;
                          }
                          else if (d_diff < -2 && d_diff > -6)
                          {
                            car_to_left = true;
                          }
                          else if (d_diff > -2 && d_diff < 2)
                          {
                            car_just_ahead = true;
                          }
                        }
                      }
                      ego_car.update_available_states(car_to_left, car_to_right);

                      // ******************************* DETERMINE BEST TRAJECTORY ***********************************
                      // where the magic happens? NOPE! I WISH - THIS APPORACH HAS BEEN ABANDONED
                      // trajectories come back in a list of s values and a list of d values (not zipped together)
                      // duration for trajectory is variable, depending on number of previous points used
                      // vector<vector<double>> frenet_traj = my_car.get_best_frenet_trajectory(predictions, duration);
                      // vector<double> traj_xy_point, best_x_traj, best_y_traj, interpolated_x_traj, interpolated_y_traj;

                      Trajectory best_frenet_traj;
                      vector<double> best_target;
                      double best_cost = 999999;
                      string best_traj_state = "";
                      for (string state : ego_car.available_states)
                      {
                        // TODO
                        // {{target_s, target_s_d, target_s_dd , target_d, target_d_d, target_d_dd}};
                        vector<double> target_state = ego_car.get_target_for_state(state, other_cars, duration, car_just_ahead);
                        // <<s,d>,<s,d>,...>
                        Trajectory possible_traj = ego_car.generate_traj_for_target(target_state, duration);

                        double current_cost = calculate_total_cost(possible_traj, ego_car, duration, other_cars);
                        if (current_cost < best_cost)
                        {
                          best_cost = current_cost;
                          best_frenet_traj = possible_traj;
                          best_traj_state = state;
                          best_target = target_state;
                        }
                      }

                      // ********************* PRODUCE NEW PATH ***********************
                      // begin by pushing the last and next-to-last point from the previous path for setting the
                      // spline the last point should be the first point in the returned trajectory, but because of
                      // imprecision, also add that point manually
                      vector<double> coarse_s_traj;
                      vector<double> coarse_x_traj;
                      vector<double> coarse_y_traj;
                      vector<double> interpolated_s_traj;
                      vector<double> interpolated_x_traj;
                      vector<double> interpolated_y_traj;

                      double prev_s = pos_s - s_dot * PATH_DT;
                      // first two points of coarse trajectory, to ensure spline begins smoothly
                      if (subpath_size >= 2)
                      {
                        coarse_s_traj.push_back(prev_s);
                        coarse_x_traj.push_back(previous_path_x[subpath_size - 2]);
                        coarse_y_traj.push_back(previous_path_y[subpath_size - 2]);
                        coarse_s_traj.push_back(pos_s);
                        coarse_x_traj.push_back(previous_path_x[subpath_size - 1]);
                        coarse_y_traj.push_back(previous_path_y[subpath_size - 1]);
                      }
                      else
                      {
                        double prev_s = pos_s - 1;
                        double prev_x = pos_x - cos(angle);
                        double prev_y = pos_y - sin(angle);
                        coarse_s_traj.push_back(prev_s);
                        coarse_x_traj.push_back(prev_x);
                        coarse_y_traj.push_back(prev_y);
                        coarse_s_traj.push_back(pos_s);
                        coarse_x_traj.push_back(pos_x);
                        coarse_y_traj.push_back(pos_y);
                      }

                      // last two points of coarse trajectory, use target_d and current s + 30,60
                      double target_s1 = pos_s + 30;
                      double target_d1 = best_target[3];
                      vector<double> target_xy1 = getXY(target_s1, target_d1, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
                      double target_x1 = target_xy1[0];
                      double target_y1 = target_xy1[1];
                      coarse_s_traj.push_back(target_s1);
                      coarse_x_traj.push_back(target_x1);
                      coarse_y_traj.push_back(target_y1);

                      double target_s2 = target_s1 + 30;
                      double target_d2 = target_d1;
                      vector<double> target_xy2 = getXY(target_s2, target_d2, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
                      double target_x2 = target_xy2[0];
                      double target_y2 = target_xy2[1];
                      coarse_s_traj.push_back(target_s2);
                      coarse_x_traj.push_back(target_x2);
                      coarse_y_traj.push_back(target_y2);

                      //next s values
                      double target_s_dot = best_target[1];
                      double current_s = pos_s;
                      double current_vel = s_dot;
                      double current_accl = s_ddot;

                      for (int i = 0; i < (NUM_PATH_POINTS - subpath_size); i++)
                      {
                        double vel_incr, accl_incr;
                        if (fabs(target_s_dot - current_vel) < 2 * VELOCITY_INCREMENT_LIMIT)
                        {
                          vel_incr = 0;
                        }
                        else
                        {
                          // arrived at VELOCITY_INCREMENT_LIMIT value empirically
                          vel_incr = (target_s_dot - current_vel) / (fabs(target_s_dot - current_vel)) * VELOCITY_INCREMENT_LIMIT;
                        }
                        current_vel += vel_incr;
                        current_s += current_vel * PATH_DT;
                        interpolated_s_traj.push_back(current_s);
                      }

                      interpolated_x_traj = interpolate_points(coarse_s_traj, coarse_x_traj, interpolated_s_traj);
                      interpolated_y_traj = interpolate_points(coarse_s_traj, coarse_y_traj, interpolated_s_traj);

                      // add previous path, if any, to next path
                      for (int i = 0; i < subpath_size; i++)
                      {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                      }
                      // add xy points from newly generated path
                      for (int i = 0; i < interpolated_x_traj.size(); i++)
                      {
                        next_x_vals.push_back(interpolated_x_traj[i]);
                        next_y_vals.push_back(interpolated_y_traj[i]);
                      }

                      // --------------------------------------------------------------------------
                      msgJson["next_x"] = next_x_vals;
                      msgJson["next_y"] = next_y_vals;

                      auto msg = "42[\"control\"," + msgJson.dump() + "]";

                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    } // end "telemetry" if
                  }
                  else
                  {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                  }
                } // end websocket if
              }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                 { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
                    {
                      ws.close();
                      std::cout << "Disconnected" << std::endl;
                    });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
