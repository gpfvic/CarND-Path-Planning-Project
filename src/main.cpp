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

                      printf("\n\n\n");
                      cout<<"#DEBUG######################################"<<endl;

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
                      vector<double> reference_waypoints_s,
                          reference_waypoints_x,
                          reference_waypoints_y,
                          reference_waypoints_dx,
                          reference_waypoints_dy;

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
                        reference_waypoints_s.push_back(current_s);
                        reference_waypoints_x.push_back(map_waypoints_x[idx]);
                        reference_waypoints_y.push_back(map_waypoints_y[idx]);
                        reference_waypoints_dx.push_back(map_waypoints_dx[idx]);
                        reference_waypoints_dy.push_back(map_waypoints_dy[idx]);
                      }

                      // interpolation parameters
                      double dist_inc = 0.5; // meters
                      int num_interpolation_points = (reference_waypoints_s[reference_waypoints_s.size() - 1] - reference_waypoints_s[0]) / dist_inc;
                      vector<double> interpolated_waypoints_s,
                          interpolated_waypoints_x,
                          interpolated_waypoints_y,
                          interpolated_waypoints_dx,
                          interpolated_waypoints_dy;

                      interpolated_waypoints_s.push_back(reference_waypoints_s[0]);
                      for (int i = 1; i < num_interpolation_points; i++)
                      {
                        interpolated_waypoints_s.push_back(reference_waypoints_s[0] + i * dist_inc);
                      }

                      interpolated_waypoints_x = interpolate_points(reference_waypoints_s, reference_waypoints_x, dist_inc, num_interpolation_points);
                      interpolated_waypoints_y = interpolate_points(reference_waypoints_s, reference_waypoints_y, dist_inc, num_interpolation_points);
                      interpolated_waypoints_dx = interpolate_points(reference_waypoints_s, reference_waypoints_dx, dist_inc, num_interpolation_points);
                      interpolated_waypoints_dy = interpolate_points(reference_waypoints_s, reference_waypoints_dy, dist_inc, num_interpolation_points);

                      // **************** DETERMINE EGO CAR PARAMETERS AND CONSTRUCT VEHICLE OBJECT ******************
                      // Vehicle class requires s,s_d,s_dd,d,d_d,d_dd - in that order
                      double pos_s, s_dot, s_ddot;
                      double pos_d, d_dot, d_ddot;
                      double pos_x, pos_y, pos_x2, pos_y2, angle, vel_x1, vel_y1,
                          pos_x3, pos_y3, vel_x2, vel_y2, acc_x, acc_y;

                      int subpath_size = min(PREVIOUS_PATH_POINTS_TO_KEEP, (int)previous_path_x.size());
                      
                      double traj_start_time = subpath_size * PATH_DT;

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
                      // for cold start
                      if(d_ddot > 9){
                        ego_car.d_dd = 9;
                      }else if(d_ddot <-9){
                        ego_car.d_dd = -9;
                      }
                      

                      // ********************* GENERATE PREDICTIONS FROM SENSOR FUSION DATA **************************
                      // The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car. 
                      //The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map. 
                      // Finally s and d are the Frenet coordinates for that car.
                      double duration = N_SAMPLES * DT - subpath_size * PATH_DT;

                      vector<Vehicle> other_cars;
                      map<int, Vehicle> predictions;
                      for (auto sf : sensor_fusion)
                      {
                        double other_car_vel = sqrt(pow((double)sf[3], 2) + pow((double)sf[4], 2));
                        Vehicle other_car = Vehicle(sf[5], other_car_vel, 0, sf[6], 0, 0);
                        other_car.traj_start_time = 0.5;
                        other_cars.push_back(other_car);
                      }

                      // finite state machine
                      // sense the other cars information, then to change the ego car's state
                      // to turn left , or right, or just keep in lane
                      bool car_turning_left = false, car_turning_right = false, car_just_ahead = false;
                      bool car_ahead_in_the_left_lane = false;
                      bool car_ahead_in_the_right_lane = false;
                      for (Vehicle other_car : other_cars)
                      {
                        double s_diff = fabs(other_car.s - car_s);
                        if (s_diff < FOLLOW_DISTANCE)
                        {
                          double d_diff = other_car.d - car_d;
                          if (d_diff > 2 && d_diff < 6)
                          {
                            car_turning_right = true;
                          }
                          else if (d_diff < -2 && d_diff > -6)
                          {
                            car_turning_left = true;
                          }
                          else if (d_diff > -2 && d_diff < 2)
                          {
                            car_just_ahead = true;
                          }
                        }
                        // deal with there is two car just with same d ahead
                        s_diff = other_car.s - car_s;
                        double d_diff = other_car.d - car_d;
                        if(s_diff < SAFETY_DISTANCE && s_diff > 0 && fabs(d_diff)<6){
                          if (d_diff > 2 && d_diff < 6)
                          {
                            car_ahead_in_the_right_lane = true;
                          }
                          else if (d_diff < -2 && d_diff > -6)
                          {
                            car_ahead_in_the_left_lane = true;
                          }
                          cout<<"s_diff = "<<s_diff<<"\t d_diff = "<<d_diff<<"\t["<<car_ahead_in_the_left_lane<<", "<<car_ahead_in_the_right_lane<<"]"<<endl;
                        }

                        // in case other car rush into ego car suddenly
                        s_diff = fabs(other_car.s - car_s);
                        d_diff = fabs(other_car.d - car_d);
                        if(s_diff <= 1.2*VEHICLE_RADIUS && d_diff<1.2*VEHICLE_RADIUS){
                          car_just_ahead = true; // speed to 0
                        }

                      }
                      ego_car.update_available_states(car_turning_left, car_turning_right, car_just_ahead, car_ahead_in_the_right_lane,car_ahead_in_the_left_lane);

                      // ******************************* DETERMINE BEST TRAJECTORY ***********************************
                      Trajectory best_frenet_traj;
                      vector<double> best_target;
                      double best_cost = 999999;
                      string best_traj_state = "";

                      for (string state : ego_car.available_states)
                      {
                        // TODO
                        printf("\n");
                        cout<<"STATE:\t"<<state<<endl;
                        
                        // {{target_s, target_s_d, target_s_dd , target_d, target_d_d, target_d_dd}};
                        vector<double> target_state = ego_car.get_target_for_state(state, other_cars, traj_start_time, duration, car_just_ahead);
                        ego_car.target_state = target_state;
                        cout<<"CURRENT State: ["<<ego_car.s<<", "<<ego_car.s_d<<", "<<ego_car.s_dd<<", "<<ego_car.d<<", "<<ego_car.d_d<<", "<<ego_car.d_dd<<"]"<<endl;
                        cout<<"TARGET  State: ["<<target_state[0]<<", "<<target_state[1]<<", "<<target_state[2]<<", "<<target_state[3]<<", "<<target_state[4]<<", "<<target_state[5]<<"]"<<endl;
                        
                        Trajectory possible_traj = ego_car.generate_traj_for_target(target_state, duration);

                        double current_cost = calculate_total_cost(possible_traj, ego_car, duration, other_cars);
                        if (current_cost < best_cost)
                        {
                          best_cost = current_cost;
                          best_frenet_traj = possible_traj;
                          best_traj_state = state;
                          best_target = target_state;
                        }
                        cout<<"CURRENT_COST = "<<current_cost<<endl;
                      }
                      printf("\n---------------\n");
                      cout<<"BEST STATE: "<<best_traj_state<<endl;
                      // cout<< "BEST Traj: ["<<best_target[0]<<","<<best_target[1]<<","<<best_target[2]<<","<<best_target[3]<<","<<best_target[4]<<","<<best_target[5]<<"]"<<endl;



                      // ********************* PRODUCE NEW PATH ***********************
                      // begin by pushing the last and next-to-last point from the previous path for setting the
                      // spline the last point should be the first point in the returned trajectory
                      vector<double> reference_s_traj;
                      vector<double> reference_x_traj;
                      vector<double> reference_y_traj;
                      vector<double> interpolated_s_traj;
                      vector<double> interpolated_x_traj;
                      vector<double> interpolated_y_traj;

                      double prev_s = pos_s - s_dot * PATH_DT;
                      // first two points of reference trajectory, to ensure spline begins smoothly
                      if (subpath_size >= 2)
                      {
                        reference_s_traj.push_back(prev_s);
                        reference_x_traj.push_back(previous_path_x[subpath_size - 2]);
                        reference_y_traj.push_back(previous_path_y[subpath_size - 2]);
                        reference_s_traj.push_back(pos_s);
                        reference_x_traj.push_back(previous_path_x[subpath_size - 1]);
                        reference_y_traj.push_back(previous_path_y[subpath_size - 1]);
                      }
                      else
                      {
                        double prev_s = pos_s - 1;
                        double prev_x = pos_x - cos(angle);
                        double prev_y = pos_y - sin(angle);
                        reference_s_traj.push_back(prev_s);
                        reference_x_traj.push_back(prev_x);
                        reference_y_traj.push_back(prev_y);
                        reference_s_traj.push_back(pos_s);
                        reference_x_traj.push_back(pos_x);
                        reference_y_traj.push_back(pos_y);
                      }

                      // last two points of reference trajectory, use target_d and current s + 30,60
                      double target_s1 = pos_s + 30;
                      double target_d1 = best_target[3];
                      vector<double> target_xy1 = getXY(target_s1, target_d1, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
                      double target_x1 = target_xy1[0];
                      double target_y1 = target_xy1[1];
                      reference_s_traj.push_back(target_s1);
                      reference_x_traj.push_back(target_x1);
                      reference_y_traj.push_back(target_y1);

                      double target_s2 = pos_s + 60;
                      double target_d2 = best_target[3];
                      vector<double> target_xy2 = getXY(target_s2, target_d2, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
                      double target_x2 = target_xy2[0];
                      double target_y2 = target_xy2[1];
                      reference_s_traj.push_back(target_s2);
                      reference_x_traj.push_back(target_x2);
                      reference_y_traj.push_back(target_y2);

                      double target_s3 = pos_s + 90;
                      double target_d3 = best_target[3];
                      vector<double> target_xy3 = getXY(target_s3, target_d3, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
                      double target_x3 = target_xy3[0];
                      double target_y3 = target_xy3[1];
                      reference_s_traj.push_back(target_s3);
                      reference_x_traj.push_back(target_x3);
                      reference_y_traj.push_back(target_y3);

                      // cout<<"*** TARGET S = "<<best_target[0]<<"\t current_s = "<<ego_car.s<<"\t target_s1 = "<<target_s1<<"\t\t target_s2 ="<<target_s2<<endl;
                      // cout<<"*** TARGET D = "<<best_target[3]<<"\t current_d = "<<ego_car.d<<"\t target_d1 = "<<target_d1<<"\t\t target_d2 = "<<target_d2<<endl;

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

                      interpolated_x_traj = interpolate_points(reference_s_traj, reference_x_traj, interpolated_s_traj);
                      interpolated_y_traj = interpolate_points(reference_s_traj, reference_y_traj, interpolated_s_traj);

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
