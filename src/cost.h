#ifndef COSTS_H
#define COSTS_H

#include "vehicle.h"
#include "constants.h"

#include <cmath>
#include <functional>
#include <iterator>
#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <iostream>

using std::string;
using std::vector;
using namespace std;


map<string, int> get_helper_data(const Vehicle &ego_car,
                                 const vector<Vehicle> &trajectory,
                                 const vector<Vehicle> &other_cars)
{
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory. ---> Not used in this project
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help
  //   differentiate between planning and executing a lane change in the
  //   cost functions.
  map<string, int> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  int intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0)
  {
    intended_lane = trajectory_last.lane - 1;
  }
  else if (trajectory_last.state.compare("PLCR") == 0)
  {
    intended_lane = trajectory_last.lane + 1;
  }
  else
  {
    intended_lane = trajectory_last.lane;
  }

  float distance_to_goal = MAX_S - trajectory_last.s;
  int final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;

  return trajectory_data;
}

float lane_speed(const vector<Vehicle> &other_cars, int lane) // Integreate Sensorfusion
{
  // All non ego vehicles in a lane have the same speed, so to get the speed
  //   limit for a lane, we can just find one vehicle in that lane.
  for (int i = 0; i < other_cars.size(); i++)
  {
    Vehicle car = other_cars[i];

    if (car.lane == lane)
    {
      return car.current_speed;
    }
  }
  return -1.0;
}

//   The weighted cost over all cost functions is computeds
//   in calculate_cost. The data from get_helper_data will be very useful in
//   your implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.
// ---> Trajectory have to be implemented into Car
//  ->>> map data have to be adjusted

float goal_distance_cost(const Vehicle &vehicle,
                         const vector<Vehicle> &trajectory,
                         const vector<Vehicle> &predictions,
                         map<string, int> &data)
{
  // Cost increases based on distance of intended lane (for planning a lane
  //   change) and final lane of trajectory.
  // Cost of being out of goal lane also becomes larger as vehicle approaches
  //   goal distance.
  float cost;
  float distance = data["distance_to_goal"];
  if (distance > 0)
  {
    cost = 1 - 2 * exp(-(abs(2.0 * vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
  }
  else
  {
    cost = 0;
  }
  float nix = 0;
  cost = max(nix, cost);
  return cost;
}

// Implement lane speed function
// car target_speed
float inefficiency_cost(const Vehicle &car,
                        const vector<Vehicle> &trajectory,
                        const vector<Vehicle> &predictions,
                        map<string, int> &data)
{
  // Cost becomes higher for trajectories with intended lane and final lane
  //  that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane.
  float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
  if (proposed_speed_intended < 0)
  {
    proposed_speed_intended = car.target_speed;
  }

  float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
  if (proposed_speed_final < 0)
  {
    proposed_speed_final = car.target_speed;
  }

  float cost = (2.0 * car.target_speed - proposed_speed_intended - proposed_speed_final) / car.target_speed;
  float nix = 0;
  cost = max(nix, cost);
  return cost;
}

float offroad_cost(const Vehicle &vehicle,
                   const vector<Vehicle> &trajectory,
                   const vector<Vehicle> &predictions,
                   map<string, int> &data)
{
  // Cost penalize trajectories that are off the streed based on distance of intended lane (for planning a lane
  //   change) and final lane of trajectory..
  float cost;
  if ((data["intended_lane"] <= vehicle.lane_max) && (data["intended_lane"] >= vehicle.lane_min))
  {
    cost = 0;
  }
  else
  {
    cost = 1;
  }
  return cost;
}

float change_lane_cost(const Vehicle &car,
                       const vector<Vehicle> &trajectory,
                       const vector<Vehicle> &predictions,
                       map<string, int> &data)
{
  // Cost penalize trajectories that are change the lane. So that a lane change have to have a
  // see able impact
  float cost;
  if (data["intended_lane"] != car.lane)
  {
    cost = 1;
  }
  else
  {
    cost = 0;
  }
  std::cout << " lane change cost" << cost << std::endl;
  return cost;
}

float speedlimit_cost(const Vehicle &car,
                      const vector<Vehicle> &trajectory,
                      const vector<Vehicle> &predictions,
                      map<string, int> &data)
{
  // Cost penalize trajectories that exeed speedlimit based on.
  float cost;

  if ((data["intended_lane"] <= car.lane_max) && (data["intended_lane"] >= car.lane_min))
  {
    cost = 0;
  }
  else
  {
    cost = 1;
  }
  std::cout << " speed limit cost" << cost << std::endl;
  return cost;
}

float compute_total_cost(const Vehicle &ego_car,
                     const vector<Vehicle> &other_cars,
                     const vector<Vehicle> &trajectory)
{
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, int> trajectory_data = get_helper_data(ego_car, trajectory, other_cars);

  float cost = 0.0;

  vector<std::function<float(const Vehicle &,
                             const vector<Vehicle> &,
                             const vector<Vehicle> &,
                             map<string, int> &)>>
      cf_list = {goal_distance_cost, inefficiency_cost, offroad_cost, change_lane_cost}; 

  vector<float> weight_list = {REACH_GOAL, EFFICIENCY, OFFROAD, LANECHANGE};

  for (int i = 0; i < cf_list.size(); ++i)
  {
    float new_cost = weight_list[i] * cf_list[i](ego_car, trajectory, other_cars, trajectory_data);
    cost += new_cost;
  }

  return cost;
}

#endif