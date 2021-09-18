#ifndef COSTS_H
#define COSTS_H

#include "vehicle.h"
#include "constants.h"
#include "helpers.h"

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

double time_diff_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  /*
    Penalizes trajectories that span a duration which is longer or 
    shorter than the duration requested.
  */
  double t = traj.t;
  return logistic(abs(t - T) / T);
}

double s_diff_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  /* Penalizes trajectories whose s coordinate (and derivatives) 
    differ from the goal.
   */
  vector<double> s = traj.s;
  vector<double> target = ego_car.state_in(T);
  vector<double> s_targ = {target[0], target[1], target[2]};
  vector<double> S = get_f_and_N_derivatives(traj.t, s, 2);
  double cost = 0;
  for (int i = 0; i < S.size(); i++)
  {
    double actual = S[i];
    double expected = s_targ[i];
    double sigma = SIGMA_S[i];
    double diff = abs(actual - expected);
    cost += logistic(diff / sigma);
  }
  return cost;
}

double d_diff_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  /* 
    Penalizes trajectories whose d coordinate (and derivatives) 
    differ from the goal.
   */
  vector<double> d_coeffs = traj.d;

  vector<double> d_dot_coeffs = differentiate(d_coeffs);
  vector<double> d_ddot_coeffs = differentiate(d_dot_coeffs);

  vector<double> D = {
      to_equation(T, d_coeffs),
      to_equation(T, d_dot_coeffs),
      to_equation(T, d_ddot_coeffs)};

  vector<double> target = ego_car.state_in(traj.t);
  vector<double> d_targ = {target[3], target[4], target[5]};
  double cost = 0.;
  for (int i = 0; i < D.size(); i++)
  {
    double actual = D[i];
    double expected = d_targ[i];
    double sigma = SIGMA_D[i];
    double diff = abs(actual - expected);
    cost += logistic(diff / sigma);
  }
  return cost;
}

double collision_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> ohter_cars)
{
  /* Binary cost function which penalizes collisions. */
  double nearest = nearest_approach_to_any_vehicle(traj, ohter_cars);
  if (nearest < 2 * VEHICLE_RADIUS)
  {
    return 1.0;
  }
  return 0.;
}

double calculate_total_cost(Trajectory traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  // Sum weighted cost functions to get total cost for trajectory.
  double cost = 0.0;

  cost += TIME_DIFF_COST * time_diff_cost(traj, ego_car, T, other_cars);
  cost += S_DIFF_COST * s_diff_cost(traj, ego_car, T, other_cars);
  cost += D_DIFF_COST * d_diff_cost(traj, ego_car, T, other_cars);
  cost += COLLISION_COST * collision_cost(traj, ego_car, T, other_cars);

  return cost;
}

#endif