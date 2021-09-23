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
#include <math.h>

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

  vector<double> target = ego_car.state_in(T);
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

double buffer_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  /* Penalizes getting close to other vehicles. */
  double nearest = nearest_approach_to_any_vehicle(traj, other_cars);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

double out_of_lane_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  double target_d = ego_car.target_state[3];
  double cost;
  for(int i=1;i<15;i++){
    double d = to_equation(i*DT, traj.d);
    cost += logistic(abs(d - target_d)/4);
  }
  return cost;
}



double efficiency_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  /* Rewards high average speeds. */
  vector<double> s = traj.s;
  double t = traj.t;
  double S = to_equation(t, s);
  double avg_v = S / t;
  vector<double> target_state = ego_car.state_in(t);
  double targ_s = target_state[0];
  double targ_v = targ_s / t;
  return logistic(2 * (targ_v - avg_v) / avg_v);
}

double total_accel_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  /*  */
  vector<double> s = traj.s;
  vector<double> d = traj.d;
  vector<double> s_dot = differentiate(s);
  vector<double> s_d_dot = differentiate(s_dot);
  double total_accel = 0.;
  double dt = T / 100.0;
  for (int i = 0; i < 100; i++)
  {
    double tt = dt * i;
    double acc = to_equation(tt, s_d_dot);
    total_accel += abs(acc * dt);
  }
  double acc_per_second = total_accel / T;
  return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC);
}

double max_accel_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  /*  */
  vector<double> s = traj.s;
  vector<double> d = traj.d;
  vector<double> s_dot = differentiate(s);
  vector<double> s_d_dot = differentiate(s_dot);
  double max_accel = 0.0;
  double dt = T / 100.0;
  for (int i = 0; i < 100; i++)
  {
    double t = dt * i;
    double acc = to_equation(t, s_d_dot);
    if (abs(acc) > max_accel)
    {
      max_accel = abs(acc);
    }
  }
  if (max_accel > MAX_ACCEL-9)
  {
    return 1;
  }
  return 0;
}

double max_jerk_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  /*  */
  vector<double> s = traj.s;
  vector<double> d = traj.d;
  vector<double> s_dot = differentiate(s);
  vector<double> s_d_dot = differentiate(s_dot);
  vector<double> jerk = differentiate(s_d_dot);
  double dt = T / 100.;
  double max_jerk = 0;
  for (int i = 0; i < 100; i++)
  {
    double jk = to_equation(dt * i, jerk);
    if (abs(jk) > max_jerk)
    {
      max_jerk = abs(jk);
    }
  }
  if (max_jerk > MAX_JERK)
  {
    return 1;
  }
  return 0;
}

double total_jerk_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  /*  */
  vector<double> s = traj.s;
  vector<double> d = traj.d;
  vector<double> s_dot = differentiate(s);
  vector<double> s_d_dot = differentiate(s_dot);
  vector<double> jerk = differentiate(s_d_dot);
  double dt = T / 100.;
  double total_jerk = 0;
  for (int i = 0; i < 100; i++)
  {
    double jk = to_equation(dt * i, jerk);
    total_jerk += abs(jk * dt);
  }
  double jerk_per_second = total_jerk / T;
  return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC);
}

double not_in_middle_lane_cost(Trajectory &traj, Vehicle ego_car, double T, vector<Vehicle> other_cars){
  // penalize not shooting for middle lane (d = 6)
  double target_d = traj.target_state[3];
  return logistic(pow(target_d-6, 2));
}

double calculate_total_cost(Trajectory traj, Vehicle ego_car, double T, vector<Vehicle> other_cars)
{
  // Sum weighted cost functions to get total cost for trajectory.
  double cost = 0.0;

  cost += S_DIFF_COST * s_diff_cost(traj, ego_car, T, other_cars);
  cost += D_DIFF_COST * d_diff_cost(traj, ego_car, T, other_cars);
  cost += COLLISION_COST * collision_cost(traj, ego_car, T, other_cars);
  cost += BUFFER_COST * buffer_cost(traj, ego_car, T, other_cars);
  cost += EFFICIENCY_COST * efficiency_cost(traj, ego_car, T, other_cars);
  cost += MAX_ACCEL_COST * max_accel_cost(traj, ego_car, T, other_cars);
  cost += MAX_JERK_COST * max_jerk_cost(traj, ego_car, T, other_cars);
  cost += TOTAL_ACCEL_COST * total_accel_cost(traj, ego_car, T, other_cars);
  cost += TOTAL_JERK_COST * total_jerk_cost(traj, ego_car, T, other_cars);
  cost += OUT_LANE_COST * out_of_lane_cost(traj, ego_car, T, other_cars);


  cout << "S_DIFF_COST = " << S_DIFF_COST * s_diff_cost(traj, ego_car, T, other_cars) << endl;
  cout << "D_DIFF_COST = " <<  D_DIFF_COST * d_diff_cost(traj, ego_car, T, other_cars) << endl;
  cout << "COLLISION_COST = " << COLLISION_COST * collision_cost(traj, ego_car, T, other_cars) << endl;
  cout << "BUFFER_COST = " << BUFFER_COST * buffer_cost(traj, ego_car, T, other_cars) << endl;
  cout<< "EFFICIENCY_COST = "<<EFFICIENCY_COST * efficiency_cost(traj, ego_car, T, other_cars)<<endl;
  cout << "MAX_ACCEL_COST = " << MAX_ACCEL_COST * max_accel_cost(traj, ego_car, T, other_cars) << endl;
  cout << "MAX_JERK_COST = " << MAX_JERK_COST * max_jerk_cost(traj, ego_car, T, other_cars) << endl;
  cout << "TOTAL_ACCEL_COST = " << TOTAL_ACCEL_COST * total_accel_cost(traj, ego_car, T, other_cars) << endl;
  cout << "TOTAL_JERK_COST = " << TOTAL_JERK_COST * total_jerk_cost(traj, ego_car, T, other_cars) << endl;
  cout << "OUT_LANE_COST = " << OUT_LANE_COST * out_of_lane_cost(traj, ego_car, T, other_cars) << endl;

  return cost;
}

#endif