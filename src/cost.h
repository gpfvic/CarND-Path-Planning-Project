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


double time_diff_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  /*
    Penalizes trajectories that span a duration which is longer or 
    shorter than the duration requested.
  */
  double t = traj.t;
  return logistic(abs(t-T)/T);
}

double s_diff_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  /* Penalizes trajectories whose s coordinate (and derivatives) 
    differ from the goal.
   */
  vector<double> s = traj.s;
  double T = traj.t;
  vector<double> target = predictions[target_vehicle_id].state_in(T);
  for(int i=0;i<target.size();i++){
    target[i] += delta[i];
  }
  vector<double> s_targ = {target[0],target[1],target[2]};
  S = get_f_and_N_derivatives(T, s, 2);
  double cost = 0;
  for (int i=0;i<S.size();i++){
    double actual = S[i];
    double expected = s_targ[i];
    double sigma = SIGMA_S[i];
    double diff = abs(actual-expected);
    cost += logistic(diff/sigma);
  }
  return cost;
}

double d_diff_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  /* 
    Penalizes trajectories whose d coordinate (and derivatives) 
    differ from the goal.
   */
  vector<double> d_coeffs = traj.d;
  double T = traj.t;

  vector<double> d_dot_coeffs = differentiate(d_coeffs);
  vector<double> d_ddot_coeffs = differentiate(d_dot_coeffs);

  vector<double> D ={
    calculate_current_position_shifting(T, d_coeffs).
    calculate_current_position_shifting(T, d_dot_coeffs),
    calculate_current_position_shifting(T, d_ddot_coeffs)
  };

  vector<double> target = predictions[target_vehicle_id].state_in(T);
  for(int i=0;i<target.size();i++){
    target[i] += delta[i];
  }
  vector<double> d_targ = {target[3],target[4],target[5]};
  double cost = 0.;
  for (int i=0;i<S.size();i++){
    double actual = S[i];
    double expected = d_targ[i];
    double sigma = SIGMA_D[i];
    double diff = abs(actual-expected);
    cost += logistic(diff/sigma);
  }
  return cost;
}

double collision_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  /* Binary cost function which penalizes collisions. */
  double nearest = nearest_approach_to_any_vehicle(traj, predictions);
  if (nearest < 2*VEHICLE_RADIUS){
    return 1.0;
  }
  return 0.;
}

double buffer_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  /* Penalizes getting close to other vehicles. */
  double nearest = nearest_approach_to_any_vehicle(traj, predictions);
  return logistic(2 *VEHICLE_RADIUS / nearest);
}

double stays_on_road_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  // TODO
  return 0.;
}

double exceeds_speed_limit_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  // TODO
  return 0.;
}

double efficiency_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  /* Rewards high average speeds. */
  vector<double> s = traj.s;
  double t = traj.t;
  double S = calculate_current_position_shifting(t, s);
  double avg_v = S / t;
  vector<double> target_state = predictions[target_vehicle_id].state_in(t);
  double targ_s = target_state[0];
  targ_v = targ_s / t;
  return logistic(2*(targ_v-avg_v)/avg_v);
}

double total_accel_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  /*  */
  vector<double> s = traj.s;
  vector<double> d = traj.d;
  vector<double> s_dot = differentiate(s);
  vector<double> s_d_dot = differentiate(s_dot);
  double total_accel = 0.;
  double dt = T / 100.0;
  for (int i=0;i<100;i++){
    double tt = dt * i;
    acc = calculate_current_position_shifting(tt, s_d_dot);
    total_accel += abs(acc*dt);
  }
  double acc_per_second = total_accel / T;
  return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC);
}

double max_accel_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  /*  */
  vector<double> s = traj.s;
  vector<double> d = traj.d;
  vector<double> s_dot = differentiate(s);
  vector<double> s_d_dot = differentiate(s_dot);
  double max_accel = 0.0;
  double dt = T/100.0;
  for (int i=0;i<100;i++){
    double t = dt*i;
    double acc = calculate_current_position_shifting(t, s_d_dot);
    if(abs(acc) > max_accel){
      max_accel = abs(acc);
    }
  }
  if(max_accel > MAX_ACCEL){
      return 1;
    }
  return 0;
}

double max_jerk_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  /*  */
  vector<double> s = traj.s;
  vector<double> d = traj.d;
  vector<double> s_dot = differentiate(s);
  vector<double> s_d_dot = differentiate(s_dot);
  vector<double> jerk = differentiate(s_d_dot);
  double dt = T/100.;
  double max_jerk = 0;
  for (int i=0;i<100;i++){
    double jk = calculate_current_position_shifting(dt*i,jerk);
    if(abs(jk)>max_jerk){
      max_jerk = abs(jk);
    }
  }
  if(max_jerk > MAX_JERK){
    return 1;
  }
  return 0;
}

double total_jerk_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
  /*  */
  vector<double> s = traj.s;
  vector<double> d = traj.d;
  vector<double> s_dot = differentiate(s);
  vector<double> s_d_dot = differentiate(s_dot);
  vector<double> jerk = differentiate(s_d_dot);
  double dt = T/100.;
  double total_jerk = 0;
  for (int i=0;i<100;i++){
    double jk = calculate_current_position_shifting(dt*i,jerk);
    total_jerk += abs(jk*dt);
  }
  double jerk_per_second = total_jerk/T;
  return logistic(jerk_per_second/EXPECTED_JERK_IN_ONE_SEC);
}


double  compute_total_cost(Trajectory &traj, int target_vehicle_id, 
                      vector<double> delta, double T, map<int,Vehicle> predictions){
{
  // Sum weighted cost functions to get total cost for trajectory.
  float cost = 0.0;

  vector<std::function<float(const Vehicle &,
                             const vector<Vehicle> &,
                             const vector<Vehicle> &,
                             map<string, int> &)>>
      cf_list = {time_diff_cost, s_diff_cost, d_diff_cost, collision_cost, 
                buffer_cost, efficiency_cost, total_accel_cost, 
                max_accel_cost, max_jerk_cost, total_jerk_cost}; 

  vector<float> weight_list = {TIME_DIFF_COST, S_DIFF_COST, D_DIFF_COST, 
                  COLLISION_COST, BUFFER_COST, EFFICIENCY_COST, TOTAL_ACCEL_COST, 
                  MAX_ACCEL_COST, MAX_JERK_COST, TOTAL_JERK_COST};

  for (int i = 0; i < cf_list.size(); ++i)
  {
    float new_cost = weight_list[i] * cf_list[i](traj, target_vehicle_id, delta, T, predictions);
    cost += new_cost;
  }

  return cost;
}




#endif