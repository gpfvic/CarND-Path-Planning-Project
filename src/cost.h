#ifndef COSTS_H
#define COSTS_H

#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>
#include <map>
#include "constants.h"
#include "helpers.h"

using namespace std;


double time_diff_cost(double target_time, double actual_time){
  /*
    Penalizes trajectories that span a duration which is longer or 
    shorter than the duration requested.
  */
 return logistic(fabs(actual_time - target_time) / target_time);
}

double traj_diff_cost(vector<double> s_traj, vector<double> target_s){
  // Penalizes trajectories whose s coordinate (and derivatives) 
  // differ from the goal. Target is s, s_dot, and s_ddot.
  int s_end = s_traj.size();
  double s1, s2, s3, s1_dot, s2_dot, s_dot_dot, cost = 0;
  s1 = s_traj[s_end -1]; // the last location of the trajectory
  s2 = s_traj[s_end -2]; // the second last location
  s3 = s_traj[s_end -3];
  s1_dot = (s1 - s2) /DT;
  s2_dot = (s2-s3)/DT;
  s_dot_dot = (s_dot1 - s_dot2)/DT;
  cost += fabs(s1-target_s[0])/SIGMA_S;
  cost += fabs(s1_dot-target_s[1])/SIGMA_S_DOT;
  cost += fabs(s1_dot_dot-target_s[2])/SIGMA_S_DOT;
  return logistic(cost);
}

double collision_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  //Binary cost function which penalizes collisions.
  double nearest = nearest_approach_to_any_vehicle(s_traj, d_straj, predictions);
  if(nearest < 2*VEHICLE_RADIUS){
    return 1;
  }else{
    return 0;
  }
}

double buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  //Penalizes getting close to other vehicles.
  double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS/ nearest);
}

double exceeds_speed_limit_cost(vector<double> s_traj){
  // Penalty getting faster than the limit speed of this lane
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  for (double s_dot : s_dot_traj){
    if(s_dot > SPEED_LIMIT){
      return 1;
    }
  }
  return 0;
}

double efficiency_cost(vector<double> s_traj){
  // Rewards high average speeds.
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  double final_s_dot, total = 0;
  final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
  return logistic((SPEED_LIMIT - final_s_dot)/SPEED_LIMIT);
}


double max_accel_cost(vector<double> s_traj){
  // Penalize if the max accel is too high
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_dot_dot_traj = velocities_for_trajectory(s_dot_traj);
  for(double s_dot_dot : s_dot_dot_traj){
    if(s_dot_dot > MAX_INSTANTANEOUS_ACCEL){
      return 1;
    }
  }
  return 0;
}

double avg_accel_cost(vector<double> s_traj){
  //Penalize higher average acceleration
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_dot_dot_traj = velocities_for_trajectory(s_dot_traj);
  double total = 0
  for(double s_dot_dot: s_dot_dot_traj){
    total += s_dot_dot;
  }
  double avg_accel = total / s_dot_dot_traj.size();
  return logistic(avg_accel/EXPECTED_ACC_IN_ONE_SEC);
}

double max_jerk_cost(vector<double> s_traj){
  // Penalize exceeding MAX_INSTANTANEOUS_JERK
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_dot_dot_traj = velocities_for_trajectory(s_dot_traj);
  vector<double> s_dot_dot_dot_traj = velocities_for_trajectory(s_dot_dot_traj);
  for(double ss: s_dot_dot_dot_traj){
    if(ss>MAX_INSTANTANEOUS_JERK){
      reurn 1;
    }
  }
  return 0;
}

double avg_jerk_cost(vector<double> s_traj){
  //Penalize higher average jerk
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_dot_dot_traj = velocities_for_trajectory(s_dot_traj);
  vector<double> s_dot_dot_dot_traj = velocities_for_trajectory(s_dot_dot_traj);

  double total = 0;
  for(double st:s_dot_dot_dot_traj){
    total += st;
  }
  double avg_jerk = total / s_dot_dot_dot_traj/size();
  return logistic(avg_jerk/EXPECTED_JERK_IN_ONE_SEC);
}

double calculate_total_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {

  double total_cost = 0;
  double col = collision_cost(s_traj, d_traj, predictions) * COLLISION_COST_WEIGHT;
  double buf = buffer_cost(s_traj, d_traj, predictions) * BUFFER_COST_WEIGHT;
  double ilb = in_lane_buffer_cost(s_traj, d_traj, predictions) * IN_LANE_BUFFER_COST_WEIGHT;
  double eff = efficiency_cost(s_traj) * EFFICIENCY_COST_WEIGHT;
  //double esl = exceeds_speed_limit_cost(s_traj) * SPEED_LIMIT_COST_WEIGHT;
  //double mas = max_accel_cost(s_traj) * MAX_ACCEL_COST_WEIGHT;
  //double aas = avg_accel_cost(s_traj) * AVG_ACCEL_COST_WEIGHT;
  //double mad = max_accel_cost(d_traj) * MAX_ACCEL_COST_WEIGHT;
  //double aad = avg_accel_cost(d_traj) * AVG_ACCEL_COST_WEIGHT;
  //double mjs = max_jerk_cost(s_traj) * MAX_JERK_COST_WEIGHT;
  //double ajs = avg_jerk_cost(s_traj) * AVG_JERK_COST_WEIGHT;
  //double mjd = max_jerk_cost(d_traj) * MAX_JERK_COST_WEIGHT;
  //double ajd = avg_jerk_cost(d_traj) * AVG_JERK_COST_WEIGHT;
  //double tdiff = time_diff_cost(target_time, actual_time) * TIME_DIFF_COST_WEIGHT;
  //double strajd = traj_diff_cost(s_traj, target_s) * TRAJ_DIFF_COST_WEIGHT;
  //double dtrajd = traj_diff_cost(d_traj, target_d) * TRAJ_DIFF_COST_WEIGHT;

}


#endif