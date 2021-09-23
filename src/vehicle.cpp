#include "vehicle.h"
#include "constants.h"
#include "helpers.h"

#include <iostream>
#include <sstream>
#include <map>
#include <fstream>
#include <cmath>
#include <iterator>
#include <random>
#include <algorithm>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle() {}

Vehicle::Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd)
{

  this->s = s;       // s position
  this->s_d = s_d;   // s dot - velocity in s
  this->s_dd = s_dd; // s dot-dot - acceleration in s
  this->d = d;       // d position
  this->d_d = d_d;   // d dot - velocity in d
  this->d_dd = d_dd; // d dot-dot - acceleration in d
  this->start_state = {s, s_d, s_dd, d, d_d, d_dd};
  state = "CS";
}

Vehicle::Vehicle(vector<double> start_state)
{
  this->start_state = start_state;
}

Vehicle::~Vehicle() {}

vector<double> Vehicle::state_in(double time)
{
  /*
    calculate the final target state after time t.
  */
  vector<double> s = {this->s, this->s_d, this->s_dd};
  vector<double> d = {this->d, this->d_d, this->d_dd};
  vector<double> state = {
      s[0] + (s[1] * time) + s[2] * time * time / 2.0,
      s[1] + s[2] * time,
      s[2],
      d[0] + (d[1] * time) + d[2] * time * time / 2.0,
      d[1] + d[2] * time,
      d[2]};
  return state;
}

void Vehicle::update_available_states(bool car_turning_left, bool car_turning_right,
                                      bool car_just_ahead, bool car_ahead_in_the_right_lane, bool car_ahead_in_the_left_lane)
{
  /*  Updates the available "states" based on the current state:
  "KL" - Keep Lane
   - The vehicle will attempt to drive its target speed, unless there is 
     traffic in front of it, in which case it will slow down.
  "LCL" or "LCR" - Lane Change Left / Right
   - The vehicle will change lanes and then follow longitudinal
     behavior for the "KL" state in the new lane. */

  this->available_states = {"KL"};
  if (this->d > 4 && !car_turning_left && !car_ahead_in_the_left_lane)
  {
    this->available_states.push_back("LCL");
  }
  if (this->d < 8 && !car_turning_right && !car_ahead_in_the_right_lane)
  {
    this->available_states.push_back("LCR");
  }
}

vector<double> Vehicle::get_target_for_state(string state,
                                             vector<Vehicle> other_cars, double traj_start_time,
                                             double duration, bool car_just_ahead)
{
  // 根据fsm状态，计算得出ego car最终的s，d
  // Returns two lists s_target and d_target in a single vector - s_target includes
  // [s, s_dot, and s_ddot] and d_target includes the same
  // If no leading car found target lane, ego car will make up PERCENT_V_DIFF_TO_MAKE_UP of the difference
  // between current velocity and target velocity. If leading car is found set target s to FOLLOW_DISTANCE
  // and target s_dot to leading car's s_dot based on other_cars
  int target_lane, current_lane = this->d / 4;
  double target_d;
  // **** TARGETS ****
  // lateral displacement : depends on state
  // lateral velocity : 0
  double target_d_d = 0;
  // lateral acceleration : 0
  double target_d_dd = 0;
  // longitudinal velocity : current velocity + max allowed accel * duration
  double target_s_d = min(this->s_d + MAX_INSTANTANEOUS_ACCEL / 4 * duration, SPEED_LIMIT);
  // longitudinal acceleration : zero ?
  double target_s_dd = 0;
  // longitudinal acceleration : difference between current/target velocity over trajectory duration?
  //double target_s_dd = (target_s_d - this->s_d) / (N_SAMPLES * DT);
  // longitudinal displacement : current displacement plus difference in current/target velocity times
  // trajectory duration
  double target_s = this->s + (this->s_d + target_s_d) / 2 * duration;

  if (state.compare("KL") == 0)
  {
    target_d = (double)current_lane * 4 + 2;
    target_lane = target_d / 4;
  }
  else if (state.compare("LCL") == 0)
  {
    target_d = ((double)current_lane - 1) * 4 + 2;
    target_lane = target_d / 4;
  }
  else if (state.compare("LCR") == 0)
  {
    target_d = ((double)current_lane + 1) * 4 + 2;
    target_lane = target_d / 4;
  }

  // replace target_s variables if there is a leading vehicle close enough
  vector<double> leading_vehicle_s_and_sdot = get_leading_vehicle_data_for_lane(target_lane, other_cars, traj_start_time, duration);
  double leading_vehicle_s = leading_vehicle_s_and_sdot[0];
  if (leading_vehicle_s - target_s < 10 && leading_vehicle_s > this->s)
  {

    target_s_d = leading_vehicle_s_and_sdot[1];

    if (fabs(leading_vehicle_s - target_s) < FOLLOW_DISTANCE)
    {
      target_s_d -= VELOCITY_SLOW_DOWN_EACH_STEP; // slow down if too close by velocity
    }

    target_s = leading_vehicle_s - FOLLOW_DISTANCE;
  }

  // emergency brake
  if (car_just_ahead)
  {
    target_s_d = 0.0;
  }

  return {target_s, target_s_d, target_s_dd, target_d, target_d_d, target_d_dd};
}

vector<double> Vehicle::get_leading_vehicle_data_for_lane(int target_lane,
                                                          vector<Vehicle> other_cars, double traj_start_time, double duration)
{
  // returns s and s_dot for the nearest (ahead) vehicle in target lane
  // this assumes the dummy vehicle will keep its lane and velocity, it will return the end position
  // and velocity (based on difference between last two positions)
  double nearest_leading_vehicle_speed = 0;
  double nearest_leading_vehicle_distance = 99999;

  for (Vehicle other_car : other_cars)
  {
    vector<double> predicted_state = other_car.state_in(duration);
    double predicted_d = predicted_state[3];
    int pred_lane = predicted_d / 4;
    if (pred_lane == target_lane)
    {
      double predicted_s = predicted_state[0];
      if (predicted_s < nearest_leading_vehicle_distance && other_car.s > this->s)
      {
        nearest_leading_vehicle_distance = predicted_state[0]; //  s
        nearest_leading_vehicle_speed = predicted_state[1];    // s_d
        // cout<<"LEADING CAR STATE: ["<<predicted_state[0]<<", "<<predicted_state[1]<<", "<<predicted_state[2]<<", "<<predicted_state[3]<<", "<<predicted_state[4]<<", "<<predicted_state[5]<<"]"<<endl;
      }
    }
  }
  return {nearest_leading_vehicle_distance, nearest_leading_vehicle_speed};
}

Trajectory Vehicle::generate_traj_for_target(vector<double> target_state, double duration)
{
  // takes a target target_state={s, s_dot, s_ddot , d, d_dot, d_ddot} and returns a Jerk-Minimized Trajectory
  // (JMT) connecting current state (s and d) to target state in a list of s points and a list of d points
  // ex. {{s1, s2, ... , sn}, {d1, d2, ... , dn}}

  Trajectory traj;
  traj.target_state = target_state;

  //vector [initial_s, init_v, init_accel, final_s, final_v, final_accel]
  vector<double> target_s = {target_state[0], target_state[1], target_state[1]};
  vector<double> target_d = {target_state[3], target_state[4], target_state[5]};
  vector<double> current_s = {this->s, this->s_d, this->s_dd};
  vector<double> current_d = {this->d, this->d_d, this->d_dd};

  // determine coefficients of optimal JMT
  // jmt return the  6 polynomial coefficents
  this->s_traj_coeffs = jmt(current_s, target_s, duration);
  this->d_traj_coeffs = jmt(current_d, target_d, duration);

  // // populate s and t trajectories at each time step
  string outd = "Ds : ";
  string outs = "Ss : ";
  vector<double> s_traj;
  vector<double> d_traj;
  for (int i = 0; i < N_SAMPLES; i++)
  {
    double t = i * duration / N_SAMPLES;
    double s_val = 0, d_val = 0;
    for (int j = 0; j < s_traj_coeffs.size(); j++)
    {
      s_val += this->s_traj_coeffs[j] * pow(t, j);
      d_val += this->d_traj_coeffs[j] * pow(t, j);
    }
    s_traj.push_back(s_val);
    d_traj.push_back(d_val);
    outd = outd + to_string(d_val) + ", ";
    outs = outs + to_string(s_val) + ", ";
  }

  // cout<<"JMT D :["<<this->d_traj_coeffs[0]<<", "<<this->d_traj_coeffs[1]<<", "<<this->d_traj_coeffs[2]<<", "<<this->d_traj_coeffs[3]<<", "<<this->d_traj_coeffs[4]<<", "<<this->d_traj_coeffs[5]<<"]"<<endl;
  // cout << outd<<endl;
  // cout << outs<<endl;
  // cout<<"Traj D="<<d_traj.back()<<"\tvs\tTarget D="<<target_d[0]<<"\t\tduration="<<duration<<endl;

  traj.s = this->s_traj_coeffs;
  traj.d = this->d_traj_coeffs;
  traj.t = duration;

  return traj;
}
