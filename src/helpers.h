#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <map>
#include <math.h>

#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"
#include "vehicle.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using namespace std;
using std::string;
using std::vector;

// Calculate distance between two points
static double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate closest waypoint to current x, y position
static int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                           const vector<double> &maps_y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
static int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                        const vector<double> &maps_y)
{
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * M_PI - angle, angle);

  if (angle > M_PI / 2)
  {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
static vector<double> getFrenet(double x, double y, double theta,
                                const vector<double> &maps_x,
                                const vector<double> &maps_y,
                                vector<double> maps_s)
{
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0)
  {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = maps_s[0];
  for (int i = 0; i < prev_wp; ++i)
  {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
static vector<double> getXY(double s, double d, const vector<double> &maps_s,
                            const vector<double> &maps_x,
                            const vector<double> &maps_y)
{
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
  {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - M_PI / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

static double logistic(double x)
{
  /*
    A function that returns a value between 0 and 1 for x in the 
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
  */
  return 2.0 / (1 + exp(-x)) - 1.0;
}

static double to_equation(double t, vector<double> coefficients)
{
  /*
    Takes the coefficients of a polynomial and creates a function of
    time from them. Then calculate.
  */
  double total = 0.0;
  for (int i = 0; i < coefficients.size(); i++)
  {
    total += coefficients[i] * pow(t, i);
  }
  return total;
}

static vector<double> differentiate(vector<double> coefficients)
{
  /* 
    Calculates the derivative of a polynomial and returns
    the corresponding coefficients.
   */
  vector<double> new_cos;
  for (int deg = 0; deg < coefficients.size() - 1; deg++)
  {
    new_cos.push_back((deg + 1) * coefficients[deg + 1]);
  }
  return new_cos;
}

static double nearest_approach(Trajectory traj, Vehicle car)
{
  double closest = 999999;
  vector<double> s_ = traj.s;
  vector<double> d_ = traj.d;

  for (int i = 0; i < 100; i++)
  {
    double t = float(i) / 100 * (traj.t+ car.traj_start_time);
    double cur_s = to_equation(t, s_);
    double cur_d = to_equation(t, d_);
    vector<double> state = car.state_in(t);
    double targ_s = state[0];
    double targ_d = state[3];
    double dist = sqrt(pow(cur_s - targ_s, 2) + pow(cur_d - targ_d, 2));
    if (dist < closest)
    {
      closest = dist;
    }
  }
  return closest;
}

static double nearest_approach_to_any_vehicle(Trajectory traj, vector<Vehicle> other_cars)
{
  /* 
    Calculates the closest distance to any vehicle during a trajectory.
   */
  double closest = 9999999;
  for (Vehicle other_car : other_cars)
  {
    double dist = nearest_approach(traj, other_car);
    if (dist < closest)
    {
      closest = dist;
    }
  }
  return closest;
}

static vector<double> get_f_and_N_derivatives(double t, vector<double> coefficients, double N)
{
  vector<double> S;
  S.push_back(to_equation(t, coefficients));
  for (int i = 0; i < N; i++)
  {
    coefficients = differentiate(coefficients);
    S.push_back(to_equation(t, coefficients));
  }
  return S;
}

static vector<double> jmt(vector<double> start, vector<double> end, double T)
{
  /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T. Get the tragectory coeffs.
    INPUTS
    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]
    end   - the desired end state for vehicle. Like "start" this is a
        length three array.
    T     - The duration, in seconds, over which this maneuver should occur.
    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    */
  MatrixXd a(3, 3);
  double T2 = T * T;
  double T3 = T2 * T;
  double T4 = T3 * T;
  double T5 = T4 * T;

  a << T3, T4, T5,
      3 * T2, 4 * T3, 5 * T4,
      6 * T, 12 * T2, 20 * T3;
  MatrixXd aInv = a.inverse();

  VectorXd b(3);
  b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T2),
      end[1] - (start[1] + start[2] * T),
      end[2] - (start[2]);

  VectorXd alpha = aInv * b;
  vector<double> coeffs = {start[0], start[1], 0.5 * start[2], alpha[0], alpha[1], alpha[2]};
  return coeffs;
}

static vector<double> interpolate_points(vector<double> ptsx, vector<double> ptsy, double interval, int output_size)
{
  // uses the spline library to interpolate points connecting a series of x and y values
  // output is output_size number of y values beginning at y[0] with specified fixed interval
  if (ptsx.size() != ptsy.size())
  {
    return {0};
  }
  vector<double> output;
  tk::spline s;
  s.set_points(ptsx, ptsy);
  for (int i = 0; i < output_size; i++)
  {
    output.push_back(s(ptsx[0] + i * interval));
  }
  return output;
}

static vector<double> interpolate_points(vector<double> ptsx, vector<double> ptsy, vector<double> eval_at_x)
{
  if (ptsx.size() != ptsy.size())
  {
    return {0};
  }
  tk::spline s;
  s.set_points(ptsx, ptsy);
  vector<double> output;
  for (double x : eval_at_x)
  {
    output.push_back(s(x));
  }
  return output;
}

#endif // HELPERS_H