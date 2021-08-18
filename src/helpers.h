#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <string>
#include <vector>
#include <cassert>
#include <algorithm>
#include <cmath>
#include <map>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"
#include "constants.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using namespace std;
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

vector<double> jmt(vector<double> start, vector<double> end, double T){
  /*
    Calculates Jerk Minimizing Trajectory for start, end and T.
    INPUTS  
    start - the vehicles initial location [s, s_dot, s_dot_dot]
    end   - the desired end location [s, s_dot, s_dot_dot]
    T     - the duration of maneuver in seconds.
    OUTPUT 
    output  - 6 polynomial coefficents for the function 
      s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
  */
  double T2 = T * T;
  double T3 = T2 * T;
  double T4 = T3 * T;
  double T5 = T4 * T;
  
  MatrixXd a(3,3);
  a <<  T3, T4, T5,
        3*T2, 4*T3, 5*T4,
        6*T,  12*T2,  20*T3;
  MatrixXd aInv = a.inverse();

  VectorXd b(3);
  b <<  end[0]  - (start[0]  +  start[1]*T  + 0.5*start[2]*T2),
        end[1]  - (             start[1]    + start[2]*T),
        end[2]  - (                           start[2]);
  VectorXd alphas = aInv * b;
  vector<double> output = {start[0],start[1],0.5*start[2],alphas[0],alphas[1],alphas[2]};
  return output;
}


vector<double> interpolate_points(vector<double> ptsx, vector<double> ptsy,
                            double interval, int output_size){
  if(ptsx.size()!=ptsy.size()){
    cout <<"ERROR! Spline: interpolate points size is not matched!" << endl;
    return {0};
  }
  tk::spline s;
  s.set_points(ptsx,ptsy);
  vector<double> output;
  for(int i=0;i<output_size;i++){
    output.push_back(s(ptsx[0]+i*interval));
  }
  return output;
}

vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y, 
                                  vector<double> eval_at_x) {
  // uses the spline library to interpolate points connecting a series of x and y values
  // output is spline evaluated at each eval_at_x point

  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
    return { 0 };
  }

  tk::spline s;
  s.set_points(pts_x,pts_y);    // currently it is required that X is already sorted
  vector<double> output;
  for (double x: eval_at_x) {
    output.push_back(s(x));
  }
  return output;
}

double nearest_approach(vector<double> s_traj, vector<double> d_traj, vector<vector<double>> prediction){
  double closest = 99999;
  for (int i=0; i<N_SAMPLES;i++){
    double current_dist = sqrt(pow(s_traj[i]-prediction[i][0],2) + pow(d_traj[i] - prediction[i][1], 2));
    if(current_dist < closest){
      closest = current_dist;
    }
  }
  return closest;
}

double nearest_approach_to_any_vehicle(vector<double> s_traj, vector<double> d_traj, 
                                      map<int, vector<vector<double>>> predictions){
  double closest = 99999;
  for(auto prediction: predictions){
    double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
    if(current_dist<closest){
      closest = current_dist;
    }
  }
  return closest;
}

double nearest_approach_to_any_vehicle_in_lane(vector<double> s_traj, vector<double> d_traj,  map<int,vector<vector<double>>> predictions) {
  double closest = 9999;
  for(auto prediction: predictions){
    double my_final_d = d_traj[d_traj.size() - 1];
    int my_lane = my_final_d / 4; //4 meters
    vector<vector<double>> pred_traj = prediction.second;
    double pred_final_d = pred_traj[pred_traj.size()-1][1];
    int pred_lane = pred_final_d / 4;
    if(my_lane == pred_lane){
      double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
      if(current_dist<closest && current_dist<120){
        closest = current_dist;
      }
    }
    return closest;
  }
}

vector<double> velocities_for_trajectory(vector<double> traj){
  // get the average velocities for the given trajectory which is made up by predicted points of the road
  vector<double> vels;
  for (int i=1;i<traj.size();i++){
    vels.push_back( (traj[i] - traj[i-1])/DT);
  }
  return vels;
}





double logistic(double x){
  return 2.0 / (1+exp(-x)) - 1.0;
}



#endif  // HELPERS_H