#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <map>

#include "Eigen-3.3/Eigen/Dense"
#include "vehicle.h"

// for convenience
using namespace std;
using std::string;
using std::vector;

struct Trajectory{
  vector<double> s; //size = 6
  vector<double> d;
  double t; //time
}

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

vector<Vehicle> check_other_vehicles_info_by_sensor_fusion(const vector<vector<double>> &sensor_fusion, Vehicle &ego_car, int &prev_size)
{

    // get the other cars information through the sensor fusion data
    vector<Vehicle> other_cars; 
    
    for (int i = 0; i < sensor_fusion.size(); i++)
    {   
        vector<int> lane_ids{0,1,2};
        float d = sensor_fusion[i][6]; 
        // Check is objeckt is in the same line
        for( int j=0; j<lane_ids.size(); j++ )
        {
            if (d<(2+4*lane_ids[j]+2) && d>(2+4*lane_ids[j]-2))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];
                //if using previous point can project s value out ...
                check_car_s += ((double)prev_size * .02 * check_speed);
                // check s values greater than mine and car's save interval distance
                if ((check_car_s > (ego_car.s - 10)) && ((check_car_s - ego_car.s) < 50))
                {
                    other_cars.push_back(Vehicle(lane_ids[j], check_car_s, check_speed, 0));
                }  
            }
        }
    }
    return other_cars;
}


double logistic(x):{
  /*
    A function that returns a value between 0 and 1 for x in the 
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
  */
  return 2.0 / (1+exp(-x)) -1.0;
}

double calculate_current_position_shifting(double t, vector<double> coefficients){
  /*
    Takes the coefficients of a polynomial and creates a function of
    time from them. Then calculate.
  */
  double total = 0.0;
  for (int i=0; i<coefficients.size();i++){
    total += coefficients[i] * pow(t, i);
  }
  return total;
}

vector<double> differentiate(vector<double> coefficients){
  /* 
    Calculates the derivative of a polynomial and returns
    the corresponding coefficients.
   */
  vector<double> new_cos;
  for(int deg=0;deg<coefficients.size()-1;deg++){
    new_cos.push_back( (deg+1)* coefficients[deg+1]);
  }
  return new_cos;
}

double nearest_approach_to_any_vehicle(Trajectory traj, map<int,Vehicle> predictions){
  /* 
    Calculates the closest distance to any vehicle during a trajectory.
   */
  double closest = 9999999;
  for(int i=0;i<predictions.size();i++){
    dist = nearest_approach(traj, predictions[i]);
    if(dist<closest){
      closest = dist;
    }
  }
  return closest;
}



double nearest_approach(Trajectory traj, Vehicle vehicle){
  double closest = 999999;
  vector<double> s_ = traj.s;
  vector<double> d_ = traj.d;
  double T = traj.t;

  for (int i=0;i<100;i++){
    t = float(i)/100 * T;
    cur_s = calculate_current_position_shifting(t, s_);
    cur_d = calculate_current_position_shifting(t, d_);
    vector<double> state = vehicle.state_in(t);
    double trag_s = state[0];
    double targ_d = state[3];
    dist = sqrt(pow(cur_s-trag_s, 2) + pow(cur_d-trag_d, 2));
    if(dist<closest){
      closest = dist;
    }
  }
  return closest;
}



vector<double> get_f_and_N_derivatives(double t, vector<double> coefficients, double N){
  vector<double> S;
  S.push_back(calculate_current_position_shifting(t,coefficients));
  for(int i=0; i<N;i++){
    coefficients =  differentiate(coefficients);
    S.push_back(calculate_current_position_shifting(t,coefficients));
  }
  return S;
}

vector<double> jmt(vector<double> start, vector<double> end, double T){
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
  MatrixXd a(3,3);
  double T2 = T*T;
  double T3 = T2*T;
  double T4 = T3*T;
  double T5 = T4*T;

  a <<    T3,   T4,   T5,
        3*T2,  4*T3, 5*T3,
        6*T,  12*T2, 20*T3;
  MatrixXd aInv = a.inverse();

  VectorXd b(3);
  b <<  end[0] - (start[0]+start[1]*T+0.5*start[2]*T2),
        end[1] - (         start[1]   + start[2]*T),
        end[2] - (                      start[2]);
  
  VectorXd alpha = aInv * b;
  vector<double> coeffs = {start[0], start[1], 0.5*start[2], alpha[0], alpha[1], alpha[2]};
  return coeffs;



}

#endif  // HELPERS_H