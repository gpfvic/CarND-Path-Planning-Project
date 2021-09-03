#include "vehicle.h"
#include "cost.h"
#include "constants.h"

#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <string>

Vehicle::Vehicle() {}

Vehicle::Vehicle(vector<double> start_state) {
  this->start_state = start_state;
}

vector<double> state_in(double time){
  vector<double> s(this->start_state.begin(), this->start_state.begin()+2);
  vector<double> d(this->start_state.begin()+3, this->start_state.end());
  vector<double> state = {
        s[0] + (s[1]*time) + pow(s[2]*t,2)/2.0,
        s[1] + s[2]*time,
        s[2],
        d[0] + (d[1]*t) + pow(d[2]*t,2)/2.0,
        d[1] + d[2]*t,
        d[2]
  };
  return state;
}