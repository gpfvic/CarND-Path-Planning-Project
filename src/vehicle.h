#ifndef VEHICLE
#define VEHICLE

#include <string>
#include <vector>
#include <map>

using std::string;
using std::vector;
using namespace std;

class Vehicle
{
public:

    int current_lane;
    

    double x;
    double y;
    double yaw;
    double s;
    double d;
    double current_speed; 
    double target_speed;
    double accel;
    string state;
    bool too_close;

    map<string, int> lane_direction_transition = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

    // Constructors
    Vehicle();

    Vehicle(int lane, double s, double v, double a, string state = "KL");

    // Destructor
    virtual ~Vehicle();

    // Vehicle functions
    void update(double x_in, double y_in, double car_s, double car_d, double car_yaw, double car_speed);

    void transition_to_next_state(vector<Vehicle> &predictions);

    vector<string> successor_states();

    vector<Vehicle> generate_trajectory(string state, vector<Vehicle> &predictions);

    vector<double> get_kinematics(vector<Vehicle> &predictions, int lane);

    vector<Vehicle> keep_lane_trajectory( vector<Vehicle> &predictions);

    vector<Vehicle> lane_change_trajectory(string state, vector<Vehicle> &predictions);

    vector<Vehicle> prep_lane_change_trajectory(string state, vector<Vehicle> &predictions);

    bool get_vehicle_behind( vector<Vehicle> &predictions, int lane, Vehicle &rCar);

    bool get_vehicle_ahead( vector<Vehicle> &predictions, int lane, Vehicle &rCar);


};
#endif
