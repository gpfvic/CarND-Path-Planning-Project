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
    // Constructors
    Vehicle();

    Vehicle(int lane, double s, double v, double a, string state = "KL");

    // Destructor
    virtual ~Vehicle();

    // Vehicle functions
    void update(double x_in, double y_in, double car_s, double car_d, double car_yaw, double car_speed);
    vector<Vehicle> choose_next_state(vector<Vehicle> &predictions);

    vector<string> successor_states();

    vector<Vehicle> generate_trajectory(string state,
                                         vector<Vehicle> &predictions);

    vector<double> get_kinematics(vector<Vehicle> &predictions, int lane);

    vector<Vehicle> keep_lane_trajectory( vector<Vehicle> &predictions);

    vector<Vehicle> lane_change_trajectory(string state,
                                           vector<Vehicle> &predictions);

    vector<Vehicle> prep_lane_change_trajectory(string state,
                                                vector<Vehicle> &predictions);


    bool get_vehicle_behind( vector<Vehicle> &predictions, int lane,
                            Vehicle &rCar);

    bool get_vehicle_ahead( vector<Vehicle> &predictions, int lane,
                           Vehicle &rCar);

    void realize_next_state(vector<Vehicle> &trajectory);

    map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

    int L = 1,lane_min = 0, lane_max = 2;
    
    int preferred_buffer = 10; // impacts "keep lane" behavior.

    int lane, s, goal_lane, goal_s, lanes_available = 3;

    double x, y, yaw, current_speed, d,target_speed, a, max_acceleration, waste_debug;

    string state;

    bool too_close;
};
#endif
