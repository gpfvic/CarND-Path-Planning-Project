#include "vehicle.h"
#include "cost.h"
#include "constants.h"

#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <string>

void Vehicle::update(double x, double y, double s, double d, double yaw, double speed)
{ 
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    // this->current_speed = speed;
}
// Initializes Vehicle
Vehicle::Vehicle(){
    this->current_lane = 1;
    this->s = 0;
    this->current_speed = 0;
    this->accel = 0;
    this->state = "KL";
    this->target_speed = SPEED_LIMIT;
}

Vehicle::Vehicle(int lane, double s, double v, double accel, string state)
{
    this->current_lane = lane;
    this->s = s;
    this->current_speed = v;
    this->accel = accel;
    this->state = state;
}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::choose_next_state(vector<Vehicle> &other_cars)
{
    /**
   * Here you can implement the transition_function code from the Behavior 
   *   Planning Pseudocode classroom concept.
   *
   * @param A other_cars map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   *
   * Functions:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and other_cars. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.

   */
    vector<string> fsm = successor_states();
    float cost;
    vector<float> costs;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = fsm.begin(); it != fsm.end(); ++it)
    {
        vector<Vehicle> trajectory = generate_trajectory(*it, other_cars);
        if (trajectory.size() != 0)
        {
            cost = compute_total_cost(*this, other_cars, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);

    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states()
{
    // Provides the possible next states given the current state for the FSM
    //   discussed in the course, with the exception that lane changes happen
    //   instantaneously, so LCL and LCR can only transition back to KL.
    vector<string> fsm;
    fsm.push_back("KL");
    if (this->state.compare("KL") == 0)
    {
        fsm.push_back("PLCL");
        fsm.push_back("PLCR");
    }
    else if (this->state.compare("PLCL") == 0)
    {
        if (current_lane != LEFT_LANE)
        {
            fsm.push_back("PLCL");
            fsm.push_back("LCL");
        }
    }
    // prepare to change to the right lane
    else if (this->state.compare("PLCR") == 0) 
    {
        if (current_lane != RIGHT_LANE)
        {
            fsm.push_back("PLCR");
            fsm.push_back("LCR");
        }
    }
    // If state is "LCL" or "LCR", then just return "KL"
    return fsm;
}

vector<Vehicle> Vehicle::generate_trajectory(string current_state, vector<Vehicle> &other_cars)
{
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    vector<Vehicle> trajectory;

    if (current_state.compare("KL") == 0)
    {
        trajectory = keep_lane_trajectory(other_cars);
    }
    // lane change left
    else if (current_state.compare("LCL") == 0 || current_state.compare("LCR") == 0)
    {
        trajectory  =lane_change_trajectory(current_state, other_cars);
    }
    // prepare lane change right 
    else if (current_state.compare("PLCL") == 0 || current_state.compare("PLCR") == 0)
    {
        trajectory = prep_lane_change_trajectory(current_state, other_cars);
    }
    return trajectory;
}
 
vector<double> Vehicle::get_kinematics(vector<Vehicle> &other_cars, int lane)
{
    // Gets next timestep kinematics (position, velocity, acceleration)
    //   for a given lane. Tries to choose the maximum velocity and acceleration,
    //   given other vehicle positions and accel/velocity constraints.
    double max_velocity_accel_limit = VELOCITY_INCREMENT_LIMIT + this->current_speed;
    double min_velocity_accel_limit = this->current_speed - VELOCITY_INCREMENT_LIMIT;
    double new_velocity;
    double new_accel;
    Vehicle car_ahead;
    Vehicle car_behind;
    double nix = 0;

    if (get_vehicle_ahead(other_cars, lane, car_ahead))
    {
        if (get_vehicle_behind(other_cars, lane, car_behind))
        {
            // must travel at the speed of traffic, regardless of preferred buffer
            new_velocity = car_ahead.current_speed;
            if (car_ahead.current_speed >= this->current_speed)  // acelearation +- depending on the situation
            {
                new_velocity = std::min(std::min(car_ahead.current_speed, max_velocity_accel_limit), this->target_speed);
            }
            else
            {
                new_velocity = std::max(std::max(car_ahead.current_speed, min_velocity_accel_limit),nix);
            }
        }
        else
        {
            double max_velocity_in_front = (car_ahead.s - this->s - FOLLOW_DISTANCE) + car_ahead.current_speed - 0.5 * (this->accel); // Does not work due to no real time simulation
            // Use Case Split Acceleration, Decceleration. To use the right limits.
            if (max_velocity_in_front >= this->current_speed)
            {
                new_velocity = std::min(std::min(max_velocity_in_front,max_velocity_accel_limit),this->target_speed);
            }
            else
            {
                new_velocity = std::max(std::max(max_velocity_in_front, min_velocity_accel_limit), nix);
            }
        }
    }
    else
    {
        new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
    }

    new_accel = new_velocity - this->current_speed; // Equation: (v_1 - v_0)/t = acceleration
    double new_position = this->s + new_velocity + new_accel / 2.0;
    
    return {new_position, new_velocity, new_accel};
}


vector<Vehicle> Vehicle::keep_lane_trajectory(vector<Vehicle> &other_cars)
{
    // Generate a keep lane trajectory.
    vector<Vehicle> trajectory = {Vehicle(current_lane, this->s, this->current_speed, this->accel, state)};
    vector<double> kinematics = get_kinematics(other_cars, this->current_lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_accel = kinematics[2];
    trajectory.push_back(Vehicle(this->current_lane, new_s, new_v, new_accel, "KL"));
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string current_state, vector<Vehicle> &other_cars)
{
    // Generate a trajectory preparing for a lane change.
    double new_s;
    double new_v;
    double new_accel;
    Vehicle vehicle_behind;
    int new_lane = this->current_lane + lane_direction_transition[current_state];
    vector<Vehicle> trajectory = {Vehicle(this->current_lane, this->s, this->current_speed, this->accel, this->state)};
    vector<double> curr_lane_new_kinematics = get_kinematics(other_cars, this->current_lane);

    if (get_vehicle_behind(other_cars, this->current_lane, vehicle_behind))
    {
        // Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_accel = curr_lane_new_kinematics[2];
    }
    else
    {
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = get_kinematics(other_cars, new_lane);
        // Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1])
        {
            best_kinematics = next_lane_new_kinematics;
        }
        else
        {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_accel = best_kinematics[2];
    }
    trajectory.push_back(Vehicle(this->current_lane, new_s, new_v, new_accel, current_state));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string current_state, vector<Vehicle> &other_cars)
{
    // Generate a lane change trajectory.
    int new_lane = this->current_lane + lane_direction_transition[current_state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    // Check if a lane change is possible (check if another vehicle occupies that spot).
    for (int i = 0; i < other_cars.size(); i++)
    {
        next_lane_vehicle = other_cars[i];
        if (((next_lane_vehicle.s > (this->s - 10)) && ((next_lane_vehicle.s - this->s) < 20)) && next_lane_vehicle.current_lane == new_lane)
        {
            // If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->current_lane, this->s, this->current_speed, this->accel, this->state));
    vector<double> kinematics = get_kinematics(other_cars, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1],
                                 kinematics[2], current_state));

    return trajectory;
}


bool Vehicle::get_vehicle_behind(vector<Vehicle> &other_cars, int lane, Vehicle &rCar)
{
    // Returns a true if a vehicle is found behind the current vehicle, false
    // otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle car_behind;
    for (int i = 0; i < other_cars.size(); i++)
    {
        car_behind = other_cars.at(i);
        if (car_behind.current_lane==this->current_lane && car_behind.s<this->s && car_behind.s>max_s)
        {
            max_s = car_behind.s;
            rCar = car_behind;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(vector<Vehicle> &other_cars, int lane, Vehicle &rCar)
{
    // Returns a true if a vehicle is found ahead of the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    double  min_s = MAX_S;  // Distance max_s from Cost function
    bool found_vehicle = false;
    Vehicle aheading_car;
    for (int i = 0; i < other_cars.size(); i++)
    {
        aheading_car = other_cars.at(i);
        if (aheading_car.current_lane == this->current_lane && aheading_car.s > (this->s-20) && aheading_car.s < min_s)  // -20 are buffer or better predict the other s
        {
            min_s = aheading_car.s;
            rCar = aheading_car;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

void Vehicle::realize_next_state(vector<Vehicle> &trajectory)
{
    // Sets state and kinematics for ego vehicle using the last state of the trajectory.
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->current_speed = next_state.current_speed;
    this->current_lane = next_state.current_lane;
    this->accel = next_state.accel;
}