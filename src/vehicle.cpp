#include "vehicle.h"
#include "cost.h"

#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <string>

void Vehicle::update(double x_in, double y_in, double s_in, double d_in, double yaw_in, double speed_in)
{ 
    x = x_in;
    y = y_in;
    s = s_in;
    d = d_in;
    yaw = yaw_in;
    waste_debug = speed_in;
}
// Initializes Vehicle
Vehicle::Vehicle(){
    this->lane = 1;
    this->s = 0;
    this->current_speed = 0;
    this->a = 0;
    this->state = "KL";
    max_acceleration = 0.224;
    this->target_speed = 49.5;
}

Vehicle::Vehicle(int lane, double s, double v, double a, string state)
{
    this->lane = lane;
    this->s = s;
    this->current_speed = v;
    this->a = a;
    this->state = state;
    max_acceleration = 0.224;
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
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
    {
        vector<Vehicle> trajectory = generate_trajectory(*it, other_cars);
        if (trajectory.size() != 0)
        {
        
            cost = calculate_cost(*this, other_cars, trajectory);
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
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if (state.compare("KL") == 0)
    {
        states.push_back("PLCL");
        states.push_back("PLCR");
    }
    else if (state.compare("PLCL") == 0)
    {
        if (lane != 0)
        {
            states.push_back("PLCL");
            states.push_back("LCL");
            
        }
    }
    else if (state.compare("PLCR") == 0)
    {
        if (lane != lanes_available - 1)
        {
            states.push_back("PLCR");
            states.push_back("LCR");
            
        }
    }
    //std::cout << " v2" << std::endl;
    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, vector<Vehicle> &other_cars)
{
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    vector<Vehicle> trajectory;

    if (state.compare("KL") == 0)
    {
        trajectory = keep_lane_trajectory(other_cars);
        //std::cout << " 3.2" << std::endl;
    }
    // lane change left
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
    {
        trajectory = lane_change_trajectory(state, other_cars);
        //std::cout << " 3.3" << std::endl;
    }
    // prepare lane change right 
    else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
    {
        trajectory = prep_lane_change_trajectory(state, other_cars);
        //std::cout << " 3.4" << std::endl;
    }
    return trajectory;
}
 
vector<double> Vehicle::get_kinematics(vector<Vehicle> &other_cars, int lane)
{
    // Gets next timestep kinematics (position, velocity, acceleration)
    //   for a given lane. Tries to choose the maximum velocity and acceleration,
    //   given other vehicle positions and accel/velocity constraints.
    double max_velocity_accel_limit = this->max_acceleration + this->current_speed;
    double min_velocity_accel_limit = this->current_speed - this->max_acceleration;
    double new_velocity;
    double new_accel;
    Vehicle car_ahead;
    Vehicle car_behind;
    double nix = 0;

    if (get_vehicle_ahead(other_cars, lane, car_ahead))
    {
        std::cout << " v4.1" << std::endl;
        if (get_vehicle_behind(other_cars, lane, car_behind))
        {
            // must travel at the speed of traffic, regardless of preferred buffer
            new_velocity = car_ahead.current_speed;
            std::cout << " v4.1.1" << std::endl;
            std::cout << " vehicle_ahead.v " << car_ahead.current_speed << std::endl;
            if (car_ahead.current_speed >= this->current_speed)  // acelearation +- depending on the situation
            {
                new_velocity = std::min(std::min(car_ahead.current_speed,
                                                 max_velocity_accel_limit),
                                        this->target_speed);
                std::cout << " new velocity  " << new_velocity << std::endl;
            }
            else
            {
                new_velocity = std::max(std::max(car_ahead.current_speed,
                                                 min_velocity_accel_limit),
                                        nix);
                std::cout << " new velocity  " << new_velocity << std::endl;
            }
        }
        else
        {
            double max_velocity_in_front = (car_ahead.s - this->s - this->preferred_buffer) + car_ahead.current_speed - 0.5 * (this->a); // Does not work due to no real time simulation
            // Use Case Split Acceleration, Decceleration. To use the right limits.
            if (max_velocity_in_front >= this->current_speed)
            {
                new_velocity = std::min(std::min(max_velocity_in_front,
                                                 max_velocity_accel_limit),
                                        this->target_speed);
                std::cout << " v4.1.2" << std::endl;
                std::cout << " max_velocity_in_front" << max_velocity_in_front << std::endl;
                std::cout << " max_velocity_accel_limit " << max_velocity_accel_limit << std::endl;
                std::cout << " a " << this->a << std::endl;
            }
            else
            {
                new_velocity = std::max(std::max(max_velocity_in_front,
                                                 min_velocity_accel_limit),
                                        nix);
                std::cout << " v4.1.3" << std::endl;
                std::cout << " max_velocity_in_front" << max_velocity_in_front << std::endl;
                std::cout << " max_velocity_accel_limit " << max_velocity_accel_limit << std::endl;
                std::cout << " a " << this->a << std::endl;
            }
        }
    }
    else
    {
        new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
        std::cout << " v4.2" << std:: endl;
    }

    new_accel = new_velocity - this->current_speed; // Equation: (v_1 - v_0)/t = acceleration
    double new_position = this->s + new_velocity + new_accel / 2.0;
    
    return {new_position, new_velocity, new_accel};
}


vector<Vehicle> Vehicle::keep_lane_trajectory(vector<Vehicle> &other_cars)
{
    // Generate a keep lane trajectory.
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->current_speed, this->a, state)};
    vector<double> kinematics = get_kinematics(other_cars, this->lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    //std::cout << " v6" << std::endl;
    std::cout << " v6 new v " << new_v  << std::endl;
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state,
                                                     vector<Vehicle> &other_cars)
{
    // Generate a trajectory preparing for a lane change.
    std::cout << " v7" << std::endl;
    double new_s;
    double new_v;
    double new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->current_speed, this->a,
                                          this->state)};
    vector<double> curr_lane_new_kinematics = get_kinematics(other_cars, this->lane);

    if (get_vehicle_behind(other_cars, this->lane, vehicle_behind))
    {
        // Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
        // std::cout << " v7.1 new_v" << new_v << std::endl;
    }
    else
    {
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = get_kinematics(other_cars, new_lane);
        //std::cout << " v7.2" <<  std::endl;
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
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    //std::cout << " v7" << std::endl;
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, vector<Vehicle> &other_cars)
{
    std::cout << " v8.0" << std::endl;
    // Generate a lane change trajectory.
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    // Check if a lane change is possible (check if another vehicle occupies
    //   that spot).
    for (int i = 0; i < other_cars.size(); i++)
    {
        next_lane_vehicle = other_cars[i];
        if (((next_lane_vehicle.s > (this->s - 10)) && ((next_lane_vehicle.s - this->s) < 20)) && next_lane_vehicle.lane == new_lane)
        {
            // If lane change is not possible, return empty trajectory.
            std::cout << " v8.1" << std::endl;
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->current_speed, this->a, this->state));
    vector<double> kinematics = get_kinematics(other_cars, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1],
                                 kinematics[2], state));

    return trajectory;
}


bool Vehicle::get_vehicle_behind(vector<Vehicle> &other_cars, int lane, Vehicle &rCar)
{
    // Returns a true if a vehicle is found behind the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (int i = 0; i < other_cars.size(); i++)
    {
        temp_vehicle = other_cars.at(i);
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s)
        {
            max_s = temp_vehicle.s;
            rCar = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(vector<Vehicle> &other_cars, int lane, Vehicle &rCar)
{
    // Returns a true if a vehicle is found ahead of the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    //std::cout << " v4.vah" << std::endl;
    double  min_s = 6945.554;  // Distance max_s from Cost function
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (int i = 0; i < other_cars.size(); i++)
    {
        temp_vehicle = other_cars.at(i);
        //std::cout << " temp_vehicle.lane" << temp_vehicle.lane << " This Lane "<< this->lane << std::endl;
        //std::cout << " temp_vehicle.s" << temp_vehicle.s << " This s " << this->s << " min s" << min_s << std::endl;
        //std::cout << " min.s" << temp_vehicle.s << " This s " << this->s << std::endl;
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > (this->s-20) && temp_vehicle.s < min_s)  // -20 are buffer or better predict the other s
        {
            min_s = temp_vehicle.s;
            rCar = temp_vehicle;
            found_vehicle = true;
            //std::cout << " v4.vah found" << std::endl;
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
    this->lane = next_state.lane;
    //this->s = next_state.s;
    this->a = next_state.a;

}