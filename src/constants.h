#ifndef CONSTANTS
#define CONSTANTS



// cost function weights
#define TIME_DIFF_COST        10
#define S_DIFF_COST           100
#define D_DIFF_COST           100
#define EFFICIENCY_COST       10
#define MAX_JERK_COST         100
#define TOTAL_JERK_COST       10
#define COLLISION_COST        100
#define MAX_ACCEL_COST        3
#define TOTAL_ACCEL_COST      1



// constants
#define SIGMA_S = [10.0, 4.0, 2.0]  // s, s_dot, s_double_dot
#define SIGMA_D = [1.0, 1.0, 1.0]
#define VEHICLE_RADIUS 1.5            // meters

#define MAX_JERK  10 // m/s/s/s
#define MAX_ACCEL 10 // m/s/s

#define EXPECTED_ACC_IN_ONE_SEC  1     // m/s
#define EXPECTED_JERK_IN_ONE_SEC   2  // m/s/s




/////////////////////////////////////////////////////////////////////////////////////////
#define REACH_GOAL 10
#define EFFICIENCY 1000
#define OFFROAD 100000
#define LANECHANGE 100
#define MAX_S 6945.554 

#define LANE_NUMBER 3
#define FOLLOW_DISTANCE 8                // distance to keep behind leading cars

#define LEFT_LANE 0
#define MIDDLE_LANE 1
#define RIGHT_LANE 2

#define SPEED_LIMIT 49.6            // mph
#define VELOCITY_INCREMENT_LIMIT 0.125

#define KEEP_IN_GOAL_LANE_ID 1




/////////////////////////////////////////////////////////////////////////////////////////
#define VEHICLE_RADIUS 1.25              // meters

#define PREVIOUS_PATH_POINTS_TO_KEEP 25
#define NUM_PATH_POINTS 50
#define PATH_DT 0.02                    // seconds

#define TRACK_LENGTH 6945.554           // meters

// number of waypoints to use for interpolation
#define NUM_WAYPOINTS_BEHIND 5
#define NUM_WAYPOINTS_AHEAD 5

// for trajectory generation/evaluation and non-ego car predictions
#define N_SAMPLES 20
#define DT 0.20                         // seconds



// cost function weights
#define COLLISION_COST_WEIGHT 99999
#define BUFFER_COST_WEIGHT 10
#define IN_LANE_BUFFER_COST_WEIGHT 1000
#define EFFICIENCY_COST_WEIGHT 10000
#define NOT_MIDDLE_LANE_COST_WEIGHT 100


// DEPRECATED CONSTANTS
#define NUM_RANDOM_TRAJ_TO_GEN 4        // the number of perturbed trajectories to generate (for each perturbed duration)
#define NUM_TIMESTEPS_TO_PERTURB 2      // the number of timesteps, +/- target time, to perturb trajectories

// sigma values for perturbing targets
#define SIGMA_S 10.0                     // s
#define SIGMA_S_DOT 3.0                 // s_dot
#define SIGMA_S_DDOT 0.1                  // s
#define SIGMA_D 0                       // d
#define SIGMA_D_DOT 0                   // d_dot
#define SIGMA_D_DDOT 0                  // d_double_dot
#define SIGMA_T 0.05

#define MAX_INSTANTANEOUS_JERK 10       // m/s/s/s
#define MAX_INSTANTANEOUS_ACCEL 10      // m/s/s

#define EXPECTED_JERK_IN_ONE_SEC 2      // m/s/s
#define EXPECTED_ACC_IN_ONE_SEC 1       // m/s

#define PERCENT_V_DIFF_TO_MAKE_UP 0.5   // the percent difference between current velocity and target velocity to allow ego car to make up in a single trajectory  

#endif