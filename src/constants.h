#ifndef CONSTANTS
#define CONSTANTS


#define MAX_S 6945.554

// cost function weights
#define TIME_DIFF_COST        10
#define S_DIFF_COST           100
#define D_DIFF_COST           100
#define EFFICIENCY_COST       10
#define MAX_JERK_COST         100
#define TOTAL_JERK_COST       10
#define COLLISION_COST        1000
#define MAX_ACCEL_COST        3
#define TOTAL_ACCEL_COST      1


#define VEHICLE_RADIUS 1.5               // meters
#define FOLLOW_DISTANCE 8.0              // distance to keep behind leading cars


// constants
const vector<double> SIGMA_S = {10.0, 4.0, 2.0}; // s, s_dot, s_double_dot
const vector<double> SIGMA_D = {1.0, 1.0, 1.0};


#define MAX_JERK  10 // m/s/s/s
#define MAX_ACCEL 10 // m/s/s

#define EXPECTED_ACC_IN_ONE_SEC  1     // m/s
#define EXPECTED_JERK_IN_ONE_SEC   2  // m/s/s

//
#define NUM_WAYPOINTS_BEHIND  5
#define NUM_WAYPOINTS_AHEAD   5

#define NUM_PATH_POINTS 50
#define PREVIOUS_PATH_POINTS_TO_KEEP 25
#define PATH_DT 0.02                    // seconds

// for trajectory generation/evaluation and non-ego car predictions
#define N_SAMPLES 20
#define DT 0.20                         // seconds

#define SPEED_LIMIT 21.5                // m/s
#define VELOCITY_INCREMENT_LIMIT 0.125

#define MAX_INSTANTANEOUS_ACCEL 10      // m/s/s




#endif