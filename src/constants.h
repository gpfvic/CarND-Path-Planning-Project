#ifndef CONSTANTS
#define CONSTANTS


#define MAX_S 6945.554

// cost function weights
#define TIME_DIFF_COST        0.6
#define S_DIFF_COST           1
#define D_DIFF_COST           1.6
#define BUFFER_COST           20
#define STAY_IN_LANE_COST     1
#define EFFICIENCY_COST       20
#define MAX_JERK_COST         1
#define TOTAL_JERK_COST       1
#define COLLISION_COST        100
#define MAX_ACCEL_COST        1
#define TOTAL_ACCEL_COST      1
#define OUT_LANE_COST         0.3
#define NOT_IN_MIDDLE_LANE_COST 1.3


#define VEHICLE_RADIUS 1.25            // meters
#define FOLLOW_DISTANCE 8.0              // distance to keep behind leading cars
#define SAFETY_DISTANCE 10.0


// constants
const vector<double> SIGMA_S = {10.0, 3.0, 2.0}; // s, s_dot, s_double_dot
const vector<double> SIGMA_D = {1.0,  1.0, 1.0};


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
#define VELOCITY_INCREMENT_LIMIT 0.1
#define VELOCITY_SLOW_DOWN_EACH_STEP  3 // m/s

#define MAX_INSTANTANEOUS_ACCEL 10      // m/s/s




#endif