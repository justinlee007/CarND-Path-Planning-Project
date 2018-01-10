#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <cstdio>
#include <vector>
#include "json.hpp"
#include "PathPlanner.h"
#include "VehicleController.h"
#include "LaneStateController.h"
#include "CoordinateUtils.h"

using json = nlohmann::json;

/**
 * An optimal and collision-free trajectory.
 */
struct Trajectory {
  vector<double> points_x; // x points of unfiltered trajectory
  vector<double> points_y; // y points of unfiltered trajectory
};

/**
 * Previous trajectory's points and Frenet end points.
 */
struct PreviousTrajectory {
  json points_x; // x points of previous jerk free trajectory
  json points_y; // y points of previous jerk free trajectory
  double end_point_s; // s Frenet end coordinate of previous trajectory
};

/**
 * Determines the optimal, collision free, jerk free and drivable trajectory for the desired behavior provided by the lane state controller.
 */
class PathPlanner {
 public:

  /**
   * Initializes the path planner.
   *
   * @param vehicle_controller pointer to the vehicle controller instance
   * @param lane_state_controller pointer to lane state controller instance
   * @param start_lane current driving lane [0=left, 1=center, 2=right]
   */
  PathPlanner(VehicleController *vehicle_controller, LaneStateController *lane_state_controller, int start_lane);

  /**
   * Set the map waypoints.
   */
  void setMapWaypoints(vector<double> &x, vector<double> &y, vector<double> &s);

  /**
   * Finds the optimal and collision-free spline-based trajectory to achieve target behavior.
   *
   * @param previous_trajectory previous trajectory provided by the simulator
   * @return
   */
  Trajectory planSplineTrajectory(PreviousTrajectory &previous_trajectory);

 private:
  static constexpr double MAX_DELTA_VELOCITY = 0.09; // Max delta velocity [m/s] between two waypoints to guarantee accelerations < 10 m/s2 jerks < 10 m/s3
  static constexpr double MIN_TIME_GAP = 2.0; // min allowed time gap to vehicle ahead [s]
  static constexpr double MIN_TIME_GAP_LANE_CHANGE = 1.0; // min allowed time gap to vehicle ahead/behind during a lane change [s]

  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  vector<double> map_waypoints_s_;

  LaneStateController *lane_state_controller_; // lane state controller instance
  VehicleController *vehicle_controller_; // vehicle controller instance
  int current_lane_; // current ego lane [0=left, 1=center, 2=right]
  int target_lane_; // target lane [0=left, 1=center, 2=right]
  double target_velocity_; // target velocity of ego vehicle [m/s] ("set speed" resp. velocity of target vehicle)
  double reference_velocity_; // reference velocity of ego vehicle [m/s]
  double time_gap_; // time gap to vehicle ahead [s]
};

#endif //PATHPLANNER_H
