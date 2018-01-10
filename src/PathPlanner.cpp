#include "PathPlanner.h"
#include "Spline/spline.h"

PathPlanner::PathPlanner(VehicleController *vehicle_controller, LaneStateController *lane_state_controller, const int start_lane) {
  vehicle_controller_ = vehicle_controller;
  lane_state_controller_ = lane_state_controller;
  current_lane_ = start_lane;
  target_lane_ = start_lane;
  target_velocity_ = vehicle_controller_->getSpeedLimitForLane(start_lane);
  reference_velocity_ = 0.0;
}

void PathPlanner::setMapWaypoints(vector<double> &x, vector<double> &y, vector<double> &s) {
  map_waypoints_x_ = x;
  map_waypoints_y_ = y;
  map_waypoints_s_ = s;
}

Trajectory PathPlanner::planSplineTrajectory(PreviousTrajectory &previous_trajectory) {

  static double previous_reference_velocity = 0.0;

  // determine current, target lane and time gap threshold for speed control
  current_lane_ = lane_state_controller_->current_lane_;
  target_velocity_ = lane_state_controller_->target_velocity_;
  double time_gap_threshold;

  if ((lane_state_controller_->current_state_ == LaneState::PREPARE_LANE_CHANGE_LEFT) || (lane_state_controller_->current_state_ == LaneState::PREPARE_LANE_CHANGE_RIGHT)) {
    target_lane_ = current_lane_;
    time_gap_threshold = MIN_TIME_GAP_LANE_CHANGE;
    time_gap_ = min(lane_state_controller_->time_gap_current_lane_front_, lane_state_controller_->time_gap_target_lane_front_);
  } else {
    target_lane_ = lane_state_controller_->target_lane_;
    time_gap_ = lane_state_controller_->time_gap_current_lane_front_;
    time_gap_threshold = MIN_TIME_GAP;
  }

  vector<double> points_x;
  vector<double> points_y;
  Vehicle ego_vehicle = vehicle_controller_->ego_vehicle_;

  double ref_x = ego_vehicle.x_;
  double ref_y = ego_vehicle.y_;
  double ref_yaw = ego_vehicle.yaw_;

  // control speed depending on time gap to vehicle ahead

  // plan trajectory to target lane
  auto prev_trajectory_size = static_cast<int>(previous_trajectory.points_x.size());

  if (prev_trajectory_size < 2) {
    // use two points that make the path tangent to the vehicle
    double prev_vehicle_x = ego_vehicle.x_ - cos(ego_vehicle.yaw_);
    double prev_vehicle_y = ego_vehicle.y_ - sin(ego_vehicle.yaw_);

    points_x.push_back(prev_vehicle_x);
    points_x.push_back(ego_vehicle.x_);

    points_y.push_back(prev_vehicle_y);
    points_y.push_back(ego_vehicle.y_);

    reference_velocity_ = ego_vehicle.v_;
  } else {
    // redefine reference state as previous path end point
    ref_x = previous_trajectory.points_x[prev_trajectory_size - 1];
    ref_y = previous_trajectory.points_y[prev_trajectory_size - 1];

    double ref_x_prev = previous_trajectory.points_x[prev_trajectory_size - 2];
    double ref_y_prev = previous_trajectory.points_y[prev_trajectory_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make the path tangent to the car
    points_x.push_back(ref_x_prev);
    points_x.push_back(ref_x);

    points_y.push_back(ref_y_prev);
    points_y.push_back(ref_y);

    reference_velocity_ = previous_reference_velocity;
  }

  // determine 3 waypoints (30 m, 60 m and 90 m) in Frenet coordinates to smooth the trajectory
  double end_point_s;

  if (prev_trajectory_size > 0) {
    end_point_s = previous_trajectory.end_point_s;
  } else {
    end_point_s = ego_vehicle.s_;
  }

  for (auto &offset: {30, 60, 90}) {
    vector<double> next_wp = getXY(end_point_s + offset, (2 + 4 * target_lane_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    points_x.push_back(next_wp[0]);
    points_y.push_back(next_wp[1]);
  }

  for (int i = 0; i < points_x.size(); ++i) {
    // shift car reference angle to 0 degrees
    double shift_x = points_x[i] - ref_x;
    double shift_y = points_y[i] - ref_y;

    points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  // create a spline to smooth the trajectory
  tk::spline spline;
  spline.set_points(points_x, points_y);

  // define the actual (x, y) points we will use for the planner
  Trajectory trajectory = Trajectory();

  // start with all of the previous path points from last time
  for (int i = 0; i < prev_trajectory_size; ++i) {
    trajectory.points_x.push_back(previous_trajectory.points_x[i]);
    trajectory.points_y.push_back(previous_trajectory.points_y[i]);
  }

  // calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
  double x_add_on = 0.0;

  // fill up the rest of our path planner after filling it with previous points
  // here we will always output 50 points
  for (int i = 1; i < (TOTAL_PREDICTION_POINTS - prev_trajectory_size); ++i) {

    // increase/decrease reference velocity
    if (time_gap_ < time_gap_threshold || reference_velocity_ > target_velocity_) {
      reference_velocity_ = max(0.5, reference_velocity_ - MAX_DELTA_VELOCITY);
    } else {
      reference_velocity_ = min(target_velocity_, reference_velocity_ + MAX_DELTA_VELOCITY);
    }

    // Calculate points along new path
    double N = (target_dist / (CONTROLLER_CYCLE_TIME * reference_velocity_));
    double x_point = x_add_on + (target_x) / N;
    double y_point = spline(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate and shift back to normal
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    trajectory.points_x.push_back(x_point);
    trajectory.points_y.push_back(y_point);
  }

  previous_reference_velocity = reference_velocity_;

  return trajectory;
}