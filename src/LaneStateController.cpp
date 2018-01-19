#include <iomanip>
#include "VehicleController.h"
#include "LaneStateController.h"

using namespace std;

LaneStateController::LaneStateController(VehicleController *controller, double ref_velocity, int start_lane) {
  vehicle_controller_ = controller;
  current_state_ = LaneState::KEEP_LANE;
  current_lane_ = start_lane;
  target_lane_ = start_lane;
  target_velocity_ = ref_velocity;
  next_vehicle_current_lane_ = nullptr;
  front_vehicle_target_lane_ = nullptr;
  rear_vehicle_target_lane_ = nullptr;
  time_gap_current_lane_front_ = DEFAULT_TIME_GAP;
  time_gap_target_lane_front_ = DEFAULT_TIME_GAP;
  time_gap_target_lane_rear_ = DEFAULT_TIME_GAP;
  ttc_current_lane_front_ = DEFAULT_TIME_TO_COLLISION;
  ttc_target_lane_front_ = DEFAULT_TIME_TO_COLLISION;
  ttc_target_lane_rear_ = DEFAULT_TIME_TO_COLLISION;
}

LaneState LaneStateController::update() {
  current_lane_ = vehicle_controller_->ego_vehicle_.lane_;
  fastest_lane_ = vehicle_controller_->getReachableFastestLane(FASTEST_LANE_FACTOR);

  updateTelemetry();

  LaneState new_state = LaneState::INITIALIZATION;
  switch (current_state_) {
    case LaneState::KEEP_LANE:new_state = executeKeepLane();
      break;
    case LaneState::PREPARE_LANE_CHANGE_LEFT:new_state = executePrepareLaneChangeLeft();
      break;
    case LaneState::LANE_CHANGE_LEFT:new_state = executeLaneChangeLeft();
      break;
    case LaneState::PREPARE_LANE_CHANGE_RIGHT:new_state = executePrepareLaneChangeRight();
      break;
    case LaneState::LANE_CHANGE_RIGHT:new_state = executeLaneChangeRight();
      break;
    default:printf("Error: Invalid behavior state.\n");
      break;
  }
  if (new_state != current_state_) {
    printf("Lane state change: current=%s, new=%s\n", getStateAsString(current_state_).c_str(), getStateAsString(new_state).c_str());
    current_state_ = new_state;
  }

  return current_state_;
}

string LaneStateController::getStateAsString(LaneState state) const {
  switch (state) {
    case LaneState::INITIALIZATION:return "INITIALIZATION";
    case LaneState::KEEP_LANE:return "KEEP_LANE";
    case LaneState::PREPARE_LANE_CHANGE_LEFT:return "PREPARE_LANE_CHANGE_LEFT";
    case LaneState::LANE_CHANGE_LEFT:return "LANE_CHANGE_LEFT";
    case LaneState::PREPARE_LANE_CHANGE_RIGHT:return "PREPARE_LANE_CHANGE_RIGHT";
    case LaneState::LANE_CHANGE_RIGHT:return "LANE_CHANGE_RIGHT";
  }
}

double LaneStateController::calculateDistance(double ego_x, double ego_y, double target_x, double target_y) {
  double delta_x = target_x - ego_x;
  double delta_y = target_y - ego_y;

  return sqrt(pow(delta_x, 2) + pow(delta_y, 2));
}

double LaneStateController::calculateTimeGap(double ego_v, double ego_x, double ego_y, double target_x, double target_y) {
  double dist = calculateDistance(ego_x, ego_y, target_x, target_y);

  return dist / ego_v;
}

double LaneStateController::calculateTimeToCollision(double ego_v, double ego_x, double ego_y, double target_v, double target_x, double target_y) {
  double dist = calculateDistance(ego_x, ego_y, target_x, target_y);
  double delta_v = ego_v - target_v;

  return abs(dist / delta_v);
}

void LaneStateController::updateTelemetry() {

  // Calculate distances, time gap and ttc for current lane
  next_vehicle_current_lane_ = vehicle_controller_->getNextVehicleDrivingAhead(current_lane_);
  Vehicle ego = vehicle_controller_->ego_vehicle_;

  // If a vehicle is ahead in the current lane, update current lane distances, time gap and ttc
  if (next_vehicle_current_lane_) {
    d_current_lane_front_ = calculateDistance(ego.x_, ego.y_, next_vehicle_current_lane_->x_, next_vehicle_current_lane_->y_);
    time_gap_current_lane_front_ = calculateTimeGap(ego.v_, ego.x_, ego.y_, next_vehicle_current_lane_->x_, next_vehicle_current_lane_->y_);
    ttc_current_lane_front_ = calculateTimeToCollision(ego.v_, ego.x_, ego.y_, next_vehicle_current_lane_->v_, next_vehicle_current_lane_->x_, next_vehicle_current_lane_->y_);
  } else {
    d_current_lane_front_ = DEFAULT_DISTANCE;
    time_gap_current_lane_front_ = DEFAULT_TIME_GAP;
    ttc_current_lane_front_ = DEFAULT_TIME_TO_COLLISION;
  }

  // Calculate distances, time gap and ttc for target lane
  if (target_lane_ == current_lane_) {
    d_target_lane_front_ = d_current_lane_front_;
    d_target_lane_rear_ = DEFAULT_DISTANCE;
    time_gap_target_lane_front_ = time_gap_current_lane_front_;
    time_gap_target_lane_rear_ = DEFAULT_TIME_GAP;
    ttc_target_lane_front_ = ttc_current_lane_front_;
    ttc_target_lane_rear_ = DEFAULT_TIME_TO_COLLISION;
  } else {
    front_vehicle_target_lane_ = vehicle_controller_->getNextVehicleDrivingAhead(target_lane_);

    // If a vehicle is ahead in the target lane, update front target lane distances, time gap and ttc
    if (front_vehicle_target_lane_) {
      d_target_lane_front_ = calculateDistance(ego.x_, ego.y_, front_vehicle_target_lane_->x_, front_vehicle_target_lane_->y_);
      time_gap_target_lane_front_ = calculateTimeGap(ego.v_, ego.x_, ego.y_, front_vehicle_target_lane_->x_, front_vehicle_target_lane_->y_);
      ttc_target_lane_front_ = calculateTimeToCollision(ego.v_, ego.x_, ego.y_, front_vehicle_target_lane_->v_, front_vehicle_target_lane_->x_, front_vehicle_target_lane_->y_);
    } else {
      d_target_lane_front_ = DEFAULT_DISTANCE;
      time_gap_target_lane_front_ = DEFAULT_TIME_GAP;
      ttc_target_lane_front_ = DEFAULT_TIME_TO_COLLISION;
    }

    rear_vehicle_target_lane_ = vehicle_controller_->getNextVehicleDrivingBehind(target_lane_);

    // If a vehicle is behind in target lane, update rear target lane distances, time gap and ttc
    if (rear_vehicle_target_lane_) {
      d_target_lane_rear_ = calculateDistance(ego.x_, ego.y_, rear_vehicle_target_lane_->x_, rear_vehicle_target_lane_->y_);
      time_gap_target_lane_rear_ = calculateTimeGap(ego.v_, ego.x_, ego.y_, rear_vehicle_target_lane_->x_, rear_vehicle_target_lane_->y_);
      ttc_target_lane_rear_ = calculateTimeToCollision(ego.v_, ego.x_, ego.y_, rear_vehicle_target_lane_->v_, rear_vehicle_target_lane_->x_, rear_vehicle_target_lane_->y_);
    } else {
      d_target_lane_rear_ = DEFAULT_DISTANCE;
      time_gap_target_lane_rear_ = DEFAULT_TIME_GAP;
      ttc_target_lane_rear_ = DEFAULT_TIME_TO_COLLISION;
    }
  }
}

bool LaneStateController::isTargetLaneClear() {
  return (time_gap_target_lane_rear_ >= MIN_TIME_GAP_LANE_CHANGE) &&
      (d_target_lane_front_ >= MIN_DISTANCE_FRONT_LANE_CHANGE) &&
      (d_target_lane_rear_ >= MIN_DISTANCE_REAR_LANE_CHANGE) &&
      (ttc_target_lane_front_ >= MIN_TTC_FRONT_LANE_CHANGE) &&
      (ttc_target_lane_rear_ >= MIN_TTC_REAR_LANE_CHANGE);
}

LaneState LaneStateController::executeKeepLane() {

  double speed_limit_current_lane = vehicle_controller_->getSpeedLimitForCurrentLane();
  double host_velocity = vehicle_controller_->ego_vehicle_.v_;

  // set target velocity
  if (next_vehicle_current_lane_ && (time_gap_current_lane_front_ <= LOWER_TIME_GAP)) {
    target_velocity_ = next_vehicle_current_lane_->v_;
  } else if (time_gap_current_lane_front_ >= UPPER_TIME_GAP) {
    target_velocity_ = speed_limit_current_lane;
  }

  // determine next behavior state
  // close vehicle ahead, prepare lane change to fastest lane
  if ((host_velocity < speed_limit_current_lane) &&
      (time_gap_current_lane_front_ <= MIN_TIME_GAP_INIT_LANE_CHANGE) &&
      (fastest_lane_ < current_lane_)) {
    target_lane_ = fastest_lane_;
    return LaneState::PREPARE_LANE_CHANGE_LEFT;
  } else if ((host_velocity < speed_limit_current_lane) &&
      (time_gap_current_lane_front_ <= MIN_TIME_GAP_INIT_LANE_CHANGE) &&
      (fastest_lane_ > current_lane_)) {
    target_lane_ = fastest_lane_;
    return LaneState::PREPARE_LANE_CHANGE_RIGHT;
  } else {
    // free driving, keep lane
    target_lane_ = current_lane_;
    return LaneState::KEEP_LANE;
  }
}

LaneState LaneStateController::executePrepareLaneChangeLeft() {

  // set target velocity
  target_velocity_ = (next_vehicle_current_lane_) ? next_vehicle_current_lane_->v_ : vehicle_controller_->getSpeedLimitForCurrentLane();

  // change lanes if it's clear
  return isTargetLaneClear() ? LaneState::LANE_CHANGE_LEFT : LaneState::PREPARE_LANE_CHANGE_LEFT;
}

LaneState LaneStateController::executeLaneChangeLeft() {

  // set target velocity
  if (front_vehicle_target_lane_ && (current_lane_ != target_lane_)) {
    // vehicle is not yet in the target lane
    target_velocity_ = front_vehicle_target_lane_->v_;
  } else if (next_vehicle_current_lane_ && (current_lane_ == target_lane_)) {
    // vehicle is already in target lane, the vehicle switched from target to current lane
    target_velocity_ = next_vehicle_current_lane_->v_;
  } else {
    target_velocity_ = vehicle_controller_->getSpeedLimitForLane(target_lane_);
  }

  // determine next state
  double d_left_lane_marker = (target_lane_ + 1) * LANE_WIDTH;
  double d_right_host_vehicle = vehicle_controller_->ego_vehicle_.d_ + (vehicle_controller_->ego_vehicle_.width_ / 2.0);

  if (d_right_host_vehicle < d_left_lane_marker) {
    return LaneState::KEEP_LANE;
  } else {
    return LaneState::LANE_CHANGE_LEFT;
  }
}

LaneState LaneStateController::executePrepareLaneChangeRight() {

  // set target velocity
  target_velocity_ = (next_vehicle_current_lane_) ? next_vehicle_current_lane_->v_ : vehicle_controller_->getSpeedLimitForCurrentLane();

  // change lanes if it's clear
  return isTargetLaneClear() ? LaneState::LANE_CHANGE_RIGHT : LaneState::PREPARE_LANE_CHANGE_RIGHT;
}

LaneState LaneStateController::executeLaneChangeRight() {

  // set target velocity
  if (front_vehicle_target_lane_ && (current_lane_ != target_lane_)) {
    // vehicle is not yet in the target lane
    target_velocity_ = front_vehicle_target_lane_->v_;
  } else if (next_vehicle_current_lane_ && (current_lane_ == target_lane_)) {
    // vehicle is already in target lane, the vehicle switched from target to current lane
    target_velocity_ = next_vehicle_current_lane_->v_;
  } else {
    target_velocity_ = vehicle_controller_->getSpeedLimitForLane(target_lane_);
  }

  // determine next state
  double d_right_lane_marker = target_lane_ * LANE_WIDTH;
  double d_left_host_vehicle = vehicle_controller_->ego_vehicle_.d_ - (vehicle_controller_->ego_vehicle_.width_ / 2.0);

  if (d_left_host_vehicle > d_right_lane_marker) {
    return LaneState::KEEP_LANE;
  } else {
    return LaneState::LANE_CHANGE_RIGHT;
  }
}
