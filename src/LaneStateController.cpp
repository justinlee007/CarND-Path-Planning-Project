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
  d_predicted_current_lane_front_ = DEFAULT_DISTANCE;
  time_gap_current_lane_front_ = DEFAULT_TIME_GAP;
  time_gap_target_lane_front_ = DEFAULT_TIME_GAP;
  time_gap_target_lane_rear_ = DEFAULT_TIME_GAP;
  time_gap_predicted_current_lane_front_ = DEFAULT_TIME_GAP;
  ttc_current_lane_front_ = DEFAULT_TIME_TO_COLLISION;
  ttc_target_lane_front_ = DEFAULT_TIME_TO_COLLISION;
  ttc_target_lane_rear_ = DEFAULT_TIME_TO_COLLISION;
  ttc_predicted_current_lane_front_ = DEFAULT_TIME_TO_COLLISION;
}

LaneState LaneStateController::update() {
  current_lane_ = vehicle_controller_->ego_vehicle_.lane_;
  fastest_lane_ = vehicle_controller_->getReachableFastestLane(FASTEST_LANE_FACTOR);

  calculateSafetyMeasures();

  LaneState new_state = LaneState::INITIALIZATION;
  switch (current_state_) {
    case LaneState::KEEP_LANE:new_state = stateKeepLane();
      break;
    case LaneState::PREPARE_LANE_CHANGE_LEFT:new_state = statePrepareLaneChangeLeft();
      break;
    case LaneState::LANE_CHANGE_LEFT:new_state = stateLaneChangeLeft();
      break;
    case LaneState::PREPARE_LANE_CHANGE_RIGHT:new_state = statePrepareLaneChangeRight();
      break;
    case LaneState::LANE_CHANGE_RIGHT:new_state = stateLaneChangeRight();
      break;
    default:cout << "Error: Invalid behavior state." << endl;
      break;
  }
  if (new_state != current_state_) {
    printf("Lane state change: current=%s, new=%s", getStateAsString(current_state_).c_str(), getStateAsString(new_state).c_str());
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

LaneState LaneStateController::stateKeepLane() {

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

bool LaneStateController::isTargetLaneSafe() {
  return (time_gap_target_lane_rear_ >= MIN_TIME_GAP_LANE_CHANGE) &&
      (d_target_lane_front_ >= MIN_DISTANCE_FRONT_LANE_CHANGE) &&
      (d_target_lane_rear_ >= MIN_DISTANCE_REAR_LANE_CHANGE) &&
      (ttc_target_lane_front_ >= MIN_TTC_FRONT_LANE_CHANGE) &&
      (ttc_target_lane_rear_ >= MIN_TTC_REAR_LANE_CHANGE);
}

LaneState LaneStateController::statePrepareLaneChangeLeft() {
  calculateSafetyMeasures();

  // set target velocity
  if (next_vehicle_current_lane_) {
    target_velocity_ = next_vehicle_current_lane_->v_;
  } else {
    target_velocity_ = vehicle_controller_->getSpeedLimitForCurrentLane();
  }

  // determine next lane state
  if (isTargetLaneSafe()) {
    return LaneState::LANE_CHANGE_LEFT;
  } else {
    // still unsafe to change to the left lane, wait
    return LaneState::PREPARE_LANE_CHANGE_LEFT;
  }
}

LaneState LaneStateController::stateLaneChangeLeft() {
  calculateSafetyMeasures();

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

LaneState LaneStateController::statePrepareLaneChangeRight() {
  calculateSafetyMeasures();

  // set target velocity
  if (next_vehicle_current_lane_) {
    target_velocity_ = next_vehicle_current_lane_->v_;
  } else {
    target_velocity_ = vehicle_controller_->getSpeedLimitForCurrentLane();
  }

  // determine next state
  if (isTargetLaneSafe()) {
    return LaneState::LANE_CHANGE_RIGHT;
  } else {
    // still unsafe to change to the right lane, wait
    return LaneState::PREPARE_LANE_CHANGE_RIGHT;
  }
}

LaneState LaneStateController::stateLaneChangeRight() {
  calculateSafetyMeasures();

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

void LaneStateController::calculateSafetyMeasures() {

  // Calculate time gap for current lane
  next_vehicle_current_lane_ = vehicle_controller_->getNextVehicleDrivingAhead(current_lane_);
  Vehicle ego = vehicle_controller_->ego_vehicle_;

  if (next_vehicle_current_lane_) {
    // found vehicle driving ahead, check time_gap
    d_current_lane_front_ = calculateDistance(ego.x_, ego.y_, next_vehicle_current_lane_->x_, next_vehicle_current_lane_->y_);
    d_predicted_current_lane_front_ = calculateDistance(ego.predicted_x_, ego.predicted_y_, next_vehicle_current_lane_->predicted_x_, next_vehicle_current_lane_->predicted_y_);
    time_gap_current_lane_front_ = calculateTimeGap(ego.v_, ego.x_, ego.y_, next_vehicle_current_lane_->x_, next_vehicle_current_lane_->y_);
    time_gap_predicted_current_lane_front_ = calculateTimeGap(ego.predicted_v_,
                                                              ego.predicted_x_,
                                                              ego.predicted_y_,
                                                              next_vehicle_current_lane_->predicted_x_,
                                                              next_vehicle_current_lane_->predicted_y_);
    ttc_current_lane_front_ = calculateTimeToCollision(ego.v_, ego.x_, ego.y_, next_vehicle_current_lane_->v_, next_vehicle_current_lane_->x_, next_vehicle_current_lane_->y_);
    ttc_predicted_current_lane_front_ = calculateTimeToCollision(ego.predicted_v_,
                                                                 ego.predicted_x_,
                                                                 ego.predicted_y_,
                                                                 next_vehicle_current_lane_->predicted_v_,
                                                                 next_vehicle_current_lane_->predicted_x_,
                                                                 next_vehicle_current_lane_->predicted_y_);
  } else {
    d_current_lane_front_ = DEFAULT_DISTANCE;
    d_predicted_current_lane_front_ = DEFAULT_DISTANCE;
    time_gap_current_lane_front_ = DEFAULT_TIME_GAP;
    time_gap_predicted_current_lane_front_ = DEFAULT_TIME_GAP;
    ttc_current_lane_front_ = DEFAULT_TIME_TO_COLLISION;
    ttc_predicted_current_lane_front_ = DEFAULT_TIME_TO_COLLISION;
  }

  // Calculate time gaps for target lane
  if (target_lane_ == current_lane_) {
    d_target_lane_front_ = d_current_lane_front_;
    d_target_lane_rear_ = DEFAULT_DISTANCE;
    time_gap_target_lane_front_ = time_gap_current_lane_front_;
    time_gap_target_lane_rear_ = DEFAULT_TIME_GAP;
    ttc_target_lane_front_ = ttc_current_lane_front_;
    ttc_target_lane_rear_ = DEFAULT_TIME_TO_COLLISION;
  } else {
    front_vehicle_target_lane_ = vehicle_controller_->getNextVehicleDrivingAhead(target_lane_);

    if (front_vehicle_target_lane_) {
      // found vehicle driving ahead in target lane, check time gap
      d_target_lane_front_ = calculateDistance(ego.x_, ego.y_, front_vehicle_target_lane_->x_, front_vehicle_target_lane_->y_);
      time_gap_target_lane_front_ = calculateTimeGap(ego.v_, ego.x_, ego.y_, front_vehicle_target_lane_->x_, front_vehicle_target_lane_->y_);
      ttc_target_lane_front_ = calculateTimeToCollision(ego.v_, ego.x_, ego.y_, front_vehicle_target_lane_->v_, front_vehicle_target_lane_->x_, front_vehicle_target_lane_->y_);
    } else {
      d_target_lane_front_ = DEFAULT_DISTANCE;
      time_gap_target_lane_front_ = DEFAULT_TIME_GAP;
      ttc_target_lane_front_ = DEFAULT_TIME_TO_COLLISION;
    }

    rear_vehicle_target_lane_ = vehicle_controller_->getNextVehicleDrivingBehind(target_lane_);

    if (rear_vehicle_target_lane_) {
      // found vehicle driving behind in target lane, check time gap
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
