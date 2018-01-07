#include "VehicleController.h"
#include "CoordinateUtils.h"

VehicleController::VehicleController() {
  number_lanes_ = 0;
}

VehicleController::~VehicleController() = default;

void VehicleController::update(double prediction_time) {
  removeOutdatedVehicles();
  average_lane_velocities_ = getAverageVelocityForAllLanes();
  lane_occupancy_ = getLaneOccupancyForAllLanes();
  generateVehicleModelPredictions(prediction_time);
}

void VehicleController::setEgoVehicle(double x, double y, double s, double d, double yaw, double velocity) {
  ego_vehicle_ = Vehicle(x, y, s, d, yaw, velocity);
  ego_vehicle_.lane_ = getDrivingLaneForVehicle(&ego_vehicle_);
}

void VehicleController::setSpeedLimitsForLanes(vector<double> speed_limits) {
  speed_limits_ = move(speed_limits);
  number_lanes_ = static_cast<int>(speed_limits_.size());
}

double VehicleController::getSpeedLimitForCurrentLane() {
  int lane = ego_vehicle_.lane_;

  if (lane >= 0 && lane < speed_limits_.size()) {
    return speed_limits_[lane];
  } else {
    return 0.0;
  }
}

double VehicleController::getSpeedLimitForLane(int lane) {
  if (lane >= 0.0 && lane < speed_limits_.size()) {
    return speed_limits_[lane];
  } else {
    return 0.0;
  }
}

void VehicleController::addVehicle(Vehicle *vehicle) {
  vehicle->lane_ = getDrivingLaneForVehicle(vehicle);
  vehicle_id_map_.insert(pair<int, Vehicle *>(vehicle->id_, vehicle));
}

void VehicleController::resetUpdateFlagForAllVehicles() {
  ego_vehicle_.updated_ = false;

  for (auto &obj: vehicle_id_map_) {
    obj.second->updated_ = false;
  }
}

void VehicleController::updateVehicle(Vehicle *vehicle) {
  auto iterator = vehicle_id_map_.find(vehicle->id_);

  if (iterator != vehicle_id_map_.end()) {
    delete (*iterator).second;
    (*iterator).second = vehicle;
  } else {
    addVehicle(vehicle);
  }
}

void VehicleController::removeOutdatedVehicles() {
  for (auto &obj: vehicle_id_map_) {
    if (!obj.second->updated_) {
      delete obj.second;
    }
  }
}

void VehicleController::removeAllVehicles() {
  for (auto &obj: vehicle_id_map_) {
    delete obj.second;
  }

  vehicle_id_map_.clear();
}

vector<Vehicle *> VehicleController::getAllVehiclesForLane(int lane) {
  vector<Vehicle *> vehicles;

  for (auto &obj: vehicle_id_map_) {
    if ((obj.second->d_ < (2 + LANE_WIDTH * lane + (LANE_WIDTH / 2.0))) && (obj.second->d_ > (2 + LANE_WIDTH * lane - (LANE_WIDTH / 2.0)))) {
      // vehicle found in given lane
      vehicles.push_back(obj.second);
    }
  }

  return vehicles;
}

Vehicle *VehicleController::getNextVehicleDrivingAhead(int lane) {
  Vehicle *next_vehicle = nullptr;

  for (auto &obj: vehicle_id_map_) {
    if (obj.second->lane_ == lane) {
      // vehicle found in given lane
      // check if it is the initial one or closer than the last stored one
      double delta_s_object = obj.second->s_ - ego_vehicle_.s_;

      if (delta_s_object >= 0 && next_vehicle == nullptr) {
        next_vehicle = obj.second;
      } else if (delta_s_object >= 0 && next_vehicle) {
        double delta_s_next = next_vehicle->s_ - ego_vehicle_.s_;

        if (delta_s_object < delta_s_next) {
          next_vehicle = obj.second;
        }
      }
    }
  }
  return next_vehicle;
}

Vehicle *VehicleController::getNextVehicleDrivingBehind(int lane) {
  Vehicle *next_vehicle = nullptr;

  for (auto obj: vehicle_id_map_) {
    if (obj.second->lane_ == lane) {
      // vehicle found in given lane
      // check if it is the initial one or closer than the last stored one
      double delta_s_object = obj.second->s_ - ego_vehicle_.s_;

      if (delta_s_object < 0 && next_vehicle == nullptr) {
        next_vehicle = obj.second;
      } else if (delta_s_object < 0 && next_vehicle) {
        double delta_s_next = next_vehicle->s_ - ego_vehicle_.s_;

        if (delta_s_object > delta_s_next) {
          next_vehicle = obj.second;
        }
      }
    }
  }
  return next_vehicle;
}

int VehicleController::getReachableFastestLane(double factor) {
  int left_lane = max(0, ego_vehicle_.lane_ - 1);
  int right_lane = min(number_lanes_ - 1, ego_vehicle_.lane_ + 1);

  if (average_lane_velocities_[left_lane] - average_lane_velocities_[ego_vehicle_.lane_] > average_lane_velocities_[ego_vehicle_.lane_] * factor) {
    return left_lane;
  } else if (average_lane_velocities_[right_lane] - average_lane_velocities_[ego_vehicle_.lane_] > average_lane_velocities_[ego_vehicle_.lane_] * factor) {
    return right_lane;
  } else {
    return ego_vehicle_.lane_;
  }
}

/*** private methods ***/

int VehicleController::getDrivingLaneForVehicle(const Vehicle *vehicle) {
  for (int lane = 0; lane < number_lanes_; ++lane) {
    if ((vehicle->d_ < (2 + LANE_WIDTH * lane + (LANE_WIDTH / 2.0))) && (vehicle->d_ > (2 + LANE_WIDTH * lane - (LANE_WIDTH / 2.0)))) {
      return lane;
    }
  }
  return -1;
}

void VehicleController::generateVehicleModelPredictions(double prediction_time) {
  ego_vehicle_.generatePredictions(prediction_time);

  for (auto &obj: vehicle_id_map_) {
    obj.second->generatePredictions(prediction_time);
  }
}

Eigen::VectorXd VehicleController::getAverageVelocityForAllLanes() {
  vector<double> sum_velocities = {0.0, 0.0, 0.0};
  vector<int> number_vehicles = {0, 0, 0};

  for (auto &obj: vehicle_id_map_) {
    double delta_s = obj.second->s_ - ego_vehicle_.s_;

    // ignore vehicles driving in ego lane
    // consider all vehicles driving ahead up to a given distance s
    if (obj.second->lane_ != ego_vehicle_.lane_ && delta_s > 0.0 && delta_s < MAX_DISTANCE_FOR_AVE_VELOCITY) {
      sum_velocities[obj.second->lane_] += obj.second->v_;
      number_vehicles[obj.second->lane_] += 1;
    }
  }

  // determine ego lane velocity on vehicle ahead only
  Vehicle *next_vehicle = getNextVehicleDrivingAhead(ego_vehicle_.lane_);

  if (next_vehicle) {
    sum_velocities[ego_vehicle_.lane_] = next_vehicle->v_;
  } else {
    sum_velocities[ego_vehicle_.lane_] = speed_limits_[ego_vehicle_.lane_];
  }
  number_vehicles[ego_vehicle_.lane_] = 1;

  Eigen::VectorXd average_lane_velocities(3);
  average_lane_velocities << (number_vehicles[0] == 0 ? speed_limits_[0] : min(speed_limits_[0], sum_velocities[0] / number_vehicles[0])),
      (number_vehicles[1] == 0 ? speed_limits_[1] : min(speed_limits_[1], sum_velocities[1] / number_vehicles[1])),
      (number_vehicles[2] == 0 ? speed_limits_[2] : min(speed_limits_[2], sum_velocities[2] / number_vehicles[2]));

  return average_lane_velocities;
}

Eigen::VectorXi VehicleController::getLaneOccupancyForAllLanes() {
  Eigen::VectorXi lane_occupancy(3);
  lane_occupancy.setZero();

  for (int i = 0; i < number_lanes_; ++i) {
    vector<Vehicle *> vehicles = getAllVehiclesForLane(i);

    for (auto &obj: vehicles) {
      double delta_s = obj->s_ - ego_vehicle_.s_;

      if (delta_s > 0.0 && delta_s < MAX_DISTANCE_LANE_OCCUPANCY) {
        lane_occupancy[i] += 1;
      }
    }
  }
  return lane_occupancy;
}
