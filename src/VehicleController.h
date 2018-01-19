#ifndef VEHICLECONTROLLER_H
#define VEHICLECONTROLLER_H

#include <cstdio>
#include <iostream>
#include <vector>
#include <map>
#include "Vehicle.h"
#include "Eigen-3.3/Eigen/Core"

static constexpr double LANE_WIDTH = 4.0; // default lane width
static constexpr double MAX_DISTANCE_FOR_AVE_VELOCITY = 100; // The average velocity is calculated for vehicles in range of x m ahead
static constexpr double MAX_DISTANCE_LANE_OCCUPANCY = 100; // The average velocity is calculated for vehicles in range of x m ahead

/**
 * Holds a list of all detected vehicles and provides methods to find the nearest vehicle in the ego or adjacent lane.
 */
class VehicleController {
 public:
  Vehicle ego_vehicle_; // ego vehicle
  vector<double> speed_limits_; // speed limits per lane [0=left, 1=center, 2=right] [m/s]
  int number_lanes_; // number of lanes
  Eigen::VectorXd average_lane_velocities_; // average velocity per lane [m/s]

  VehicleController();

  ~VehicleController();

  /**
   * Updates the vehicle controller predictions, lane assignments, etc.
   *
   * @param prediction_time Number of seconds to predict into the future.
   */
  void update(double prediction_time);

  /**
   * Sets ego vehicle.
   *
   * @param x x map coordinate
   * @param y y map coordinate
   * @param s s Frenet coordinate
   * @param d d Frenet coordinate
   * @param yaw yaw angle [rad]
   * @param velocity velocity [m/s]
   */
  void setEgoVehicle(double x, double y, double s, double d, double yaw, double velocity);

  /**
   * Sets the individual speed limit for each lane.
   *
   * @param speed_limits Vector of speed limits for each lane [m/s] [0=left, 1=center, 2=right]
   */
  void setSpeedLimitsForLanes(vector<double> speed_limits);

  /**
   * Gets the speed limit for the current lane.
   *
   * @return the speed limit [m/s].  If the ego vehicle is driving off-road, 0 m/s is returned.
   */
  double getSpeedLimitForCurrentLane();

  /**
   * Gets the speed limit for the given lane.
   *
   * @param lane id [0=left, 1=center, 2=right]
   * @return the speed limit [m/s] for the requested lane or 0 for non-existent lanes
   */
  double getSpeedLimitForLane(int lane);

  /**
   * Adds a vehicle to the vehicle controller map.
   */
  void addVehicle(Vehicle *vehicle);

  /**
   * Reset the update flag for all vehicles.
   */
  void resetUpdateFlagForAllVehicles();

  /**
   * Update vehicle attributes / dynamics.  If the vehicle id is not found, a new vehicle is added.
   */
  void updateVehicle(Vehicle *vehicle);

  /**
   * Removes all outdated vehicles (updated_ = false).
   */
  void removeOutdatedVehicles();

  /**
   * Removes all vehicles.
   */
  void removeAllVehicles();

  /**
   * Finds the next vehicle driving ahead in given lane.
   *
   * @param lane id [0=left, 1=center, 2=right]
   * @return the next vehicle driving in the given lane, NULL if no vehicle found
   */
  Vehicle *getNextVehicleDrivingAhead(int lane);

  /**
   * Finds the next vehicle driving behind in given lane.
   *
   * @param lane id [0=left, 1=center, 2=right]
   * @return the next vehicle driving in the given lane, NULL if no vehicle found
   */
  Vehicle *getNextVehicleDrivingBehind(int lane);

  /**
   * Gets the fastest lane which is either the adjacent or ego lane.
   *
   * @param factor Percentage the fastest lane velocity needs to be higher than the current lane
   * @return the id of the fastest reachable lane [-1=unknown, 0=left, 1=center, 2=right]
   */
  int getReachableFastestLane(double factor = 1.0);

 private:
  map<int, Vehicle *> vehicle_id_map_; // Map of vehicle models <id, Vehicle>

  /**
   * Determines the driving lane for the vehicle.
   *
   * @return the driving lane [-1=unknown, 0=left, 1=center, 2=right]
   */
  int getDrivingLaneForVehicle(const Vehicle *vehicle);

  /**
   * Predicts all vehicle trajectories the given seconds into the future.
   *
   * @param prediction_time Number of seconds to predict into the future
   */
  void generateVehiclePredictions(double prediction_time);

  /**
   * Determines average velocity for all lanes based on vehicles driving ahead.
   * If a lane is free, the speed limit is returned.  For the ego lane only the next vehicle driving ahead is considered.
   */
  Eigen::VectorXd getAverageVelocityForAllLanes();
};

#endif //VEHICLECONTROLLER_H
