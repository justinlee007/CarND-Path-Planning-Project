#ifndef LANESTATECONTROLLER_H
#define LANESTATECONTROLLER_H

#include <cstdio>
#include <vector>
#include "VehicleController.h"

using namespace std;

/**
 * A FSM for the lane state controller.
 */
enum class LaneState {
  INITIALIZATION = 0,
  KEEP_LANE,
  PREPARE_LANE_CHANGE_LEFT,
  LANE_CHANGE_LEFT,
  PREPARE_LANE_CHANGE_RIGHT,
  LANE_CHANGE_RIGHT
};

// Sensor fusion index constants [id, x, y, vx, vy, s, d]
static const int SENSOR_FUSION_INDEX_ID = 0;
static const int SENSOR_FUSION_INDEX_X = 1;
static const int SENSOR_FUSION_INDEX_Y = 2;
static const int SENSOR_FUSION_INDEX_VX = 3;
static const int SENSOR_FUSION_INDEX_VY = 4;
static const int SENSOR_FUSION_INDEX_S = 5;
static const int SENSOR_FUSION_INDEX_D = 6;

/**
 * Uses data from VehicleController The lane state controller suggests the next lane state.  It does not consider path planning.
 */
class LaneStateController {

 public:
  VehicleController *vehicle_controller_; // contains a list of vehicles based on sensor fusion
  LaneState current_state_; // current lane state
  int current_lane_; // current lane ID
  int target_lane_; // target lane ID
  int fastest_lane_; // fastest lane ID
  Vehicle *next_vehicle_current_lane_; // next vehicle driving ahead in ego lane
  Vehicle *front_vehicle_target_lane_; // next vehicle driving ahead in target lane
  Vehicle *rear_vehicle_target_lane_; // next vehicle driving behind in target lane
  double target_velocity_; // target velocity (selected vehicle) [m/s]
  double d_current_lane_front_; // distance to vehicle ahead [m]
  double d_target_lane_front_; // distance to vehicle ahead in target lane [m]
  double d_target_lane_rear_; // distance to vehicle behind in target lane [m]
  double d_predicted_current_lane_front_; // predicted distance to vehicle ahead [m]
  double time_gap_current_lane_front_; // time gap to vehicle ahead [s]
  double time_gap_target_lane_front_; // time gap to vehicle ahead in target lane [s]
  double time_gap_target_lane_rear_; // time gap to vehicle behind in target lane [s]
  double time_gap_predicted_current_lane_front_; // predicted time gap to vehicle ahead [s]
  double ttc_current_lane_front_; // time-to-collision to vehicle ahead [s]
  double ttc_target_lane_front_; // time-to-collision to vehicle ahead in target lane [s]
  double ttc_target_lane_rear_; // time-to-collision to vehicle behind in target lane [s]
  double ttc_predicted_current_lane_front_; // predicted time-to-collision to vehicle ahead [s]

  /**
   * Initializes the first state.
   *
   * @param controller Pointer to the controller instance including all vehicles
   * @param ref_velocity Reference velocity [m/s]
   * @param start_lane ID of current lane [0=left, 1=center, 2=right]
   */
  LaneStateController(VehicleController *controller, double ref_velocity, int start_lane = 1);

  /**
   * Plans the next lane state.
   *
   * @return the next lane state
   */
  LaneState update();

  /**
   * @return the behavior state as a user-readable string
   */
  string getStateAsString(LaneState state) const;

  /**
   * Returns the target lane ID for the current lane state.
   *
   * @return the target lane [0=left, 1=center, 2=right]
   */
  int getTargetLane();

 private:
  static const double LOWER_TIME_GAP = 1.5; // min allowed time gap to vehicle ahead [s]
  static const double UPPER_TIME_GAP = 3.5; // time gap [s] to drive with speed limit
  static const double MIN_TIME_GAP_INIT_LANE_CHANGE = 3.0; // min allowed time gap to initiate a lane change [s]
  static const double MIN_TIME_GAP_LANE_CHANGE = 1.0; // min required time gap to vehicle in target lane [s]
  static const double MIN_DISTANCE_FRONT_LANE_CHANGE = 10.0; // min required distance to vehicle ahead in target lane [m]
  static const double MIN_DISTANCE_REAR_LANE_CHANGE = 10.0; // min required distance to vehicle behind in target lane [m]
  static const double MIN_TTC_FRONT_LANE_CHANGE = 6.0; // min required time-to-collision to vehicle ahead in target lane [s]
  static const double MIN_TTC_REAR_LANE_CHANGE = 6.0; // min required time-to-collision to vehicle behind in target lane [s]
  static const double FASTEST_LANE_FACTOR = 0.08; // the fastest lane velocity need to  be x% faster than the current [%]

  static const double DEFAULT_TIME_GAP = 9999.9; // default resp. max time gap [s]
  static const double DEFAULT_TIME_TO_COLLISION = 9999.9; // default resp. max time-to-collision [s]
  static const double DEFAULT_DISTANCE = 9999.9; // default resp. max distance to vehicles [m]

  /**
   * Calculates distance between two vehicles.
   *
   * @param ego_x x map coordinate of ego vehicle [m]
   * @param ego_y y map coordinate of ego vehicle [m]
   * @param target_x x map coordinate of target vehicle [m]
   * @param target_y y map coordinate of target vehicle [m]
   * @return the distance between the given vehicles [m]
   */
  double calculateDistance(double ego_x, double ego_y, double target_x, double target_y);

  /**
   * Calculates time gap between two vehicles.
   *
   * @param ego_v Velocity of ego vehicle [m/s]
   * @param ego_x x map coordinate of ego vehicle [m]
   * @param ego_y y map coordinate of ego vehicle [m]
   * @param target_x x map coordinate of target vehicle [m]
   * @param target_y y map coordinate of target vehicle [m]
   * @return the time gap between given vehicles [s]
   */
  double calculateTimeGap(double ego_v, double ego_x, double ego_y, double target_x, double target_y);

  /**
   * Calculates time-to-collision between two vehicles.
   *
   * @param ego_v Velocity of ego vehicle [m/s]
   * @param ego_x x map coordinate of ego vehicle [m]
   * @param ego_y y map coordinate of ego vehicle [m]
   * @param target_v Velocity of target vehicle [m/s]
   * @param target_x x map coordinate of target vehicle [m]
   * @param target_y y map coordinate of target vehicle [m]
   * @return the time-to-collision between given vehicles [s].
   */
  double calculateTimeToCollision(double ego_v, double ego_x, double ego_y, double target_v, double target_x, double target_y);

  /**
   * Calculates the safety time gaps, TTC, and distances for the current and target lane.
   */
  void calculateSafetyMeasures();

  /**
   * Checks whether the target lane is safe to drive or not.
   *
   * @return true if the lane change is safe
   */
  bool isTargetLaneSafe();

  /********** FINITE STATE MACHINE **********/

  /**
   * Vehicle keeps the lane as long the reference velocity could be kept.
   *
   * @return the next lane state.
   */
  LaneState stateKeepLane();

  /**
   * Vehicle prepares for left lane change (e.g. adjusting speed, waiting for a safe gap,...).
   *
   * @return the next lane state
   */
  LaneState statePrepareLaneChangeLeft();

  /**
   * Vehicle performs a left lane change.
   *
   * @return the next lane state
   */
  LaneState stateLaneChangeLeft();

  /**
   * Vehicle prepares for right lane change (e.g. adjusting speed, waiting for a safe gap,...).
   *
   * @return the next lane state
   */
  LaneState statePrepareLaneChangeRight();

  /**
   * Vehicle performs a right lane change.
   *
   * @return the next lane state
   */
  LaneState stateLaneChangeRight();
};

#endif //LANESTATECONTROLLER_H
