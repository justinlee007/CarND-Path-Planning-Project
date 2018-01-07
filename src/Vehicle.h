#ifndef VEHICLE_H
#define VEHICLE_H

#include <cstdio>
#include <iostream>
#include <vector>

using namespace std;

static const double DEFAULT_VEHICLE_WIDTH = 2.0; // default vehicle width [m]

/**
 * Contains the actual vehicle dynamics and predicts the state based on a standard bicycle model.
 */
class VehicleModel {
 public:
  int id_; // vehicle ID
  double x_; // x position [m]
  double y_; // y position [m]
  double vx_; // velocity in x direction [m/s]
  double vy_; // velocity in y direction [m/s]
  double ax_; // acceleration in x direction [m/s^2]
  double ay_; // acceleration in y direction [m/s^2]
  double v_; // velocity [m/s]
  double a_; // acceleration [m/s^2]
  double yaw_; // yaw angle (orientation) [rad]
  double s_; // frenet s coordinate
  double d_; // frenet d coordinate
  double width_; // vehicle width [m]
  bool updated_; // true if attributes are up-to date

  int lane_; // lane id [-1=unknown, 0-left, 1-center, 2-right]

  double predicted_x_; // predicted x positions [m]
  double predicted_y_; // predicted y positions [m]
  double predicted_v_; // predicted velocity [m/s]

  vector<double> predicted_trajectory_x_; // predicted trajectory, x positions [m]
  vector<double> predicted_trajectory_y_; // predicted trajectory, y positions [m]
  vector<double> predicted_trajectory_vx_; // predicted trajectory, velocity in x direction [m/s]
  vector<double> predicted_trajectory_vy_; // predicted trajectory, velocity in y direction [m/s]
  vector<double> predicted_trajectory_ax_; // predicted trajectory, acceleration in x direction [m/s^2]
  vector<double> predicted_trajectory_ay_; // predicted trajectory, acceleration in y direction [m/s^2]
  vector<double> predicted_trajectory_v_; // predicted trajectory, velocity [m/s]
  vector<double> predicted_trajectory_a_; // predicted trajectory, acceleration [m/s^2]
  vector<double> predicted_trajectory_yaw_; // predicted trajectory, yaw angle (orientation) [rad]
  vector<double> predicted_trajectory_s_; // predicted trajectory, frenet s coordinates
  vector<double> predicted_trajectory_d_; // predicted trajectory, frenet d coordinates

  /**
   * Constructor initializes a default vehicle model.
   */
  VehicleModel();

  /**
   * Constructor initializes a vehicle with the standard set of parameters.
   *
   * @param id ID of vehicle track.
   * @param x map x-position [m]
   * @param y map y-position [m]
   * @param vx velocity in x direction [m/s]
   * @param vy velocity in y direction [m/s]
   * @param s frenet s coordinate
   * @param d frenet s coordinate
   */
  VehicleModel(int id, double x, double y, double vx, double vy, double s, double d);

  /**
   * Constructor initializes a vehicle with the standard set of parameters.
   *
   * @param x map x-position [m]
   * @param y map y-position [m]
   * @param s frenet s coordinate
   * @param d frenet d coordinate
   * @param yaw yaw angle [rad]
   * @param velocity velocity [m/s]
   */
  VehicleModel(double x, double y, double s, double d, double yaw, double velocity);

  /**
   * Generate predictions for all vehicle trajectories for the given seconds into the future.
   *
   * @param prediction_time (in seconds)
   */
  void generatePredictions(double prediction_time);
};

#endif //VEHICLE_H
