#ifndef COORDINATEUTILS_H
#define COORDINATEUTILS_H

#include <cstdio>
#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

class CoordinateUtils {

 public:

  static const int START_LANE = 1; // starting lane = center lane
  static constexpr double SPEED_LIMIT = 49.5 * 0.44704; // max allowed speed [m/s]
  static constexpr double CONTROLLER_CYCLE_TIME = 0.02; // Cycle time of the vehicle controller between two trajectory points [s]
  static const int TOTAL_PREDICTION_POINTS = 50; // number of prediction points for e.g. the trajectory planner or vehicle model
  static constexpr double TOTAL_PREDICTION_TIME = CONTROLLER_CYCLE_TIME * (double) TOTAL_PREDICTION_POINTS;

  /**
   * For converting back and forth between radians and degrees.
   * @return PI
   */
  const double pi();

  /**
   * Converts degrees to radians.
   */
  double deg2rad(double x);

  /**
   * Converts radians to degrees.
   */
  double rad2deg(double x);

  /**
   * Converts mph to m/s.
   * @param velocity in mph
   */
  double mph2mps(double velocity);

  /**
   * Converts m/s to mph.
   * @param velocity
   */
  double mps2mph(double velocity);

  /**
   * Euclidian distance
   */
  double distance(double x1, double y1, double x2, double y2);

  /**
   * Finds the closest waypoint to Ego
   */
  int closestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

  /**
   * Finds next way point in front of Ego
   */
  int nextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  /**
   * Transform from Cartesian (x, y) coordinates to Frenet (s, d) coordinates
   */
  vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  /**
   * Transform from Frenet (s, d) coordinates to Cartesian (x, y)
   */
  vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

};

#endif //COORDINATEUTILS_H
