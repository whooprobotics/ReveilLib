#pragma once

#include <sstream>
#include <string>
#include <vector>
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/units/all_units.hh"

namespace rev {
/**
 * @brief Implementation of the Sparkfun Optical Tracking Odometry Sensor
 * 
 */
class OTOS {
 public:
  /**
   * @brief Constructs a new OTOS object
   */
  OTOS();

  /**
   * @brief Gets the current x value
   * 
   * @returns x
   */
  double get_x();

  /**
   * @brief Gets the current y value
   * 
   * @returns y
   */
  double get_y();

  /**
   * @brief Gets the current theta value
   * 
   * @returns theta
   */
  double get_h();

  /**
   * @brief Refreshes the sensor with new data
   * 
   */
  void update();

  /**
   * @brief Resets the current position to {0, 0, 0}
   */
  void reset();

 private:
  double x;
  double y;
  double h;

  std::string input;
  std::string token;
  std::vector<std::string> tokens;
  std::stringstream ss;
};
} // namespace rev
