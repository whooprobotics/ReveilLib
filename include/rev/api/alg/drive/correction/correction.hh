#pragma once
#include "rev/api/alg/odometry/odometry.hh"

#include <tuple>

namespace rev {
/**
 * @brief Interface for correction algorithms
 *
 */
class Correction {
 public:
  /**
   * @brief Applies correction to the input
   *
   * @param current_state The current OdometryState
   * @param target_state The position being targeted
   * @param start_state The state of the robot when the active segment began
   * @param drop_early The distance from the segment target point at which this segment will halt
   * @param powers The input powers
   * @return std::tuple<double, double> The adjusted powers
   */
  virtual std::tuple<double, double> apply_correction(
      OdometryState current_state,
      Position target_state,
      Position start_state,
      QLength drop_early,
      std::tuple<double, double> powers) = 0;
};
}  // namespace rev