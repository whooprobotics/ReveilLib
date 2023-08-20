#pragma once
#include "rev/api/alg/odometry/odometry.hh"

#include <tuple>

namespace rev {
/**
 * @brief Interface for correction algorithms
 *
 */
class Correction {
  /**
   * @brief Applies correction to the input
   *
   * @param current_state The current OdometryState
   * @param target_state The position being targeted
   * @param powers The input powers
   * @return std::tuple<double, double> The adjusted powers
   */
  virtual std::tuple<double, double> apply_correction(
      OdometryState current_state,
      Position target_state,
      std::tuple<double, double> powers) = 0;
};
}  // namespace rev