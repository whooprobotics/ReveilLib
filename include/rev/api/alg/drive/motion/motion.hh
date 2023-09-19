#pragma once
#include "rev/api/alg/odometry/odometry.hh"

#include <tuple>

namespace rev {
/**
 * @brief Interface for generating raw motor powers
 *
 */
class Motion {
 public:
  /**
   * @brief Generate motor powers.
   *
   * This is intended for use as the initial generation of motor powers.
   * Correction should be applied later.
   *
   * @param current_state The current state when this method is called
   * @param target_state The target state being approached
   * @param start_state The position occupied when the current segment gained control
   * @param drop_early The distance from the target point at which this segment should end
   * 
   * @return std::tuple<double, double>
   */
  virtual std::tuple<double, double> gen_powers(OdometryState current_state,
                                                Position target_state,
                                                Position start_state,
                                                QLength drop_early) = 0;
};
}  // namespace rev