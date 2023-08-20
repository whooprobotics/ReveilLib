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
   * @param current_state
   * @param target_state
   * @return std::tuple<double, double>
   */
  virtual std::tuple<double, double> gen_powers(OdometryState current_state,
                                                Position target_state) = 0;
};
}  // namespace rev