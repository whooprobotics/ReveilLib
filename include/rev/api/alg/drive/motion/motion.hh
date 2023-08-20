#pragma once
#include "rev/api/alg/odometry/odometry.hh"

#include <tuple>

namespace rev {
/**
 * @brief Interface for generating raw motor powers
 *
 */
class Motion {
  virtual std::tuple<double, double> gen_powers(OdometryState current_state,
                                                Position target_state) = 0;
};
}  // namespace rev