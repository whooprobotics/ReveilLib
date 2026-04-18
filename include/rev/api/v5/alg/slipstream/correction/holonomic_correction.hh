#pragma once

#ifdef PLATFORM_BRAIN

#include "rev/api/v5/alg/odometry/odometry.hh"
#include "rev/api/v5/alg/slipstream/power.hh"

namespace rev {
/**
 * @brief Interface for holonomic correction algorithms
 * 
 */
class HolonomicCorrection {
 public:
  /**
   * @brief Applies correction to the input
   * 
   * @param current_state The current OdometryState
   * @param target_state The position being targeted
   * @param start_state The state of the robot when the active segment begins
   * @param drop_early The distance from the segment target point at which the
   * segment will halt
   * @param powers The input powers
   * @return SlipstreamPower The corrected powers
   */
  virtual SlipstreamPower apply_correction(OdometryState current_state,
                                           Position target_state,
                                           Position start_state,
                                           QLength drop_early,
                                           SlipstreamPower powers) = 0;
};
} // namespace rev

#endif