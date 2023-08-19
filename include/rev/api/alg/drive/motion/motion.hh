#pragma once
#include "rev/api/odometry/odometry.hh"

/**
 * @brief Interface for generating raw motor powers
 * 
 */
class Motion {
  virtual std::tuple<double, double> genPowers(OdometryState current_state,
                                               Position target_state) = 0;
};