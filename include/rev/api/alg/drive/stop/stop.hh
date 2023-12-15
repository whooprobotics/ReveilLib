#pragma once

#include "rev/api/alg/odometry/odometry.hh"

namespace rev {

enum class stop_state { GO, COAST, BRAKE, EXIT };

/**
 * @brief Interface for stop controllers
 *
 */
class Stop {
 public:
  virtual stop_state get_stop_state(OdometryState current_state,
                                    Position target_state,
                                    Position start_state,
                                    QLength drop_early) = 0;

  virtual double get_coast_power() = 0;
};

}  // namespace rev