#pragma once

#include "rev/api/alg/odometry/odometry.hh"

namespace rev {

enum class stop_state { GO, COAST, BRAKE };

class Stop {
 public:
  virtual stop_state get_stop_state(OdometryState current_state,
                                    Position target_state,
                                    Position start_state,
                                    QLength drop_early);

  virtual double get_coast_power();
};

}  // namespace rev