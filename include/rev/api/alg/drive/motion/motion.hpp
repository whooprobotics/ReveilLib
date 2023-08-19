#include "rev/api/odometry/odometry.hpp"

class Motion {
  virtual std::tuple<double, double> genPowers(OdometryState current_state,
                                               Position target_state) = 0;
};