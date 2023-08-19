#pragma once
#include "rev/api/alg/odometry/odometry.hpp"

class Correction {
  virtual std::tuple<double, double> applyCorrection(
      OdometryState current_state,
      Position target_state,
      std::tuple<double, double> powers) = 0;
};