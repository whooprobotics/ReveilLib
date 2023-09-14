#include "rev/api/alg/drive/correction/no_correction.hh"

std::tuple<double, double> rev::NoCorrection::apply_correction(
    rev::OdometryState current_state,
    rev::Position target_state,
      Position start_state,
      QLength drop_early,
    std::tuple<double, double> powers) {
  return powers;
}
