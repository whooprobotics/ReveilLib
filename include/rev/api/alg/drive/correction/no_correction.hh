#include "rev/api/alg/drive/correction/correction.hh"

namespace rev {
class NoCorrection : public Correction {
  std::tuple<double, double> apply_correction(
      OdometryState current_state,
      Position target_state,
      std::tuple<double, double> powers) override;
};
}  // namespace rev