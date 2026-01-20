#pragma once

#ifdef PLATFORM_BRAIN
#include <memory>
#include "rev/api/v5/alg/drive/correction/correction.hh"

namespace rev {

/**
 * @brief Dummy class implementing Correction that does nothing
 *
 * This is mostly for testing, but also allows for having an eventual system
 * which does not rely on correction whatsoever
 */
class NoCorrection : public Correction {
 public:
  /**
   * @brief Applies correction to the input
   *
   * @param current_state
   * @param target_state
   * @param powers
   * 
   * @return std::tuple<double, double>
   */
  std::tuple<double, double> apply_correction(
      OdometryState current_state,
      Position target_state,
      Position start_state,
      QLength drop_early,
      std::tuple<double, double> powers) override;

  /**
   * @brief Shorthand for creating a new NoCorrection object
   * 
   * @return std::shared_ptr<NoCorrection> newly constructed NoCorrection object
   */
  std::shared_ptr<NoCorrection> operator&() {
    return std::make_shared<NoCorrection>(*this);
  }
};
}  // namespace rev

#endif