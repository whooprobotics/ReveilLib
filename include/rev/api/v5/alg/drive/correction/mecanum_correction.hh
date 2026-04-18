#pragma once

#ifdef PLATFORM_BRAIN

#include <memory>
#include "rev/api/v5/alg/reckless/correction/correction.hh"
#include "rev/api/common/units/q_length.hh"

namespace rev {
/**
 * Implementing some correction alg
 */
class MecanumCorrection : public Correction {
 public:
  /**
     * @brief Applies correction to the input
     *
     * @param current_state The current OdometryState
     * @param target_state The position being targeted
     * @param powers The input powers
     * @return std::tuple<double, double> The adjusted powers
     */
  std::tuple<double, double> apply_correction( OdometryState current_state,
      Position target_state,
      Position start_state,
      QLength drop_early,
      std::tuple<double, double> powers) override;

  /**
   * @brief Construct a new Mecanum Correction object
   *
   * @param ik_correction Some correction constant to be tuned
   * @param imax_error Error threshold
   */
   MecanumCorrection();

  std::shared_ptr<MecanumCorrection> operator&() {
    return std::make_shared<MecanumCorrection>(*this);
  }

 private:
  double ik_correction_;
  QLength imax_error_;
};

}

#endif