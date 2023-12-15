#pragma once

#include "rev/api/alg/drive/correction/correction.hh"
#include "rev/api/units/q_length.hh"

namespace rev {
/**
 * @brief Implements the PiLons Correction Algorithm used by team 5225A in VRC
 * In The Zone
 *
 * PiLons correction follows the following:
 *
 *  1. Find the distance from the target position to the nearest point on the
 * line the robot is currently following. Save this as `xerr`
 *  2. Calculate the global angle from where the robot is to the target point
 *  3. Find the difference in the angle the robot is facing to the angle the
 * robot needs to face to be facing towards the target point. This should be the
 * minimum angle needed. Call this `angle`.
 *  4. If `xerr` is greater than the maximum allowable `xerr`, `correction = k
 * * angle`
 *  5. If `correction == 0`, skip correcting. If `correction > 0`, multiply the
 * right power by `exp(-correction)`. If `correction < 0`, multiply left power
 * by `exp(correction)`
 */
class PilonsCorrection : public Correction {
 public:
  /**
   * @brief Applies correction to the input
   *
   * @param current_state The current OdometryState
   * @param target_state The position being targeted
   * @param powers The input powers
   * @return std::tuple<double, double> The adjusted powers
   */
  std::tuple<double, double> apply_correction(
      OdometryState current_state,
      Position target_state,
      Position start_state,
      QLength drop_early,
      std::tuple<double, double> powers) override;

  /**
   * @brief Construct a new Pilons Correction object
   *
   * @param ikCorrection The constant of correction. `8.0` was used by Pilons,
   * but tuning is recommended.
   * @param imaxError The error threshold over which correction will be applied
   */
  PilonsCorrection(double ik_correction, QLength imax_error);

 private:
  double k_correction;
  QLength max_error;
};

}  // namespace rev