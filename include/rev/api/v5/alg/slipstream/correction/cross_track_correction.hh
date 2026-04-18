#pragma once

#ifdef PLATFORM_BRAIN

#include <memory>
#include "rev/api/v5/alg/slipstream/correction/holonomic_correction.hh"
#include "rev/api/common/units/q_length.hh"

namespace rev {
/**
 * @brief Implements cross-track correction algorithm for holonomic drives
 * Adapted from the Pilons Algorithm for mecanum/holonomic systems
 *
 * Cross-track correction follows the following:
 *
 *  1. Calculate the cross-track error: the perpendicular distance from the
 * robot's current position to the desired path line between start and target
 *  2. Calculate the heading error: the difference between the robot's current
 * heading and the desired heading to reach the target
 *  3. If the cross-track error exceeds the maximum allowable threshold,
 * apply correction
 *  4. For translation, apply a lateral correction proportional to cross-track
 * error to move the robot back toward the desired path
 *  5. For rotation, apply a rotational correction proportional to heading
 * error to orient the robot toward the target
 */
class CrossTrackCorrection : public HolonomicCorrection {
 public:
  /**
   * @brief Applies correction to the input powers
   *
   * @param current_state The current OdometryState
   * @param target_state The position being targeted
   * @param start_state The position when the current segment began
   * @param drop_early The distance from target at which segment should end
   * @param powers The input powers from the motion controller
   * @return SlipstreamPower The corrected powers for all four wheels
   */
  SlipstreamPower apply_correction(OdometryState current_state,
                                   Position target_state,
                                   Position start_state,
                                   QLength drop_early,
                                   SlipstreamPower powers) override;

  /**
   * @brief Construct a new Cross Track Correction object
   *
   * @param cross_track_gain The gain for cross-track error correction.
   * Higher values provide stronger correction but may cause oscillation
   * @param heading_gain The gain for heading error correction.
   * Higher values provide stronger rotational correction
   * @param max_cross_track_error The error threshold over which correction
   * will be applied. Small values provide tighter path following
   * @param max_heading_error The heading error threshold over which
   * rotational correction will be applied
   */
  CrossTrackCorrection(double icross_track_gain,
                       double iheading_gain,
                       QLength imax_cross_track_error,
                       QAngle imax_heading_error);

  std::shared_ptr<CrossTrackCorrection> operator&() {
    return std::make_shared<CrossTrackCorrection>(*this);
  }

 private:
  double cross_track_gain;
  double heading_gain;
  QLength max_cross_track_error;
  QAngle max_heading_error;
};

}

#endif