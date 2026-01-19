#pragma once

#include <memory>
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/reckless/segment.hh"
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_time.hh"

namespace rev {
/**
 * @brief Path segment for turning for use with Reckless controller,
 * implementing Walker Campbell's turn algorithm
 *
 */
class RecklessTurnSegment : public RecklessSegment {
 public:
  /**
   * @brief Constructs a new RecklessTurnSegment object
   * 
   * @param imax_power Maximum turning power
   * @param icoast_power Turning coast power
   * @param iangle The angle to face the robot towards
   * @param iharsh_coeff The harsh braking coefficient
   * @param icoast_coeff The coast braking coefficient
   * @param ibrake_time The maximum time that should be spent braking
   */
  RecklessTurnSegment(double imax_power,
                      double icoast_power,
                      QAngle iangle,
                      double iharsh_coeff,
                      double icoast_coeff,
                      QTime ibrake_time);

  /**
   * @brief Current state of the turn segment
   */
  enum class TurnState { FULLPOWER, COAST, BRAKE };

  /**
   * @brief Initializes the segment to turn the robot
   * 
   * @param initial_state The current odometry state when this segment is initialized
   */
  void init(OdometryState initial_state);

  /**
   * @brief Update the segment with new sensor data
   * 
   * @return SegmentStatus The current status of the segment
   */
  SegmentStatus step(OdometryState current_state);

  /**
   * @brief Cleans up the turn segment
   */
  void clean_up();

  /**
   * @brief Shorthand for creating a new LookAt object
   * 
   * @return std::shared_ptr<LookAt> newly constructed LookAt object
   */
  std::shared_ptr<RecklessSegment> operator&() {
    return std::make_shared<RecklessTurnSegment>(*this);
  }

  static std::shared_ptr<RecklessTurnSegment> create(double imax_power,
                                                     double icoast_power,
                                                     QAngle iangle,
                                                     double iharsh_coeff,
                                                     double icoast_coeff,
                                                     QTime ibrake_time) {
    return std::make_shared<RecklessTurnSegment>(imax_power, icoast_power,
                                                 iangle, iharsh_coeff,
                                                 icoast_coeff, ibrake_time);
  }

  /**
   * @brief Returns the progress of the segment
   * 
   * @return Segment progress
   */
  double progress() override { return 0; }

 private:
  double max_power;
  double harsh_coeff;
  double coast_coeff;
  double coast_power;
  uint32_t brake_time;
  QAngle angle_goal = 0_deg;
  QAngle start_angle;
  QAngle angle_difference;
  QAngle target_relative_original;
  QAngle target_relative;
  int left_direction = 0;
  int right_direction = 0;
  int brake_start_time = -1;
  TurnState controller_state{TurnState::FULLPOWER};
};

using TurnSegment = RecklessTurnSegment;

}  // namespace rev
