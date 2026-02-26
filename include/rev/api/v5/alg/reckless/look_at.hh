#pragma once

#ifdef PLATFORM_BRAIN
#include <memory>
#include "rev/api/v5/alg/odometry/odometry.hh"
#include "rev/api/v5/alg/reckless/segment.hh"
#include "rev/api/v5/alg/reckless/turn_segment.hh"
#include "rev/api/common/units/q_angle.hh"
#include "rev/api/common/units/q_length.hh"
#include "rev/api/common/units/q_time.hh"

namespace rev {
/**
 * @brief Implements a point turn segment to face the robot towards a set of coordinates
 * 
 */
class LookAt : public RecklessSegment {
 public:
  /**
   * @brief Constructs a new LookAt object
   * 
   * @param imax_power Maximum turning power
   * @param icoast_power Turning coast power
   * @param itarget_position The point to face the robot towards
   * @param idrop_angle The angle at which the segment should end. You probably should leave this at 0, as it has not been tested.
   * @param iharsh_coeff The harsh braking coefficient
   * @param icoast_coeff The coast braking coefficient
   * @param ibrake_time The maximum time that should be spent braking
   */
  LookAt(
      double imax_power, 
      double icoast_power, 
      Position itarget_position, 
      QAngle idrop_angle, 
      double iharsh_coeff, 
      double icoast_coeff, 
      QTime ibrake_time
  );

  /**
   * @brief Constructs a new LookAt object
   * 
   * @param imax_power Maximum turning power
   * @param icoast_power Turning coast power
   * @param itarget_position The point to face the robot towards
   * @param idrop_angle The angle at which the segment should end. You probably should leave this at 0, as it has not been tested.
   * @param iharsh_coeff The harsh braking coefficient
   * @param icoast_coeff The coast braking coefficient
   * @param ibrake_time The maximum time that should be spent braking
   * @param itimeout The maximum amount of time in the turn before it breaks out
   */
  LookAt(
      double imax_power, 
      double icoast_power, 
      Position itarget_position, 
      QAngle idrop_angle, 
      double iharsh_coeff, 
      double icoast_coeff, 
      QTime ibrake_time,
      QTime itimeout
  );

  /**
    * @brief Constructs a new LookAt object
    * 
    * @param imax_power Maximum turning power
    * @param icoast_power Turning coast power
    * @param itarget_position The point to face the robot towards
    * @param iangle_offset The amount of offset to add to the computed angle from 
    * the target position, EX: 180_deg to look backwards
    * @param idrop_angle The angle at which the segment should end. You probably should leave this at 0, as it has not been tested.
    * @param iharsh_coeff The harsh braking coefficient
    * @param icoast_coeff The coast braking coefficient
    * @param ibrake_time The maximum time that should be spent braking
    */
  LookAt(
    double imax_power, 
    double icoast_power, 
    Position itarget_position, 
    QAngle iangle_offset,
    QAngle idrop_angle, 
    double iharsh_coeff, 
    double icoast_coeff, 
    QTime ibrake_time
  );

  /**
    * @brief Constructs a new LookAt object
    * 
    * @param imax_power Maximum turning power
    * @param icoast_power Turning coast power
    * @param itarget_position The point to face the robot towards
    * @param iangle_offset The amount of offset to add to the computed angle from 
    * the target position, EX: 180_deg to look backwards
    * @param idrop_angle The angle at which the segment should end. You probably should leave this at 0, as it has not been tested.
    * @param iharsh_coeff The harsh braking coefficient
    * @param icoast_coeff The coast braking coefficient
    * @param ibrake_time The maximum time that should be spent braking
    * @param itimeout The maximum amount of time in the turn before it breaks out
    */
  LookAt(
    double imax_power, 
    double icoast_power, 
    Position itarget_position, 
    QAngle iangle_offset,
    QAngle idrop_angle, 
    double iharsh_coeff, 
    double icoast_coeff, 
    QTime ibrake_time,
    QTime itimeout
  );
  
  /**
   * @brief Initialize the segment to turn the robot
   * 
   * @param initial_state The current robot state when this segment is initialized
   */
  void init(OdometryState initial_state) override;

  /**
   * @brief Update the segment with new sensor data
   * 
   * @return SegmentStatus The current status of the segment
   */
  SegmentStatus step(OdometryState current_state) override;

  /**
   * @brief Cleans up the turn segment
   */
  void clean_up() override;

  /**
   * @brief Shorthand for creating a new LookAt object
   * 
   * @return std::shared_ptr<LookAt> newly constructed LookAt object
   */
  std::shared_ptr<LookAt> operator&() {
      return std::make_shared<LookAt>(*this);
  }

  static std::shared_ptr<LookAt> create(
      double imax_power, 
      double icoast_power, 
      Position itarget_position, 
      QAngle idrop_angle, 
      double iharsh_coeff, 
      double icoast_coeff, 
      QTime ibrake_time) {
    return std::make_shared<LookAt>(
        imax_power,
        icoast_power,
        itarget_position,
        idrop_angle,
        iharsh_coeff,
        icoast_coeff,
        ibrake_time
    );
  }

  /**
   * @brief Returns the progress of the segment
   * 
   * @return Segment progress
   */
  double progress() override { return 0; }

 private:
  RecklessTurnSegment turn_segment;

  double max_power;
  double coast_power;

  Position start_position;
  Position target_position;

  QAngle angle_goal = 0_deg;
  QAngle angle_offset;
  QAngle drop_angle;

  double harsh_coeff;
  double coast_coeff;

  QTime brake_time;

  uint32_t timeout = 0;

};

}  // namespace rev

#endif