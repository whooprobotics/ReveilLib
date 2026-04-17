#include "rev/api/alg/slipstream/correction/cross_track_correction.hh"
#include <cmath>
#include <algorithm>
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"

namespace rev {

// Helper function to normalize angle to [-pi, pi]
// Not stolen from anybody
QAngle normalize_angle(QAngle angle) {
    double radians = angle.convert(radian);
    while (radians > M_PI) {
        radians -= 2 * M_PI;
    }
    while (radians < -M_PI) {
        radians += 2 * M_PI;
    }
    return radians * radian;
}

CrossTrackCorrection::CrossTrackCorrection(double icross_track_gain,
                                         double iheading_gain,
                                         QLength imax_cross_track_error,
                                         QAngle imax_heading_error)
    : cross_track_gain(icross_track_gain),
      heading_gain(iheading_gain),
      max_cross_track_error(imax_cross_track_error),
      max_heading_error(imax_heading_error)
    {}

SlipstreamPower CrossTrackCorrection::apply_correction(
    OdometryState current_state,
    Position target_state,
    Position start_state,
    QLength drop_early,
    SlipstreamPower powers) {

  Pose current_pos = current_state.pos;

  // Calculate the desired path vector from start to target
  PointVector path_vector = {
      target_state.x - start_state.x,
      target_state.y - start_state.y
  };

  QLength path_length = sqrt(square(path_vector.x) + square(path_vector.y));

  // If path is too short, no correction needed
  if (path_length < 0.01_in) {
      return powers;
  }

  // Calculate cross-track error (perpendicular distance from robot to desired path)
  PointVector start_to_robot = {current_pos.x - start_state.x,current_pos.y - start_state.y};

  // Project robot position onto the path line
  QArea dot_product = (start_to_robot.x * path_vector.x + start_to_robot.y * path_vector.y);
  QLength projection_scalar = dot_product / path_length;

  PointVector projection_point = {
      start_state.x + (projection_scalar * path_vector.x) / path_length,
      start_state.y + (projection_scalar * path_vector.y) / path_length
  };

  // Cross-track error vector (perpendicular to path)
  PointVector cross_track_error_vector = {
      current_pos.x - projection_point.x,
      current_pos.y - projection_point.y
  };

  QLength cross_track_error = sqrt(cross_track_error_vector.x * cross_track_error_vector.x +
                                  cross_track_error_vector.y * cross_track_error_vector.y);

  // Calculate desired heading
  // Desired heading is specified by user (last part of the pose)
  QAngle desired_heading = target_state.theta;

  // Calculate heading error
  QAngle heading_error = normalize_angle(desired_heading - current_pos.theta);

  // Initialize correction components
  double lateral_correction = 0.0;
  double rotational_correction = 0.0;

  // Apply cross-track correction if error exceeds threshold
  if (cross_track_error > max_cross_track_error) {
      // Calculate the angle of the cross-track error vector
      QAngle cross_track_angle = atan2(cross_track_error_vector.y, cross_track_error_vector.x);

      // Convert to robot-relative coordinates
      QAngle robot_relative_error_angle = normalize_angle(cross_track_angle - current_pos.theta);

      // Apply correction to move back toward the path
      // Negative sign because we want to move opposite to the error direction
      lateral_correction = -cross_track_gain * cross_track_error.convert(meter) *
                          sin(robot_relative_error_angle).get_value();
  }

  // Apply heading correction if error exceeds threshold
  if (abs(heading_error) > max_heading_error) {
      rotational_correction = heading_gain * heading_error.convert(radian);
  }

  // Convert corrections to mecanum wheel powers
  // lateral_correction: positive = strafe right, negative = strafe left
  // rotational_correction: positive = turn counterclockwise, negative = turn clockwise

  double strafe_correction = lateral_correction;
  double rotation_correction = rotational_correction;

  // Apply corrections
  SlipstreamPower corrected_powers;
  corrected_powers.front_left_forward = powers.front_left_forward - strafe_correction - rotation_correction;
  corrected_powers.front_right_forward = powers.front_right_forward + strafe_correction + rotation_correction;
  corrected_powers.rear_left_forward = powers.rear_left_forward + strafe_correction - rotation_correction;
  corrected_powers.rear_right_forward = powers.rear_right_forward - strafe_correction + rotation_correction;

  // Normalize pows
  double max_power = std::max({
      std::abs(corrected_powers.front_left_forward),
      std::abs(corrected_powers.front_right_forward),
      std::abs(corrected_powers.rear_left_forward),
      std::abs(corrected_powers.rear_right_forward)
  });

  if (max_power > 1.0) {
      corrected_powers.front_left_forward /= max_power;
      corrected_powers.front_right_forward /= max_power;
      corrected_powers.rear_left_forward /= max_power;
      corrected_powers.rear_right_forward /= max_power;
  }

  return corrected_powers;
}

} // namespace rev
