#ifdef PLATFORM_BRAIN

#include <cmath>
#include <algorithm>
#include "rev/api/v5/alg/slipstream/motion/mecanum_constant_motion.hh"
#include "rev/api/common/units/q_angle.hh"
#include "rev/api/common/units/q_angular_speed.hh"
#include "rev/api/common/units/q_length.hh"
#include "rev/api/common/units/q_speed.hh"
#include "rev/api/common/units/r_quantity.hh"

namespace rev {

/**
 * Keeps constant power moving towards the target
 * ALSO attempts to turn the robot at constant speed so that
 * it finishes turning when it reaches the target
 */
MecanumConstantMotion::MecanumConstantMotion(double ipower) : power(fabs(ipower)) {}

SlipstreamPower MecanumConstantMotion::gen_powers(
    OdometryState current_state,
    Position target_state,
    Position start_state,
    QLength drop_early) {

    // Calculate vector from current position to target
    PointVector to_target = {target_state.x - current_state.pos.x, target_state.y - current_state.pos.y};
    QLength distance_to_target = sqrt(to_target.x * to_target.x + to_target.y * to_target.y);

    // Check if we're close enough to stop
    if (distance_to_target < drop_early) {
        return {0, 0, 0, 0};
    }

    // Calculate the angle from current position to target in global coordinates
    QAngle global_target_angle = atan2(to_target.y, to_target.x);

    // Convert to robot-relative angle (strafe angle relative to robot's current heading)
    QAngle strafe_angle = global_target_angle - current_state.pos.theta;

    // Normalize angle to [-pi, pi]
    while (strafe_angle.convert(radian) > M_PI) {
        strafe_angle = strafe_angle - 2 * M_PI * radian;
    }
    while (strafe_angle.convert(radian) < -M_PI) {
        strafe_angle = strafe_angle + 2 * M_PI * radian;
    }

    // Calculate desired heading change to face toward target
    QAngle desired_heading = global_target_angle;
    QAngle heading_error = desired_heading - current_state.pos.theta;

    // Normalize heading error to [-pi, pi]
    while (heading_error.convert(radian) > M_PI) {
        heading_error = heading_error - 2 * M_PI * radian;
    }
    while (heading_error.convert(radian) < -M_PI) {
        heading_error = heading_error + 2 * M_PI * radian;
    }

    // Estimate time to complete translation at current power
    double max_rpm = 600; // TODO: remove hard coded value
    QLength wheel_diameter = 2.75_in; // TODO: remove hard coded value
    QSpeed translation_speed = power * max_rpm / 60_s * wheel_diameter / 2;
    QTime estimated_translation_time =  distance_to_target / translation_speed;

    // Calculate required rotation speed to finish at same time as translation
    double rotational_component = 0.0;
    if (estimated_translation_time > 0.01_s) { // Avoid division by zero
        QAngularSpeed required_rotation_speed = heading_error / estimated_translation_time;
        QAngularSpeed max_angular_speed = max_rpm / 60_s * 2 * M_PI * radian;

        rotational_component = (required_rotation_speed / max_angular_speed).get_value();
    }

    // Calculate forward and strafe components for translation
    double forward_component = power * cos(strafe_angle).get_value();
    double strafe_component = power * sin(strafe_angle).get_value();

    // Mecanum wheel kinematics with coordinated rotation:
    // front_left = forward - strafe - rotation
    // front_right = forward + strafe + rotation
    // rear_left = forward + strafe - rotation
    // rear_right = forward - strafe + rotation
    double front_left = forward_component - strafe_component - rotational_component;
    double front_right = forward_component + strafe_component + rotational_component;
    double rear_left = forward_component + strafe_component - rotational_component;
    double rear_right = forward_component - strafe_component + rotational_component;

    // Normalize powers to ensure none exceed [-1, 1] while maintaining ratios
    double max_power = std::max({std::abs(front_left), std::abs(front_right),
                                std::abs(rear_left), std::abs(rear_right)});

    if (max_power > 1.0) {
        front_left /= max_power;
        front_right /= max_power;
        rear_left /= max_power;
        rear_right /= max_power;
    }

    return {front_left, front_right, rear_left, rear_right};
}
} // namespace rev

#endif