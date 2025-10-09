#include "rev/api/alg/reckless/motion/mecanum_constant_motion.hh"
#include <cmath>
#include <algorithm>
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"
#include "rev/api/units/r_quantity.hh"

namespace rev {

/**
 * Keeps constant power moving towards the target
 * No turning towards the target, this should be handled in the correction
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

    // Convert to robot-relative angle
    QAngle strafe_angle = global_target_angle - current_state.pos.theta;

    // Normalize angle to [-pi, pi]
    while (strafe_angle.convert(radian) > M_PI) {
        strafe_angle = strafe_angle - 2 * M_PI * radian;
    }
    while (strafe_angle.convert(radian) < -M_PI) {
        strafe_angle = strafe_angle + 2 * M_PI * radian;
    }

    // Calculate forward and strafe components
    double forward_component = power * cos(strafe_angle).get_value();
    double strafe_component = power * sin(strafe_angle).get_value();

    // Mecanum wheel kinematics:
    // front_left = forward - strafe
    // front_right = forward + strafe
    // rear_left = forward + strafe
    // rear_right = forward - strafe
    double front_left = forward_component - strafe_component;
    double front_right = forward_component + strafe_component;
    double rear_left = forward_component + strafe_component;
    double rear_right = forward_component - strafe_component;

    // Normalize powers to ensure none exceed [-1, 1]
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
