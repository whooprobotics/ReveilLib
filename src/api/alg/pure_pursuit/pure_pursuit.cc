#include "rev/api/alg/pure_pursuit/pure_pursuit.hh"

namespace rev {

PurePursuitSegment::PurePursuitSegment(std::shared_ptr<Motion> imotion,
                                       std::shared_ptr<Correction> icorrection,
                                       std::shared_ptr<Stop> istop,
                                       QLength look_ahead_distance,
                                       QLength wheelbase,
                                       double ikp,
                                       double iki,
                                       double ikd):
    motion(imotion),
    correction(icorrection),
    stop(istop),
    look_ahead_distance(look_ahead_distance),
    wheelbase(wheelbase),
    kp(ikp),
    ki(iki),
    kd(ikd),
    current_idx(0)
    {}

PointVector PurePursuitSegment::find_target_point(OdometryState current_state) {
    size_t target_idx = current_idx;
    PointVector target_point = path_waypoints.back(); // Default to the last waypoint

    for (size_t i = current_idx; i < path_waypoints.size(); ++i) {
        QLength dx = path_waypoints[i].x - current_state.pos.x;
        QLength dy = path_waypoints[i].y - current_state.pos.y;
        QLength distance = sqrt(square(dx) + square(dy));

        if (distance >= look_ahead_distance) {
            target_idx = i; 
            target_point = path_waypoints[i];
            break;
        }
    }

    // Update the current index
    current_idx = target_idx;

    return target_point;
}

// Helper method to calculate the remaining distance to the end of the path
QLength PurePursuitSegment::calculate_remaining_distance(OdometryState current_state) {
    QLength dx = path_waypoints.back().x - current_state.pos.x;
    QLength dy = path_waypoints.back().y - current_state.pos.y;
    QLength distance = sqrt(square(dx) + square(dy));
    return distance;
}

std::tuple<double, double> PurePursuitSegment::PID(OdometryState current_state, double base_power) {
    // Calculate error in position (Euclidean distance to last_point)
    QLength error_x = last_point.x - current_state.pos.x;
    QLength error_y = last_point.y - current_state.pos.y;
    QLength error = sqrt(square(error_x) + square(error_y));
    
    // Calculate the angle to the final point
    QAngle desired_theta = atan2(error_y, error_x);
    QAngle current_theta = current_state.pos.theta; 
    double angle_error = (desired_theta - current_theta).convert(radian);
    
    // Normalize the angle_error to [-pi, pi]
    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;
    
    // Proportional term
    double P = kp * angle_error;
    
    // Integral term
    integral += angle_error;
    double I = ki * integral;
    
    // Derivative term
    double derivative = angle_error - prev_error;
    double D = kd * derivative;
    
    // Update prev_error
    prev_error = angle_error;
    
    // Compute the steering correction
    double steering_correction = P + I + D;
    

    double left_power = base_power + steering_correction;
    double right_power = base_power - steering_correction;
    
    // Clamp motor powers to valid range (e.g., -100 to 100)
    left_power = std::max(-100.0, std::min(100.0, left_power));
    right_power = std::max(-100.0, std::min(100.0, right_power));
    
    return std::make_tuple(left_power, right_power);
}

void PurePursuitSegment::init(OdometryState initial_state) {
}

// Step function implementation
SegmentStatus PurePursuitSegment::step(OdometryState current_state) {

    new_state = this->stop->get_stop_state(current_state, {last_point.x, last_point.y, 0_deg},
                                                    start_point, this->drop_early);

    //Prevent status from regressing
    if (last_status.status == SegmentStatusType::NEXT ||
        new_state == stop_state::EXIT)
    return last_status = SegmentStatus::next();

    if (last_status.status == SegmentStatusType::BRAKE ||
        new_state == stop_state::BRAKE)
    return last_status = SegmentStatus::brake();

    // Find the target point at the look-ahead distance
    PointVector target = find_target_point(current_state);

    // If current_idx is out of bounds, stop the robot
    if (current_idx >= path_waypoints.size()) {
        std::cout << "PurePursuitSegment: Reached end of path. Initiating brake." << std::endl;
        return SegmentStatus::brake();
    }

    pows = motion->gen_powers(
            current_state,
            {target.x, target.y, 0_deg},
            {path_waypoints[0].x, path_waypoints[0].y, 0_deg},
            0.0_ft
        );
    
    if (!(target == last_point)) {
        // Apply correction to the motor powers
        corrected_pows = correction->apply_correction(
            current_state,
            {target.x, target.y, 0_deg},
            {path_waypoints[0].x, path_waypoints[0].y, 0_deg},
            0.0_ft,
            pows
        );
    }
    else {
        // run PID loop
        corrected_pows = PID(current_state, std::get<0>(pows));
    }


    return SegmentStatus::drive(corrected_pows);
}

void PurePursuitSegment::clean_up() {
    // Clean-up implementation (if needed)
}

} // namespace rev