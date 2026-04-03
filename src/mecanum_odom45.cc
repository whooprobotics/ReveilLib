#include "mecanum_odom45.hh"
#include "mecanum_util.hh"

void Odom45::set_physical_distances(double horizontal_distance_from_cog, double vertical_distance_from_cog) {
    this->horizontal_distance_from_cog = horizontal_distance_from_cog;
    this->vertical_distance_from_cog = vertical_distance_from_cog;
}

void Odom45::set_position(point position, double orientation_deg, double right_tracker_position, double left_tracker_position) {
    this->right_tracker_position = right_tracker_position;
    this->left_tracker_position = left_tracker_position;
    this->position = position;
    this->orientation_deg = orientation_deg;
}

void Odom45::update_position(double right_tracker_position, double left_tracker_position, double orientation_deg) {
    double right_delta = right_tracker_position - this->right_tracker_position;
    double left_delta = left_tracker_position - this->left_tracker_position;
    double orientation_delta = orientation_deg - this->orientation_deg;

    this->right_tracker_position = right_tracker_position;
    this->left_tracker_position = left_tracker_position;
    this->orientation_deg = orientation_deg;
    
    orientation_delta = reduce_negative_180_to_180(orientation_delta);
    double orientation_delta_rad = to_rad(orientation_delta);

    if (right_delta == 0.0 && left_delta == 0.0 && orientation_delta == 0.0) return;

    const double inv_sqrt2 = 1.0 / sqrt(2.0);

    double forward  = (right_delta + left_delta) * inv_sqrt2;
    double side = (right_delta - left_delta) * inv_sqrt2;

    double local_X_position;
    double local_Y_position;

    if (fabs(orientation_delta) == 0) {
        local_X_position  = side;
        local_Y_position = forward;
    } else {
        local_X_position = (2 * sin(orientation_delta_rad / 2)) * ((side / orientation_delta_rad) + horizontal_distance_from_cog); 
        local_Y_position = (2 * sin(orientation_delta_rad / 2)) * ((forward / orientation_delta_rad) + vertical_distance_from_cog);
    }

    double avg_heading_deg = reduce_negative_180_to_180(orientation_deg - 0.5f * orientation_delta);

    double polar_r = sqrt(local_X_position * local_X_position + local_Y_position * local_Y_position);
    double polar_angle = atan2(local_Y_position, local_X_position) - to_rad(avg_heading_deg);

    double dX = polar_r * cos(polar_angle);
    double dY = polar_r * sin(polar_angle);

    position.x += dX;
    position.y += dY;
}
