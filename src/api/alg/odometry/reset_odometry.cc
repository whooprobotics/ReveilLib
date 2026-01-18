#include "rev/api/alg/odometry/reset_odometry.hh"
#include "rev/api/units/all_units.hh"
#include "rev.hh"

#define WALL_TOP_Y     70
#define WALL_BOTTOM_Y -70
#define WALL_LEFT_X   -70
#define WALL_RIGHT_X   70

#define WALL_TOP_ANGLE_OFFSET    270
#define WALL_BOTTOM_ANGLE_OFFSET 270
#define WALL_LEFT_ANGLE_OFFSET    90
#define WALL_RIGHT_ANGLE_OFFSET   90

#define SENSOR_FRONT_ANGLE_OFFSET   0
#define SENSOR_BACK_ANGLE_OFFSET  180
#define SENSOR_LEFT_ANGLE_OFFSET   90
#define SENSOR_RIGHT_ANGLE_OFFSET 270

float to_rad(float angle) {
    return angle * (M_PI / 180.0);
}

rev::ResetOdometry::ResetOdometry(const std::vector<rev::DistanceReset>& distance_sensors, std::shared_ptr<rev::Odometry> odom) :
    distance_sensors(distance_sensors), odom(odom)
{};

float rev::ResetOdometry::get_reset_axis_pos(rev::DistancePosition sensor_position, rev::WallPosition wall_position, float angle) {
    int index = -1;
    for (size_t i = 0; i < distance_sensors.size(); ++i) {
        if (distance_sensors[i].position() == sensor_position) {
            index = i;
        }
    }
    if (index < 0) { return 0; }
    
    const float sensor_offset = to_sensor_offset_constant(sensor_position);
    const float wall_offset = to_wall_angle_constant(wall_position);
    const float wall_pos = to_wall_pos_constant(wall_position);
    
    const float distance = distance_sensors[index].get() / 25.4f; // Convert mm to inches
    const float x_offset = distance_sensors[index].x_center_offset();
    const float y_offset = distance_sensors[index].y_center_offset();
    const float theta = angle + wall_offset + sensor_offset; 

    const bool reset_x = (wall_position == rev::WallPosition::LEFT_WALL || wall_position == rev::WallPosition::RIGHT_WALL);
    const bool reset_y = (wall_position == rev::WallPosition::TOP_WALL || wall_position == rev::WallPosition::BOTTOM_WALL);
    
    if (reset_x) {
        return wall_pos + (std::cos(to_rad(theta)) * distance) - (std::cos(to_rad(angle)) * x_offset) - (std::sin(to_rad(angle)) * y_offset);
    }
    if (reset_y) {
        return wall_pos + (std::sin(to_rad(theta)) * distance) + (std::sin(to_rad(angle)) * x_offset) - (std::cos(to_rad(angle)) * y_offset);
    }

    return NAN;
}

bool rev::ResetOdometry::reset_axis(rev::DistancePosition sensor_position, rev::WallPosition wall_position, float max_reset_distance) {
    const double angle = odom->get_state().pos.theta.convert(degree);
    const float new_pos = get_reset_axis_pos(sensor_position, wall_position, angle);

    const bool reset_x = (wall_position == rev::WallPosition::TOP_WALL || wall_position == rev::WallPosition::BOTTOM_WALL) ? true : false; 

    const float odom_x = odom->get_state().pos.x.convert(inch);
    const float odom_y = odom->get_state().pos.y.convert(inch);
    
    const rev::QLength qnew_pos = new_pos * inch;
    const rev::QLength qodom_x = odom_x * inch;
    const rev::QLength qodom_y = odom_y * inch;

    if (reset_x && std::abs(new_pos - odom_x) < max_reset_distance) {
        odom->set_position({ qnew_pos, qodom_y, odom->get_state().pos.theta });
        pros::lcd::print(4, "New X: %f", new_pos);
        return true;
    }
    if (!reset_x && std::abs(new_pos - odom_y) < max_reset_distance) {
        odom->set_position({ qodom_x, qnew_pos, odom->get_state().pos.theta });
        pros::lcd::print(5, "New Y: %f", new_pos);
        return true;
    } 
    
    if (reset_x) {
        pros::lcd::print(4, "X Failed: %f", new_pos);
    } else {
        pros::lcd::print(5, "Y Failed: %f", new_pos);
    }

    return false;
}

std::vector<rev::DistanceReset>& rev::ResetOdometry::get_distance_sensors() {
    return distance_sensors;
}

float rev::ResetOdometry::to_sensor_offset_constant(rev::DistancePosition sensor_pos) {
    switch (sensor_pos) {
        case rev::DistancePosition::FRONT_SENSOR:
            return SENSOR_FRONT_ANGLE_OFFSET;
        case rev::DistancePosition::REAR_SENSOR:
            return SENSOR_BACK_ANGLE_OFFSET;
        case rev::DistancePosition::LEFT_SENSOR:
            return SENSOR_LEFT_ANGLE_OFFSET;
        case rev::DistancePosition::RIGHT_SENSOR:
            return SENSOR_RIGHT_ANGLE_OFFSET;
    }
}

float rev::ResetOdometry::to_wall_pos_constant(rev::WallPosition wall_pos) {
    switch (wall_pos) {
        case rev::WallPosition::TOP_WALL:
            return WALL_TOP_Y;
        case rev::WallPosition::BOTTOM_WALL:
            return WALL_BOTTOM_Y;
        case rev::WallPosition::LEFT_WALL:
            return WALL_LEFT_X;
        case rev::WallPosition::RIGHT_WALL:
            return WALL_RIGHT_X;
    }
}

float rev::ResetOdometry::to_wall_angle_constant(rev::WallPosition wall_pos) {
    switch (wall_pos) {
        case rev::WallPosition::TOP_WALL:
            return WALL_TOP_ANGLE_OFFSET;
        case rev::WallPosition::BOTTOM_WALL:
            return WALL_BOTTOM_ANGLE_OFFSET;
        case rev::WallPosition::LEFT_WALL:
            return WALL_LEFT_ANGLE_OFFSET;
        case rev::WallPosition::RIGHT_WALL:
            return WALL_RIGHT_ANGLE_OFFSET;
    }
}
