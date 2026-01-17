#include "rev/api/hardware/sensors/distance_reset.hh"

rev::DistanceReset::DistanceReset(int port, rev::DistancePosition position, float x_center_offset, float y_center_offset) :
    pros::Distance(port), port_(port), position_(position), x_center_offset_(x_center_offset), y_center_offset_(y_center_offset), name_(to_sensor_name(position))
{};

rev::DistancePosition rev::DistanceReset::position() const { return position_; }
float rev::DistanceReset::x_center_offset() const { return x_center_offset_; }
float rev::DistanceReset::y_center_offset() const { return y_center_offset_; }
void rev::DistanceReset::x_center_offset(float new_offset) { x_center_offset_ = new_offset; }
void rev::DistanceReset::y_center_offset(float new_offset) { y_center_offset_ = new_offset; }

std::string rev::DistanceReset::to_sensor_name(rev::DistancePosition sensor_pos) {
    switch (sensor_pos) {
        case rev::DistancePosition::FRONT_SENSOR:
            return "front_distance_sensor";
        case rev::DistancePosition::REAR_SENSOR:
            return "rear_distance_sensor";
        case rev::DistancePosition::LEFT_SENSOR:
            return "left_distance_sensor";
        case rev::DistancePosition::RIGHT_SENSOR:
            return "right_distance_sensor";
    }
}