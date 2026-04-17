#include "distance_reset.hh"
#include <cmath>
#include <iostream>
#include <string>

#define WALL_TOP_Y 70.0f
#define WALL_BOTTOM_Y -70.0f
#define WALL_LEFT_X -70.0f
#define WALL_RIGHT_X 70.0f

#define WALL_TOP_ANGLE_OFFSET 270.0f
#define WALL_BOTTOM_ANGLE_OFFSET 270.0f
#define WALL_LEFT_ANGLE_OFFSET 90.0f
#define WALL_RIGHT_ANGLE_OFFSET 90.0f

#define SENSOR_FRONT_ANGLE_OFFSET 0.0f
#define SENSOR_BACK_ANGLE_OFFSET 180.0f
#define SENSOR_LEFT_ANGLE_OFFSET 90.0f
#define SENSOR_RIGHT_ANGLE_OFFSET 270.0f

#define ANSI_GREEN "\033[32m"
#define ANSI_BRIGHT_GREEN "\033[92m"
#define ANSI_RED "\033[31m"
#define ANSI_BRIGHT_RED "\033[91m"
#define ANSI_RESET "\033[0m"

static inline float to_rad(float deg) {
  return deg * M_PI / 180.0f;
}

namespace rev {

distance::distance(uint8_t port,
                   distance_position position,
                   float x_center_offset,
                   float y_center_offset)
    : pros::Distance(port),
      port_(port),
      position_(position),
      x_center_offset_(x_center_offset),
      y_center_offset_(y_center_offset),
      name_(to_sensor_name(position)) {}

uint8_t distance::port() const {
  return port_;
}
distance_position distance::position() const {
  return position_;
}
float distance::x_center_offset() const {
  return x_center_offset_;
}
float distance::y_center_offset() const {
  return y_center_offset_;
}
void distance::x_center_offset(float new_offset) {
  x_center_offset_ = new_offset;
}
void distance::y_center_offset(float new_offset) {
  y_center_offset_ = new_offset;
}
const std::string distance::name() const {
  return name_;
}

std::string distance::to_sensor_name(distance_position pos) {
  switch (pos) {
    case distance_position::FRONT_SENSOR:
      return "front_distance_sensor";
    case distance_position::REAR_SENSOR:
      return "rear_distance_sensor";
    case distance_position::LEFT_SENSOR:
      return "left_distance_sensor";
    case distance_position::RIGHT_SENSOR:
      return "right_distance_sensor";
  }
  return "";
}

distance_reset::distance_reset(const std::vector<rev::distance>& sensors)
    : distance_sensors(sensors) {}

std::vector<rev::distance>& distance_reset::get_distance_sensors() {
  return distance_sensors;
}

float distance_reset::to_sensor_offset_constant(distance_position pos) {
  switch (pos) {
    case distance_position::FRONT_SENSOR:
      return SENSOR_FRONT_ANGLE_OFFSET;
    case distance_position::REAR_SENSOR:
      return SENSOR_BACK_ANGLE_OFFSET;
    case distance_position::LEFT_SENSOR:
      return SENSOR_LEFT_ANGLE_OFFSET;
    case distance_position::RIGHT_SENSOR:
      return SENSOR_RIGHT_ANGLE_OFFSET;
  }
  return 0.0f;
}

float distance_reset::to_wall_pos_constant(wall_position pos) {
  switch (pos) {
    case wall_position::TOP_WALL:
      return WALL_TOP_Y;
    case wall_position::BOTTOM_WALL:
      return WALL_BOTTOM_Y;
    case wall_position::LEFT_WALL:
      return WALL_LEFT_X;
    case wall_position::RIGHT_WALL:
      return WALL_RIGHT_X;
  }
  return 0.0f;
}

float distance_reset::to_wall_angle_constant(wall_position pos) {
  switch (pos) {
    case wall_position::TOP_WALL:
      return WALL_TOP_ANGLE_OFFSET;
    case wall_position::BOTTOM_WALL:
      return WALL_BOTTOM_ANGLE_OFFSET;
    case wall_position::LEFT_WALL:
      return WALL_LEFT_ANGLE_OFFSET;
    case wall_position::RIGHT_WALL:
      return WALL_RIGHT_ANGLE_OFFSET;
  }
  return 0.0f;
}

float distance_reset::get_reset_axis_pos(distance_position sensor_pos,
                                         wall_position wall_pos,
                                         float angle) {
  int index = -1;
  for (size_t i = 0; i < distance_sensors.size(); ++i) {
    if (distance_sensors[i].position() == sensor_pos) {
      index = static_cast<int>(i);
    }
  }
  if (index < 0)
    return 0.0f;

  const float sensor_offset = to_sensor_offset_constant(sensor_pos);
  const float wall_offset = to_wall_angle_constant(wall_pos);
  const float wall_pos_val = to_wall_pos_constant(wall_pos);

  const float dist_in = distance_sensors[index].get() / 25.4f;  // mm -> inches
  const float x_offset = distance_sensors[index].x_center_offset();
  const float y_offset = distance_sensors[index].y_center_offset();
  const float theta = angle + wall_offset + sensor_offset;

  const bool reset_x = (wall_pos == wall_position::LEFT_WALL ||
                        wall_pos == wall_position::RIGHT_WALL);
  const bool reset_y = (wall_pos == wall_position::TOP_WALL ||
                        wall_pos == wall_position::BOTTOM_WALL);

  if (reset_x) {
    return wall_pos_val + (std::cos(to_rad(theta)) * dist_in) -
           (std::cos(to_rad(angle)) * x_offset) -
           (std::sin(to_rad(angle)) * y_offset);
  }
  if (reset_y) {
    return wall_pos_val + (std::sin(to_rad(theta)) * dist_in) +
           (std::sin(to_rad(angle)) * x_offset) -
           (std::cos(to_rad(angle)) * y_offset);
  }

  return NAN;
}

bool distance_reset::reset_axis(distance_position sensor_position,
                                wall_position wall,
                                float max_reset_distance,
                                std::shared_ptr<Odometry> odom,
                                int reset_attempts) {
  const auto state = odom->get_state();
  const float heading = state.pos.theta.convert(degree);

  float pos_sum = 0.0f;
  for (int i = 0; i < reset_attempts; ++i) {
    pos_sum += get_reset_axis_pos(sensor_position, wall, heading);
  }
  const float new_pos = pos_sum / static_cast<float>(reset_attempts);

  const bool reset_x =
      (wall == wall_position::LEFT_WALL || wall == wall_position::RIGHT_WALL);

  const float odom_x = state.pos.x.convert(inch);
  const float odom_y = state.pos.y.convert(inch);

  auto fmt_coord = [](float x, float y) {
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
  };

  if (reset_x && std::abs(new_pos - odom_x) < max_reset_distance) {
    odom->set_position({new_pos * inch, state.pos.y, state.pos.theta});
    std::cout << ANSI_GREEN << "Reset Odom X Position Successfully"
              << ANSI_RESET << "\n";
    std::cout << ANSI_BRIGHT_GREEN << "Old: " << fmt_coord(odom_x, odom_y)
              << " -> New: " << fmt_coord(new_pos, odom_y) << ANSI_RESET
              << "\n";
    return true;
  }
  if (!reset_x && std::abs(new_pos - odom_y) < max_reset_distance) {
    odom->set_position({state.pos.x, new_pos * inch, state.pos.theta});
    std::cout << ANSI_GREEN << "Reset Odom Y Position Successfully"
              << ANSI_RESET << "\n";
    std::cout << ANSI_BRIGHT_GREEN << "Old: " << fmt_coord(odom_x, odom_y)
              << " -> New: " << fmt_coord(odom_x, new_pos) << ANSI_RESET
              << "\n";
    return true;
  }

  if (reset_x) {
    std::cout << ANSI_RED << "Reset Odom X Position Failed" << ANSI_RESET
              << "\n";
    std::cout << ANSI_BRIGHT_RED << "Old: " << fmt_coord(odom_x, odom_y)
              << " -> New: " << fmt_coord(new_pos, odom_y) << ANSI_RESET
              << "\n";
  } else {
    std::cout << ANSI_RED << "Reset Odom Y Position Failed" << ANSI_RESET
              << "\n";
    std::cout << ANSI_BRIGHT_RED << "Old: " << fmt_coord(odom_x, odom_y)
              << " -> New: " << fmt_coord(odom_x, new_pos) << ANSI_RESET
              << "\n";
  }
  return false;
}

}  // namespace rev
