#pragma once

#ifdef PLATFORM_BRAIN

#include "pros/distance.hpp"
#include "rev/api/v5/alg/odometry/odometry.hh"
#include "rev/api/common/units/q_length.hh"
#include "rev/api/common/units/q_angle.hh"
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace rev {

enum class distance_position { FRONT_SENSOR, REAR_SENSOR, LEFT_SENSOR, RIGHT_SENSOR };
enum class wall_position { TOP_WALL, BOTTOM_WALL, LEFT_WALL, RIGHT_WALL };

inline constexpr distance_position front_sensor = distance_position::FRONT_SENSOR;
inline constexpr distance_position rear_sensor  = distance_position::REAR_SENSOR;
inline constexpr distance_position left_sensor  = distance_position::LEFT_SENSOR;
inline constexpr distance_position right_sensor = distance_position::RIGHT_SENSOR;

inline constexpr wall_position top_wall    = wall_position::TOP_WALL;
inline constexpr wall_position bottom_wall = wall_position::BOTTOM_WALL;
inline constexpr wall_position left_wall   = wall_position::LEFT_WALL;
inline constexpr wall_position right_wall  = wall_position::RIGHT_WALL;

class distance : public pros::Distance {
 public:
  /**
   * @brief Creates a distance sensor with its position and offset from tracking center.
   * @param port The V5 port number (1-21).
   * @param position The face of the robot the sensor is mounted to.
   * @param x_center_offset Horizontal offset from tracking center (right = +, left = -), in inches.
   * @param y_center_offset Vertical offset from tracking center (forward = +, backward = -), in inches.
   */
  distance(uint8_t port, distance_position position, float x_center_offset, float y_center_offset);

  uint8_t port() const;
  distance_position position() const;
  float x_center_offset() const;
  float y_center_offset() const;
  void x_center_offset(float new_offset);
  void y_center_offset(float new_offset);
  const std::string name() const;

 private:
  std::string to_sensor_name(distance_position pos);

  uint8_t port_;
  distance_position position_;
  float x_center_offset_;
  float y_center_offset_;
  std::string name_;
};

class distance_reset {
 public:
  /**
   * @brief Wraps mik::distance sensors into a group.
   * Two perpendicular sensors (e.g. front + left) can reset both axes at once.
   */
  explicit distance_reset(const std::vector<rev::distance>& distance_sensors);

  /**
   * @brief Computes the robot's X or Y position from a wall distance reading.
   * @param sensor_pos The sensor to use.
   * @param wall_pos The wall the sensor is facing.
   * @param angle Current robot heading in degrees.
   * @return Computed axis position in inches, or NAN if inapplicable.
   */
  float get_reset_axis_pos(distance_position sensor_pos, wall_position wall_pos, float angle);

  /**
   * @brief Resets the odom X or Y coordinate using a distance sensor wall reading.
   * Left/right walls reset X; top/bottom walls reset Y.
   * @param sensor_position The sensor to use.
   * @param wall The wall the sensor is facing.
   * @param max_reset_distance Maximum allowed position correction in inches.
   * @param odom Odometry instance whose position will be updated.
   * @param reset_attempts Number of readings to average before applying the reset. Defaults to 1.
   * @return true if the reset was applied, false if it exceeded max_reset_distance.
   */
  bool reset_axis(distance_position sensor_position, wall_position wall,
                  float max_reset_distance, std::shared_ptr<Odometry> odom,
                  int reset_attempts = 1);

  std::vector<rev::distance>& get_distance_sensors();

 private:
  float to_sensor_offset_constant(distance_position sensor_pos);
  float to_wall_pos_constant(wall_position wall_pos);
  float to_wall_angle_constant(wall_position wall_pos);

  std::vector<rev::distance> distance_sensors;
};

}  // namespace rev

#endif