#pragma once
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_angular_speed.hh"
#include "rev/api/units/q_length.hh"
#include "rev/api/units/q_speed.hh"
#include "rev/api/units/r_quantity.hh"
#include "rev/util/math/pose.hh"

namespace rev {

/**
 * @brief A structure representing a position and facing direction
 *
 */
// struct Position {
//   QLength x;
//   QLength y;
//   QAngle facing;
// };
typedef struct Pose Position;

/**
 * @brief A structure representing a velocity
 *
 */
struct Velocity {
  QSpeed xv;
  QSpeed yv;
  QAngularSpeed angular;
};

/**
 * @brief A structure packaging a velocity and position into one
 *
 */
struct OdometryState {
  Position pos;
  Velocity vel;
};

/**
 * @brief Interface for odometry implementations
 *
 */
class Odometry {
 public:
  /**
   * @brief Get the current position and velocity of the robot
   *
   * @return OdometryState
   */
  virtual OdometryState get_state() = 0;
  /**
   * @brief Set the position of the controller
   *
   * @param pos
   */
  virtual void set_position(Position pos) = 0;
  /**
   * @brief Set the position to (0,0)
   *
   * THIS WILL ALSO RESET HEADING, WHICH YOU MIGHT NOT WANT
   *
   */
  virtual void reset_position() = 0;
};
}  // namespace rev