#pragma once
#include "rev/api/units/r_quantity.hh"
#include "rev/api/units/q_length.hh"
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_speed.hh"
#include "rev/api/units/q_angular_speed.hh"

namespace rev {
struct Position {
  QLength x;
  QLength y;
  QAngle facing;
};

struct Velocity {
  QSpeed xv;
  QSpeed yv;
  QAngularSpeed angular;
};

struct OdometryState {
  Position pos;
  Velocity vel;
};

class Odometry {
  virtual OdometryState get_state() = 0;
  virtual void set_position(Position pos) = 0;
  virtual void reset_position() = 0;
};
}  // namespace rev