#pragma once
#include "rev/api/unit/unit.hh"

namespace rev {
struct Position {
  Length x;
  Length y;
  Angle facing;
};

struct Velocity {
  Speed xv;
  Speed yv;
  AngularSpeed angular;
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