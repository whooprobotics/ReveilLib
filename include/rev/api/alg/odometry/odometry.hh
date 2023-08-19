#pragma once
#include "rev/api/unit/unit.hh"

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
  virtual OdometryState getState() = 0;
  virtual void setPosition(Position pos) = 0;
  virtual void resetPosition() = 0;
};