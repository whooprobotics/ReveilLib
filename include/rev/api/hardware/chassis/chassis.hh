#pragma once

namespace rev {
class Chassis {
  virtual void drive(double left, double right) = 0;
  virtual void drive_vector(double forward, double yaw) = 0;
};
}  // namespace rev