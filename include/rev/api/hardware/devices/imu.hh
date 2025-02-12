#pragma once
#include "pros/imu.hpp"
#include "rev/api/hardware/devices/gyroscope/gyroscope.hh"

namespace rev {

class Imu : Gyroscope {
  public:
    Imu(int port);
    double get_position();
    bool is_calibrating();
  private:
    pros::Imu inertial;
};

} // namespace rev