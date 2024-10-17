#pragma once
#include "rev/api/hardware/devices/gyroscope/gyroscope.hh"
#include "pros/imu.hpp"

namespace rev {

class Imu : Gyroscope {
  public:
    Imu(int port);
    double get_heading();
    bool is_calibrating();
  private:
    pros::Imu inertial;
};

} // namespace rev