#pragma once
#include "rev/api/hardware/devices/gyroscope/gyroscope.hh"
#include "pros/imu.hpp"

namespace rev {

class DualImu : Gyroscope {
  public:
    DualImu(int port1, int port2);
    double get_heading();
    bool is_calibrating();
  private:
    pros::Imu inertial1;
    pros::Imu inertial2;
};

} // namespace rev