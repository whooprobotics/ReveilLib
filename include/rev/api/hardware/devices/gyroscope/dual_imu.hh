#pragma once
#include "rev/api/hardware/devices/gyroscope/gyroscope.hh"
#include "pros/imu.hpp"
#include <algorithm>

namespace rev {

class DualImu : public Gyroscope {
  public:
    DualImu(int port1, int port2);
    double get_heading() override;
    bool is_calibrating() override;
  private:
    pros::Imu inertial1;
    pros::Imu inertial2;
};

} // namespace rev