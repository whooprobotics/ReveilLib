#pragma once
#include "rev/api/hardware/devices/gyroscope/gyroscope.hh"
#include "pros/imu.hpp"

namespace rev {

class Imu : public Gyroscope {
  public:
    Imu(int port);
    double get_heading() override;
    bool is_calibrating() override;
  private:
    pros::Imu inertial;
};

} // namespace rev