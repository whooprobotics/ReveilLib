#pragma once
#include <algorithm>
#include "pros/imu.hpp"
#include "rev/api/hardware/devices/gyroscope/gyroscope.hh"

namespace rev {

class DualImu : public Gyroscope {
 public:
  DualImu(int port1, int port2);
  double get_heading() override;
  bool is_calibrating() override;
  std::pair<std::uint8_t, std::uint8_t> check_port() override;

 private:
  pros::Imu inertial1;
  pros::Imu inertial2;
  std::uint8_t port1;
  std::uint8_t port2;
};

}  // namespace rev