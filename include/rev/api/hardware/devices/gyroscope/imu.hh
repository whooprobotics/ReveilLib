#pragma once
#include "pros/imu.hpp"
#include "rev/api/hardware/devices/gyroscope/gyroscope.hh"
#include <utility>

namespace rev {

class Imu : public Gyroscope {
 public:
  Imu(int port);
  double get_heading() override;
  bool is_calibrating() override;
  std::pair<std::uint8_t, std::uint8_t> check_port() override;

 private:
  pros::Imu inertial;
  std::uint8_t port;
};

}  // namespace rev