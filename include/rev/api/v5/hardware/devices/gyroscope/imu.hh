#pragma once

#ifdef PLATFORM_BRAIN
#include <utility>
#include "pros/imu.hpp"
#include "rev/api/v5/hardware/devices/gyroscope/gyroscope.hh"

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

#endif