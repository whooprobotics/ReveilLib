#pragma once
#include "pros/imu.hpp"
#include "rev/api/hardware/devices/gyroscope/gyroscope.hh"
#include <utility>

namespace rev {

/**
 * @brief Implementation of the Gyroscope interface with one Vex IMU
 * 
 */
class Imu : public Gyroscope {
 public:
  /**
   * @brief Constructs a new Imu object based on Vex IMU
   * 
   */
  Imu(int port);

  /**
   * @brief Gets the current heading of the sensor
   * @returns Current facing angle of the IMU, normalized to [-180, 180]
   */
  double get_heading() override;

  /**
   * @brief Checks if the IMU device is calibrating
   * 
   */
  bool is_calibrating() override;

  /**
   * @brief Verifies that the sensor port is properly connected
   * @returns A pair <port, 0> if the port is not connected, zeros otherwise
   */
  std::pair<std::uint8_t, std::uint8_t> check_port() override;

 private:
  pros::Imu inertial;
  std::uint8_t port;
};
}  // namespace rev