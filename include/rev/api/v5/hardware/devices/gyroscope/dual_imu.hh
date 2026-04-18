#pragma once

#ifdef PLATFORM_BRAIN
#include <algorithm>
#include "pros/imu.hpp"
#include "rev/api/v5/hardware/devices/gyroscope/gyroscope.hh"

namespace rev {

/**
 * @brief Implementation of the Gyroscope interface with two Vex IMUs
 * 
 */
class DualImu : public Gyroscope {
 public:
  /**
   * @brief Constructs a new DualImu object based on Vex IMUs
   * 
   * @param port1 Port for first IMU
   * @param port2 Port for second IMU
   */
  DualImu(int port1, int port2);

  /**
   * @brief Gets the average heading of the sensors
   * 
   * @returns Average current facing angle of the IMUs, normalized to [-180, 180]
   */
  double get_heading() override;

  /**
   * @brief Calibrates both sensors
   * 
   */
  void calibrate() override;

  /**
   * @brief Checks if either of the IMU devices are calibrating
   * 
   */
  bool is_calibrating() override;

  /**
   * @brief Verifies that the sensor ports are properly connected
   * 
   * @returns Ports that are not properly connected, zeros otherwise
   */
  std::pair<std::uint8_t, std::uint8_t> check_ports() override;

 private:
  pros::Imu inertial1;
  pros::Imu inertial2;
  std::uint8_t port1;
  std::uint8_t port2;
};

}  // namespace rev

#endif