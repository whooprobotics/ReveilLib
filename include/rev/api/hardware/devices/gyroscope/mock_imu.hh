#pragma once
#include "rev/api/hardware/devices/gyroscope/gyroscope.hh"
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {

/**
 * @brief Simulated implementation of the Gyroscope class
 */
class MockImu : public Gyroscope {
 public:
  /**
   * @brief Constructs a new MockImu simulated device
   * @param initial_angle Initial angle the MockImu is initialized to
   */
  MockImu(double initial_angle);

  /**
   * @brief Gets the current heading of the sensor
   * @returns Current facing angle of the MockImu, normalized to [-180, 180]
   */
  double get_heading();

  /**
   * @brief Sets the angle
   * @param new_angle New facing angle in degrees, normalized to [-180, 180]
   */
  void set_angle(double new_angle);

  /**
   * @brief Checks if the Gyroscope is calibrating
   * @returns false since it's a simulated device
   */
  bool is_calibrating();

  /**
   * @brief Checks if the ports are configured correctly
   * @returns Zeros since it's a simulated device
   */
  std::pair<uint8_t, uint8_t> check_port();

 private:
  double angle = 0.0;
};

}  // namespace rev