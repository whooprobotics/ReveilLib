#pragma once

#ifdef PLATFORM_BRAIN
#include <sstream>
#include <string>
#include <vector>
#include "odometry.hh"
#include "pros/rtos.hpp"
#include "rev/api/v5/async/async_runnable.hh"
#include "rev/api/v5/hardware/devices/optical/otos.hh"

namespace rev {
/**
 * @brief Odometry implementation using the Sparkfun Optical Tracking Odometry Sensor
 *
 */
class OpticalOdometry : public Odometry, public AsyncRunnable {
 public:
  /**
   * @brief Get the current position
   *
   * The implementation of this is thread-safe
   *
   * @return OdometryState Current robot position
   */
  OdometryState get_state() override;
  
  /**
   * @brief Sets the current robot position
   * @param pos The desired x, y (QLength), and theta (QAngle)
   */
  void set_position(Position pos) override;

  /**
   * @brief Resets the current robot position to {0, 0, 0}
   */
  void reset_position() override;

  /**
   * @brief Updates the current robot position with new data from sensors
   */
  void step() override;

  /**
   * @brief Constructs a new OpticalOdometry object
   * @param sensor std::shared_ptr<OTOS> pointing to the sensor to collect data from
   * @param ilongitudinal_offset QLength offset of the OTOS to the right of the center of the robot
   * @param ilateral_offset QLength offset of the OTOS behidn the robot center of rotation
   */
  OpticalOdometry(std::shared_ptr<OTOS> sensor,
                  QLength ilongitudinal_offset,
                  QLength ilateral_offset);

 private:
  std::shared_ptr<OTOS> optical_sensor;  // Tracking sensor
  pros::Mutex current_position_mutex;

  OdometryState current_position{{0_in, 0_in, 0_deg},
                                 {0_mps, 0_mps, 0_deg / second}};

  // Offset of the OTOS to the right of the center of the robot
  QLength longitudinal_offset;
  // Likewise, for the OTOS position backward from the center of rotation
  QLength lateral_offset;
};
}  // namespace rev

#endif