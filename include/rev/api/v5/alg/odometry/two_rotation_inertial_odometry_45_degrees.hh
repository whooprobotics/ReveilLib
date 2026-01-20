#pragma once

#ifdef PLATFORM_BRAIN
#include "odometry.hh"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "rev/api/v5/async/async_runnable.hh"
#include "rev/api/v5/hardware/devices/gyroscope/gyroscope.hh"
#include "rev/api/v5/hardware/devices/rotation_sensors/rotary_sensors.hh"
namespace rev {
/**
 * @brief Odometry implementation using 2 tracking wheels mounted at
 * a 45 degree angle to each other and an inertial
 *
 */
class TwoRotationInertialOdometry45Degrees : public Odometry,
                                             public AsyncRunnable {
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
   * 
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
   * @brief Constructs a new TwoRotationInertialOdometry45Degrees object
   * 
   * @param left_sensor std::shared_ptr<RotarySensor> pointing to the left sensor
   * @param right_sensor std::shared_ptr<RotarySensor> pointing to the right sensor
   */
  TwoRotationInertialOdometry45Degrees(
      std::shared_ptr<rev::RotarySensor> left_sensor,
      std::shared_ptr<rev::RotarySensor> right_sensor,
      std::shared_ptr<rev::Gyroscope> iinertial,
      QLength ilongitudinal_wheel_diameter = 3.25 * inch,
      QLength ilateral_wheel_diameter = 3.25 * inch,
      QLength ilongitudinal_wheel_offset = 0 * inch,
      QLength ilateral_wheel_offset = 0 * inch);

 private:
  /*
   * Which sensor is which doesn't matter frankly, left and right are
   * used for convention. The sensor readings are averaged out.
   */

  // Sensor on left side of robot
  std::shared_ptr<rev::RotarySensor> left_sensor;
  // Sensor on right side of robot
  std::shared_ptr<rev::RotarySensor> right_sensor;
  // Inertial sensor from which the robot yaw will be read
  std::shared_ptr<rev::Gyroscope> inertial;

  // Mutex for updating odometry position
  pros::Mutex current_position_mutex;

  // Current odometry state
  OdometryState current_position{{0_in, 0_in, 0_deg},
                                 {0_mps, 0_mps, 0_deg / second}};

  // Used for getting differences
  double longitude_ticks_last;
  double latitude_ticks_last;

  // Only used for velocity calculation
  double heading_ticks_last;

  // We call this init instead of last because it is used for absolutes
  double heading_ticks_init;

  // Time of last call
  int32_t time_last = -1;

  bool is_initialized = false;

  // Wheel sizes
  QLength left_wheel_diameter;
  QLength right_wheel_diameter;

  // Longitudinal offset to the right of the center of the robot
  QLength longitudinal_wheel_offset;
  // Likewise, for the lateral wheel backward from the center of rotation
  QLength lateral_wheel_offset;
};
}  // namespace rev

#endif