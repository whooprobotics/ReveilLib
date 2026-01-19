#pragma once
#include "odometry.hh"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "rev/api/async/async_runnable.hh"
#include "rev/api/hardware/devices/gyroscope/gyroscope.hh"
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {
/**
 * @brief Odometry implementation using 2 tracking wheels and an inertial
 *
 */
class TwoRotationInertialOdometry : public Odometry, public AsyncRunnable {
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
   * @brief Constructs a new TwoRotationInertialOdometry object
   * 
   * @param ilongitudinal_sensor std::shared_ptr<RotarySensor> pointing to the sensor along robot's longitudinal axis
   * @param ilateral_sensor std::shared_ptr<RotarySensor> pointing to the sensor along robot's lateral axis
   * @param iinertial std::shared_ptr<Gyroscope> pointing to the inertial sensors
   * @param ilongitudinal_wheel_diameter Diameter of the wheel on the longitudinal sensor
   * @param ilateral_wheel_diameter Diameter of the wheel on the lateral sensor
   * @param ilongitudinal_wheel_offset Offset of the longitudinal wheel to the right of the center of the robot
   * @param ilateral_wheel_offset Offset of the lateral wheel backward from the center of rotation
   */
  TwoRotationInertialOdometry(
      std::shared_ptr<RotarySensor> ilongitudinal_sensor,
      std::shared_ptr<RotarySensor> ilateral_sensor,
      std::shared_ptr<Gyroscope> iinertial,
      QLength ilongitudinal_wheel_diameter = 3.25 * inch,
      QLength ilateral_wheel_diameter = 3.25 * inch,
      QLength ilongitudinal_wheel_offset = 0 * inch,
      QLength ilateral_wheel_offset = 0 * inch);

 private:
  std::shared_ptr<rev::RotarySensor>
      longitudinal_sensor;  // Sensor indicating forward motion.
                            // Moving the robot forward should cause
                            // the position of this to increase.
  std::shared_ptr<rev::RotarySensor>
      lateral_sensor;  // Sensor indicating motion to the right.
                       // Moving the robot right should cause the
                       // position of this to increase.
  std::shared_ptr<rev::Gyroscope>
      inertial;  // Inertial sensor from which the robot yaw will be read

#ifndef OFF_ROBOT_TESTS
  pros::Mutex current_position_mutex;
#endif
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
  QLength longitudinal_wheel_diameter;
  QLength lateral_wheel_diameter;

  // Offset of the longitudinal wheel to the right of the center of the robot
  QLength longitudinal_wheel_offset;
  // Likewise, for the lateral wheel backward from the center of rotation
  QLength lateral_wheel_offset;
};
}  // namespace rev