#pragma once

#ifdef PLATFORM_BRAIN
#include "odometry.hh"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "rev/api/v5/async/async_runnable.hh"
#include "rev/api/v5/hardware/devices/gyroscope/gyroscope.hh"
#include "rev/api/v5/hardware/devices/rotation_sensors/rotary_sensors.hh"

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
   * @param ivertical_distance_from_cog Vertical distance from the center, positive is behind the center, negative is in front.
   * @param ihorizontal_distance_from_cog Horizontal distance from the center, positive is to the right of the center, negative is to the left.
   */
  TwoRotationInertialOdometry(
      std::shared_ptr<RotarySensor> ilongitudinal_sensor,
      std::shared_ptr<RotarySensor> ilateral_sensor,
      std::shared_ptr<Gyroscope> iinertial,
      QLength ilongitudinal_wheel_diameter = 3.25 * inch,
      QLength ilateral_wheel_diameter = 3.25 * inch,
      QLength ivertical_distance_from_cog = 0 * inch,
      QLength ihorizontal_distance_from_cog = 0 * inch);

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

  // Offsets
  QLength vertical_distance_from_cog;
  QLength horizontal_distance_from_cog;
};

}  // namespace rev

#endif