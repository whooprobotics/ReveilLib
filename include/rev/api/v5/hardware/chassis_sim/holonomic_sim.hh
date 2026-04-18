#pragma once

#ifdef PLATFORM_BRAIN

#include "rev/api/v5/async/async_runnable.hh"
#include "rev/api/v5/hardware/chassis/holonomic_chassis.hh"
#include "rev/api/v5/alg/odometry/odometry.hh"
#include "rev/api/common/units/q_speed.hh"
#include "rev/api/common/units/q_angular_speed.hh"
#include "rev/api/common/units/q_frequency.hh"
#include "rev/api/common/units/q_time.hh"
#include "rev/api/v5/alg/slipstream/power.hh"

namespace rev {

class HolonomicSim : public HolonomicChassis, public Odometry, public AsyncRunnable {
 public:
  HolonomicSim(QSpeed iv_max,
               QAngularSpeed iw_max,
               QFrequency ilinear_decay_rate,
               QFrequency iangular_decay_rate,
               QFrequency ilinear_brake_decay_rate,
               QFrequency iangular_brake_decay_rate);

  // HolonomicChassis implementation requirements

  /**
   * @brief Drive the robot in a mecanum drive style
   * 
   * @param power Struct storing all motor powers for holonomic drives
   */
  void drive_holonomic(SlipstreamPower power) override;

  /**
   * @brief Heuristic function for a holonomic drive from driver controls
   * 
   * @param forward The amount the robot is to move forward
   * @param yaw The amount the robot is to turn
   * @param strafe The amount the robot is to strafe
   */
  void drive_holonomic(double forward, double yaw, double strafe) override;

  // Chassis implementation requirements

  /**
   * @brief Moves the virtual robot
   *
   * @param left From [-1.0, 1.0]. Applies a voltage to the motor group.
   * @param right From [-1.0, 1.0]. Applies a voltage to the motor group.
   */
  void drive_tank(double left, double right) override;
  
  /**
   * @brief Moves the virtual robot
   *
   * @param forward From [-1.0, 1.0]. The forward component of the motion.
   * @param yaw From [-1.0, 1.0]. The yaw component of the motion.
   */
  void drive_arcade(double forward, double yaw) override;
  
  /**
   * @brief Sets the brake types of all motors to brake
   */
  void set_brake_harsh() override;
  
  /**
   * @brief Sets the brake types of all motors to coast
   */
  void set_brake_coast() override;
  
  /**
   * @brief Stops all of the motors
   */
  void stop() override;

  // Odometry implementation requirements

  /**
   * @brief Get the current position and velocity of the robot
   *
   * @return OdometryState
   */
  OdometryState get_state() override;
  
  /**
   * @brief Set the position of the controller
   *
   * @param pos
   */
  void set_position(Position pos) override;
  
  /**
   * @brief Set the position to (0,0)
   *
   * THIS WILL ALSO RESET HEADING, WHICH YOU MIGHT NOT WANT
   *
   */
  void reset_position() override;

  // AsyncRunnable implementation requirements

  /**
   * @brief The step function for the runnable controller. This will be invoked
   * once every tdelta milliseconds.
   *
   */
  void step() override;

 private:
  const QSpeed v_max;
  const QAngularSpeed w_max;
  const QFrequency linear_decay_rate;
  const QFrequency angular_decay_rate;
  const QFrequency linear_brake_decay_rate;
  const QFrequency angular_brake_decay_rate;

  double forward_power{0.0};
  double strafe_power{0.0};
  double yaw_power{0.0};
  
  QSpeed vx_current{0.0_mps};  // velocity in x direction
  QSpeed vy_current{0.0_mps};  // velocity in y direction
  QAngularSpeed w_current{0.0_rpm}; // angular velocity

  bool use_harsh_brake_mode{false};

  int32_t time_h = -1;

  OdometryState state;
};

}  // namespace rev

#endif