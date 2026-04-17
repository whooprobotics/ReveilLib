#include "rev/api/async/async_runnable.hh"
#include "rev/api/hardware/chassis_sim/chassis_sim.hh"

namespace rev {

class DriftlessSim : public ChassisSim, public AsyncRunnable {
 public:
  DriftlessSim(QSpeed iv_max,
               QAngularSpeed iw_max,
               QFrequency ilinear_decay_rate,
               QFrequency iangular_decay_rate,
               QFrequency ilinear_brake_decay_rate,
               QFrequency iangular_brake_decay_rate);

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

  double linear_power{0.0};
  double angular_power{0.0};
  QSpeed v_current{0.0_mps};
  QAngularSpeed w_current{0.0_rpm};

  bool use_harsh_brake_mode{false};

  int32_t time_h = -1;

  OdometryState state;
};

}  // namespace rev