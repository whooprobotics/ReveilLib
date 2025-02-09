/**
This file primarily consists of code modified from PROS 3.8, and is therefore
governed by the Mozilla Public License. You can view the license at
include/pros/LICENSE
*/

#pragma once
#include <cstdint>
#include "pros/motors.h"

namespace rev {
using pros::motor_brake_mode_e_t;
using pros::motor_encoder_units_e_t;
using pros::motor_gearset_e_t;

/**
 * @brief Interface for implementing motors
 *
 * This is intended to be 1:1 with pros motors but a few things were removed for
 * various reasons, such as just being deprecated or useless (we don't have
 * enough time for every little feature)
 */
class AnyMotor {
 public:
  /****************************************************************************/
  /**                         Motor movement functions                       **/
  /**                                                                        **/
  /**          These functions allow programmers to make motors move         **/
  /****************************************************************************/
  /**
   * Sets the voltage for the motor from -127 to 127.
   *
   * This is designed to map easily to the input from the controller's analog
   * stick for simple opcontrol use. The actual behavior of the motor is
   * analogous to use of motor_move(), or motorSet() from the PROS 2 API.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param voltage
   *        The new motor voltage from -127 to 127
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t move(std::int32_t voltage) const = 0;

  /**
   * Sets the target absolute position for the motor to move to.
   *
   * This movement is relative to the position of the motor when initialized or
   * the position when it was most recently reset with
   * pros::Motor::set_zero_position().
   *
   * \note This function simply sets the target for the motor, it does not block
   * program execution until the movement finishes.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param position
   *        The absolute position to move to in the motor's encoder units
   * \param velocity
   *        The maximum allowable velocity for the movement in RPM
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t move_absolute(const double position,
                                     const std::int32_t velocity) const = 0;

  /**
   * Sets the relative target position for the motor to move to.
   *
   * This movement is relative to the current position of the motor as given in
   * pros::Motor::motor_get_position(). Providing 10.0 as the position parameter
   * would result in the motor moving clockwise 10 units, no matter what the
   * current position is.
   *
   * \note This function simply sets the target for the motor, it does not block
   * program execution until the movement finishes.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param position
   *        The relative position to move to in the motor's encoder units
   * \param velocity
   *        The maximum allowable velocity for the movement in RPM
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t move_relative(const double position,
                                     const std::int32_t velocity) const = 0;

  /**
   * Sets the velocity for the motor.
   *
   * This velocity corresponds to different actual speeds depending on the
   * gearset used for the motor. This results in a range of +-100 for
   * E_MOTOR_GEARSET_36, +-200 for E_MOTOR_GEARSET_18, and +-600 for
   * E_MOTOR_GEARSET_6. The velocity is held with PID to ensure consistent
   * speed, as opposed to setting the motor's voltage.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param velocity
   *        The new motor velocity from -+-100, +-200, or +-600 depending on the
   *        motor's gearset
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t move_velocity(const std::int32_t velocity) const = 0;

  /**
   * Sets the output voltage for the motor from -12000 to 12000 in millivolts.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param voltage
   *        The new voltage value from -12000 to 12000
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t move_voltage(const std::int32_t voltage) const = 0;

  /**
   * Stops the motor using the currently configured brake mode.
   *
   * This function sets motor velocity to zero, which will cause it to act
   * according to the set brake mode. If brake mode is set to MOTOR_BRAKE_HOLD,
   * this function may behave differently than calling move_absolute(0)
   * or move_relative(0).
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t brake(void) const = 0;

  /**
   * Changes the output velocity for a profiled movement (motor_move_absolute()
   * or motor_move_relative()). This will have no effect if the motor is not
   * following a profiled movement.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param velocity
   *        The new motor velocity from +-100, +-200, or +-600 depending on the
   *        motor's gearset
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t modify_profiled_velocity(
      const std::int32_t velocity) const = 0;

  /****************************************************************************/
  /**                        Motor telemetry functions                       **/
  /**                                                                        **/
  /**    These functions allow programmers to collect telemetry from motors  **/
  /****************************************************************************/

  /**
   * Gets the actual velocity of the motor.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \return The motor's actual velocity in RPM or PROS_ERR_F if the operation
   * failed, setting errno.
   */
  virtual double get_actual_velocity(void) const = 0;

  /**
   * Gets the direction of movement for the motor.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \return 1 for moving in the positive direction, -1 for moving in the
   * negative direction, and PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t get_direction(void) const = 0;

  /**
   * Checks if the motor is stopped.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \note Although this function forwards data from the motor, the motor
   * presently does not provide any value. This function returns PROS_ERR with
   * errno set to ENOSYS.
   *
   * \return 1 if the motor is not moving, 0 if the motor is moving, or PROS_ERR
   * if the operation failed, setting errno
   */
  virtual std::int32_t is_stopped(void) const = 0;

  /**
   * Gets the raw encoder count of the motor at a given timestamp.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param[in] timestamp
   *            A pointer to a time in milliseconds for which the encoder count
   *            will be returned. If NULL, the timestamp at which the encoder
   *            count was read will not be supplied
   *
   * \return The raw encoder count at the given timestamp or PROS_ERR if the
   * operation failed.
   */
  virtual std::int32_t get_raw_position(
      std::uint32_t* const timestamp) const = 0;

  /**
   * Gets the absolute position of the motor in its encoder units.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \return The motor's absolute position in its encoder units or PROS_ERR_F
   * if the operation failed, setting errno.
   */
  virtual double get_position(void) const = 0;

  /****************************************************************************/
  /**                      Motor configuration functions                     **/
  /**                                                                        **/
  /**  These functions allow programmers to configure the behavior of motors **/
  /****************************************************************************/

  /**
   * Sets the position for the motor in its encoder units.
   *
   * This will be the future reference point for the motor's "absolute"
   * position.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param position
   *        The new reference position in its encoder units
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t set_zero_position(const double position) const = 0;

  /**
   * Sets the "absolute" zero position of the motor to its current position.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t tare_position(void) const = 0;

  /**
   * Sets one of motor_brake_mode_e_t to the motor.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param mode
   *        The motor_brake_mode_e_t to set for the motor
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t set_brake_mode(
      const motor_brake_mode_e_t mode) const = 0;

  /**
   * Sets one of motor_encoder_units_e_t for the motor encoder.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param units
   *        The new motor encoder units
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t set_encoder_units(
      const motor_encoder_units_e_t units) const = 0;

  /**
   * Sets one of motor_gearset_e_t for the motor.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param gearset
   *        The new motor gearset
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t set_gearing(const motor_gearset_e_t gearset) const = 0;

  /**
   * Sets the reverse flag for the motor.
   *
   * This will invert its movements and the values returned for its position.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \param reverse
   *        True reverses the motor, false is default
   *
   * \return 1 if the operation was successful or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t set_reversed(const bool reverse) = 0;

  /**
   * Gets the brake mode that was set for the motor.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \return One of motor_brake_mode_e_t, according to what was set for the
   * motor, or E_MOTOR_BRAKE_INVALID if the operation failed, setting errno.
   */
  virtual motor_brake_mode_e_t get_brake_mode(void) const = 0;

  /**
   * Gets the encoder units that were set for the motor.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * ENODEV - The port cannot be configured as a motor
   *
   * \return One of motor_encoder_units_e_t according to what is set for the
   * motor or E_MOTOR_ENCODER_INVALID if the operation failed.
   */
  virtual motor_encoder_units_e_t get_encoder_units(void) const = 0;
};
}  // namespace rev