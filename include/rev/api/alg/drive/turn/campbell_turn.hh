#pragma once

#include <memory>
#include "rev/api/alg/drive/turn/turn.hh"
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/async/async_awaitable.hh"
#include "rev/api/async/async_runnable.hh"
#include "rev/api/hardware/chassis/chassis.hh"  // For chassis model

namespace rev {
/**
 * @brief Class implementing Walker Campbell's turn algorithm under the Turn
 * interface
 *
 */
class CampbellTurn : public Turn,
                     public AsyncRunnable,
                     public AsyncAwaitable {  // Rename this if you want
 public:
  CampbellTurn(std::shared_ptr<Chassis> ichassis,
               std::shared_ptr<Odometry> iodometry,
               double ikP1,
               double ikP2);  // Constructor function

  /**
   * @brief Starts a turn towards an absolute heading
   *
   * @param max_power The maximum power the controller will output
   * @param angle The absolute heading the controller will target
   */
  void turn_to_target_absolute(double max_power, QAngle angle) override;

  /**
   * @brief Steps the controller, for use with AsyncRunner.
   *
   */
  void step() override;

  /**
   * @brief Blocks until the current turn completes
   *
   */
  void await() override;

  /**
   * @brief Tells if the controller is working or has completed its motion
   *
   * @return true if the controller is not doing anything
   * @return false if the controller is still working
   */
  bool is_completed();

 private:
  std::shared_ptr<Chassis> chassis;
  std::shared_ptr<Odometry> odometry;
  double kP1;
  double kP2;
  double max_power = 0;
  TurnState controller_state{TurnState::INACTIVE};
  QAngle angle_difference;
  QAngle target_relative_original;
  QAngle target_relative;
  double coast_turn_power = 0.175;  // 0.175 from testing
  int left_direction = 0;
  int right_direction = 0;
  int brake_start_time = -1;
  QAngle angle_goal = 0_deg;
};
};  // namespace rev