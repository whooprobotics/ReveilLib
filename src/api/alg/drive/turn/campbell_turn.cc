
// You don't need a #pragma once here, because this isn't a header file and
// won't be included into anything
#include "rev/api/alg/drive/turn/campbell_turn.hh"
#include "api.h"
#include "rev/api/alg/odometry/odometry.hh"

#ifdef OFF_ROBOT_TESTS
#include <chrono>
using namespace std::chrono;
#define GET_TIME \
  duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()
#else
#define GET_TIME pros::millis()
#endif

namespace rev {
// The "CampbellTurn::" here just tells it that we are implementing a method
// that is a member of the CampbellTurn class
CampbellTurn::CampbellTurn(std::shared_ptr<Chassis> ichassis,
                           std::shared_ptr<Odometry> iodometry,
                           double ikP1,
                           double ikP2) {
  // Implement initialization stuff here, basically set up class member
  // variables
  kP1 = ikP1;  // was 0.2 in testing
  kP2 = ikP2;  // was 0.05 in testing
  chassis = ichassis;
  odometry = iodometry;
}
void CampbellTurn::turn_to_target_absolute(double imax_power, QAngle iangle) {
  max_power = imax_power;
  angle_goal = iangle;

  angle_difference = angle_goal - odometry->get_state().pos.facing;

  target_relative_original =
      angle_difference -
      360 * std::floor((angle_difference.convert(degree) + 180) / 360) * degree;

  angle_goal = odometry->get_state().pos.facing + target_relative_original;

  target_relative =
      angle_difference -
      360 * std::floor((angle_difference.convert(degree) + 180) / 360) * degree;
  controller_state = TurnState::FULLPOWER;

  if (target_relative <
      0 * degree) {  // if direction is negative, flip directions
    left_direction = -1;
    right_direction = 1;
  } else {
    left_direction = 1;
    right_direction = -1;
  }
}
void CampbellTurn::step() {
  if (controller_state == TurnState::INACTIVE) {
    return;
  }

  OdometryState state = odometry->get_state();

  // Full power turn
  if (controller_state == TurnState::FULLPOWER) {
    chassis->drive_tank(max_power * left_direction,
                        max_power * right_direction);
    // printf("full power\n");
  }
  // Low power turn
  else if (controller_state == TurnState::COAST) {
    chassis->drive_tank(left_direction * coast_turn_power,
                        right_direction * coast_turn_power);
    // printf("Coast\n");
  }
  // Activating hard brakes
  else if (controller_state == TurnState::BRAKE) {
    // If we haven't started our braking, we need to get the current time and
    // then start braking
    if (brake_start_time == -1) {
      // printf("start brake\n");
      brake_start_time = pros::millis();
      chassis->set_brake_harsh();
      chassis->stop();
    }
    // pros::delay(250); //pros::delay shouldn't be used in non-blocking
    // functions
    if (brake_start_time < GET_TIME - 250) {  // Check if 250ms has elapsed
      chassis->set_brake_coast();
      // printf("End brake\n");
      controller_state = TurnState::INACTIVE;  // set inactive and =-1 so it can
                                               // be called again
      brake_start_time = -1;
    }
  }
  // Disable motors/no power (normal coast mode) // is bad -> can interfer with
  // other code trying to manage chassis else if (controller_state ==
  // TurnState::INACTIVE)
  // {
  //     chassis->drive_tank(0,0);
  // }

  angle_difference = angle_goal - state.pos.facing;

  // Check Current angle/angular velocity and set controller_state
  target_relative =
      angle_difference -
      360 * std::floor((angle_difference.convert(degree) + 180) / 360) * degree;
  // Remain in fullpower if we are currently in fullpower and we arent ready to
  // advance
  // If its already set to fullpower, dont need to set again
  // if (fabs(target_relative.convert(degree)) >
  //         fabs(odometry->get_state().vel.angular.convert(degree / second) *
  //              kP1) &&
  //     controller_state == TurnState::FULLPOWER) {
  //       printf("Setting fullpower\n");
  //   controller_state = TurnState::FULLPOWER;
  // }
  // printf("target_relative:%f\n",fabs(target_relative.convert(degree)));
  // printf("angular_velocity * k1 = %f
  // \n",fabs(odometry->get_state().vel.angular.convert(degree / second) *
  // kP1));
  // printf("angular_velocity * k2 = %f
  // \n",fabs(odometry->get_state().vel.angular.convert(degree / second) *
  // kP2));
  // Start slowdown if we are ready for that and we haven't started
  // harsh-braking
  if (fabs(target_relative.convert(degree)) <
          fabs(odometry->get_state().vel.angular.convert(degree / second) *
               kP1) &&
      controller_state != TurnState::BRAKE &&
      controller_state != TurnState::COAST) {
    // printf("Setting coast\n");
    controller_state = TurnState::COAST;
  }
  // Harsh-brake if we're at that point
  if (fabs(target_relative.convert(degree)) <
          fabs(odometry->get_state().vel.angular.convert(degree / second) *
               kP2) &&
      controller_state != TurnState::BRAKE) {
    // printf("Setting brake\n");
    controller_state = TurnState::BRAKE;
  }

  // Implement the actual controller logic
  // This should basically behave as a "state transition" function
  // It should not block execution, instead it should just make its checks,
  // update the motor outputs if needed, and immediately return How to achieve
  // that is up to you
}
// You will need implementations for every method

bool CampbellTurn::is_completed() {
  return (controller_state == TurnState::INACTIVE);
}
};  // namespace rev