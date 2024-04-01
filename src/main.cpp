#include "main.h"
#include "rev/rev.hh"

//#include <iostream>

// Slingo test
// MotorGroup drive_left = {15, 18, -19, -20};
// MotorGroup drive_right = {4, 6, -7, -9};

pros::Motor_Group leftd({-1, -2, -3, -4});
pros::Motor_Group rightd({12, 13, 14, 15});

pros::Rotation fwd(5);
pros::Rotation rgt(16, true);
pros::Imu imu(14);
pros::Controller controller(pros::controller_id_e_t::E_CONTROLLER_MASTER);

std::shared_ptr<rev::SkidSteerChassis> chassis =
    std::make_shared<rev::SkidSteerChassis>(leftd, rightd);

pros::Motor test_motor(15);
using namespace rev;

void on_center_button() {}

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  // controller.print(0, 0, "furk");
  // test_motor.move_voltage(12000);
  pros::delay(2000);
  std::shared_ptr<rev::TwoRotationInertialOdometry> odom =
      std::make_shared<rev::TwoRotationInertialOdometry>(
          fwd, rgt, imu, 2.09_in, 2.75_in, 4.75_in, 0.5_in);
  
  odom->set_position({1.5_ft, 10.5_ft, 135_deg});

  // odom->set_position({215_in, 49_in, 16_deg});
  AsyncRunner odom_runner(odom);

  /*std::shared_ptr<rev::CampbellTurn> turnc =
  std::make_shared<CampbellTurn>(chassis, odom, 0.2, 0.05);

  AsyncRunner turn_runner (turnc);

  pros::delay(2000);
  turnc->turn_to_target_absolute(0.7, -15_deg);
  //pros::delay(2000);
  turnc->turn_to_target_absolute(0.7, -30_deg);*/

  std::shared_ptr<rev::Reckless> reckless =
      std::make_shared<Reckless>(chassis, odom);

  AsyncRunner reckless_runner(reckless);

  std::shared_ptr<rev::CampbellTurn> turn =
      std::make_shared<CampbellTurn>(chassis, odom, 0.2, 0.05);

  AsyncRunner turn_runner(turn);

  pros::delay(2000);

  const double kP = 0.0;
  const double kB = 0.015;

  /*turn->turn_to_target_absolute(0.7, 45_deg);

  while (!turn->is_completed())
    pros::delay(20);*/

  reckless->go(RecklessPath()
                   .with_segment(RecklessPathSegment(
                       std::make_shared<CascadingMotion>(0.7, kP, kB,
                                                         40_in / second, 0.07),
                       std::make_shared<PilonsCorrection>(2, 0.5_in),
                       std::make_shared<SimpleStop>(0.1_s, 0.2_s, 0.4),
                       {4_ft, 8_ft, 0_deg}, 0_in)

                                     )
                   .with_segment(RecklessPathSegment(
                       std::make_shared<CascadingMotion>(0.7, kP, kB,
                                                         40_in / second, 0.07),
                       std::make_shared<PilonsCorrection>(2, 0.5_in),
                       std::make_shared<SimpleStop>(.1_s, 0.2_s, 0.4),
                       {8_ft, 8_ft, 45_deg}, 0_in))
                   );

  while(!reckless->is_completed()) pros::delay(20);
  printf("Completed motion");

  // reckless->go(RecklessPath());

  turn_runner.~AsyncRunner();  // Stop AsyncRunner

  turn->turn_to_target_absolute(
      0.7, 45_deg);  // Attempt another turn with AsyncRunner deleted, this
                     // should fail

  while (true) {
    // printf("loop\n");
    auto pose = odom->get_state().pos;
    std::cout << pose.x.convert(foot) << "ft, " << pose.y.convert(foot) << ","
              << pose.theta.convert(degree) << "deg" << std::endl;

    // if(controller.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A))
    //     odom->set_position({215_in, 49_in, 16_deg});

    // chassis->drive_arcade(
    //     (double)controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y)/100,
    //     (double)controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_X)/100
    //);

    pros::delay(250);
  }
}