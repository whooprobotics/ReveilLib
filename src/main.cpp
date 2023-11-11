#include "main.h"
#include "rev/rev.hh"
#include "rev/api/units/all_units.hh"
#include "rev/api/async/async_runner.hh"
#include "rev/api/alg/drive/turn/campbell_turn.hh"
#include "rev/api/alg/odometry/two_rotation_inertial_odometry.hh"
#include "rev/api/hardware/chassis/skid_steer_chassis.hh"

//#include <iostream>
// Gets all the motor ports
pros::Motor left1(15);
pros::Motor left2(18);
pros::Motor left3(-19);
pros::Motor left4(-20);
pros::Motor right1(4);
pros::Motor right2(6);
pros::Motor right3(-7);
pros::Motor right4(-9);
// Puts motors into groups, runs them all together by side
pros::Motor_Group motor_left({left1, left2, left3, left4});
pros::Motor_Group motor_right({right1, right2, right3, right4});
std::shared_ptr<rev::SkidSteerChassis> turnChassis = std::make_shared<rev::SkidSteerChassis>(motor_left,motor_right);
pros::Rotation fwd (5);
pros::Rotation rgt (16, true);
pros::Imu imu (14);

using namespace rev;

void on_center_button() {}

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    std::shared_ptr<rev::TwoRotationInertialOdometry> odom = std::make_shared<rev::TwoRotationInertialOdometry>(
        fwd,
        rgt,
        imu,
        2.09_in,
        2.75_in,
        4.75_in,
        0.5_in
    ); // Sets up the odometry controller

    AsyncRunner odom_runner (odom); // Sets up the thread which runs the odom

    pros::delay(2000);

    std::shared_ptr<rev::CampbellTurn> turn = std::make_shared<rev::CampbellTurn>(
        turnChassis,
        odom,
        0.2,
        0.05
    ); // Sets up the odometry controller

    AsyncRunner turn_runner (turn);

    turn->turn_to_target_absolute(.7,90_deg);

    while(true) { // This loop just prints things
        auto pose = odom->get_state().pos;
        std::cout << pose.x.convert(inch) << "in, " << pose.y.convert(inch) << "in, " << pose.facing.convert(degree) << "deg" << std::endl;

        pros::delay(250);
    }
}