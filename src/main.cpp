#include "main.h"
#include "rev/rev.hh"
#include "rev/api/units/all_units.hh"
#include "rev/api/async/async_runner.hh"

//#include <iostream>

pros::Rotation fwd (5);
pros::Rotation rgt (16, true);
pros::Imu imu (14);
pros::Controller controller (pros::controller_id_e_t::E_CONTROLLER_MASTER);

pros::Motor test_motor (15);
using namespace rev;

void on_center_button() {}

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    //controller.print(0, 0, "furk");
    //test_motor.move_voltage(12000);
    std::shared_ptr<rev::TwoRotationInertialOdometry> odom = std::make_shared<rev::TwoRotationInertialOdometry>(
        fwd,
        rgt,
        imu,
        2.09_in,
        2.75_in,
        4.75_in,
        0.5_in
    );

    AsyncRunner odom_runner (odom);

    while(true) {
        //printf("loop\n");
        auto pose = odom->get_state().pos;
        std::cout << pose.x.convert(inch) << "in, " << pose.y.convert(inch) << "in, " << pose.facing.convert(degree) << "deg" << std::endl;

        pros::delay(250);
    }
}
