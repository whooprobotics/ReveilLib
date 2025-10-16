#include "main.h"
#include "rev/api/alg/reckless/path.hh"
#include "rev/api/alg/reckless/turn_segment.hh"
#include "rev/api/alg/reckless/look_at.hh"
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"
#include "rev/api/hardware/devices/rotation_sensors/rotation_sensor.hh"
#include "rev/rev.hh"
#include "rev/api/hardware/chassis/mecanum_chassis.hh"
#include <vector>
#include <string>

using std::shared_ptr, std::make_shared, std::vector, std::string;

rev::MotorGroup front_left({-3, 5});
rev::MotorGroup back_left({1, -2});
rev::MotorGroup front_right({-7, 8});
rev::MotorGroup back_right({9, -10});

shared_ptr<rev::MecanumChassis> chassis = make_shared<rev::MecanumChassis>(front_left, front_right, back_left, back_right);
pros::Controller controller(pros::E_CONTROLLER_MASTER);



void on_center_button() {}

void initialize() {
  pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  // controller.print(0, 0, "furk");
  while(true) {
    /*
     * TEST MECANUM DRIVE
    */
    double forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double strafe = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    double turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;
    chassis->drive_holonomic(forward, turn, strafe);

    pros::lcd::print(0, "Left Y: %d", forward);
    pros::lcd::print(1, "Left X: %d", strafe);
    pros::lcd::print(2, "Right X: %d", turn);
    /*
     * END TEST MECANUM DRIVE
     */


    /*
     * TEST MECANUM TANK DRIVE
     */
    // double left = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    // double right = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;
    // chassis->drive_arcade(left, right);
    pros::delay(20);
  }



}