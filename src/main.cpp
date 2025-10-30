#include "main.h"
#include "rev/api/alg/reckless/path.hh"
#include "rev/api/alg/reckless/turn_segment.hh"
#include "rev/api/alg/reckless/look_at.hh"
#include "rev/api/alg/slipstream/correction/cross_track_correction.hh"
#include "rev/api/alg/slipstream/motion/mecanum_constant_motion.hh"
#include "rev/api/alg/slipstream/slipstream.hh"
#include "rev/api/alg/stop/simple_holonomic_stop.hh"
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"
#include "rev/api/hardware/devices/rotation_sensors/rotation_sensor.hh"
#include "rev/rev.hh"
#include "rev/api/hardware/chassis/mecanum_chassis.hh"
#include "rev/api/hardware/chassis/asterisk_chassis.hh"
#include "rev/api/units/q_time.hh"
#include "rev/api/units/q_length.hh"
#include "rev/api/alg/slipstream/mecanum_segment.hh"
#include <memory>
#include <vector>
#include <string>
#include <iostream>

using std::shared_ptr, std::make_shared, std::vector, std::string, std::cout, std::endl;
using namespace rev;

rev::MotorGroup front_left({-3, 5});
rev::MotorGroup back_left({1, -2});
rev::MotorGroup front_right({-17, 8});
rev::MotorGroup back_right({9, -10});
rev::Motor center_left(11);
rev::Motor center_right(-12);

// shared_ptr<rev::MecanumChassis> chassis = make_shared<rev::MecanumChassis>(front_left, front_right, back_left, back_right);
shared_ptr<rev::AsteriskChassis> chassis = make_shared<rev::AsteriskChassis>(front_left, front_right, back_left, back_right, center_left, center_right);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void on_center_button() {}

void initialize() {
  pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  shared_ptr<rev::MecanumChassis> chassis = make_shared<rev::MecanumChassis>(front_left, front_right, back_left, back_right);
  shared_ptr<rev::Odometry> odom;
  // chassis->drive_holonomic(0.5, 0.5, 0.5);

  shared_ptr<rev::Slipstream> slipstream;

  // Set start position and consts
  rev::QTime harsh = 0.06_s;
  rev::QTime coast = 0.2_s;

  // Turn constants
  double harsh_turn = 0.085;
  double coast_turn = 0.23;
  rev::QTime brake_time = 0.1_s;

  // Powers
  double rush_power = 0.87;
  double fast_power = 0.75;
  double medium_power = 0.5;
  double slow_power = 0.32;
  double coast_power = 0.25;

  // Motion controllers
  shared_ptr<rev::MecanumConstantMotion> rush_motion = make_shared<rev::MecanumConstantMotion>(rush_power);
  shared_ptr<rev::MecanumConstantMotion> fast_motion = make_shared<rev::MecanumConstantMotion>(fast_power);
  shared_ptr<rev::MecanumConstantMotion> medium_motion = make_shared<rev::MecanumConstantMotion>(medium_power);
  shared_ptr<rev::MecanumConstantMotion> slow_motion = make_shared<rev::MecanumConstantMotion>(slow_power);
  shared_ptr<rev::CrossTrackCorrection> ct_correction = make_shared<rev::CrossTrackCorrection>(2, 1, 0.5_in, 3_deg);

  cout << "Starting route" << endl;
  uint8_t timeout = 0;

  rev::Pose start_position = {19.8_in, 48_in, -22.1_deg};
  odom->set_position(start_position);

  // test movement
  slipstream->go(
      {
        // Move fwd
        &MecanumSegment(
          rush_motion,
          ct_correction,
          make_shared<rev::SimpleHolonomicStop>(0_s, 0_s, coast_power),
          {40.2_in, 40_in, 0_deg}, 0_in),
      }
  );
}

int signum(double x) {
  if (x >= 0)
    return 1;
  else return -1;
}

void opcontrol() {
  // controller.print(0, 0, "furk");
  while(true) {
    /*
     * TEST MECANUM DRIVE
    */
    double left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double left_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    double right_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;
    int exp = 3;
    double forward = pow(left_y, exp);
    double strafe = pow(left_x, exp);
    double turn = pow(right_x, exp);

    chassis->drive_holonomic(forward, turn, strafe);

    // pros::lcd::print(0, "Left Y: %d", forward);
    // pros::lcd::print(1, "Left X: %d", strafe);
    // pros::lcd::print(2, "Right X: %d", turn);
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
