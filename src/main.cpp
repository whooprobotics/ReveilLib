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
#include "rev/api/hardware/devices/gyroscope/imu.hh"
#include "rev/rev.hh"
#include "rev/api/hardware/chassis/mecanum_chassis.hh"
#include "rev/api/hardware/chassis/asterisk_chassis.hh"
#include "rev/api/alg/odometry/two_rotation_inertial_odometry_45_degrees.hh"
#include "rev/api/units/q_time.hh"
#include "rev/api/units/q_length.hh"
#include "rev/api/alg/slipstream/mecanum_segment.hh"
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include "rev/api/hardware/chassis/butterfly_chassis.hh"

using std::shared_ptr, std::make_shared, std::vector, std::string, std::cout, std::endl;
using namespace rev;

rev::MotorGroup front_left({3, -5});
rev::MotorGroup back_left({-1, 2});
rev::MotorGroup front_right({17, -8});
rev::MotorGroup back_right({-9, 10});
rev::Motor center_left(-11);
rev::Motor center_right(12);

shared_ptr<rev::ReadOnlyRotarySensor> left_enc = make_shared<rev::QuadEncoder>(static_cast<int>('A'), static_cast<int>('B'), false);
shared_ptr<rev::ReadOnlyRotarySensor> right_enc = make_shared<rev::QuadEncoder>(static_cast<int>('C'), static_cast<int>('D'), true);
shared_ptr<rev::Gyroscope> imu = make_shared<rev::Imu>(14);

// shared_ptr<rev::MecanumChassis> chassis = make_shared<rev::MecanumChassis>(front_left, front_right, back_left, back_right);

// butterfly chassis initialization
// #define left_piston_port 'G'
// #define right_piston_port 'H'
// pros::ADIDigitalOut left_piston (left_piston_port);
// pros::ADIDigitalOut right_piston (right_piston_port);
// shared_ptr<rev::ButterflyChassis> chassis = make_shared<rev::ButterflyChassis>(front_left, front_right, back_left, back_right, left_piston, right_piston);

shared_ptr<rev::AsteriskChassis> chassis = make_shared<rev::AsteriskChassis>(front_left, front_right, back_left, back_right, center_left, center_right);
shared_ptr<rev::TwoRotationInertialOdometry45Degrees> odom = make_shared<rev::TwoRotationInertialOdometry45Degrees>(left_enc, right_enc, imu, 2.46_in, 2.46_in);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

shared_ptr<AsyncRunner> odom_runner;

void on_center_button() {}

void initialize() {
  pros::lcd::initialize();

  odom_runner = make_shared<AsyncRunner>(odom);
  std::cout << "Initialized" << std::endl;
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
          {40.2_in, 40_in}, 0_in),
      }
  );
}

int signum(double x) {
  if (x >= 0)
    return 1;
  else return -1;
}

void reset_odom() {
  odom->reset_position();
}

void opcontrol() {
  // controller.print(0, 0, "furk");

  while(true) {
    /*
     * TEST MECANUM DRIVE
    */
    double left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double left_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    double right_x = -1 * controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;
    int exp = 3;
    double forward = pow(left_y, exp);
    double strafe = pow(left_x, exp);
    double turn = pow(right_x, exp);

    chassis->drive_holonomic(forward, turn, strafe);

    OdometryState current_state = odom->get_state();

    pros::lcd::print(0, "X: %f", current_state.pos.x.convert(inch));
    pros::lcd::print(1, "Y: %f", current_state.pos.y.convert(inch));
    pros::lcd::print(2, "a: %f", current_state.pos.theta.convert(degree));
    // pros::lcd::print(0, "L encoder: %f", left_enc->get_position());
    // pros::lcd::print(1, "R encoder: %f", right_enc->get_position());
    // pros::lcd::print(2, "IMU heading: %f", imu->get_heading());

    // std::cout << "Odom X: " << current_state.pos.x.convert(inch) << std::endl;
    // std::cout << "Odom Y: " << current_state.pos.y.convert(inch) << std::endl;
    // std::cout << "Odom A: " << current_state.pos.theta.convert(degree) << std::endl;
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



  
  // Butterfly teleop
  /* while(true) {
    double left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double left_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    double right_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
      chassis->toggle_pistons();
    }

    int exp = 3;
    double forward = pow(left_y, exp);
    double strafe = pow(left_x, exp);
    double turn = pow(right_x, exp);

    chassis->drive_auto(forward, turn, strafe);

    pros::lcd::print(0, "Left Y: %d", forward);
    pros::lcd::print(1, "Left X: %d", strafe);
    pros::lcd::print(2, "Right X: %d", turn);
    pros::lcd::print(3, "Piston state: %d", chassis->get_pistons());
    pros::delay(20);
  } */


}
