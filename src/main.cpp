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
#include "rev/api/alg/pid/pid.hh"

using std::shared_ptr, std::make_shared, std::vector, std::string, std::cout, std::endl;
using namespace rev;

rev::MotorGroup front_left({11, -12});
rev::MotorGroup back_left({-14, 15});
rev::MotorGroup front_right({-10, 19});
rev::MotorGroup back_right({17, -16});
rev::Motor center_left(13);
rev::Motor center_right(-8);

rev::Motor intake(1);

pros::Motor lever1(-2, pros::E_MOTOR_GEAR_RED);
pros::Motor lever2(9, pros::E_MOTOR_GEAR_RED);

pros::Motor_Group lever({lever1, lever2});
pros::ADIDigitalOut scraper('D');
pros::ADIDigitalOut lift('C');
pros::ADIDigitalOut hood('B');

// shared_ptr<rev::ReadOnlyRotarySensor> right_enc = make_shared<rev::QuadEncoder>(static_cast<int>('E'), static_cast<int>('F'), false);
// shared_ptr<rev::ReadOnlyRotarySensor> left_enc = make_shared<rev::QuadEncoder>(static_cast<int>('H'), static_cast<int>('G'), true);
// shared_ptr<rev::Imu> imu = make_shared<rev::Imu>(14);

// shared_ptr<rev::MecanumChassis> chassis = make_shared<rev::MecanumChassis>(front_left, front_right, back_left, back_right);

// butterfly chassis initialization
// #define left_piston_port 'G'
// #define right_piston_port 'H'
// pros::ADIDigitalOut left_piston (left_piston_port);
// pros::ADIDigitalOut right_piston (right_piston_port);
// shared_ptr<rev::ButterflyChassis> chassis = make_shared<rev::ButterflyChassis>(front_left, front_right, back_left, back_right, left_piston, right_piston);

shared_ptr<rev::AsteriskChassis> chassis = make_shared<rev::AsteriskChassis>(front_left, front_right, back_left, back_right, center_left, center_right);
rev::PID lever_pid(.01, 0.0, 0);
double lever_target = 0;
// shared_ptr<rev::TwoRotationInertialOdometry45Degrees> odom = make_shared<rev::TwoRotationInertialOdometry45Degrees>(left_enc, right_enc, imu, 2.46_in, 2.46_in);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// shared_ptr<AsyncRunner> odom_runner;

void on_center_button() {}

void initialize() {
  // pros::lcd::initialize();

  // odom_runner = make_shared<AsyncRunner>(odom);
  // std::cout << "Initialized" << std::endl;
}

void disabled() {}

void competition_initialize() {}

void pilons_test() {
  // rev::MotorGroup left({11, -12, 13, -14, 15});
  // rev::MotorGroup right({-16, 17, -18, 19, -20});

  // shared_ptr<rev::ReadOnlyRotarySensor> right_enc = make_shared<rev::QuadEncoder>(static_cast<int>('C'), static_cast<int>('D'), false);
  // shared_ptr<rev::ReadOnlyRotarySensor> left_enc = make_shared<rev::QuadEncoder>(static_cast<int>('F'), static_cast<int>('E'), true); 
  // shared_ptr<rev::Imu> skid_imu = make_shared<rev::Imu>(5);

  // shared_ptr<rev::SkidSteerChassis> skid_chassis = make_shared<rev::SkidSteerChassis>(left, right);
  // shared_ptr<rev::TwoRotationInertialOdometry45Degrees> skid_odom = make_shared<rev::TwoRotationInertialOdometry45Degrees>(left_enc, right_enc, skid_imu, 2.46_in, 2.46_in);

  // shared_ptr<rev::Reckless> reckless = make_shared<rev::Reckless>(skid_chassis, skid_odom);

  // rev::AsyncRunner reckless_runner(reckless);
  // rev::AsyncRunner odom_runner(skid_odom);

  // odom->set_position({40.5_in, 0_in, 270_deg});

  // reckless->go(
  //   {
  //   &PilonsSegment(
  //     &ConstantMotion(0.75),
  //     &PilonsCorrection(2, 0.5_in),
  //     make_shared<SimpleStop>(0.06_s, 0.2_s, 0.25),
  //     {20_in, 0_in, 270_deg}, 0_in),
  //   &PilonsSegment(
  //     &ConstantMotion(0.75),
  //     &PilonsCorrection(2, 0.5_in),
  //     make_shared<SimpleStop>(0.06_s, 0.2_s, 0.25),
  //     {4_in, 20_in, 0_deg}, 0_in)
  //   }
  // );

  // reckless->await();
}

// void autonomous() {
//   rev::MotorGroup left({3, -5, -1, 2});
//   rev::MotorGroup right({17, -8, -9, 10});



//   // shared_ptr<rev::MecanumChassis> chassis = make_shared<rev::MecanumChassis>(front_left, front_right, back_left, back_right);
//   // shared_ptr<rev::SkidSteerChassis> skid_chassis = make_shared<rev::SkidSteerChassis>(left, right);

//   // chassis->drive_holonomic(0.5, 0.5, 0.5);

//   // shared_ptr<rev::Slipstream> slipstream = make_shared<rev::Slipstream>(chassis, odom);

//   // Set start position and consts
//   rev::QTime harsh = 0.06_s;
//   rev::QTime coast = 0.2_s;

//   // Turn constants
//   double harsh_turn = 0.085;
//   double coast_turn = 0.23;
//   rev::QTime brake_time = 0.1_s;

//   // Powers
//   double rush_power = 0.87;
//   double fast_power = 0.75;
//   double medium_power = 0.5;
//   double slow_power = 0.32;
//   double coast_power = 0.25;

//   // Motion controllers
//   shared_ptr<rev::MecanumConstantMotion> rush_motion = make_shared<rev::MecanumConstantMotion>(rush_power);
//   shared_ptr<rev::MecanumConstantMotion> fast_motion = make_shared<rev::MecanumConstantMotion>(fast_power);
//   shared_ptr<rev::MecanumConstantMotion> medium_motion = make_shared<rev::MecanumConstantMotion>(medium_power);
//   shared_ptr<rev::MecanumConstantMotion> slow_motion = make_shared<rev::MecanumConstantMotion>(slow_power);
//   shared_ptr<rev::CrossTrackCorrection> ct_correction = make_shared<rev::CrossTrackCorrection>(2, 1, 0.5_in, 3_deg);

//   cout << "Starting route" << endl;
//   uint8_t timeout = 0;

//   rev::Pose start_position = {0_in, 0_in, 0_deg};
//   odom->set_position(start_position);

//   // test movement
//   cout << "Starting test movement" << endl;
//   slipstream->go(
//       {
//         // Move fwd
//         &MecanumSegment(
//           rush_motion,
//           ct_correction,
//           make_shared<rev::SimpleHolonomicStop>(0_s, 0_s, coast_power),
//           {40.2_in, 40_in, 0_deg}, 0_in),
//       }
//   );


// }

// int signum(double x) {
//   if (x >= 0)
//     return 1;
//   else return -1;
// }

// void reset_odom() {
//   odom->reset_position();
// }

float reduce_0_to_360(float angle) {
    while(!(angle >= 0 && angle < 360)) {
        if(angle < 0) { angle += 360; }
        if(angle >= 360) { angle -= 360; }
    }
    return angle;
}

float deadband(float input, float width){
    if (std::abs(input) < width) { return 0; }
    return input;
}

void opcontrol() {
  pros::delay(1000);
  // controller.print(0, 0, "furk");

  bool scraper_state = false;
  while(true) {
    /*
     * TEST MECANUM DRIVE
    */
    double left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double left_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    double right_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

    double forward = deadband(left_y, 0.05);
    double strafe = deadband(left_x, 0.05);
    double turn = deadband(right_x, 0.05);
    // double angle = reduce_0_to_360(imu->get_heading());

    // float robotFwd =  forward * std::cos(angle) + strafe * std::sin(angle);
    // float robotStrafe = -forward * std::sin(angle) + strafe * std::cos(angle);


    // SlipstreamPower power;
    // power.front_left_forward = forward + strafe + turn;
    // power.front_right_forward = forward - strafe - turn;
    
    // power.rear_left_forward = forward - strafe + turn;
    // power.rear_right_forward = forward + strafe - turn;

    chassis->drive_holonomic(forward, turn, strafe);
    


    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lever_target = 1000;
      hood.set_value(1);
    } else {
      lever_target = 0;
      hood.set_value(0);
    }

    lever.move_voltage(12000 * lever_pid.update(lever_target, lever.get_positions()[0]));

    
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move_voltage(12000); 
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      intake.move_voltage(-12000); 
    } else {
      if (scraper_state) {
        intake.move_voltage(12000); 
      } else {
        intake.move(0);
      }
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      scraper_state = true;
      scraper.set_value(scraper_state);
    } else {
      scraper_state = false;
    }

    // chassis->drive_holonomic(forward, turn, strafe);

    // OdometryState current_state = odom->get_state();

    // pros::lcd::print(0, "X: %f", current_state.pos.x.convert(inch));
    // pros::lcd::print(1, "Y: %f", current_state.pos.y.convert(inch));
    // pros::lcd::print(2, "a: %f", current_state.pos.theta.convert(degree));
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
