#include "rev/rev.hh"
#include "robot-config.hh"

Odom45 odom45; // Odom using +y coordinate scheme
Constants constants; // Global constants

rev::MotorGroup front_left({11, -12});
rev::MotorGroup back_left({-14, 15});
rev::MotorGroup front_right({-10, 19});
rev::MotorGroup back_right({17, -16});
rev::Motor center_left(13);
rev::Motor center_right(-8);

rev::Motor intake(2);

pros::Motor lever1(-1, pros::E_MOTOR_GEAR_RED);
pros::Motor lever2(9, pros::E_MOTOR_GEAR_RED);

pros::Motor_Group lever({lever1, lever2});
pros::ADIDigitalOut scraper('A');
pros::ADIDigitalOut lift('C');
pros::ADIDigitalOut hood('B');

rev::AsteriskChassis chassis(front_left, front_right, back_left, back_right, center_left, center_right);

pros::Controller controller(pros::E_CONTROLLER_MASTER);
rev::QuadEncoder left_encoder('H', 'H', false);
rev::QuadEncoder right_encoder('I', 'I', false);
pros::IMU imu(20);