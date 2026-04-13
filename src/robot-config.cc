#include "rev/rev.hh"
#include "robot-config.hh"

namespace rev { Constants constants; } // Global constants

rev::MotorGroup front_left({11, 12});
rev::MotorGroup back_left({19, 20});
rev::MotorGroup front_right({-14, -15});
rev::MotorGroup back_right({-17, -18});
rev::Motor center_left(13);
rev::Motor center_right(-16);

pros::Optical color_sensor(8);

pros::Motor intake(-2, pros::E_MOTOR_GEAR_BLUE);
pros::Motor back_intake(-10, pros::E_MOTOR_GEAR_BLUE);

//pros::Motor lever1(-1, pros::E_MOTOR_GEAR_RED);
pros::Motor lever(-9, pros::E_MOTOR_GEAR_GREEN); //controlled by up & down

//pros::Motor_Group lever(lever1, lever2);
pros::ADIDigitalOut scraper({7, 'A'});
pros::ADIDigitalOut lift('C');
pros::ADIDigitalOut hood('B');
pros::ADIDigitalOut lever_piston('A');
pros::ADIDigitalOut descore_piston('D');

rev::AsteriskChassis chassis(front_left, front_right, back_left, back_right, center_left, center_right);

pros::Controller controller(pros::E_CONTROLLER_MASTER);
rev::QuadEncoder left_encoder('H', 'G', false);
rev::QuadEncoder right_encoder('E', 'F', false);
pros::IMU imu(1);

// rev::Slipstream slipstream;