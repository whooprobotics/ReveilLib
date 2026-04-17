#include "rev/rev.hh"
#include "robot_testing/rev2_config.hh"

rev::MotorGroup front_left({11, 12});
rev::MotorGroup back_left({19, 20});
rev::MotorGroup front_right({-14, -15});
rev::MotorGroup back_right({-17, -18});
rev::Motor center_left(13);
rev::Motor center_right(-16);

pros::Optical color_sensor(8);

pros::Motor front_intake(-2, pros::E_MOTOR_GEAR_BLUE);
pros::Motor back_intake(-10, pros::E_MOTOR_GEAR_BLUE);

//pros::Motor lever1(-1, pros::E_MOTOR_GEAR_RED);
pros::Motor lever(-9, pros::E_MOTOR_GEAR_GREEN); //controlled by up & down

//pros::Motor_Group lever(lever1, lever2);
pros::ADIDigitalOut scraper({7, 'A'});
pros::ADIDigitalOut lift('C');
pros::ADIDigitalOut hood('B');
pros::ADIDigitalOut lever_piston('A');
pros::ADIDigitalOut descore_piston('D');

std::shared_ptr<rev::AsteriskChassis> chassis = std::make_shared<rev::AsteriskChassis>(front_left, front_right, back_left, back_right, center_left, center_right);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

std::shared_ptr<rev::QuadEncoder> forward_enc = std::make_shared<rev::QuadEncoder>('E', 'F', true); // Rev2
std::shared_ptr<rev::QuadEncoder> sideways_enc = std::make_shared<rev::QuadEncoder>('G', 'H', true); // Rev2

// std::shared_ptr<rev::QuadEncoder> forward_enc = std::make_shared<rev::QuadEncoder>('H', 'G', false); // Megatron
// std::shared_ptr<rev::QuadEncoder> sideways_enc = std::make_shared<rev::QuadEncoder>('E', 'F', false); // Megatron

// std::shared_ptr<rev::Imu> imu = std::make_shared<rev::Imu>(7); // Megatron
std::shared_ptr<rev::Imu> imu = std::make_shared<rev::Imu>(1); // Rev2

QLength odom_wheel_size = 2_in; // Rev2
// QLength odom_wheel_size = 2.41_in; // Megatron
 
// std::shared_ptr<rev::TwoRotationInertialOdometry45Degrees> odom = std::make_shared<rev::TwoRotationInertialOdometry45Degrees>(
//     sideways_enc, forward_enc, imu,
//     odom_wheel_size, odom_wheel_size,
//     3_in, 0_in
// );
std::shared_ptr<rev::TwoRotationInertialOdometry> odom = std::make_shared<rev::TwoRotationInertialOdometry>(
    forward_enc, sideways_enc, imu,
    odom_wheel_size, odom_wheel_size,
    2.75_in, 1.9_in
);

std::shared_ptr<rev::Slipstream> slipstream = std::make_shared<rev::Slipstream>(chassis, odom);

rev::AsyncRunner odom_runner(odom);
rev::AsyncRunner slipstream_runner(slipstream);
