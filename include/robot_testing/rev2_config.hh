#pragma once

#include "rev/rev.hh"

#ifdef PLATFORM_BRAIN

extern rev::MotorGroup front_left;
extern rev::MotorGroup back_left;
extern rev::MotorGroup front_right;
extern rev::MotorGroup back_right;
extern rev::Motor center_left;
extern rev::Motor center_right;

extern pros::Optical color_sensor;

extern pros::Motor front_intake;
extern pros::Motor back_intake;

extern pros::Motor lever;
extern pros::ADIDigitalOut scraper;
extern pros::ADIDigitalOut lift;
extern pros::ADIDigitalOut hood;
extern pros::ADIDigitalOut lever_piston;
extern pros::ADIDigitalOut descore_piston;

extern pros::Controller controller;
extern std::shared_ptr<rev::QuadEncoder> forward_enc;
extern std::shared_ptr<rev::QuadEncoder> sideways_enc;
extern std::shared_ptr<rev::Imu> imu;

extern std::shared_ptr<rev::AsteriskChassis> chassis;

extern std::shared_ptr<rev::TwoRotationInertialOdometry> odom; // Rev2
// extern std::shared_ptr<TwoRotationInertialOdometry45Degrees> odom; // Megatron

extern rev::QLength odom_wheel_size;

void init();

extern std::shared_ptr<rev::Slipstream> slipstream;

extern rev::AsyncRunner odom_runner;
extern rev::AsyncRunner slipstream_runner;

#endif