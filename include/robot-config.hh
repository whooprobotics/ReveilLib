#pragma once

extern rev::MotorGroup front_left;
extern rev::MotorGroup back_left;
extern rev::MotorGroup front_right;
extern rev::MotorGroup back_right;
extern rev::Motor center_left;
extern rev::Motor center_right;

extern rev::Motor intake;

extern pros::Motor lever1;
extern pros::Motor lever2;

extern pros::Motor_Group lever;
extern pros::ADIDigitalOut scraper;
extern pros::ADIDigitalOut lift;
extern pros::ADIDigitalOut hood;

extern rev::AsteriskChassis chassis;

extern pros::Controller controller;
extern rev::QuadEncoder left_encoder;
extern rev::QuadEncoder right_encoder;
extern pros::IMU imu;

rev::Slipstream slipstream;