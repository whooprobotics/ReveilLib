#include "main.h"
#include "rev/rev.hh"

using namespace rev;

// Setting up tank drive chassis
Motor_Group left_drive({1, 2, -3});
Motor_Group right_drive({4, 5, 6});

auto chassis = m_s<SkidSteerChassis>(left_drive, right_drive);

/*
  Setting Up Odometry
    - When moving forward robot position X should increase
    - When moving right robot position Y should increase
*/  

auto imu = m_s<Imu>(7);

auto left_encoder = m_s<QuadEncoder>('A', 'B', false);
auto right_encoder = m_s<QuadEncoder>('C', 'D', true);

auto odom = m_s<TwoRotationInertialOdometry45Degrees>(
  left_encoder, right_encoder, imu, 
  2.46_in, 2.46_in // Wheel Diameters
);


// Controller Setup
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Setting up ReveilLib
auto reckless = m_s<Reckless>(chassis, odom);
AsyncRunner odom_runner(odom);
AsyncRunner reckless_runner(reckless);

void initialize() {
  pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

/* 
  Sample autonomous 
  Robot will:
    - Drive forward 24 inches while correcting with pilons
    - Turn 90 degrees right
    - Drive forward another 24 inches while correcting with pilons
    - Drive back to origin while correcting with pilons
    - Look at a point 5 inches in front of the robot
*/
void autonomous() {
 odom->set_position({0_in, 0_in, 0_deg});
 reckless->go({
    &PilonsSegment(
      &ConstantMotion(1),
      &PilonsCorrection(2, 0.5_in),
      &SimpleStop(60_ms, 200_ms, 0.25),
      {0_in, 24_in}, 0_in
    ),
    &TurnSegment(
      0.75, 0.25,
      90_deg,
      0.06, 0.2, 100_ms
    ),
    &PilonsSegment(
      &ConstantMotion(1),
      &PilonsCorrection(2, 0.5_in),
      &SimpleStop(60_ms, 200_ms, 0.25),
      {24_in, 24_in}, 0_in
    ),
    &PilonsSegment(
      &ConstantMotion(1),
      &PilonsCorrection(2, 0.5_in),
      &SimpleStop(60_ms, 200_ms, 0.25),
      {0_in, 0_in}, 0_in
    ),
    &LookAt(
      0.75, 0.25,
      {5_in, 0_in}, 0_deg,
      0.06, 0.2, 100_ms
    )
 });
 reckless->await();
}

void opcontrol() {
  odom->set_position({0_in, 0_in, 0_deg});

  while (true) {
    double left_power = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double right_power = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0;

    chassis->drive_arcade(left_power, right_power);
    
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      autonomous();
    }

    // Print odometry output
    pros::lcd::print(0, "X: %f", odom->get_state().pos.x.convert(inch));
    pros::lcd::print(1, "Y: %f", odom->get_state().pos.y.convert(inch));
    pros::lcd::print(2, "Theta: %f", odom->get_state().pos.theta.convert(degree));
    pros::lcd::print(3, "Right Encoder: %f", right_encoder->get_position());
    pros::lcd::print(4, "Left Encoder: %f", left_encoder->get_position());

    pros::delay(20);
  }
}