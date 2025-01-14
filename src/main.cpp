#include "main.h"
#include "rev/rev.hh"
#include "rev/api/hardware/devices/optical/otos.hh"
#include "rev/api/async/async_runner.hh"
#include "rev/api/alg/odometry/optical_odometry.hh"

using namespace rev;
using std::shared_ptr, std::make_shared;

// void on_center_button() {}

void initialize() {
  pros::lcd::initialize();
}

// void disabled() {}

// void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  
  shared_ptr<rev::OTOS> otos = make_shared<rev::OTOS>();
  shared_ptr<OpticalOdometry> odom = make_shared<OpticalOdometry>(otos, 0_in, 0_in);
  shared_ptr<AsyncRunner> odom_runner = make_shared<AsyncRunner>(odom);

  while (true) {
    pros::lcd::print(0, "X pose: %f", odom->get_state().pos.x.convert(inch));
    pros::lcd::print(1, "Y pose: %f", odom->get_state().pos.y.convert(inch));
    pros::lcd::print(2, "H pose: %f", odom->get_state().pos.theta.convert(degree));
    pros::delay(20);
  }
}