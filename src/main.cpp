#include "rev/rev.hh"

using std::shared_ptr, std::make_shared, std::vector, std::string, std::cout, std::endl;
using namespace rev;

// Devices have moved into robot-config

double odom_wheel_diameter = 2.41; // in inches

PID lever_pid(.1, 0.0, 0, 0);
double lever_target = 0;

void initialize() {
  pros::lcd::initialize();

  imu.reset(true);

  slipstream.set_constants({
    .drive_kp = 1.5,
    .drive_ki = 0,
    .drive_kd = 10,
    .drive_starti = 0,
  
    .drive_settle_error = 1,
    .drive_settle_time = 100_ms,
    .drive_large_settle_error = 3,
    .drive_large_settle_time = 500_ms,
    .drive_timeout = 5000_ms,
  
    .drive_exit_error = 0_in,
    .drive_min_speed = 0,
    .drive_max_speed = 12,
  
    .turn_kp = .4,
    .turn_ki = 0.03,
    .turn_kd = 3,
    .turn_starti = 15,
  
    .turn_settle_error = 1,
    .turn_settle_time = 100_ms,
    .turn_large_settle_error = 3,
    .turn_large_settle_time = 500_ms,
    .turn_timeout = 3000_ms,
  
    .turn_exit_error = 0_deg,
    .turn_min_speed = 0,
    .turn_max_speed = 12,
  });
}

// Go crazy dawg

void test_mecanum() {
  slipstream.go({
    &MecanumToPose({24_in, 0_in, 0_deg}),
    &MecanumToPose({24_in, 24_in, 90_deg}),
    &MecanumToPose({0_in, 24_in, 180_deg}),
    &MecanumToPose({0_in, 0_in, 270_deg}),
  });
}

// Drive Code
void opcontrol() {
  bool scraper_state = false;
  bool lift_state = false;
  while(true) {
    pros::lcd::print(2, "Theta: %f", imu.get_heading());
    pros::lcd::print(3, "Right Encoder: %f", right_encoder.get_position());
    pros::lcd::print(4, "Left Encoder: %f", left_encoder.get_position());


    double left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double left_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    double right_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

    double forward = deadband(left_y, 0.05);
    double strafe = deadband(left_x, 0.05);
    double turn = deadband(right_x, 0.05);

    double lever_speed = 1;

    chassis.drive_holonomic(forward, turn, strafe);
    
    scraper.set_value(scraper_state);
    lift.set_value(lift_state);

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      test_mecanum();
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      lift_state = !lift_state;
    }

    // hood goes up immediately, lever follows 250ms later; hood goes down 500ms after release
    static uint32_t r1_press_time = 0;
    static uint32_t r1_release_time = 0;
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      if (r1_press_time == 0) r1_press_time = pros::millis();
      if (pros::millis() - r1_press_time < 2000) intake.move_voltage(12000);
      else intake.move(0);
      r1_release_time = 0;
      hood.set_value(0);
      if (pros::millis() - r1_press_time >= 250) lever_target = 1000;
    } else {
      r1_press_time = 0;
      lever_target = 0;
      if (r1_release_time == 0) r1_release_time = pros::millis();
      if (pros::millis() - r1_release_time >= 1000) hood.set_value(1);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lever_target = 1000;
      hood.set_value(0);
      lever_pid.kp = .008;
      // lever_speed = .28;
    } else {
      lever_pid.kp = .1;
    }

    lever.move_voltage((1000 * lever_pid.compute(lever_target - lever.get_positions()[0])) );

    // lift state needs to be fixed so that it goes down when B is release

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move_voltage(12000); 
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      lift_state = true;
      intake.move_voltage(-12000); 
    } else {
      if (scraper_state) {
        intake.move_voltage(12000); 
      } else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.move(0);
      }
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      scraper_state = true;
      lift_state = true;
    } else {
      scraper_state = false;
    }

    pros::delay(20);
  }
}
