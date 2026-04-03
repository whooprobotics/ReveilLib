#include "rev/rev.hh"

using std::shared_ptr, std::make_shared, std::vector, std::string, std::cout, std::endl;
using namespace rev;

// Devices have moved into robot-config

double odom_wheel_diameter = 2.75; // in inches

PID lever_pid(.01, 0.0, 0, 0);
double lever_target = 0;

static void set_coordinates(double x, double y, double angle) {
  static auto get_left_enc_inches = []() {
    return left_encoder.get_position() * (odom_wheel_diameter * M_PI / 360.0); 
  };
  
  static auto get_right_enc_inches = []() {
    return right_encoder.get_position() * (odom_wheel_diameter * M_PI / 360.0); 
  };

  odom45.set_position({x, y}, angle, get_right_enc_inches(), get_left_enc_inches());

  static pros::Task odom_task([](){
    while (true) {
      odom45.update_position(get_right_enc_inches(), get_left_enc_inches(), imu.get_heading());
      pros::delay(10);
    }
  });
}

void initialize() {
  pros::lcd::initialize();
  odom45.set_physical_distances(3.25, 5.75); // in inches, horizontal distance from cog, vertical distance from cog
}

void test_mecanum() {
  set_coordinates(0, 0, 0);
  mecanum_to_pose(24, 0, 90);
}

// Drive Code
void opcontrol() {
  pros::delay(1000);

  bool scraper_state = false;
  while(true) {
    double left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double left_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    double right_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

    double forward = deadband(left_y, 0.05);
    double strafe = deadband(left_x, 0.05);
    double turn = deadband(right_x, 0.05);

    double lever_speed = 1;

    chassis.drive_holonomic(forward, turn, strafe);
    
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lever_target = 1000;
      lift.set_value(1);
      hood.set_value(0);
    } else {
      lever_target = 0;
      lift.set_value(0);
      hood.set_value(1);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lever_target = 1000;
      hood.set_value(0);
      lever_speed = .28;
    } else {
      lever_speed = 1;
    }

    lever.move_voltage((12000 * lever_pid.compute(lever_target - lever.get_positions()[0])) * lever_speed );

    
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
    } else {
      scraper_state = false;
    }
    scraper.set_value(scraper_state);

    pros::delay(20);
  }
}
