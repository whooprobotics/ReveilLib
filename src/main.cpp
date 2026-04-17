#include "rev/rev.hh"
#include "robot_testing/rev2_config.hh"
#include "robot_testing/rev2_macros.hh"

using std::shared_ptr, std::make_shared, std::vector, std::string, std::cout, std::endl;
using namespace rev;


// Message me if this doesnt work, i can show you a video on it working on
// mikgen. Also, if this doesnt work, read the commit logs, its not the code's
// fault, its probably something else, and if you message me about it not
// working, i will probably just send you a video of it working and then you can
// figure out what you did wrong on your end.
// void test_mecanum() {
//   odom->set_position({0_in, 0_in, 0_deg});
//   turn(0_deg, Turn{.min_speed = 4, .timeout = 2000_ms});
//   drive(12_in, Drive{.center_max_speed = 0});
//   drive(-12_in);
//   drive(0_in, 24_in);
//   turn(0_deg, Turn{.min_speed = 4});
//   drive(24_in, 24_in, 90_deg);
//   turn(180_deg);
//   drive(0_in, 0_in);
//   turn(0_deg);
// }

void initialize() {
  // Initialize LCD so we can print to it for debugging
  pros::lcd::initialize();

  // Initialize color sensor and set integration time and led power
  color_sensor.set_integration_time(5);
  color_sensor.set_led_pwm(75);

  // Makes lever go to start no matter hat position it starts in, 
  // and also zeros the position so we can use move_absolute with it
  reset_lever();

  // These constants work, if the algo does not work
  // ITS NOT THE CONSTANTS FAULT READ THE COMMIT LOGS, NOT THE CONSTANTS FAULT
  // slipstream->set_constants({.drive_kp = 1.5,
  //                            .drive_ki = 0,
  //                            .drive_kd = 10,
  //                            .drive_starti = 0,

  //                            .drive_settle_error = 1,
  //                            .drive_settle_time = 100_ms,
  //                            .drive_large_settle_error = 3,
  //                            .drive_large_settle_time = 500_ms,
  //                            .drive_timeout = 5000_ms,

  //                            .drive_exit_error = 0_in,
  //                            .drive_min_speed = 0,
  //                            .drive_max_speed = 8,

  //                            .turn_kp = .4,
  //                            .turn_ki = 0,
  //                            .turn_kd = 3,
  //                            .turn_starti = 0,

  //                            .turn_settle_error = 1,
  //                            .turn_settle_time = 100_ms,
  //                            .turn_large_settle_error = 3,
  //                            .turn_large_settle_time = 500_ms,
  //                            .turn_timeout = 3000_ms,

  //                            .turn_exit_error = 0_deg,
  //                            .turn_min_speed = 0,
  //                            .turn_max_speed = 12,

  //                            .center_max_speed = 10});

  // Calibrate the imu
  imu->calibrate();

  pros::Task AntiJamTask(anti_jam);
  pros::Task Intake_Task(intake_task);
  pros::Task Color_Task(color_task);
}

void opcontrol() {

  while(true) {
    // Print out telemetry for debugging
    pros::lcd::print(2, "Theta: %f", imu->get_heading());
    pros::lcd::print(3, "X: %f", odom->get_state().pos.x.convert(inch));
    pros::lcd::print(4, "Y: %f", odom->get_state().pos.y.convert(inch));
    pros::lcd::print(5, "Sideways Encoder: %f", sideways_enc->get_position());
    pros::lcd::print(6, "Forward Encoder: %f", forward_enc->get_position());


    // Field-centric driving toggle
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      field_centric_enabled = !field_centric_enabled;
    }
    drive();

    // intake state control
    toggle_intake = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B); // if the B button is pressed, we want to reset the anti-jam system, this allows the driver to clear jams without having to stop and wait for the system to reset on its own
    eject(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
    outtake(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2));

    // use buttons to control lever scoring
    score_shallow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || (score_shallow && score); // if the B button is held, the robot will do a shallower score, which can be useful in certain situations, also if the score button is released but the lever is still moving, we want to keep the score_shallow variable true until the lever is done moving to prevent it from changing mid-score
    score         = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) || score_shallow;
    lever_code();

    // If the left button is pressed, run the mecanum test function
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {test_mecanum();}
    

    // If the right button is pressed, toggle the lift state and set the hood to the default position
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      lift_state = !lift_state;
      set_hood(false);
    }

    set_scraper(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y));
    set_lift(lift_state);
    set_descore(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN));

    // Delay so brain no exploded
    pros::delay(20);
  }
}
