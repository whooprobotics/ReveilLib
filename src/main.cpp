#include "rev/rev.hh"

using std::shared_ptr, std::make_shared, std::vector, std::string, std::cout, std::endl;
using namespace rev;
  
// Message me if this doesnt work, i can show you a video on it working on mikgen.
// Also, if this doesnt work, read the commit logs, its not the code's fault, its probably something else, and if you message me about it not working, i will probably just send you a video of it working and then you can figure out what you did wrong on your end.
void test_mecanum() {
  odom->set_position({0_in, 0_in, 0_deg});
  slipstream->go({
    &MecanumToPose({24_in, 0_in, 0_deg}),
    &MecanumToPose({24_in, 24_in, 90_deg}),
    &MecanumToPose({0_in, 24_in, 180_deg}),
    &MecanumToPose({0_in, 0_in, 270_deg}),
  });
  slipstream->await();
}

void initialize() {
  pros::lcd::initialize();
  color_sensor.set_integration_time(5);
  color_sensor.set_led_pwm(75);

  // Makes lever go to start
  while (lever.get_torque() < .25) {
    lever.move_voltage(-4000);
  }
  
  lever.move_voltage(0);
  lever.set_zero_position(0);


  // These constants work, if the algo does not work 
  // ITS NOT THE CONSTANTS FAULT READ THE COMMIT LOGS, NOT THE CONSTANTS FAULT
  slipstream->set_constants({
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
    .turn_max_speed = 12
  });

  imu->calibrate();
  
  
}

enum Color {
  RED = 0,
  BLUE = 1,
  NONE = 2
};

void field_centric() {
    double throttle = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0, 0.05);
    double strafe   = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0, 0.05);
    double turn     = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, 0.05);

    // const robotFwd =  throttle * Math.cos(to_rad(angle)) + strafe * Math.sin(to_rad(angle));
    // const robotStrafe = -throttle * Math.sin(to_rad(angle)) + strafe * Math.cos(to_rad(angle));

    double angle = imu->get_heading() * M_PI / 180.0;
    double robotFwd    =  throttle * std::cos(angle) + strafe * std::sin(angle);
    double robotStrafe =  -throttle * std::sin(angle) + strafe * std::cos(angle);

    SlipstreamPower power{
      .front_left_forward  = robotFwd + turn + robotStrafe,
      .front_right_forward = robotFwd - turn - robotStrafe,
      .rear_left_forward   = robotFwd + turn - robotStrafe,
      .rear_right_forward  = robotFwd - turn + robotStrafe
    };
  
    power.clamp_powers();

    chassis->drive_holonomic(robotFwd, turn, robotStrafe);
}

inline static bool is_sorting = false;
inline static bool is_stalled = false;
inline static bool front_intake_stalled = false;
inline static bool back_intake_stalled= false;

Color detect_color() {
  int hue = color_sensor.get_hue();

  if ((hue < 5 || hue > 350) && color_sensor.get_proximity() > 90) {
    return Color::RED;
  } else if ((hue > 200 && hue < 230) && color_sensor.get_proximity() > 90) {
    return Color::BLUE;
  } else {
    return Color::NONE;
  }
}

void color_sort(Color color, Color team_color) {
  if (color != team_color && color != Color::NONE) {
    is_sorting = true;
    back_intake.move_voltage(-12000);
    pros::delay(175);
    is_sorting = false;
  }
}

void color_task() {
  Color Team_Color = Color::RED;
  while (true) {
    Color detected_color = detect_color();
    color_sort(detected_color, Team_Color);
    // pros::lcd::print(5, Team_Color == Color::RED ? "RED" : "BLUE");
    // pros::lcd::print(6, detected_color == Color::RED ? "RED" : detected_color == Color::BLUE ? "BLUE" : "UNKNOWN");
    pros::delay(10);
  }
}
pros::Task Color_Task(color_task);

void reset_jam() {
  front_intake_stalled = false;
  back_intake_stalled = false;
  is_stalled = false;
}
void antijam() {
  int stall_timeout = 500; // ms
  int frontstall_time = 0;
  int backstall_time = 0;
  
  double front_intake_torque_threshold = .27 ; // Nm
  double back_intake_torque_threshold = .27; // Nm
  double i = 0;
  bool is_front_stalling = false;
  bool is_back_stalling = false;

  while (true) {
    // If the torque exceeds the threshold, start a timer, if the torque is still above the threshold after the timer runs out, we can assume the intake is stalled and stop the motors until the torque drops back down
    if (!is_front_stalling && intake.get_torque() > front_intake_torque_threshold) {
      frontstall_time = pros::millis();
      is_front_stalling = true;
    } 
    if (!is_back_stalling && back_intake.get_torque() > back_intake_torque_threshold) {
      backstall_time = pros::millis();
      is_back_stalling = true;
    } 
    
    // If the torque drops back down below 90% of the threshold, we can assume it was a false positive and reset the stall state
    if (is_front_stalling && intake.get_torque() < front_intake_torque_threshold*0.9) {
      is_front_stalling = false;
    }
    if (is_back_stalling && back_intake.get_torque() < back_intake_torque_threshold*0.9) {
      is_back_stalling = false;
    }

    // If the timer runs out and the torque is still above the threshold, we can assume the intake is stalled and stop the motors until the torque drops back down
    if (is_front_stalling && pros::millis() - frontstall_time > stall_timeout) {
      front_intake_stalled = true;
      controller.rumble("--");
      intake.move_voltage(0);
    }
    if (is_back_stalling && pros::millis() - backstall_time > stall_timeout){
      back_intake_stalled = true;
      controller.rumble("-");
      back_intake.move_voltage(0);
    }

    // graph the torque values for testing in terminal
    cout << "(" << i << ", " << intake.get_torque() << "), "; 
    cout << "(" << i << ", " << back_intake.get_torque() << ") " << endl; 
    i += 0.02;

    // If either intake is stalled, set the is_stalling flag to true, otherwise set it to false
    is_stalled = front_intake_stalled || back_intake_stalled;

    pros::delay(20);
  }
}
pros::Task AntiJamTask(antijam);

// Drive Code
void opcontrol() {
  bool scraper_state = false;
  bool descore_state = false;
  bool lift_state = false;
  bool intake_in_state = true;

  while(true) {
    pros::lcd::print(2, "Theta: %f", imu->get_heading());
    pros::lcd::print(3, "X: %f", odom->get_state().pos.x.convert(inch));
    pros::lcd::print(4, "Y: %f", odom->get_state().pos.y.convert(inch));
    pros::lcd::print(5, "Sideways Encoder: %f", sideways_enc->get_position());
    pros::lcd::print(6, "Forward Encoder: %f", forward_enc->get_position());

    double left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double left_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    double right_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

    // double forward = deadband(left_y, 0.05);
    // double strafe = deadband(left_x, 0.05);
    // double turn = deadband(right_x, 0.05);

    double lever_speed = 1;

    //chassis.drive_holonomic(forward, turn, strafe);
    field_centric();

    scraper.set_value(scraper_state);
    lift.set_value(lift_state);
    descore_piston.set_value(descore_state);

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      test_mecanum();
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      lift_state = !lift_state;
      hood.set_value(0);
    }

    // R1 & R2 controls, moves back and front intake
    if(!is_sorting) {
      if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) && !is_sorting) {
        if (!is_stalled) {
          intake_in_state = !intake_in_state;
        } else {
          reset_jam();
        }
      }
      if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        reset_jam();
        back_intake.move_voltage(-12000);
        intake.move_voltage(-12000);
      } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        reset_jam();
        back_intake.move_voltage(-12000);
        intake.move_voltage(0);
      } else if (intake_in_state){
        intake.move_voltage(12000 * !front_intake_stalled);
        back_intake.move_voltage(12000 * !back_intake_stalled);
      } else {
        back_intake.move(0);
        intake.move(0);
      }
    }
    
    
    // lever piston goes up immediately, lever follows 200ms later; lever piston goes down 500ms after release
    static uint32_t score_press_time = 0;
    static uint32_t lever_up_time = 0;
    static uint32_t lever_retract_time = 0;
    //static uint32_t up_release_time = 0;
    static bool lever_actuating = false;
    static bool lever_retracting = false;

    static int lever_end_position = 720;
    static int lever_rest_position = 220;
    static int lever_retract_score_position = 0;

    static int lever_score_velocity = 100; // percent
    static int lever_retract_velocity = 100; // percent

    static int lever_timeout = 800; // ms
    static int lever_score_pause = 100; // ms
    static int lever_retract_pause = 100; // ms
    static int hood_timeout = 1200; // ms

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) || lever_actuating || lever_retracting) {
      if (!lever_actuating && !lever_retracting) {
        score_press_time = pros::millis();
        reset_jam();
        intake_in_state = true;
        lever.move_absolute(lever_retract_score_position, lever_retract_velocity);
        hood.set_value(1);
        lever_actuating = true;
      }

      if (lever_actuating && pros::millis() - score_press_time >= lever_score_pause) {
        lever_piston.set_value(1);
        lever.move_voltage(lever_score_velocity * 120); // percent to millivolts
      }

      if (lever_actuating && (lever.get_position() >= lever_end_position || pros::millis() - score_press_time >= lever_timeout)) {
        lever_up_time = pros::millis();
        lever.move_voltage(0);
        lever_actuating = false;
        lever_retracting = true;
      }

      if (lever_retracting && pros::millis() - lever_up_time >= lever_retract_pause) {
        lever.move_absolute(lever_rest_position, lever_retract_velocity);
        lever_piston.set_value(0);
        lever_retract_time = pros::millis();
        lever_retracting = false;
      }
    } else {
      lever.move_absolute(lever_rest_position, lever_retract_velocity);
      lever_piston.set_value(0);
    }

    if(!lever_actuating && !lever_retracting && pros::millis() - lever_retract_time >= hood_timeout) {
      hood.set_value(0);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      scraper_state = true;
    } else {
      scraper_state = false;
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      descore_state = true;
    } else {
      descore_state = false;
    }

    pros::delay(20);
  }
}
