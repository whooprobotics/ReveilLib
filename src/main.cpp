#include "rev/rev.hh"

using std::shared_ptr, std::make_shared, std::vector, std::string, std::cout, std::endl;
using namespace rev;

void drive(QLength distance, rev::Drive params = {}) {
  slipstream->go({rev::MecanumToDistance::create(distance, params)});
  slipstream->await();
}

void drive(QLength x, QLength y, rev::Drive params = {}) {
  slipstream->go({rev::MecanumToPoint::create({x, y}, params)});
  slipstream->await();
}

void drive(QLength x, QLength y, QAngle angle, rev::Drive params = {}) {
  slipstream->go({rev::MecanumToPose::create({x, y, angle}, params)});
  slipstream->await();
}

void turn(QAngle angle, rev::Turn params = {}) {
  slipstream->go({rev::MecanumTurnToAngle::create(angle, params)});
  slipstream->await();
}

void turn(QLength x, QLength y, rev::Turn params = {}) {
  slipstream->go({rev::MecanumTurnToPoint::create({x, y}, params)});
  slipstream->await();
}

// Message me if this doesnt work, i can show you a video on it working on
// mikgen. Also, if this doesnt work, read the commit logs, its not the code's
// fault, its probably something else, and if you message me about it not
// working, i will probably just send you a video of it working and then you can
// figure out what you did wrong on your end.
void test_mecanum() {
  odom->set_position({0_in, 0_in, 0_deg});
  turn(0_deg, Turn{.min_speed = 4, .timeout = 2000_ms});
  drive(12_in, Drive{.center_max_speed = 0});
  drive(-12_in);
  drive(0_in, 24_in);
  turn(0_deg, Turn{.min_speed = 4});
  drive(24_in, 24_in, 90_deg);
  turn(180_deg);
  drive(0_in, 0_in);
  turn(0_deg);

}

void reset_lever() {
  u_int32_t lever_reset_timeout = 500; // ms

  u_int32_t lever_reset_start_time = pros::millis();
  while (lever.get_torque() < 0.25 || pros::millis() - lever_reset_start_time > lever_reset_timeout) {
    lever.move_voltage(-4000);
  }
  lever.move_voltage(0);
  lever.set_zero_position(0);
}

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
  slipstream->set_constants({.drive_kp = 1.5,
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
                             .drive_max_speed = 8,

                             .turn_kp = .4,
                             .turn_ki = 0,
                             .turn_kd = 3,
                             .turn_starti = 0,

                             .turn_settle_error = 1,
                             .turn_settle_time = 100_ms,
                             .turn_large_settle_error = 3,
                             .turn_large_settle_time = 500_ms,
                             .turn_timeout = 3000_ms,

                             .turn_exit_error = 0_deg,
                             .turn_min_speed = 0,
                             .turn_max_speed = 12,

                             .center_max_speed = 10});

  // Calibrate the imu
  imu->calibrate();
}
// Color enum for color sorting
enum Color {
  RED = 0,
  BLUE = 1,
  NONE = 2
};

// Code for field centric control dont touch ts very nice

static bool field_centric_enabled = true;
void drive() {
    double throttle = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0, 0.05);
    double strafe   = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0, 0.05);
    double turn     = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, 0.05);

    if (!field_centric_enabled) {
      chassis->drive_holonomic(throttle, turn, strafe);
      return;
    }

    // field centric
    double angle = imu->get_heading() * M_PI / 180.0;
    double robotFwd    =  throttle * std::cos(angle) + strafe * std::sin(angle);
    double robotStrafe =  -throttle * std::sin(angle) + strafe * std::cos(angle);

    chassis->drive_holonomic(robotFwd, turn, robotStrafe);
}

// Variables for anti-jam system
static bool is_sorting = false;
static bool is_stalled = false;
static bool front_intake_stalled = false;
static bool back_intake_stalled = false;

// Anti-jam system, if the torque on either intake exceeds the threshold for a certain amount of time, 
// it cuts power to the intakes until the torque drops back down, indicating the jam has been cleared

void anti_jam() {

  int stall_timeout = 500; // ms
  int frontstall_time = 0;
  int backstall_time = 0;
  
  double front_intake_torque_threshold = .27 ; // Nm
  double back_intake_torque_threshold = .27; // Nm
  double i = 0;
  bool is_front_stalling = false;
  bool is_back_stalling = false;

  while (true) {
    // If the torque exceeds the threshold, start a timer, 
    // if the torque is still above the threshold after the timer runs out, 
    // we can assume the intake is stalled and stop the motors until the torque drops back down
    if (!is_front_stalling && front_intake.get_torque() > front_intake_torque_threshold) {
      frontstall_time = pros::millis();
      is_front_stalling = true;
    } 
    if (!is_back_stalling && back_intake.get_torque() > back_intake_torque_threshold) {
      backstall_time = pros::millis();
      is_back_stalling = true;
    } 
    
    // If the torque drops back down below 90% of the threshold, 
    // we can assume it was a false positive and reset the stall state
    if (is_front_stalling && front_intake.get_torque() < front_intake_torque_threshold*0.9) {
      is_front_stalling = false;
    }
    if (is_back_stalling && back_intake.get_torque() < back_intake_torque_threshold*0.9) {
      is_back_stalling = false;
    }

    // If the timer runs out and the torque is still above the threshold, 
    // we can assume the intake is stalled and stop the motors until the torque drops back down
    if (is_front_stalling && pros::millis() - frontstall_time > stall_timeout) {
      front_intake_stalled = true;
      controller.rumble("--");
      front_intake.move_voltage(0);
    }
    if (is_back_stalling && pros::millis() - backstall_time > stall_timeout){
      back_intake_stalled = true;
      controller.rumble("-");
      back_intake.move_voltage(0);
    }

    // Graph the torque values for testing in terminal
    // cout << "(" << i << ", " << intake.get_torque() << "), "; 
    // cout << "(" << i << ", " << back_intake.get_torque() << ") " << endl; 
    i += 0.02;

    // If either intake is stalled, set the is_stalling flag to true, otherwise set it to false
    is_stalled = front_intake_stalled || back_intake_stalled;

    pros::delay(20);
  }
}
void reset_jam() { // Resets the anti-jam system, setting all the stalled variables to false 
  front_intake_stalled = false;
  back_intake_stalled = false;
  is_stalled = false;
}
pros::Task AntiJamTask(anti_jam);

// Color detection and sorting code, returns the detected color 
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
// Sorts the color, if the detected color is not the same as the team color and is not NONE, 
// it runs the intake in reverse for a short amount of time to eject the wrong colored object
// Task for constantly checking the color sensor and sorting the objects, 
// runs in a separate thread so it can run concurrently with driver control
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

static bool reset = false;
static bool eject_state = false; // highest precednece, if this is true, the robot will reverse both intakes to clear all blocks out
static bool outtake_state = false; // if this is true, the robot will reverse the back intake and briefly reverse the front intake to score in low goal without jamming
static bool intake_in_state = false; // if this is true, the intakes will run forward to take in blocks, if false, the intakes will stop, 
void intake_control() {
  // Intake control, if the intake is not currently sorting, allow the driver to control the intake, 
  // otherwise ignore driver input to prevent interference with the sorting process
  static int outtake_reverse_timer = 0; // ms
  static bool outtaking = false; 
  if (reset) { // if we are currently resetting the jam, we want to ignore driver input until the jam is cleared to prevent interference with the reset process
    // Toggle the intake state, if the intake is currently stalled, reset the anti-jam system instead of toggling the intake
    if (!is_stalled) {
      intake_in_state = !intake_in_state;
    } else {
      reset_jam();
    }
    reset = false;
  }

  if(!is_sorting) {
    // If the left trigger is held, run both intakes in reverse to eject objects, 
    // if the right trigger is held, run just the back intake in reverse, but run the front intake in reverse for a short amount of time to help eject the blocks
    // otherwise run the intakes based on the intake state, if the intake is stalled, dont run it to prevent further stalling
    if (eject_state) {
      reset_jam();
      back_intake.move_voltage(-12000);
      front_intake.move_voltage(-12000);
    } else if (outtake_state) {
      reset_jam();
      back_intake.move_voltage(-12000);
      front_intake.move_voltage(-12000);
      if (!outtaking) {
        outtake_reverse_timer = pros::millis();
        outtaking = true;
      }
      if (outtaking && pros::millis() - outtake_reverse_timer < 100) {
        front_intake.move_voltage(-12000);
      } else {
        front_intake.move_voltage(0);
      }
    } else if (intake_in_state){
      front_intake.move_voltage(12000 * !front_intake_stalled);
      back_intake.move_voltage(12000 * !back_intake_stalled);
    } else {
      back_intake.move(0);
      front_intake.move(0);
    }
  }

  if (!outtake_state) {
    outtaking = false;
  }
}
void intake(bool state) {
  intake_in_state = state;
}
void outtake(bool state) {
  outtake_state = state;
}
void eject(bool state) {
  eject_state = state;
}
void stop_intake() {
  eject_state = false;
  outtake_state = false;
  intake_in_state = false;
}
void intake_task() {
  while (true) {
    intake_control();
    pros::delay(20);
  }
}
pros::Task Intake_Task(intake_task);

static bool score = false;
static bool score_shallow = false;
static bool lift_state = false;
void lever_code(){
  // lever piston goes up immediately, lever follows 200ms later; lever piston goes down 500ms after release
  static uint32_t score_press_time = 0;
  static uint32_t lever_up_time = 0;
  static uint32_t lever_retract_time = 0;
  //static uint32_t up_release_time = 0;
  static bool lever_actuating = false;
  static bool lever_retracting = false;

  static int lever_end_position = 750;
  static int lever_rest_position = 180;
  static int lever_retract_score_position = 50;

  static int lever_score_velocity = 100; // percent
  static int lever_retract_velocity = 100; // percent

  static int lever_score_timeout = 800; // ms
  static int lever_retract_timeout = 1000; // ms
  static int lever_score_pause = 200; // ms
  static int lever_retract_pause = 100; // ms
  static int hood_timeout = 1200; // ms

  score_shallow &= (score || lever_actuating || lever_retracting); // if score_shallow is true, we want to keep it true until the lever is done moving to prevent it from changing mid-score, also if the score button is released but the lever is still moving, we want to keep the score_shallow variable true until the lever is done moving to prevent it from changing mid-score
  
  // If the score button is pressed, start the scoring process, which involves moving the lever and actuating the piston in a certain sequence with specific timing to achieve the desired scoring motion, also set the hood to the scoring position
  if (score || lever_actuating || lever_retracting) {
    // If the lever is not currently moving, start the scoring process by setting the score press time, 
    // resetting the anti-jam system, setting the intake state to in, 
    //moving the lever to the retract position, and setting the hood to the scoring position
    if (!lever_actuating && !lever_retracting) {
      score_press_time = pros::millis();
      reset_jam();

      if (!score_shallow) {
        lever.move_voltage(-lever_score_velocity * 120); // percent to millivolts // if score_shallow is true, the lever will not move all the way down, allowing for a shallower score that can be useful in certain situations
      } else if (lift_state) {
        lever.move_relative(175, lever_score_velocity); // move lever up a little bit to help with shallower scoring, this is only used if score_shallow is true, which is when the B button is held, allowing for a shallower score
      }
      hood.set_value(1); // set hood to the scoring position
      lever_actuating = true;

      intake_in_state = false; // set intake to out to eject the object, will be set back to in after the lever retracts
    }

    // After the score pause time has gone by, move the lever to the scoring position
    if (lever_actuating && (pros::millis() - score_press_time >= lever_score_pause || lever.get_position() <= lever_retract_score_position)) {
      lever_piston.set_value(!(score_shallow && !lift_state)); // if score_shallow is true and barrel is down, the piston will not extend all the way up, allowing for a shallower score that can be useful in certain situations
      lever.move_voltage(lever_score_velocity * 120); // percent to millivolts
    }

    // After the lever timeout has gone by or the lever has reached the end position, stop the lever and start the retract process
    if (lever_actuating && (lever.get_position() >= lever_end_position || pros::millis() - score_press_time >= lever_score_timeout)) {
      lever_up_time = pros::millis();
      lever.move_voltage(0);
      lever_actuating = false;
      lever_retracting = true;
    }

    // After the retract pause time has gone by, move the lever back to the rest position and lower the piston
    if (lever_retracting && pros::millis() - lever_up_time >= lever_retract_pause) {
      lever.move_absolute(lever_rest_position, lever_retract_velocity);
      lever_piston.set_value(0);
      lever_retract_time = pros::millis();
      intake_in_state = false; // set intake back to in
    }

    if (lever_retracting && (lever.get_position() <= lever_rest_position + 10 /*|| pros::millis() - lever_retract_time >= lever_retract_timeout*/)) { // if the lever is close enough to the rest position, we can assume it has finished retracting and reset the retracting state just in case the timing was off
      lever_retracting = false;
      intake_in_state = true; // set intake back to in
      score = false; // reset score to prevent re-entering the scoring process without a new button press

    }
    
  } else { // If the R1 button is not pressed, reset the lever and piston to the default positions
    lever.move_absolute(lever_rest_position, lever_retract_velocity);
    lever_piston.set_value(0);
  }

  // After the hood timeout has gone by, lower the hood back down
  if(!lever_actuating && !lever_retracting && pros::millis() - lever_retract_time >= hood_timeout) {
    hood.set_value(0);
  }
}
void score_lever(bool score_s = false) {
  score = true;
  score_shallow = score_s;

  while (score) {
    lever_code();
    pros::delay(20);
  }
}

// Variables for driver control states
static bool scraper_state = false;
static bool descore_state = false;
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

    // Update the states of the various actuators
    scraper.set_value(scraper_state);
    lift.set_value(lift_state);
    descore_piston.set_value(descore_state);

    // intake state control
    reset = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B); // if the B button is held, we want to reset the anti-jam system, this allows the driver to clear jams without having to stop and wait for the system to reset on its own, which can save valuable time during a match
    eject_state = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    outtake_state = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    // use buttons to control lever scoring
    score_shallow = (score_shallow && score) || controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1); // if the B button is held, the robot will do a shallower score, which can be useful in certain situations, also if the score button is released but the lever is still moving, we want to keep the score_shallow variable true until the lever is done moving to prevent it from changing mid-score
    score         = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) || score_shallow;
    lever_code();

    // If the left button is pressed, run the mecanum test function
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      test_mecanum();
    }

    // If the right button is pressed, toggle the lift state and set the hood to the default position
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      lift_state = !lift_state;
      hood.set_value(0);
    }

    // If the down button is pressed, set the scraper state to true, otherwise set it to false
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      scraper_state = true;
    } else {
      scraper_state = false;
    }

    // If the Y button is pressed, set the descore piston state to true, otherwise set it to false
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      descore_state = true;
    } else {
      descore_state = false;
    }
    
    // Delay so brain no exploded
    pros::delay(20);
  }
}
