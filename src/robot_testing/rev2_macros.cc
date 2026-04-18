#include "robot_testing/rev2_config.hh"
#include "robot_testing/rev2_macros.hh"

using rev::QLength, rev::QAngle;

void driveTo(QLength distance, rev::Drive params) {
  slipstream->go({rev::MecanumToDistance::create(distance, params)});
  slipstream->await();
}

void driveTo(QLength x, QLength y, rev::Drive params) {
  slipstream->go({rev::MecanumToPoint::create({x, y}, params)});
  slipstream->await();
}

void driveTo(QLength x, QLength y, QAngle angle, rev::Drive params) {
  slipstream->go({rev::MecanumToPose::create({x, y, angle}, params)});
  slipstream->await();
}

void turnTo(QAngle angle, rev::Turn params) {
  slipstream->go({rev::MecanumTurnToAngle::create(angle, params)});
  slipstream->await();
}

void turnTo(QLength x, QLength y, rev::Turn params) {
  slipstream->go({rev::MecanumTurnToPoint::create({x, y}, params)});
  slipstream->await();
}

// Code for field centric control dont touch ts very nice -- I touched btw :)
bool field_centric_enabled = true;
void drive() {
    double throttle = rev::deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0, 0.05);
    double strafe   = rev::deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0, 0.05);
    double turn     = rev::deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, 0.05);

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

// anti-jam state variables
bool is_sorting = false;
bool is_stalled = false;
bool front_intake_stalled = false;
bool back_intake_stalled = false;

// intake state variables
bool toggle_intake = false;
bool eject_state = false; // highest precednece, if this is true, the robot will reverse both intakes to clear all blocks out
bool outtake_state = false; // if this is true, the robot will reverse the back intake and briefly reverse the front intake to score in low goal without jamming
bool intake_in_state = false; // if this is true, the intakes will run forward to take in blocks, if false, the intakes will stop,

// scoring state variables
bool score = false;
bool score_shallow = false;
bool lever_score_fail = false;
bool lever_retract_fail = false;
bool lift_state = false;

// Piston States
void set_scraper(bool state) {
  scraper.set_value(state);
}
void set_descore(bool state) {
  descore_piston.set_value(state);
}
void set_lift(bool state) {
  lift.set_value(state);
}
void set_hood(bool state) {
  hood.set_value(state);
}

// Anti-jam system, if the torque on either intake exceeds the threshold for a certain amount of time, 
// it cuts power to the intakes until the torque drops back down, indicating the jam has been cleared
void anti_jam() {
  static int stall_timeout = 500; // ms
  static int frontstall_time = 0;
  static int backstall_time = 0;
  
  static double front_intake_torque_threshold = .27 ; // Nm
  static double back_intake_torque_threshold = .27; // Nm
  static double i = 0;
  static bool is_front_stalling = false;
  static bool is_back_stalling = false;

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


// Color detection and sorting code, returns the detected color 
// Sorts the color, if the detected color is not the same as the team color and is not NONE, 
// it runs the intake in reverse for a short amount of time to eject the wrong colored object
// Task for constantly checking the color sensor and sorting the objects, 
// runs in a separate thread so it can run concurrently with driver control
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



// function to enable and disable solor sorting
static bool kill_color_sorting = false; 
void kill_sorting(bool state) {
  kill_color_sorting = state;
}

bool is_sorting_on() {
  return kill_color_sorting;
}

static Color TEAM = Color::RED; // default team color is red, can be set w this function
void set_team(Color team_color) {
  TEAM = team_color;
}

Color get_team() {
  return TEAM;
}

void color_sort(Color color, Color team_color) {
  static uint32_t sort_start_time = 0;
  static uint32_t sort_timeout = 175;

  static uint32_t sort_total_start_time = 0;
  static uint32_t sort_total_timeout = 400;

  if (color != team_color && color != Color::NONE) {
    if (!is_sorting) sort_total_start_time = pros::millis();
    is_sorting = true;
    back_intake.move_voltage(-12000);
    front_intake.move_voltage(6000);
    sort_start_time = pros::millis();
  }
  if (is_sorting && pros::millis() - sort_start_time > sort_timeout && pros::millis() - sort_total_start_time > sort_total_timeout){
    is_sorting = false;
  }
}


void color_task() {
  Color Team_Color = TEAM;
  while (true) {
    Color detected_color = detect_color();
    color_sort(detected_color, Team_Color);
    pros::delay(10);
  }
}

void intake_control() {
  // Intake control, if the intake is not currently sorting, allow the driver to control the intake, 
  // otherwise ignore driver input to prevent interference with the sorting process
  static int outtake_reverse_timer = 0; // ms
  static bool outtaking = false; 
  if (toggle_intake) { 
    // Toggle the intake state, if the intake is currently stalled, reset the anti-jam system instead of toggling the intake
    if (!is_stalled) {
      intake_in_state = !intake_in_state;
    } else {
      reset_jam();
    }
    toggle_intake = false;
  }

  if(!is_sorting) {
    // If the left trigger is held, run both intakes in reverse to eject objects, 
    // if the right trigger is held, run just the back intake in reverse, but run the front intake in reverse for a short amount of time to help eject the blocks
    // otherwise run the intakes based on the intake state, if the intake is stalled, dont run it to prevent further stalling
    if (eject_state) {
      reset_jam(); // eject overrides jamming
      back_intake.move_voltage(-12000);
      front_intake.move_voltage(-12000);
    } else if (outtake_state) {
      reset_jam(); // outtake overrides jamming
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
      // does not override jamming
      front_intake.move_voltage(12000 * !front_intake_stalled); // turn on if not stalled
      back_intake.move_voltage(12000 * !back_intake_stalled);   // turn on if not stalled
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
  reset_jam();
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


//---------------------------------------------lever control code---------------------------------------------

// Makes lever go to start no matter hat position it starts in, 
// and also zeros the position so we can use move_absolute with it
void reset_lever() {
  u_int32_t lever_reset_timeout = 1000; // ms

  u_int32_t lever_reset_start_time = pros::millis();
  while (true) {
    if (pros::millis() - lever_reset_start_time > lever_reset_timeout) break;
    if (lever.get_torque() > 0.25) break;

    lever.move_voltage(-4000);
  }
  lever.move_voltage(0);
  lever.set_zero_position(0);
  lever_control();
}

bool lever_actuating = false;
bool lever_retracting = false;
uint32_t score_press_time = 0;
void lever_control() {
  // lever piston goes up immediately, lever follows 200ms later; lever piston goes down 500ms after release
  //static uint32_t score_press_time = 0;
  static uint32_t lever_up_time = 0;
  static uint32_t lever_retract_time = 0;

  static bool lever_paused = false;
  static bool lever_setting_up = false;

  static uint32_t lever_end_position = 750;
  static uint32_t lever_rest_position = 180;
  static uint32_t lever_retract_score_position = 50;

  static uint32_t lever_score_velocity = 100; // percent
  static uint32_t lever_retract_velocity = 100; // percent

  static uint32_t lever_score_timeout = 800; // ms
  static uint32_t lever_retract_timeout = 1000; // ms
  static uint32_t lever_score_pause = 200; // ms
  static uint32_t lever_retract_pause = 100; // ms
  static uint32_t hood_timeout = 1200; // ms

  // if score_shallow is true, we want to keep it true until the lever is done moving to prevent it from changing mid-score, 
  // also if the score button is released but the lever is still moving, 
  // we want to keep the score_shallow variable true until the lever is done moving to prevent it from changing mid-score
  score_shallow &= (score || lever_actuating || lever_retracting || lever_paused || lever_setting_up); 
  
  // If the score button is pressed, start the scoring process, which involves moving the lever and actuating the piston in a certain sequence with specific timing to achieve the desired scoring motion, also set the hood to the scoring position
  if (score || lever_actuating || lever_retracting || lever_paused || lever_setting_up) {
    // If the lever is not currently moving, start the scoring process by setting the score press time, 
    // resetting the anti-jam system, setting the intake state to in, 
    // moving the lever to the retract position, and setting the hood to the scoring position
    if (!lever_actuating && !lever_retracting && !lever_paused && !lever_setting_up) {
      score_press_time = pros::millis();

      if (!score_shallow) {
        lever.move_voltage(-lever_score_velocity * 120); // percent to millivolts // if score_shallow is true, the lever will not move all the way down, allowing for a shallower score that can be useful in certain situations
      } else if (lift_state) {
        lever.move_relative(175, lever_score_velocity); // move lever up a little bit to help with shallower scoring, this is only used if score_shallow is true, which is when the B button is held, allowing for a shallower score
      }
      lever_setting_up = true;
      lever_score_fail = false;
      lever_retract_fail = false;

      set_hood(true); // set hood to the scoring position
      intake(false); // set intake to stop, will be set back to in after the lever retracts
    }

    // After the score pause time has gone by, move the lever to the scoring position
    if (lever_setting_up && (pros::millis() - score_press_time >= lever_score_pause || lever.get_position() <= lever_retract_score_position)) {
      lever_piston.set_value(!(score_shallow && !lift_state)); // if score_shallow is true and barrel is down, the piston will not extend all the way up, allowing for a shallower score that can be useful in certain situations
      lever.move_voltage(lever_score_velocity * 120); // percent to millivolts
      lever_actuating = true;
      lever_setting_up = false;
    }

    // After the lever timeout has gone by or the lever has reached the end position, stop the lever and start the retract process
    if (lever_actuating && (lever.get_position() >= lever_end_position || pros::millis() - score_press_time >= lever_score_timeout)) {
      lever_up_time = pros::millis();
      lever.move_voltage(0);
      lever_actuating = false;
      lever_paused = true;
      
      // lever timed out and failed to score
      lever_score_fail = lever.get_position() < lever_end_position;                     
    }

    // After the retract pause time has gone by, move the lever back to the rest position and lower the piston
    if (lever_paused && pros::millis() - lever_up_time >= lever_retract_pause) {
      lever_retracting = true;
      lever_paused = false;
      lever.move_absolute(lever_rest_position, lever_retract_velocity);
      lever_piston.set_value(0);
      lever_retract_time = pros::millis();
      //intake(false); // turn intake off
    }

    if (lever_retracting && (lever.get_position() <= lever_rest_position + 10 || pros::millis() - lever_retract_time >= lever_retract_timeout)) { // if the lever is close enough to the rest position, we can assume it has finished retracting and reset the retracting state just in case the timing was off
      lever_retracting = false;
      score = false; // reset score to prevent re-entering the scoring process without a new button press
      intake(true); // set intake back to in

      // lever timed out and failed to retract
      lever_retract_fail = lever.get_position() > lever_rest_position + 10;                     
    }
    
  } else { // If the R1 button is not pressed, reset the lever and piston to the default positions
    if (!lever_score_fail && !lever_retract_fail) lever.move_absolute(lever_rest_position, lever_retract_velocity);
    //lever.move_voltage(0);
    lever_piston.set_value(0);
    recover_lever();
  }

  // After the hood timeout has gone by, lower the hood back down
  if(!lever_actuating && !lever_retracting && !lever_paused && !lever_setting_up && pros::millis() - lever_retract_time >= hood_timeout) {
    set_hood(false);
  }
}

bool score_lever(bool score_shall_O, bool retry) {
  score = true;
  score_shallow = score_shall_O;

  bool failure = false;
  while (score) {
    lever_control();
    failure |= lever_score_fail || lever_retract_fail;

    if (retry && !score && failure) recover_lever();
    pros::delay(20);
  }
  return !failure;
}

// outtakes and retries to retract if lever_retract_fail
// rescores if lever_score_fail
// currently untested
void recover_lever() {
  static int retry_count = 0;
  static bool recovering = false;
  if (lever_retract_fail && retry_count < 5) {
    std::cout << "Retract Fail detected. count = " << retry_count << std::endl;

    score = true;
    
    lever_actuating = true;
    lever.move_voltage(12000);
    score_press_time = pros::millis();

    set_hood(true);
    outtake(true);
    intake(false);

    lift_state = true;
    lever_retract_fail = false;
    lever_score_fail = false;

    if (!recovering) {retry_count = 0; recovering = true;}
    retry_count++;
    lever_control();

  } else if (lever_score_fail && retry_count < 2) {
    std::cout << "Score Fail detected. count = " << retry_count << std::endl;

    score = true;
    score_shallow = true;
    lever_score_fail = false;
    lever_retract_fail = false;
    retry_count++;
    lever_control();

  } else if (!lever_score_fail && !lever_retract_fail) {
    retry_count = 0;
  } else {
    recovering = false;
  }
}
