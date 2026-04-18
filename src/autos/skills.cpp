#include "rev/rev.hh"
#include "autos/autos.h"
#include "robot_testing/rev2_config.hh"
#include "robot_testing/rev2_macros.hh"

using namespace rev;

void onslaught_skills_1() {
    odom->set_position({-46.75_in, -13.5_in, 90_deg});
// put down scraper and strafe the parking zone blocks
    set_scraper(true);
    pros::delay(500);
    driveTo(-46.75_in, 7_in, 90_deg);
// put up scraper and strafe to the matchloader
    set_scraper(false);
    driveTo(-46.75_in, -47_in, 90_deg);
// go backward and intake the 6 blocks
    //set_scraper(true);
    kill_sorting(true); // disable color sorting to intake all blocks
    pros::delay(500);
    intake(true);
    driveTo(-59.28_in, -47_in, 90_deg);
    pros::delay(3000);

// strafe to the side of the long goal
    driveTo(-32_in, -60_in, 90_deg);
  
// drive forward to the far side of the long goal
    driveTo(35_in, -60_in, 90_deg);
// strafe and turn to align to the long goal
    driveTo(35_in, -48_in, 270_deg);
    set_lift(true);
  
// drive forward and score the 6 blocks
    driveTo(30_in, -48_in, 270_deg);
    score_lever();
    score_lever();

// put down scraper and intake 6 blocks
    //scraper.set_value(true);
    driveTo(59_in, -48_in, 270_deg);
    pros::delay(3000);

// drive to long goal and score 6 blocks
    //scraper.set_value(false);
    driveTo(30_in, -48_in, 270_deg);
    score_lever();
    score_lever();
}

void onslaught_skills_2() {
// strafe to the two red block near the wall and intake them
    driveTo(46.3_in, -61.5_in, 180_deg);
    set_lift(false); // to go under the goal

// move backwards, turn to the blocks under the long goal, and begin intaking
    driveTo(43.4_in, -53.3_in, 270_deg);
    driveTo(20.9_in, -60.5_in, 290_deg);

// turn on color sort, strafe to the blocks under the goal, and get the red ones
    kill_sorting(false); // enable color sorting to only intake the red blocks
    driveTo(0_in, -48_in, 270_deg);

// turn to the line of balls
    turnTo(0_deg);

// move forward rlly slow and intake all the balls, sorting out blue
    driveTo(0_in, -24_in, 0_deg);

// strafe and make the back intake face the low goal and outtake the balls into the low goal
    driveTo(-12_in, -12_in, 225_deg);
    intake(false);
    outtake(true);
    pros::delay(2000);
}