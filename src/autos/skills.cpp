#include "rev/rev.hh"
#include "autos/autos.h"
#include "robot_testing/rev2_config.hh"
#include "robot_testing/rev2_macros.hh"


void onslaught_skills_1() {
  odom->set_position({-46.75_in, -16_in, 90_deg});
// put down scraper and strafe the parking zone blocks
    scraper.set_value(true);
    drive(-46.75_in, 7_in, 90_deg);
// put up scraper and strafe to the matchloader
    scraper.set_value(false);
    drive(-46.75_in, -47_in, 90_deg);
// go backward and intake the 6 blocks
    scraper.set_value(true);
    drive(-59.28_in, -47_in, 90_deg);
// strafe to the side of the long goal
    drive(-32_in, -60_in, 90_deg);
// drive forward to the far side of the long goal
    drive(35_in, -60_in, 90_deg);
// strafe and turn to align to the long goal
    drive(35_in, -47_in, 270_deg);
// drive forward and score the 6 blocks
    drive(30_in, -47_in, 270_deg);
// put down scraper and intake 6 blocks
    scraper.set_value(true);
    drive(59_in, -47_in, 270_deg);
// drive to long goal and score 6 blocks
    scraper.set_value(false);
    drive(30_in, -47_in, 270_deg);
}

void onslaught_skills_2() {}