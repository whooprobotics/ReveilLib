#include "rev/rev.hh"
#include "autos/autos.h"
#include "robot_testing/rev2_config.hh"
#include "robot_testing/rev2_macros.hh"

using namespace rev;

void printOdomShi(){
  while (true)
  {
    std::cout << "(" << odom->get_state().pos.x.convert(inch) << ", " << odom->get_state().pos.y.convert(inch) << ")" << std::endl;
    pros::delay(80);
  }
}

void onslaught_skills_1() {
    pros::Task PRINT_ODOM(printOdomShi);
    odom->set_position({-46.75_in, -15_in, 90_deg});
// put down scraper and strafe the parking zone blocks
    set_scraper(true);
    pros::delay(500);
    driveTo(-46.75_in, 7_in, 90_deg);
// put up scraper and strafe to the matchloader
    set_scraper(false);
    driveTo(-46.75_in, -47.5_in, 90_deg);
// go backward and intake the 6 blocks
    set_scraper(true);
    kill_sorting(true); // disable color sorting to intake all blocks
    intake(true);
    pros::delay(750);
    
    driveTo(-58_in, -47_in, 90_deg);
    driveTo(-3.0, -3.0, Drive{ .timeout = 750_ms });//delays while ramming wall to intake blocks
    pros::delay(300);
    driveTo(-3.0, -3.0, Drive{ .timeout = 500_ms });
    pros::delay(300);
    driveTo(-3.0, -3.0, Drive{ .timeout = 500_ms });
    pros::delay(300);
    driveTo(-3.0, -3.0, Drive{ .timeout = 500_ms });

// strafe to the side of the long goal
    driveTo(-32_in, -60_in, 90_deg);
    set_scraper(false);
  
// drive forward to the far side of the long goal
    driveTo(35_in, -60_in, 90_deg);

// strafe and turn to align to the long goal
    driveTo(35_in, -48_in, 270_deg);
    set_lift(true);
  
// drive forward and score the 6 blocks
    driveTo(29_in, -48_in, 270_deg);
    score_lever();
    score_lever();

// put down scraper and intake 6 blocks
    scraper.set_value(true);
    driveTo(60_in, -46.5_in, 270_deg, Drive{ .max_speed = 7 });
    driveTo(-3.0, -3.0, Drive{ .timeout = 3000_ms });//delays while ramming wall to intake blocks

// drive to long goal and score 6 blocks
    scraper.set_value(false);
    driveTo(29_in, -48_in, 270_deg);
    score_lever();
    score_lever();
    PRINT_ODOM.remove();
}

void onslaught_skills_2() {
    // strafe to the 2 blocks near the wall and intake them
    driveTo(46.32_in, -61.56_in, 180_deg);

    // strafe to the line of blocks and prepare to intake them
    driveTo(24_in, -30_in);
    driveTo(2_in, -22_in, 225_deg);

    // intake only the 3 reds (turn on color sorting)
    kill_sorting(false);
    driveTo(0_in, -33_in, Drive{ .max_speed = 2 });

    // strafe to the side and face the long goal balls
    driveTo(-11.3_in, -37_in, 150_deg, Drive{ .max_speed = 3 });

    // drive forward and intake the 2 blocks
    driveTo(-7_in, -47_in, 100_deg, Drive{ .max_speed = 3 });

    // strafe to the low goal and score the 7 blocks
    driveTo(-13.5_in, -13.5_in, 225_deg);
}