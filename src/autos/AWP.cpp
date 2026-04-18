#include "autos/autos.h"
#include "rev/rev.hh"
#include "robot_testing/rev2_config.hh"
#include "robot_testing/rev2_macros.hh"

void AWP() {
  // im sorry raj its gone chat

  odom->set_position({-47_in, -16.75_in, 90_deg});
  // Drive to scraper
  kill_sorting(true);

  set_scraper(true);
  driveTo(-47_in, -47_in, 90_deg, Drive{.max_speed = 12});
  // Go forward to start match loading
  intake(true);
  // Go back to score
  driveTo(-56.5_in, -47_in, 90_deg, Drive{. max_speed = 6 });
  driveTo(-3.0, -3.0, Drive{ .timeout = 2000_ms });
  
  set_lift(true);
  // Drive back to the long goal
  driveTo(-31.5_in, -47_in, 90_deg);

  score_lever(true);

  pros::delay(1000);
  
  driveTo(-56.5_in, -47_in, 90_deg, Drive{}, true);
  pros::delay(200);
  score_lever(false);
  slipstream->await();
  
  driveTo(-3.0, -3.0, Drive{ .timeout = 3000_ms });

  set_scraper(false);
  intake(false);

  driveTo(-51.25_in, -47_in, 90_deg, Drive{.min_speed = 3, .exit_error = 2_in});
  driveTo(-13.5_in, -13.5_in, 225_deg);
  outtake(true);
  pros::delay(1000);
  outtake(false);

  driveTo(5_in, Drive{ .max_speed = 12, .min_speed = 5, .exit_error = 2_in });
  set_descore(false);
  driveTo(-29.55_in, -35.99_in, 90_deg);

  set_descore(true);
  driveTo(-14.55_in, -35.99_in, 90_deg, Drive{ .timeout = 5000_ms, .turn_settle = { .settle_time = 0_ms, .large_settle_time = 0_ms }});
  
}