#include <gtest/gtest.h>
#include <iostream>
#include "rev/api/alg/reckless/call.hh"
#include "rev/api/alg/reckless/turn_segment.hh"
#include "rev/api/hardware/chassis_sim/driftless_sim.hh"
#include "rev/rev.hh"
#include "rev/api/units/q_time.hh"
#include "rev/api/units/q_angle.hh"
using namespace rev;
const QAngle tolerance = 2_deg;

bool withinTolerance(QAngle currentAngle, QAngle goalAngle){
    return (abs(currentAngle-goalAngle) < tolerance);
}

TEST(Call, RecklessVerifications) {
  using namespace rev;
  auto sim = std::make_shared<DriftlessSim>(60_in / second, 200_rpm, 5_Hz, 5_Hz,
                                            20_Hz, 20_Hz);

  AsyncRunner simrunner1(sim);

  std::shared_ptr<rev::Reckless> reckless =
      std::make_shared<Reckless>(sim, sim);

  AsyncRunner reckless_runner(reckless);

  double max_power = 0.5;
  double coast_power = 0.1;
  QAngle angle = 90_deg;
  double harsh_coeff = 0.2; //idk what these should be
  double coast_coeff = 0.2; // idk what these should be
  QTime brake_time = .2_s;

  //reckless->go(RecklessPath().with_segment(rev::Call(set_flag)));
  reckless->go(RecklessPath().with_segment(rev::RecklessTurnSegment(max_power, coast_power, angle, harsh_coeff, coast_coeff, brake_time)));

  while (!reckless->is_completed()) {
    pros::delay(500);
  }



  EXPECT_TRUE(withinTolerance(sim->get_state().pos.theta,angle));
}