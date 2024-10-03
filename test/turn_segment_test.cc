#include "rev/api/alg/reckless/turn_segment.hh"
#include <gtest/gtest.h>
#include <iostream>
#include "rev/api/alg/reckless/call.hh"
#include "rev/api/hardware/chassis_sim/driftless_sim.hh"
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_time.hh"
#include "rev/rev.hh"
using namespace rev;
const QAngle tolerance = 3_deg;

bool withinTolerance(QAngle currentAngle, QAngle goalAngle) {
  QAngle angle_difference = goalAngle - currentAngle;
  QAngle target_relative_original =
      angle_difference -
      360 * std::floor((angle_difference.convert(degree) + 180) / 360) * degree;
  return (abs(target_relative_original) < tolerance);
}

void angleTester(double max_power = 0.75,
                 double coast_power = 0.3,
                 QAngle angle = 90_deg,
                 double harsh_coeff = 0.17,
                 double coast_coeff = 0.3,
                 QTime brake_time = 0.2_s) {
  using namespace rev;
  auto sim = std::make_shared<DriftlessSim>(60_in / second, 200_rpm, 5_Hz, 5_Hz,
                                            20_Hz, 20_Hz);

  AsyncRunner simrunner1(sim);

  std::shared_ptr<rev::Reckless> reckless =
      std::make_shared<Reckless>(sim, sim);

  AsyncRunner reckless_runner(reckless);

  // reckless->go(RecklessPath().with_segment(rev::Call(set_flag)));
  reckless->go(RecklessPath().with_segment(rev::RecklessTurnSegment(
      max_power, coast_power, angle, harsh_coeff, coast_coeff, brake_time)));

  while (!reckless->is_completed()) {
    pros::delay(500);
  }
  std::cout << sim->get_state().pos.theta.convert(degree) << std::endl;
  EXPECT_TRUE(withinTolerance(sim->get_state().pos.theta, angle));
}

TEST(TurnTests, TurnTest90) {
  angleTester();
}

TEST(TurnTests, TurnTest360) {
  angleTester(0.75, 0.3, 360_deg);
}