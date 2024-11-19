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
  std::cout << "GOAL: " << goalAngle.convert(degree) << std::endl;
  std::cout << "ANGLE: " << currentAngle.convert(degree) << std::endl;
  std::cout << "Test Results: " << target_relative_original.convert(degree)
            << std::endl;
  return (abs(target_relative_original) < tolerance);
}

void angleTester(double max_power = 0.7,
                 double coast_power = 0.3,
                 QAngle angle = 90_deg,
                 double harsh_coeff = 0.10,
                 double coast_coeff = 0.5,
                 QTime brake_time = 0.25_s) {
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
    std::cout << sim->get_state().pos.theta.convert(degree) << " / "
              << angle.convert(degree) << " degrees" << std::endl;
    std::cout << sim->get_state().vel.angular.convert(degree / second)
              << " deg/s" << std::endl;
    pros::delay(50);  // 10 for hard testing, 500 for typical
  }
  std::cout << sim->get_state().pos.theta.convert(degree) << std::endl;
  EXPECT_TRUE(withinTolerance(sim->get_state().pos.theta, angle));
}

TEST(TurnTests, TurnTest_90) {
  angleTester();
}

TEST(TurnTests, TurnTest_45) {
  angleTester(.70, 0.3, 45_deg);
}

TEST(TurnTests, TurnTest_60) {
  angleTester(.70, 0.3, 60_deg);
}

TEST(TurnTests, TurnTest_120) {
  angleTester(.70, 0.3, 120_deg);
}

TEST(TurnTests, TurnTest_180) {
  angleTester(.70, 0.3, 180_deg);
}

TEST(TurnTests, TurnTest_360) {
  angleTester(.70, 0.3, 360_deg);
}

TEST(TurnTests, TurnTest_Neg_90) {
  angleTester(.70, 0.3, -90_deg);
}

TEST(TurnTests, TurnTest_Neg_45) {
  angleTester(.70, 0.3, -45_deg);
}

TEST(TurnTests, TurnTest_Neg_60) {
  angleTester(.70, 0.3, -60_deg);
}

TEST(TurnTests, TurnTest_Neg_120) {
  angleTester(.70, 0.3, -120_deg);
}

TEST(TurnTests, TurnTest_Neg_180) {
  angleTester(.70, 0.3, -180_deg);
}

TEST(TurnTests, TurnTest_Neg_360) {
  angleTester(.70, 0.3, -360_deg);
}

// Corner cases

TEST(TurnTests, TurnTest_Tolerance) {
  angleTester(.70, 0.3, tolerance);
}

TEST(TurnTests, TurnTest_400) {
  angleTester(.70, 0.3, 400_deg);
}

TEST(TurnTests, TurnTest_0) {
  angleTester(.70, 0.3, 0_deg);
}
