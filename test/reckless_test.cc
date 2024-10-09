#include <gtest/gtest.h>
#include <iostream>
#include "rev/api/hardware/chassis_sim/driftless_sim.hh"
#include "rev/rev.hh"

TEST(Cascade, RecklessVerifications) {
  using namespace rev;
  auto sim = std::make_shared<DriftlessSim>(60_in / second, 200_rpm, 5_Hz, 5_Hz,
                                            20_Hz, 20_Hz);

  AsyncRunner simrunner1(sim);

  std::shared_ptr<rev::Reckless> reckless =
      std::make_shared<Reckless>(sim, sim);

  AsyncRunner reckless_runner(reckless);

  const double kP = 0.0;
  const double kB = 0.015;

  reckless->go(RecklessPath()
                   .with_segment(PilonsSegment(
                       std::make_shared<CascadingMotion>(1, kP, kB,
                                                         60_in / second, 0.07),
                       std::make_shared<PilonsCorrection>(2, 0.5_in),
                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                       {2_ft, 0_ft, 0_deg}, 0_in)

                                     )
                   .with_segment(PilonsSegment(
                       std::make_shared<CascadingMotion>(1, kP, kB,
                                                         60_in / second, 0.07),
                       std::make_shared<PilonsCorrection>(2, 0.5_in),
                       std::make_shared<SimpleStop>(.1_s, 0.2_s, 0.4),
                       {4_ft, 1_ft, 45_deg}, 0_in))
                   .with_segment(PilonsSegment(
                       std::make_shared<CascadingMotion>(1, kP, kB,
                                                         60_in / second, 0.07),
                       std::make_shared<PilonsCorrection>(2, 0.5_in),
                       std::make_shared<SimpleStop>(0.075_s, 0.2_s, 0.4),
                       {0_ft, 0_ft, 0_deg}, 0_in)

                                     ));

  while (!reckless->is_completed()) {
    auto state = sim->get_state();
    std::cout << state.pos.x.convert(inch) << "in, "
              << state.pos.y.convert(inch) << "in" << std::endl;

    pros::delay(500);
  }
}