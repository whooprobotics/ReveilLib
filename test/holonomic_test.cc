#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include "rev/api/alg/slipstream/correction/cross_track_correction.hh"
#include "rev/api/alg/slipstream/motion/mecanum_constant_motion.hh"
#include "rev/api/alg/slipstream/mecanum_segment.hh"
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/alg/slipstream/slipstream.hh"
#include "rev/api/alg/stop/simple_holonomic_stop.hh"
#include "rev/api/async/async_runner.hh"
#include "rev/rev.hh"
#include "rev/api/hardware/chassis_sim/holonomic_sim.hh"
#include "rev/api/units/q_length.hh"

using std::cout, std::endl, std::make_shared, std::shared_ptr;
using namespace rev;

TEST(HolonomicTest, Test1) {
  cout << "Beginning holonomic test" << endl;

  auto sim = make_shared<HolonomicSim>(60_in / second, 200_rpm, 5_Hz, 5_Hz,
                                            20_Hz, 20_Hz);

  AsyncRunner simrunner1(sim);
  cout << "Made simrunner" << endl;

  std::shared_ptr<Slipstream> slipstream = make_shared<Slipstream>(sim, sim);
  cout << "Made slipstream" << endl;

  AsyncRunner slipstreamrunner(slipstream);
  cout << "Made slipstreamrunner" << endl;

  // actually test motion

  double power = 0.75;
  double slow_power = 0.32;
  double coast_power = 0.25;
  shared_ptr<MecanumConstantMotion> motion = make_shared<MecanumConstantMotion>(power);
  shared_ptr<CrossTrackCorrection> ct_correction = make_shared<CrossTrackCorrection>(2, 1, 0.5_in, 3_deg);

  slipstream->go(
    {
      &MecanumSegment(
        motion,
        ct_correction,
        make_shared<SimpleHolonomicStop>(0.1_s, 0_s, coast_power),
        {40_in, 40_in, 30_deg}, 0_in),
      &MecanumSegment(
        motion,
        ct_correction,
        make_shared<SimpleHolonomicStop>(0.1_s, 0_s, coast_power),
        {90_in, 40_in, 90_deg}, 0_in),
      &MecanumSegment(
        motion,
        ct_correction,
        make_shared<SimpleHolonomicStop>(0.1_s, 0_s, coast_power),
        {30_in, 100_in, -30_deg}, 0_in),
    }
  );

  cout << "Made it go" << endl;

  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(20); // Set timeout duration

  while (!slipstream->is_completed()) {
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - start_time > timeout) {
      std::cout << "Timeout reached, stopping the simulation." << std::endl;
      break;
    }

    auto state = sim->get_state();
    std::cout << state.pos.x.convert(inch) << "in, "
              << state.pos.y.convert(inch) << "in, "
              << state.pos.theta.convert(degree) << " deg" << std::endl;

    auto progress = slipstream->progress();
    std::cout << "Progress: " << progress << std::endl;

    pros::delay(100);
  }

}
