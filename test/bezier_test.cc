#include <gtest/gtest.h>
#include <iostream>
#include "rev/api/hardware/chassis_sim/driftless_sim.hh"
#include "rev/api/async/async_runner.hh"
#include "rev/rev.hh"
#include "rev/api/alg/path_gen/bezier_curves.hh"
#include <iostream>
using std::cout, std::endl;

// TEST(Bezier, BezierVerification1){
//   cout << "Beginning" << endl;
//   using namespace rev;
//   auto sim = std::make_shared<DriftlessSim>(60_in / second, 200_rpm, 5_Hz, 5_Hz,
//                                             20_Hz, 20_Hz);

//   AsyncRunner simrunner1(sim);
//   cout << "Made simrunner" << endl;

//   std::shared_ptr<rev::Reckless> reckless =
//       std::make_shared<Reckless>(sim, sim);
//   cout << "Made reckless" << endl;

//   AsyncRunner reckless_runner(reckless);
//   cout << "Made reckless runner" << endl;

//   std::vector<PointVector> path_points {{0_in, 0_in}, {2_ft, 2_ft}, {4_ft, 4_ft}};
//   cout << "Made path points" << endl;

//   reckless->go(RecklessPath()
//     .with_segment(BezierSegment(
//                       std::make_shared<PilonsCorrection>(2, 0.5_in),
//                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
//                       path_points,
//                       0.7,
//                       5,
//                       1_in)));
//   cout << "Made it go" << endl;

//   while (!reckless->is_completed()) {
//     auto state = sim->get_state();
//     std::cout << state.pos.x.convert(inch) << "in, "
//               << state.pos.y.convert(inch) << "in" << std::endl;

//     pros::delay(100);
//   }
// }

TEST(Bezier, BezierVerification2){
  cout << "Beginning" << endl;
  using namespace rev;
  auto sim = std::make_shared<DriftlessSim>(60_in / second, 200_rpm, 5_Hz, 5_Hz,
                                            20_Hz, 20_Hz);

  AsyncRunner simrunner1(sim);
  cout << "Made simrunner" << endl;

  std::shared_ptr<rev::Reckless> reckless =
      std::make_shared<Reckless>(sim, sim);
  cout << "Made reckless" << endl;

  AsyncRunner reckless_runner(reckless);
  cout << "Made reckless runner" << endl;

  std::vector<PointVector> path_points {{0_in, 0_in}, {0_in, 6_in}, {12_in, 6_in}, {12_in, 0_in}, {24_in, 0_in}};
  cout << "Made path points" << endl;

  reckless->go(RecklessPath()
    .with_segment(BezierSegment(
                      std::make_shared<PilonsCorrection>(2, 0.5_in),
                      std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                      path_points,
                      0.2)));
  cout << "Made it go" << endl;

  while (!reckless->is_completed()) {
    auto state = sim->get_state();
    std::cout << state.pos.x.convert(inch) << "in, "
              << state.pos.y.convert(inch) << "in" << std::endl;

    pros::delay(100);
  }
}

// TEST(Bezier, BezierVerification3){
//   cout << "Beginning" << endl;
//   using namespace rev;
//   auto sim = std::make_shared<DriftlessSim>(60_in / second, 200_rpm, 5_Hz, 5_Hz,
//                                             20_Hz, 20_Hz);

//   AsyncRunner simrunner1(sim);
//   cout << "Made simrunner" << endl;

//   std::shared_ptr<rev::Reckless> reckless =
//       std::make_shared<Reckless>(sim, sim);
//   cout << "Made reckless" << endl;

//   AsyncRunner reckless_runner(reckless);
//   cout << "Made reckless runner" << endl;

//   std::vector<PointVector> path_points {{3_in, 3_in}, {0_in, 6_in}, {24_in, 12_in}, {48_in, 16_in}, {50_in, 25_in}};
//   cout << "Made path points" << endl;

//   reckless->go(RecklessPath()
//     .with_segment(BezierSegment(
//                       std::make_shared<PilonsCorrection>(2, 0.5_in),
//                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
//                       path_points,
//                       0.2)));
//   cout << "Made it go" << endl;

//   while (!reckless->is_completed()) {
//     auto state = sim->get_state();
//     std::cout << state.pos.x.convert(inch) << "in, "
//               << state.pos.y.convert(inch) << "in" << std::endl;

//     pros::delay(100);
//   }
// }