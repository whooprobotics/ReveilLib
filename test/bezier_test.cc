#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include "rev/api/hardware/chassis_sim/driftless_sim.hh"
#include "rev/api/async/async_runner.hh"
#include "rev/rev.hh"
#include "rev/api/alg/path_gen/bezier_curves.hh"
#include "rev/util/plotting/plotting_functions.hh"

using std::cout, std::endl;
const double kP = 0.0;
const double kB = 0.015;

std::vector<PointVector> generate_bezier_points(std::vector<PointVector> path_points, std::size_t resolution){
  std::vector<PointVector> bezier_points {};
  PointVector start_point = {0_in, 0_in};
  path_points.insert(path_points.begin(), start_point);
  std::size_t current_idx = 0;

  for (std::size_t t = 0; t < resolution; ++t) {
    double t_value = static_cast<double>(t) / (resolution - 1);
    std::vector<PointVector> temp_points = path_points; // Initialize temp_points with path_points

    for (std::size_t layer = 1; layer < path_points.size(); ++layer) {
      for (std::size_t idx = 0; idx + layer < path_points.size(); ++idx) {
        temp_points[idx] = (1-t_value) * temp_points[idx] + t_value * temp_points[idx + 1];
      }
    }
    bezier_points.push_back(temp_points[0]);
  }
  return bezier_points;
}

TEST(Bezier, BezierVerification1){
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

  std::vector<PointVector> path_points {{0_in, 0_in}, {2_ft, 2_ft}, {4_ft, 4_ft}};
  cout << "Made path points" << endl;

  reckless->go(RecklessPath()
    .with_segment(BezierSegment(
               std::make_shared<CascadingMotion>(0.7, kP, kB, 40_in / second, 0.07),
                      std::make_shared<PilonsCorrection>(2, 0.5_in),
                      std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                      path_points,
                      5,
                      1_in)));
  cout << "Made it go" << endl;

  std::vector<PointVector> robor_points {};
  while (!reckless->is_completed()) {
    auto state = sim->get_state();
    std::cout << state.pos.x.convert(inch) << "in, "
              << state.pos.y.convert(inch) << "in" << std::endl;
    robor_points.push_back({state.pos.x, state.pos.y});
    pros::delay(100);
  }
  // go plot stuff
  std::vector<PointVector> bezier_points = generate_bezier_points(path_points, 5);
  generate_plot(bezier_points, robor_points);

}

// TEST(Bezier, BezierVerification2){
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

//   std::vector<PointVector> path_points {{0_in, 0_in}, {0_in, 6_in}, {12_in, 6_in}, {12_in, 0_in}, {24_in, 0_in}};
//   cout << "Made path points" << endl;

//   reckless->go(RecklessPath()
//     .with_segment(BezierSegment(
//                std::make_shared<CascadingMotion>(0.7, kP, kB, 40_in / second, 0.07),
//                       std::make_shared<PilonsCorrection>(2, 0.5_in),
//                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
//                       path_points
//                       )));
//   cout << "Made it go" << endl;

//   while (!reckless->is_completed()) {
//     auto state = sim->get_state();
//     std::cout << state.pos.x.convert(inch) << "in, "
//               << state.pos.y.convert(inch) << "in" << std::endl;

//     pros::delay(100);
//   }
// }

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
//                std::make_shared<CascadingMotion>(0.7, kP, kB, 40_in / second, 0.07),
//                       std::make_shared<PilonsCorrection>(2, 0.5_in),
//                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
//                       path_points
//                       )));
//   cout << "Made it go" << endl;

//   while (!reckless->is_completed()) {
//     auto state = sim->get_state();
//     std::cout << state.pos.x.convert(inch) << "in, "
//               << state.pos.y.convert(inch) << "in" << std::endl;

//     pros::delay(100);
//   }
// }

// TEST(Bezier, BezierVerification4){
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

//   std::vector<PointVector> path_points {{0_in, 0_in}, {6_in, 6_in}, {12_in, 24_in}};
  
//   cout << "Made path points" << endl;

//   reckless->go(RecklessPath()
//     .with_segment(BezierSegment(
//                       std::make_shared<CascadingMotion>(0.2, kP, kB, 40_in / second, 0.07),
//                       std::make_shared<PilonsCorrection>(2, 0.5_in),
//                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
//                       path_points,
//                       4
//                       )));
//   cout << "Made it go" << endl;

//   while (!reckless->is_completed()) {
//     auto state = sim->get_state();
//     std::cout << state.pos.x.convert(inch) << "in, "
//               << state.pos.y.convert(inch) << "in" << std::endl;

//     pros::delay(100);
//   }
// }