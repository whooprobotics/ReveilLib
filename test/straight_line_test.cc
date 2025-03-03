#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <tuple>
#include "rev/api/hardware/chassis_sim/driftless_sim.hh"
#include "rev/api/async/async_runner.hh"
#include "rev/rev.hh"
#include "rev/api/alg/path_gen/straight_lines.hh"
#include "rev/util/plotting/plotting_functions.hh"

using std::cout, std::endl;
const double kP = 0.0;
const double kB = 0.015;

const double bez_kp = 0.2;
const double bez_ki = 0.0;
const double bez_kd = 0.0;
const std::tuple<double, double, double> pid_constants = {bez_kp, bez_ki, bez_kd};
const QLength wheelbase = 13_in;

std::vector<PointVector> generate_path_waypoints(std::vector<PointVector> path_points, std::size_t resolution) {
  std::vector<PointVector> path_waypoints {};
  PointVector start_point = {0_in, 0_in};
  path_points.insert(path_points.begin(), start_point);
  std::size_t current_idx = 0;
	std::size_t res_per_line = resolution / path_points.size();
  for (std::size_t i = 0; i < path_points.size() - 1; ++i){
		PointVector start = path_points[i];
		PointVector end = path_points[i + 1];
		for (std::size_t j = 0; j < res_per_line; ++j){
			path_waypoints.push_back({start.x + (end.x - start.x) * j / res_per_line,
																			start.y + (end.y - start.y) * j / res_per_line});
		}
	}

  return path_waypoints; 
}

TEST(Straight, StraightVerification1){
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

  std::vector<PointVector> path_points {{0_in, 0_in}, {2_ft, 0_ft}, {0_ft, -2_ft}, {-2_ft, 0_ft}, {-4_ft, 0_ft}};
  cout << "Made path points" << endl;

  reckless->go(RecklessPath()
    .with_segment(StraightSegments(
              			  std::make_shared<CascadingMotion>(0.7, kP, kB, 40_in / second, 0.07),
                      std::make_shared<PilonsCorrection>(2, 0.5_in),
                      std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                      path_points,
                      pid_constants,
                      wheelbase,
                      30,
                      12_in
                      )));
  cout << "Made it go" << endl;

  std::vector<PointVector> robor_points {};
  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(20); // Set timeout duration

  while (!reckless->is_completed()) {
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - start_time > timeout) {
      std::cout << "Timeout reached, stopping the simulation." << std::endl;
      break;
    }

    auto state = sim->get_state();
    std::cout << state.pos.x.convert(inch) << "in, "
              << state.pos.y.convert(inch) << "in" << std::endl;
    robor_points.push_back({state.pos.x, state.pos.y});
    pros::delay(100);
  }
  // go plot stuff
  std::vector<PointVector> path_waypoints = generate_path_waypoints(path_points, 20);
  generate_plot(path_waypoints, robor_points);

}
