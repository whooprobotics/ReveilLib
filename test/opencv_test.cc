#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <tuple>
#include "rev/api/hardware/chassis_sim/driftless_sim.hh"
#include "rev/api/async/async_runner.hh"
#include "rev/rev.hh"
#include "rev/api/alg/path_gen/bezier_curves.hh"
#include "rev/util/plotting/plotting_opencv.hh"

using namespace rev;
using std::cout, std::endl;
const double kP = 0.0;
const double kB = 0.015;

const double bez_kp = 0.2;
const double bez_ki = 0.0;
const double bez_kd = 0.0;
const std::tuple<double, double, double> pid_constants = {bez_kp, bez_ki, bez_kd};
const QLength wheelbase = 13_in;

std::vector<PointVector> robor_points {};
void update_robor_points(std::shared_ptr<rev::Reckless> reckless, std::shared_ptr<rev::DriftlessSim> sim){
  while (!reckless->is_completed()) {
      auto state = sim->get_state();
      std::cout << state.pos.x.convert(inch) << "in, "
                << state.pos.y.convert(inch) << "in" << std::endl;
      robor_points.push_back({state.pos.x, state.pos.y});
      pros::delay(100);
    }
}
  
TEST(Autons, auton1){
  cout << "Beginning" << endl;
  auto sim = std::make_shared<DriftlessSim>(60_in / second, 200_rpm, 5_Hz, 5_Hz,
                                            20_Hz, 20_Hz);

  AsyncRunner simrunner1(sim);
  cout << "Made simrunner" << endl;

  std::shared_ptr<rev::Reckless> reckless =
      std::make_shared<Reckless>(sim, sim);
  cout << "Made reckless" << endl;

  AsyncRunner reckless_runner(reckless);
  cout << "Made reckless runner" << endl;

  // DRIVE TO START POINT
//   reckless->go(RecklessPath()
//                    .with_segment(RecklessPathSegment(
//                        std::make_shared<CascadingMotion>(1, kP, kB,
//                                                          60_in / second, 0.07),
//                        std::make_shared<PilonsCorrection>(2, 0.5_in),
//                        std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
//                        {1_ft, 3.5_ft, 0_deg}, 0_in)
//                                     ));

//   reckless->await();
  // END OF DRIVING TO START
  
 reckless->go(RecklessPath()
    .with_segment(BezierSegment(
               std::make_shared<CascadingMotion>(0.7, kP, kB, 40_in / second, 0.07),
                      std::make_shared<PilonsCorrection>(2, 0.5_in),
                      std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                      std::vector<PointVector>{{1_ft, 3.5_ft}, {3_ft, 3.5_ft},{6_ft, 2_ft}}, 
                      pid_constants,
                      wheelbase,
                      30,
                      6_in
                      )));
  update_robor_points(reckless, sim);

//   reckless->go(RecklessPath()
//                    .with_segment(RecklessPathSegment(
//                        std::make_shared<CascadingMotion>(1, kP, kB,
//                                                          60_in / second, 0.07),
//                        std::make_shared<PilonsCorrection>(2, 0.5_in),
//                        std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
//                        {3_ft, 3_ft, 0_deg}, 0_in)

//                                      )
//                    .with_segment(RecklessPathSegment(
//                        std::make_shared<CascadingMotion>(1, kP, kB,
//                                                          60_in / second, 0.07),
//                        std::make_shared<PilonsCorrection>(2, 0.5_in),
//                        std::make_shared<SimpleStop>(.1_s, 0.2_s, 0.4),
//                        {0_ft, 0_ft, 45_deg}, 0_in))
//                 );



  update_robor_points(reckless, sim);
  reckless->await();


  display_robor_path(robor_points);
  
}