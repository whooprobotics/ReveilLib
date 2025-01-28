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
  
TEST(Autons, sonic1){ //AWP-Simple Autonomous
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

  // DRIVE TO START POINT (0.25_ft, 5_ft, -45_deg)
  reckless->go(RecklessPath()
                   .with_segment(PilonsSegment(
                       std::make_shared<CascadingMotion>(1, kP, kB,
                                                         60_in / second, 0.07),
                       std::make_shared<PilonsCorrection>(2, 0.5_in),
                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                       {0.25_ft, 5_ft, 0_deg}, 0_in)
                                    ));

  reckless->await();

  // reckless->go(RecklessPath()                   /////////////////////////////TURN TO -45_deg
  //                   .with_segment(CampbellTurn(
  //                       std::make_shared<CascadingMotion>(1, kP, kB,
  //                                                         60_in / second, 0.07),
  //                       std::make_shared<PilonsCorrection>(2, 0.5_in),
  //                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
  //                       -45_deg,
  //                       3_ft,
  //                       0_deg,
  //                       true)
  //                                   ));
  // END OF DRIVING TO START
  
  // SPIN CONVEYOR


   reckless->go(RecklessPath()
                   .with_segment(PilonsSegment(
                       std::make_shared<CascadingMotion>(1, kP, kB,
                                                         60_in / second, 0.07),
                       std::make_shared<PilonsCorrection>(2, 0.5_in),
                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                       {1_ft, 4.5_ft, 0_deg}, 0_in))
                    .with_segment(PilonsSegment(
                        std::make_shared<CascadingMotion>(1, kP, kB,
                                                          60_in / second, 0.07),
                        std::make_shared<PilonsCorrection>(2, 0.5_in),
                        std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                        {2_ft, 6_ft, 0_deg}, 0_in))
                    .with_segment(PilonsSegment(
                        std::make_shared<CascadingMotion>(1, kP, kB,
                                                          60_in / second, 0.07),
                        std::make_shared<PilonsCorrection>(2, 0.5_in),
                        std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                        {3_ft, 7_ft, 0_deg}, 0_in))
                      );
  
  update_robor_points(reckless, sim);
  // Pick up mogo at ~progress = 2
  //Start intakine when finished
  

   reckless->go(RecklessPath()
                   .with_segment(PilonsSegment(
                       std::make_shared<CascadingMotion>(1, kP, kB,
                                                         60_in / second, 0.07),
                       std::make_shared<PilonsCorrection>(2, 0.5_in),
                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                       {0.9_ft, 6.1_ft, 0_deg}, 0_in))
                    .with_segment(PilonsSegment(
                        std::make_shared<CascadingMotion>(1, kP, kB,
                                                          60_in / second, 0.07),
                        std::make_shared<PilonsCorrection>(2, 0.5_in),
                        std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                        {1_ft, 4.5_ft, 0_deg}, 0_in))
                    .with_segment(PilonsSegment(
                          std::make_shared<CascadingMotion>(1, kP, kB,
                                                            60_in / second, 0.07),
                          std::make_shared<PilonsCorrection>(2, 0.5_in),
                          std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                          {4_ft, 2_ft, 0_deg}, 0_in)) // Needs to somehow intake top ring
                      );

  update_robor_points(reckless, sim);
  // Start intaking at ~ progress = 1

  //Turn to 180 deg

  display_robor_path(robor_points);
  
}

TEST(Autons, knuckles1){ //AWP-Simple Autonomous
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

  // DRIVE TO START POINT (0.75_ft, 8_ft, -90_deg)
  reckless->go(RecklessPath()
                   .with_segment(PilonsSegment(
                       std::make_shared<CascadingMotion>(1, kP, kB,
                                                         60_in / second, 0.07),
                       std::make_shared<PilonsCorrection>(2, 0.5_in),
                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                       {0.75_ft, 8_ft, -90_deg}, 0_in)
                                    ));
  reckless->await();

  reckless->go(RecklessPath()
                   .with_segment(PilonsSegment( //1 -> rush middle mobile goal
                       std::make_shared<CascadingMotion>(1, kP, kB,
                                                         60_in / second, 0.07),
                       std::make_shared<PilonsCorrection>(2, 0.5_in),
                       std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                       {6_ft, 6_ft, -45_deg}, 0_in))
                    .with_segment(PilonsSegment( //2 -> ring kind of near to top left neg corner
                        std::make_shared<CascadingMotion>(1, kP, kB,
                                                          60_in / second, 0.07),
                        std::make_shared<PilonsCorrection>(2, 0.5_in),
                        std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                        {2_ft, 9_ft, 0_deg}, 0_in))
                      );

  update_robor_points(reckless, sim);
  // clamp mogo/ mogo rush @ progress = 1
  // intake @ progress = 1.5 -> 2
  // spin conveyor @ progress = 2

  reckless->go(RecklessPath()
                    .with_segment(PilonsSegment( //2.5 -> back up
                        std::make_shared<CascadingMotion>(1, kP, kB,
                                                          60_in / second, 0.07),
                        std::make_shared<PilonsCorrection>(2, 0.5_in),
                        std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                        {1.5_ft, 8.5_ft, 45_deg}, 0_in))
                    .with_segment(PilonsSegment( //3 -> ring left of centerline mogo
                        std::make_shared<CascadingMotion>(1, kP, kB,
                                                          60_in / second, 0.07),
                        std::make_shared<PilonsCorrection>(2, 0.5_in),
                        std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                        {3.75_ft, 9_ft, 45_deg}, 0_in))
                      );
  update_robor_points(reckless, sim);
  // intake @ progress = 1.5 -> 2
  // spin conveyor @ progress = 2

  reckless->go(RecklessPath()
                    .with_segment(PilonsSegment( //3.5 -> back up
                        std::make_shared<CascadingMotion>(1, kP, kB,
                                                          60_in / second, 0.07),
                        std::make_shared<PilonsCorrection>(2, 0.5_in),
                        std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                        {3_ft, 8.75_ft, 90_deg}, 0_in))
                    .with_segment(PilonsSegment( //4 -> bottom of ladder towards opposite positive corner
                        std::make_shared<CascadingMotion>(1, kP, kB,
                                                          60_in / second, 0.07),
                        std::make_shared<PilonsCorrection>(2, 0.5_in),
                        std::make_shared<SimpleStop>(0_s, 0.2_s, 0.4),
                        {5.5_ft, 7_ft, 135_deg}, 0_in))
                      );
  update_robor_points(reckless, sim);

  display_robor_path(robor_points);

}