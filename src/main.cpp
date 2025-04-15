#include "main.h"
#include "rev/api/alg/reckless/path.hh"
#include "rev/api/alg/reckless/turn_segment.hh"
#include "rev/api/alg/reckless/look_at.hh"
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"
#include "rev/api/hardware/devices/rotation_sensors/rotation_sensor.hh"
#include "rev/rev.hh"
#include <vector>
#include <string>

using std::shared_ptr, std::make_shared, std::vector, std::string;

rev::Motor_Group leftd({17, -18, 19, -20});
rev::Motor_Group rightd({1, -2, 3, -4});
rev::Motor conveyor(16);



// pros::Imu imu(4);
shared_ptr<rev::DualImu> imu = make_shared<rev::DualImu>(6, 7);
shared_ptr<rev::ReadOnlyRotarySensor> right_enc = make_shared<rev::QuadEncoder>(static_cast<int>('C'), static_cast<int>('D'), true);
shared_ptr<rev::ReadOnlyRotarySensor> left_enc = make_shared<rev::QuadEncoder>(static_cast<int>('E'), static_cast<int>('F'), true);
// pros::Rotation fwd(1, true);
// pros::Rotation rgt(14);
// pros::Controller controller(pros::controller_id_e_t::E_CONTROLLER_MASTER);

shared_ptr<rev::SkidSteerChassis> chassis = make_shared<rev::SkidSteerChassis>(leftd, rightd);

// pros::Motor test_motor(15);
using namespace rev;

void on_center_button() {}

void initialize() {
  pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  // controller.print(0, 0, "furk");
  // test_motor.move_voltage(12000);
  
//   pros::delay(2000);
  
  
//   shared_ptr<rev::TwoRotationInertialOdometry45Degrees> odom =
//     make_shared<rev::TwoRotationInertialOdometry45Degrees>(
//     left_enc, right_enc, imu, 2.46_in, 2.46_in);

//   odom->set_position({0_ft, 0_ft, 0_deg});

//   // odom->set_position({215_in, 49_in, 16_deg});
//   AsyncRunner odom_runner(odom);

//   /*std::shared_ptr<rev::CampbellTurn> turnc =
//   std::make_shared<CampbellTurn>(chassis, odom, 0.2, 0.05);

//   AsyncRunner turn_runner (turnc);

//   pros::delay(2000);
//   turnc->turn_to_target_absolute(0.7, -15_deg);
//   //pros::delay(2000);
//   turnc->turn_to_target_absolute(0.7, -30_deg);*/

//   shared_ptr<rev::Reckless> reckless =
//       make_shared<Reckless>(chassis, odom);

//   AsyncRunner reckless_runner(reckless);

//   const double kP = 0.0;
//   const double kB = 0.015;

//   /*turn->turn_to_target_absolute(0.7, 45_deg);

//   while (!turn->is_completed())
//     pros::delay(20);*/

//   PilonsSegmentParams SLOW = {
//       make_shared<CascadingMotion>(0.4, kP, kB, 40_in / second, 0.07),
//       make_shared<PilonsCorrection>(2, 0.5_in),
//       make_shared<SimpleStop>(0.1_s, 0.2_s, 0.4),
//   };

//   PilonsSegmentParams MEDIUM = {
//       make_shared<CascadingMotion>(0.7, kP, kB, 60_in / second, 0.07),
//       make_shared<PilonsCorrection>(2, 0.5_in),
//       make_shared<SimpleStop>(0.1_s, 0.2_s, 0.4),
//   };

//   PilonsSegmentParams FAST = {
//       make_shared<CascadingMotion>(0.9, kP, kB, 80_in / second, 0.07),
//       make_shared<PilonsCorrection>(2, 0.5_in),
//       make_shared<SimpleStop>(0.1_s, 0.2_s, 0.4),
//   };

//   // test new reckless syntax
// //   reckless->go({PilonsSegment::create(SLOW, {20_in, 0_in}),
// //                 PilonsSegment::create(MEDIUM, {20_in, 20_in, 90_deg}),
// //                 PilonsSegment::create(FAST, {0_in, 20_in}),
// //                 &PilonsSegment(SLOW, {0_in, 0_in})});

//   reckless->go(RecklessPath()
//                 //    .with_segment(PilonsSegment(
//                 //        std::make_shared<CascadingMotion>(0.7, kP, kB,
//                 //                                          40_in / second, 0.07),
//                 //        std::make_shared<PilonsCorrection>(2, 0.5_in),
//                 //        std::make_shared<SimpleStop>(0.1_s, 0.2_s, 0.4),
//                 //        {5_ft, 0_ft, 0_deg}, 0_in)

//                 //                      )
//                    .with_segment(LookAt(
//                     1.0, 0.25, {5_ft, 5_ft, 0_deg}, 0_deg, 0.085, 0.23, 0.1_s
//                    ))
//                 );

//   while (!reckless->is_completed())
//     pros::delay(20);
//   printf("Completed motion");

  // reckless->go(RecklessPath());

  //   turn_runner.~AsyncRunner();  // Stop AsyncRunner

  //   turn->turn_to_target_absolute(
  //       0.7, 45_deg);  // Attempt another turn with AsyncRunner deleted, this
  //                      // should fail

  //   while (true) {
  //     printf("loop\n");
  //     auto pose = odom->get_state().pos;
  //     std::cout << pose.x.convert(foot) << "ft, " << pose.y.convert(foot) <<
  //     ","
  //              << pose.theta.convert(degree) << "deg" << std::endl;

  //     chassis->drive_arcade(
  //         (double)controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y)/100,
  //         (double)controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_X)/100
  //     );

  //     pros::delay(250);
  //   }

  rev::Optical color(15);
  // Testing MotorGroup.check_ports();
  while(true) {
    std::uint8_t left_ports = leftd.check_ports();
    std::uint8_t right_ports = rightd.check_ports();
    uint8_t conv_port = conveyor.check_port();
    std::pair<uint8_t, uint8_t> left_enc_port = left_enc->check_port();
    std::pair<uint8_t, uint8_t> right_enc_port = right_enc->check_port();
    std::pair<uint8_t, uint8_t> dualimu_ports = imu->check_port();

    pros::lcd::print(0, "Left ports: %d", left_ports);
    pros::lcd::print(1, "Right ports: %d", right_ports);
    pros::lcd::print(2, "Conv port: %d", conv_port);
    pros::lcd::print(3, "IMU ports: %d %d", dualimu_ports.first, dualimu_ports.second);
    

    pros::delay(20);
  }



}