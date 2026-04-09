#include "robots/soundwave.hh"
#include "rev/rev.hh"
#include "rev/api/common/units/all_units.hh"

using namespace rev::literals;
using rev::PilonsSegment, rev::PilonsParams;
using rev::TurnSegment, rev::TurnSegmentParams;
using rev::LookAt, rev::LookAtParams;

using namespace rev;

namespace soundwave {

rev::MotorGroup left_drive({-16, 17, -18, 19, -20});
rev::MotorGroup right_drive({11, -12, 13, -14, 15});

std::shared_ptr<rev::Imu> imu = std::make_shared<rev::Imu>(8);
std::shared_ptr<rev::SkidSteerChassis> chassis = std::make_shared<rev::SkidSteerChassis>(left_drive, right_drive);

std::shared_ptr<rev::QuadEncoder> left_encoder = std::make_shared<rev::QuadEncoder>('E', 'F', false);
std::shared_ptr<rev::QuadEncoder> right_encoder = std::make_shared<rev::QuadEncoder>('G', 'H', false);

std::shared_ptr<rev::TwoRotationInertialOdometry45Degrees> odom = std::make_shared<rev::TwoRotationInertialOdometry45Degrees>(
  left_encoder, right_encoder, imu,
  2.46_in, 2.46_in // Wheel Diameters
);

std::shared_ptr<rev::Reckless> reckless = std::make_shared<rev::Reckless>(chassis, odom);

rev::AsyncRunner odom_runner(odom);
rev::AsyncRunner reckless_runner(reckless);

void init() {}

void auton() {
  pros::delay(3000);
  reckless->go({
    // &PilonsSegment({1_ft, -1_ft})
    // &TurnSegment(90_deg)
    &LookAt({2_ft, 1_ft})
  });

  rev::OdometryState current;
  while(reckless->progress() < 0.8) {
    current = odom->get_state(); 
    std::cout << "x: " << current.pos.x.convert(inch) << "\ty: " << current.pos.y.convert(inch) << "\ta: " << current.pos.theta.convert(degree) << std::endl;
    pros::delay(10);
  }
  reckless->await();

  while(true) {
    current = odom->get_state(); 
    std::cout << "x: " << current.pos.x.convert(inch) << "\ty: " << current.pos.y.convert(inch) << "\ta: " << current.pos.theta.convert(degree) << std::endl;
    pros::delay(10);
  }
}

}  // namespace soundwave