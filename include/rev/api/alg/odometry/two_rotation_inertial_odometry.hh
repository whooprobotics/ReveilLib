#include "odometry.hh"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
namespace rev {
class TwoRotationInertialOdometry : public Odometry {
 public:
  OdometryState get_state() override;
  void set_position(Position pos) override;
  void reset_position() override;

 private:
  pros::Rotation longitudinal_sensor;  // Sensor indicating forward motion.
                                       // Moving the robot forward should cause
                                       // the position of this to increase.
  pros::Rotation lateral_sensor;       // Sensor indicating motion to the right.
                                  // Moving the robot right should cause the
                                  // position of this to increase.
  pros::Imu inertial;  // Inertial sensor from which the robot yaw will be read
};
};  // namespace rev