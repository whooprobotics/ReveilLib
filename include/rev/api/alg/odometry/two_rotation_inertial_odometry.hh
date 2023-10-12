#include "odometry.hh"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "rev/api/async/async_runnable.hh"
namespace rev {
class TwoRotationInertialOdometry : public Odometry, public AsyncRunnable {
 public:
  /**
   * @brief Get the current position
   *
   * The implementation of this is thread-safe
   *
   * @return OdometryState
   */
  OdometryState get_state() override;
  void set_position(Position pos) override;
  void reset_position() override;
  void step() override;

 private:
  pros::Rotation longitudinal_sensor;  // Sensor indicating forward motion.
                                       // Moving the robot forward should cause
                                       // the position of this to increase.
  pros::Rotation lateral_sensor;       // Sensor indicating motion to the right.
                                  // Moving the robot right should cause the
                                  // position of this to increase.
  pros::Imu inertial;  // Inertial sensor from which the robot yaw will be read

  pros::Mutex current_position_mutex;
  OdometryState current_position;
};
};  // namespace rev