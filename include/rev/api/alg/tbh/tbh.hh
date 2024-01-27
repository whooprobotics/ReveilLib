
#include "rev/api/async/async_runnable.hh"
#include "rev/api/units/all_units.hh"

namespace rev {

class TBH_Controller : public AsyncRunnable {
 public:
  TBH_Controller(std::shared_ptr<pros::MotorGroup> imotor_group,
                 std::shared_ptr<pros::Rotation> irotation_sensor,
                 double& ikI);  // Constructor function /
                                // declare arguments..?

  void step() override;  // for async //
  // override -> makes the compiler give error if issue instead of runtime

  void setTarget(double itarget);

  void controllerSet(double ivalue);

  double getTarget();

  double getProcessValue();

  double getOutput();

  double getMaxOutput();

  double getMinOutput();

  double getError();

  void setOutputLimits(double imax, double imin);

  void setControllerSetTargetLimits(double itargetMax, double itargetMin);

  void reset();

 private:  // declare all variables
  std::shared_ptr<pros::MotorGroup> motor_group;
  std::shared_ptr<pros::Rotation> rotation_sensor;
  double kI;

  double error{0};
  double target{0};
  double output{0};
  double outputSum{0};
  double takeBackHalf{0};
  double outputMax{1};
  double outputMin{-1};
  double controllerSetTargetMax{1};
  double controllerSetTargetMin{-1};
  bool controllerIsDisabled{false};
};
};  // namespace rev