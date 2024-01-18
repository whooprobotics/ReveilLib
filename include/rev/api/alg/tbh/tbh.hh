
#include "rev/api/async/async_runnable.hh"
#include "rev/api/units/all_units.hh"

namespace rev {

class TBH_Controller : public AsyncRunnable {
 public:
  TBH_Controller(pros::MotorGroup& imotor_group,
                 double& ikI);  // Constructor function /
                                // declare arguments..?

  double step(double inewReading);  // for async

  void setTarget(double itarget);

  void controllerSet(double ivalue);

  double getTarget();

  double getProcessValue();

  double getOutput();

  double getMaxOutput();

  double getMinOutput();

  double getError();

  bool isSettled();

  void setSampleTime(okapi::QTime isampleTime);  // ################## NEEDS THE
                                                 // REVEIL VERSION..?

  void setOutputLimits(double imax, double imin);

  void setControllerSetTargetLimits(double itargetMax, double itargetMin);

  void reset();

  void flipDisable();

  void flipDisable(bool iisDisabled);

  bool isDisabled();

 private:  // declare all variables
  pros::MotorGroup* motor_group;
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