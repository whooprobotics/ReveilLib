
#include "rev/api/alg/tbh/tbh.hh"
#include "api.h"

namespace rev {
TBH_Controller::TBH_Controller(std::shared_ptr<pros::MotorGroup> imotor_group,
                               std::shared_ptr<pros::Rotation> irotation_sensor,
                               double& ikI)
    : motor_group(imotor_group), rotation_sensor(irotation_sensor), kI(ikI) {}

void TBH_Controller::step() {
  // Get the new error
  double nerror = getError();
  // Adjust the controller accordingly using an I loop
  output += kI * nerror;

  // Heres the take back half magic
  // If the sign of the error has changed since the last step, cut back the
  // output halfway to the last time this code ran
  if (std::copysign(1.0, error) != std::copysign(1.0, nerror)) {
    output = 0.5 * (output + takeBackHalf);
    takeBackHalf = output;
  }
  error = nerror;
  motor_group->move_voltage(output);
}

void TBH_Controller::controllerSet(double ivalue) {
  target =
      ((ivalue + 1) / 2) * (controllerSetTargetMax - controllerSetTargetMin) +
      controllerSetTargetMin;
  // sets target to the "ratio" of i is to -1,1 as controller max/min
}
void TBH_Controller::setTarget(double itarget) {
  target = itarget;
}
double TBH_Controller::getTarget() {
  return target;
}
double TBH_Controller::getProcessValue() {
  return rotation_sensor->get_velocity();
}
double TBH_Controller::getOutput() {
  return output;
}
double TBH_Controller::getMaxOutput() {
  return outputMax;
}
double TBH_Controller::getMinOutput() {
  return outputMin;
}
double TBH_Controller::getError() {
  return getTarget() - getProcessValue();
}

void TBH_Controller::setOutputLimits(double imax, double imin) {
  outputMax = imax;
  outputMin = imin;
}
void TBH_Controller::setControllerSetTargetLimits(double itargetMax,
                                                  double itargetMin) {
  controllerSetTargetMax = itargetMax;
  controllerSetTargetMin = itargetMin;
}
void TBH_Controller::reset() {
  target = 0;
  takeBackHalf = 0;
  output = 0;
  outputSum = 0;
  error = 0;
}

};  // namespace rev