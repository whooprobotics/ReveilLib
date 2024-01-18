
#include "rev/api/alg/tbh/tbh.hh"
#include "api.h"

namespace rev {
TBH_Controller::TBH_Controller(pros::Motor_Group& imotor_group, double& ikI)
    : motor_group(&imotor_group), kI(ikI) {}

double TBH_Controller::step(double inewReading) {
  if (controllerIsDisabled)
    return 0;

  loopDtTimer
      ->placeHardMark();  /// ############## HOW DO I GET TIME WITH REVEIL
  if (loopDtTimer->getDtFromHardMark() >= sampleTime) {
    stepVel(inewReading);  //// ############ IN OKAPI CAME FROM PID CONTROLLER
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

    loopDtTimer->clearHardMark();

    settledUtil->isSettled(
        error);  /// ########### NEEDS TO BE ADDED TO CONSTRUCTOR
  }
}

void TBH_Controller::controllerSet(double ivalue) {
  target = okapi::remapRange(
      ivalue, -1, 1,
      controllerSetTargetMin,  /// ############ NEEDS TO CHANGE TO REVEIL
      controllerSetTargetMax);
}
double TBH_Controller::getTarget() {
  return target;
}
double TBH_Controller::getTarget() {
  return target;
}
double TBH_Controller::getProcessValue() {
  return velMath->getVelocity().convert(
      okapi::rpm);  /// NEED TO CHANGE TO USE REVEIL
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
bool TBH_Controller::isSettled() {
  return isDisabled()
             ? true
             : settledUtil->isSettled(error);  /// #### NEEDS CONSTRUCTOR
}
void TBH_Controller::setSampleTime(
    okapi::QTime isampleTime) {  /// ######### CHANGE TO USE TIME FROM REVEIL
  sampleTime = isampleTime;
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
void TBH_Controller::flipDisable() {
  controllerIsDisabled = !controllerIsDisabled;
}
void TBH_Controller::flipDisable(bool iisDisabled) {
  controllerIsDisabled = iisDisabled;
}
bool TBH_Controller::isDisabled() {
  return controllerIsDisabled;
}

};  // namespace rev