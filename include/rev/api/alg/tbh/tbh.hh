
#include "rev/api/async/async_runnable.hh"
#include "rev/api/units/all_units.hh"

pros::Motor motor_1(1);
pros::Motor_Group motor_group({motor_1});
namespace rev {

class TBH_Controller : public AsyncRunnable {
 public:
  TBH_Controller();  // Constructor function / declare arguments..?

  void setMotorPower(double power) {
    // set power to motor in voltage
    motor_group.move_voltage(
        power);  // is this right...? //
                 // ########################################################################
  }

  ENCODERTYPE_IDK getMotorEncoder() {
    // returns encoder value
    return motor_group.get_encoder_units();
  }

  void setVelocity() {}

  void calculateSpeed() {
    // get current encoder value
    // compare to last retrieved encoder value and time from last
    // save current encoder and time
    // calculate rpm from previous and current time/encoder
  }

  void updateVoltageTBH() {
    // do the crazy math stuff
  }

  void step() override;  // needed for Async

 private:  // declare all variables
           // https://gist.github.com/jpearman/6761b53919101f07c400
// Update inteval (in mS) for the flywheel control loop
#define FW_LOOP_SPEED = 1

// Maximum power we want to send to the flywheel motors
#define FW_MAX_POWER = 1

  long counter;  ///< loop counter used for debug

  // encoder tick per revolution
  float ticks_per_rev;  ///< encoder ticks per revolution

  // Encoder
  long e_current;  ///< current encoder count
  long e_last;     ///< current encoder count

  // velocity measurement
  float v_current;  ///< current velocity in rpm
  long v_time;      ///< Time of last velocity calculation

  // TBH control algorithm variables
  long target;          ///< target velocity
  long current;         ///< current velocity
  long last;            ///< last velocity
  float error;          ///< error between actual and target velocities
  float last_error;     ///< error last time update called
  float gain;           ///< gain
  float drive;          ///< final drive out of TBH (0.0 to 1.0)
  float drive_at_zero;  ///< drive at last zero crossing
  long first_cross;     ///< flag indicating first zero crossing
  float drive_approx;   ///< estimated open loop drive

  // final motor drive
  long motor_drive;  ///< final motor control value

  // #############################################################################
};
};  // namespace rev