#include "rev/api/alg/reckless/motion/mecanum_constant_motion.hh"

namespace rev {

MecanumConstantMotion::MecanumConstantMotion(double ipower) : power(fabs(ipower)) {}

SlipstreamPower MecanumConstantMotion::gen_powers(
    OdometryState current_state,
    Position target_state,
    Position start_state,
    QLength drop_early) {

  /*
   * TODO: implement actual constant motion code
   * Should linearly move robot and turn robot so that it finishes both at same time
   *
   */

   double front_left_forward = 0.0;
   double front_right_forward = 0.0;
   double rear_left_forward = 0.0;
   double rear_right_forward = 0.0;

  return {front_left_forward, front_right_forward, rear_left_forward, rear_right_forward};
}
} // namespace rev