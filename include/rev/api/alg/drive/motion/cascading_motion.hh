#pragma once
#include "rev/api/alg/drive/motion/motion.hh"
#include "rev/api/units/all_units.hh"

namespace rev {
/**
 * @brief Motion class in which the outputs attempt to approach a specific
 * velocity
 *
 * The target velocity is determined by a formula. For pilons, this formula was
 *
 * `45 * (1 - exp(0.07 * error))`, though they used some extra stuff such as a
 * dropEarly parameter. This formula will likely need tuning for other robots
 * and for v5. It seems to assume a max velocity of about 45 inches per second,
 * which is well within reason for a V4 X drive running 4" omni wheels on speed
 * geared motors, as 45in/s requires ~152rpm and the speed geared V4 motors are
 * capable of approximately 160rpm
 *
 * It may also be worth considering using dependency injection to allow the
 * user to completely replace the velocity selection calculation, but more
 * research needs to be conducted to determine if this would be necessary. Such
 * a solution may have a high performance impact for a small amount of (or no)
 * gain. Another good solution may be a quadratic function. Regardless of the
 * choice here, DI will always be available to a programmer who wishes to
 * implement their own motion controller.
 *
 * For a V5 robot running 3.25" wheels in a skid steer configuration on blue
 * motors geared down by a ratio of 5:3, you can expect
 * 360rpm to be the reasonable max. This leaves you with a theoretical maximum
 * velocity of about 60 inches per second.
 *
 * As for the other constant, 0.07, the meaning of that one is less clear. It
 * will likely require tuning in order to make it usable. We will call this one
 * `k_v` for reference.
 *
 * The output power is then calculated as
 *
 * ```cpp
 * double final_power = (k_b * v_target + k_p * (v_target - v)) * sgn(power);
 * ```
 *
 * This may need some experimentation to determine if it is ideal. Round should
 * not be used on v5, as the power in the context of ReveilLib is a float
 * [-1.0, 1.0], unlike RobotC's int [-128, 127]
 */
class CascadingMotion : public Motion {
 public:
  std::tuple<double, double> gen_powers(OdometryState current_state,
                                        Position target_state,
                                        Position start_state,
                                        QLength drop_early) override;

  /**
   * @brief Construct a new Cascading Motion controller
   *
   * @param ipower The power to run this motion at
   * @param ik_p The proportional constant for approaching the target velocity
   * @param ik_b The feed-forward constant for the target velocity
   * @param imax_v The maximum target velocity of the controller
   * @param ik_v The `k_v` constant for the controller
   */
  explicit CascadingMotion(double ipower,
                           double ik_p,
                           double ik_b,
                           QSpeed imax_v = 60 * inch / second,
                           double ik_v = 0.07);

 private:
  double power;
  double k_p;
  double k_b;
  QSpeed max_v;
  double k_v;
};
}  // namespace rev