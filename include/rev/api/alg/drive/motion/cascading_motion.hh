#include "rev/api/alg/drive/motion/motion.hh"

namespace rev {
/**
 * @brief Motion class in which the outputs attempt to approach a specific
 * velocity
 *
 * The target velocity is determined by a formula. For pilons, this formula was
 *
 * `45 * (1 - exp(0.07 * error))`, though they used some extra stuff such as a
 * dropEarly parameter
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
                                        Position target_state) override;

  /**
   * @brief Construct a new Cascading Motion controller
   *
   * @param ipower The power to run this motion at
   * @param ik_p The proportional constant for approaching the target velocity
   * @param ik_b The feed-forward constant for the target velocity
   */
  explicit CascadingMotion(double ipower, double ik_p, double ik_b);

 private:
  double power;
  double k_p;
  double k_b;
};
}  // namespace rev