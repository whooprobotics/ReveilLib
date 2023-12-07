#pragma once
#include "rev/api/alg/drive/motion/motion.hh"

namespace rev {
/**
 * @brief Motion class in which the outputs are proportional to longitudinal
 * error
 *
 * This could potentially cause issues if the longitudinal error gets to 0
 * outside of the settling area.
 */
class ProportionalMotion : public Motion {
 public:
  std::tuple<double, double> gen_powers(OdometryState current_state,
                                        Position target_state,
                                        Position start_state,
                                        QLength drop_early) override;

  /**
   * @brief Construct a new Proportional Motion object
   *
   * @param ipower The maximum power the controller will output
   * @param ik_p A constant determining the power to be applied per inch from
   * target
   */
  explicit ProportionalMotion(double ipower, double ik_p);

 private:
  double power;
  double k_p;
};
}  // namespace rev