#pragma once
#include "rev/api/alg/drive/motion/motion.hh"

namespace rev {
/**
 * @brief Motion class in which the outputs are of a constant power
 *
 * This class takes an input power and just spits out a drive tuple with 2
 * powers of that exact value, like pilons mttSimple
 */
class ConstantMotion : public Motion {
 public:
  std::tuple<double, double> gen_powers(OdometryState current_state,
                                        Position target_state,
                                        Position start_state,
                                        QLength drop_early) override;

  /**
   * @brief Construct a new Constant Motion object
   *
   * @param ipower
   */
  explicit ConstantMotion(double ipower);

 private:
  double power;
};
}  // namespace rev