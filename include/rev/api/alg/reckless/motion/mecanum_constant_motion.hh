#pragma once
#include <memory>
#include "rev/api/alg/reckless/motion/slipstream_motion.hh"
#include "rev/api/alg/slipstream/power.hh"

namespace rev {
/**
 * @brief Motion class in which the outputs are of a constant power
 *
 * This class takes an input power and just spits out a drive tuple with 2
 * powers of that exact value, like pilons mttSimple
 */
class MecanumConstantMotion : public SlipstreamMotion {
 public:
  SlipstreamPower gen_powers(OdometryState current_state,
                             Position target_state,
                             Position start_state,
                             QLength drop_early) override;

  /**
   * @brief Construct a new Constant Motion object
   *
   * @param ipower
   */
  explicit MecanumConstantMotion(double ipower);

  std::shared_ptr<MecanumConstantMotion> operator&() {
    return std::make_shared<MecanumConstantMotion>(*this);
  }

 private:
  double power;
};
}  // namespace rev