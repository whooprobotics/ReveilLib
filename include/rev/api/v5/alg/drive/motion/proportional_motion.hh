#pragma once

#ifdef PLATFORM_BRAIN
#include <memory>
#include "rev/api/v5/alg/drive/motion/motion.hh"

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
  /**
   * @brief Generate motor powers.
   * 
   * This is intended for use as the initial generation of motor powers.
   * Correction should be applied later.
   * 
   * @param current_state The current state when this method is called
   * @param target_state The target state being approached
   * @param start_state The position occupied when the current segment gained control
   * @param drop_early The distance from the target point at which this segment should end
   * 
   * @return std::tuple<double, double> Motor powers for a differential drive
   */
  std::tuple<double, double> gen_powers(OdometryState current_state,
                                        Position target_state,
                                        Position start_state,
                                        QLength drop_early) override;

  /**
   * @brief Construct a new Proportional Motion object
   *
   * @param ipower The maximum power the controller will output
   * @param ik_p A constant determining the power to be applied per inch from target
   */
  explicit ProportionalMotion(double ipower, double ik_p);

  /**
   * @brief Shorthand for creating a new ProportionalMotion object
   * 
   * @return std::shared_ptr<ProportionalMotion> newly constructed ProportionalMotion object
   */
  std::shared_ptr<ProportionalMotion> operator&() {
    return std::make_shared<ProportionalMotion>(*this);
  }

 private:
  double power;
  double k_p;
};
}  // namespace rev

#endif