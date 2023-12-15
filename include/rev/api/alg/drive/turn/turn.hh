#pragma once

#include "rev/api/units/all_units.hh"

namespace rev {
/**
 * @brief Interface for turn controllers
 *
 */
class Turn {
 public:
  /**
   * @brief Starts a turn towards an absolute heading
   *
   * @param max_power The maximum power the controller will output
   * @param angle The absolute heading the controller will target
   */
  virtual void turn_to_target_absolute(double max_power, QAngle angle) = 0;
};

/**
 * @brief Possible turn controller states
 *
 */
enum class TurnState { INACTIVE, FULLPOWER, COAST, BRAKE };

}  // namespace rev
