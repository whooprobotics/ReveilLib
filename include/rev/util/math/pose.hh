#pragma once

#include "rev/util/math/point_vector.hh"

namespace rev {
struct Pose : public PointVector {
  QAngle theta;

  /**
   * @brief Converts this (absolute) pose to a pose relative to the supplied
   * reference
   *
   * This should be exactly the inverse of `to_absolute`
   *
   * @param reference
   * @return Pose
   */
  Pose to_relative(const Pose reference) const;

  /**
   * @brief Converts this (relative) pose to an absolute pose using the
   * reference origin
   *
   * This should be exactly the inverse of `to_relative`
   *
   * @param reference
   * @return Pose
   */
  Pose to_absolute(const Pose reference) const;
};

constexpr bool operator==(Pose lhs, Pose rhs) {
  // 1 degree precision should be enough
  if (abs(lhs.theta - rhs.theta) > 1_deg)
    return false;

  return ((PointVector)lhs == (PointVector)rhs);
}

}  // namespace rev