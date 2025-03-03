#pragma once

#include "rev/util/math/point_vector.hh"

namespace rev {
struct Pose : public PointVector {
  QAngle theta;

  constexpr Pose(QLength x, QLength y) : PointVector{x, y}, theta(0_deg) {}
  constexpr Pose(QLength x, QLength y, QAngle angle)
      : PointVector{x, y}, theta(angle) {}
  constexpr Pose(const Pose& other)
      : PointVector{other.x, other.y}, theta(other.theta) {}
  constexpr Pose() : PointVector{0_m, 0_m}, theta(0_deg) {}

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