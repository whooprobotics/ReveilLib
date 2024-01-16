#pragma once

#include "rev/api/units/all_units.hh"

namespace rev {

struct PointVector {
  QLength x;
  QLength y;
};

// + and - operators
constexpr PointVector operator+(const PointVector& lhs,
                                const PointVector& rhs) {
  return PointVector{lhs.x + rhs.x, lhs.y + rhs.y};
}

constexpr PointVector operator-(const PointVector& lhs,
                                const PointVector& rhs) {
  return PointVector{lhs.x - rhs.x, lhs.y - rhs.y};
}

// Dot product operator
constexpr QArea operator*(const PointVector& lhs, const PointVector& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

constexpr bool operator==(PointVector lhs, PointVector rhs) {
  // Millimeter precision is enough here
  if (abs(lhs.x - rhs.x) > 1_mm)
    return false;
  if (abs(lhs.y - rhs.y) > 1_mm)
    return false;

  return true;
}

// 2d vector absolute value, equivalent to distance from origin
constexpr QLength abs(const PointVector lhs) {
  return sqrt(lhs.x * lhs.x + lhs.y * lhs.y);
}

// Unitization function, unitizes to 1_m abs
constexpr PointVector unitize(const PointVector lhs) {
  Number kd = abs(lhs) / meter;
  return PointVector{lhs.x / kd, lhs.y / kd};
}

}  // namespace rev