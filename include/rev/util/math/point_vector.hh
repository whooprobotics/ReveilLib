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

// Scalar multiplication operator
constexpr PointVector operator*(const Number& lhs, const PointVector& rhs) {
  return {lhs * rhs.x, lhs * rhs.y};
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

/**
 * @brief Projects `lhs` onto `rhs`
 *
 * @param lhs The vector being projected
 * @param rhs A vector in the direction lhs is being projected into
 * @return PointVector The vector projection of `lhs` parallel to `rhs`
 */
constexpr PointVector projection(const PointVector lhs, const PointVector rhs) {
  Number ratio = (lhs * rhs) / (rhs * rhs);
  return ratio * rhs;
}

/**
 * @brief Finds the vector rejection of `lhs` onto `rhs`
 *
 * @param lhs The vector being projected
 * @param rhs The vector which `lhs` will be projected onto
 * @return PointVector The vector projection of `lhs` orthogonal to `rhs`
 */
constexpr PointVector rejection(const PointVector lhs, const PointVector rhs) {
  PointVector proj = projection(lhs, rhs);
  return lhs - proj;
}

}  // namespace rev