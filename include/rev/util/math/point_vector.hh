#pragma once

#include "rev/api/units/all_units.hh"

namespace rev {

/**
 * @brief Structure representing a 2D vector on a plane
 *
 */
struct PointVector {
  QLength x;
  QLength y;
};

/**
 * @brief 2D Vector addition operator
 *
 * @param lhs
 * @param rhs
 * @return constexpr PointVector
 */
constexpr PointVector operator+(const PointVector& lhs,
                                const PointVector& rhs) {
  return PointVector{lhs.x + rhs.x, lhs.y + rhs.y};
}

/**
 * @brief 2D vector subtraction operator
 *
 * @param lhs
 * @param rhs
 * @return constexpr PointVector
 */
constexpr PointVector operator-(const PointVector& lhs,
                                const PointVector& rhs) {
  return PointVector{lhs.x - rhs.x, lhs.y - rhs.y};
}

/**
 * @brief 2D vector dot product operator
 *
 * @param lhs
 * @param rhs
 * @return constexpr QArea
 */
constexpr QArea operator*(const PointVector& lhs, const PointVector& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

/**
 * @brief 2D vector scalar multiplication operator
 *
 * @param lhs The scalar to multiply by
 * @param rhs The vector being multiplied
 * @return constexpr PointVector
 */
constexpr PointVector operator*(const Number& lhs, const PointVector& rhs) {
  return {lhs * rhs.x, lhs * rhs.y};
}

/**
 * @brief 2D vector comparison operator
 *
 * @param lhs
 * @param rhs
 * @return true if the two vectors are within 1 millimeter of one another in
 * each direction
 * @return false if the two vectors are not within 1 millimeter of one another
 */
constexpr bool operator==(PointVector lhs, PointVector rhs) {
  // Millimeter precision is enough here
  if (abs(lhs.x - rhs.x) > 1_mm)
    return false;
  if (abs(lhs.y - rhs.y) > 1_mm)
    return false;

  return true;
}

/**
 * @brief 2D vector magnitude function
 *
 * @param lhs
 * @return QLength The magnitude of the vector
 */
constexpr QLength abs(const PointVector lhs) {
  return sqrt(lhs.x * lhs.x + lhs.y * lhs.y);
}

/**
 * @brief Unitization function
 *
 * @param lhs The vector to be unitized
 * @return constexpr PointVector A vector of length 1 in the same direction as
 * `lhs`
 */
constexpr PointVector unitize(const PointVector lhs) {
  if (abs(lhs) == 0_m)
    return PointVector{1_m, 0_m};
  Number kd = abs(lhs) / meter;
  return PointVector{lhs.x / kd, lhs.y / kd};
}

/**
 * @brief Unit vector from angle function
 *
 * @param lhs The angle for which we are generating the unit vector
 * @return constexpr PointVector A unit vector of length 1 meter
 */
constexpr PointVector unit_from_angle(const QAngle lhs) {
  return PointVector{meter * cos(lhs), meter * sin(lhs)};
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