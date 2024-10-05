#pragma once
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"
#include "rev/util/math/point_vector.hh"
namespace rev {
/**
 * @brief Implementation of signum
 *
 * @tparam T
 * @param val
 * @return int
 */
template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

/**
 * @brief calculates the radius of a circular arc given the starting point, starting angle, and final position
 *
 * @param first_point Position, aka a PointVector plus a QAngle
 * @param next_point PointVector
 * @return double
 *
 */
QLength calculate_radius(Position first_point, PointVector next_point);

}  // namespace rev