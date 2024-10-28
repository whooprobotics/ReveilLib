#pragma once
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"
#include "rev/util/math/point_vector.hh"
#include <tuple>
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
 * @return QLength and a bool of whether the inside wheels are the left wheels
 *
 */
QLength calculate_radius(PointVector first_point, Position current_pos, PointVector next_point);

/**
 * @brief Calculates the relative linear velocity of the inside part of the such that the angular velocities stay the same
 *
 *
 *
 *  @return double
 */
Number calculate_inside_ratio(QLength chassis_width, QLength arc_radius);

}  // namespace rev