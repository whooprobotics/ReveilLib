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

}  // namespace rev