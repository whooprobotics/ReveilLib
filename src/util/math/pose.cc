#include "rev/util/math/pose.hh"

namespace rev {

/**
 * @brief Converts this (absolute) pose to a pose relative to the supplied
 * reference
 *
 * This should be exactly the inverse of `to_absolute`
 *
 * @param reference
 * @return Pose
 */
Pose Pose::to_relative(const Pose reference) const {
  // First shift the input, then rotate it
  QLength x_shift = this->x - reference.x;
  QLength y_shift = this->y - reference.y;

  return Pose{x_shift * cos(reference.theta) + y_shift * sin(reference.theta),
              x_shift * -sin(reference.theta) + y_shift * cos(reference.theta),
              this->theta - reference.theta};
}

/**
 * @brief Converts this (relative) pose to an absolute pose using the
 * reference origin
 *
 * This should be exactly the inverse of `to_relative`
 *
 * @param reference
 * @return Pose
 */
Pose Pose::to_absolute(const Pose reference) const {
  // First rotate the input, then shift it

  QLength x_shift =
      this->x * cos(reference.theta) - this->y * sin(reference.theta);
  QLength y_shift =
      this->x * sin(reference.theta) + this->y * cos(reference.theta);

  return Pose{reference.x + x_shift, reference.y + y_shift,
              reference.theta + this->theta};
}
}  // namespace rev