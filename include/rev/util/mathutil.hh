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