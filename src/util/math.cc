#include "rev/util/math.hh"

template <typename T>
int rev::sgn(T val) {
  return (T(0) < val) - (val < T(0));
}