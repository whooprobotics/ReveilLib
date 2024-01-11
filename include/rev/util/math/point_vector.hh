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

}  // namespace rev