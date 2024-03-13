#include "rev/util/math/point_vector.hh"
#include <gtest/gtest.h>

TEST(MathTests, PointVectorTests) {
  using namespace rev;
  PointVector a{3_m, 6_m};
  PointVector b{4_m, 0_m};

  // Test dot product
  EXPECT_FLOAT_EQ((a * b).get_value(), 12.);

  // Test addition
  PointVector c = a + b;
  EXPECT_FLOAT_EQ(c.x.get_value(), 7.);
  EXPECT_FLOAT_EQ(c.y.get_value(), 6.);

  // Test subtraction
  PointVector d = a - b;
  EXPECT_FLOAT_EQ(d.x.get_value(), -1.);
  EXPECT_FLOAT_EQ(d.y.get_value(), 6.);
}