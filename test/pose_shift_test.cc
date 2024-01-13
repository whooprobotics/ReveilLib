#include <gtest/gtest.h>
#include "rev/util/math/point_vector.hh"
#include "rev/util/math/pose.hh"

TEST(MathTests, PoseShiftInversionTest) {
  using namespace rev;
  Pose a{3_m, 6_m, 45_deg};
  Pose b{4_m, 0_m, 0_deg};

  // Inversion test
  EXPECT_TRUE(a.to_absolute(b).to_relative(b) == a);
  EXPECT_TRUE(a.to_relative(b).to_absolute(b) == a);
}

TEST(MathTests, ToAbsoluteTest) {
  using namespace rev;
  Pose a{3_m, 6_m, 45_deg};
  Pose b{4_m, 0_m, 0_deg};

  // Expected result if a is relative to b
  Pose c = {7_m, 6_m, 45_deg};

  // test c
  EXPECT_TRUE(a.to_absolute(b) == c);

  // Expected result if b is relative to a
  c = {5.82842_m, 8.82842_m, 45_deg};

  // test c
  EXPECT_TRUE(b.to_absolute(a) == c);
}

TEST(MathTests, ToRelativeTest) {
  using namespace rev;
  Pose a{7_m, 6_m, 45_deg};
  Pose b{4_m, 0_m, 0_deg};

  // Expected result if a is referenced relative to b
  Pose c = {3_m, 6_m, 45_deg};

  // test c
  EXPECT_TRUE(a.to_relative(b) == c);

  // Expected result if b is referenced relative to a
  c = {-6.3639610306_m, -2.12132034_m, -45_deg};

  // test c
  EXPECT_TRUE(b.to_relative(a) == c);
}