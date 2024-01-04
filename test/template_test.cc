#include <gtest/gtest.h>

TEST(DriveTest, BasicAssertions) {
  EXPECT_STRNE("hello", "world");
}