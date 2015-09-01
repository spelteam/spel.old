#include <gtest/gtest.h>
#include <score.hpp>

namespace SPEL
{
  TEST(scoreTest, OperatorsTest)
  {
    Score s1(1.0, "CHD", 1.0);
    Score s2(2.0, "CHD", 1.0);
    Score s3(1.0, "CHD", 1.0);
    Score s4(1.0, "Score4", 1.0);

    EXPECT_TRUE(s1 < s2);
    EXPECT_TRUE(s2 > s1);
    EXPECT_TRUE(s3 == s1);
    EXPECT_FALSE(s1 == s2);
    EXPECT_FALSE(s4 == s1);
    EXPECT_TRUE(s1 != s2) << "   1.0*1.0 != 2.0*1.0 must be 'true'\n";
    EXPECT_TRUE(s1 != s4);

    Score s5 = s3;

    EXPECT_EQ(s5.getScore(), s3.getScore());
    EXPECT_EQ(s5.getDetName(), s3.getDetName());
    EXPECT_FLOAT_EQ(s5.getCoeff(), s3.getCoeff());
  }
}
