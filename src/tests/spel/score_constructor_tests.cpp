#include <gtest/gtest.h>
#include <score.hpp>

namespace SPEL
{
  TEST(scoreTest, ConstructorTest)
  {
    Score s1;

    EXPECT_EQ(0, s1.getScore());
    EXPECT_TRUE(s1.getDetName().empty());
    EXPECT_FLOAT_EQ(1.0f, s1.getCoeff());

    float score = 12.5f;
    string detName = "Test Name";
    float coeff = 0.74f;
    Score s2(score, detName, coeff);

    EXPECT_EQ(score, s2.getScore());
    EXPECT_EQ(detName, s2.getDetName());
    EXPECT_FLOAT_EQ(coeff, s2.getCoeff());
  }
}
