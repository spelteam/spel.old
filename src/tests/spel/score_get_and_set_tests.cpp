#include <gtest/gtest.h>
#include <score.hpp>

namespace SPEL
{
  TEST(scoreTest, GetAndSetTest)
  {
    float score = 5.5f;
    string detName = "detName";
    float coeff = 0.25f;

    Score s1;
    s1.setScore(score);
    s1.setDetName(detName);
    s1.setCoeff(coeff);

    EXPECT_EQ(score, s1.getScore());
    EXPECT_EQ(detName, s1.getDetName());
    EXPECT_FLOAT_EQ(coeff, s1.getCoeff());
  }
}
