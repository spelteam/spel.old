#include <gtest/gtest.h>
#include <interpolation.hpp>

namespace SPEL
{
  class InterpolationTest : public testing::Test{
  protected:
    //init
    //nothing to init here
    /*virtual void SetUp(){}*/

    //clear
    //nothing to clear here
    /*virtual void TearDown(){}*/
  protected:
    Interpolation interpolation;
  };

  TEST_F(InterpolationTest, GetFrametype){
    EXPECT_EQ(FRAMETYPE::INTERPOLATIONFRAME, interpolation.getFrametype());
  }
}
