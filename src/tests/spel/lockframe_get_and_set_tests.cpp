#include <gtest/gtest.h>
#include <lockframe.hpp>

namespace SPEL
{
  class LockframeTest : public testing::Test{
  protected:
    //init
    //nothing to init here
    /*virtual void SetUp(){}*/

    //clear
    //nothing to clear here
    /*virtual void TearDown(){}*/
  protected:
    Lockframe lockframe;
  };

  TEST_F(LockframeTest, GetFrametype){
    EXPECT_EQ(FRAMETYPE::LOCKFRAME, lockframe.getFrametype());
  }
}
