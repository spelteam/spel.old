#include <gtest/gtest.h>
#include <hogDetector.hpp>

namespace SPEL
{
  TEST(HOGDetectorTests, GetAndSetTest)
  {
    HogDetector hd;
    int id = 3;
    hd.setID(id);

    EXPECT_EQ(id, hd.getID());
  }
}
