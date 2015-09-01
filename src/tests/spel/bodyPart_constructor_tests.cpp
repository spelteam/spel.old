#include <gtest/gtest.h>
#include <bodyPart.hpp>

namespace SPEL
{
  TEST(bodyPartTest, ConstructorTest)
  {
    BodyPart bp1;
    EXPECT_EQ(0, bp1.getPartID());
    EXPECT_TRUE(bp1.getPartName().empty());
    EXPECT_EQ(0, bp1.getParentJoint());
    EXPECT_EQ(0, bp1.getChildJoint());
    EXPECT_FALSE(bp1.getIsOccluded());
    EXPECT_EQ(0, bp1.getExpectedDistance());

    int partID = 3;
    string partName = "Part Name";
    int parentJoint = 0;
    int childJoint = 0;
    bool isOccluded = true;
    float spaceLength = 1.343f;

    BodyPart bp2(partID, partName, parentJoint, childJoint);
    EXPECT_EQ(partID, bp2.getPartID());
    EXPECT_EQ(partName, bp2.getPartName());
    EXPECT_EQ(parentJoint, bp2.getParentJoint());
    EXPECT_EQ(childJoint, bp2.getChildJoint());
    EXPECT_FALSE(bp2.getIsOccluded());
    EXPECT_EQ(0, bp2.getExpectedDistance());

    BodyPart bp3(partID, partName, parentJoint, childJoint, isOccluded);
    EXPECT_EQ(partID, bp3.getPartID());
    EXPECT_EQ(partName, bp3.getPartName());
    EXPECT_EQ(parentJoint, bp3.getParentJoint());
    EXPECT_EQ(childJoint, bp3.getChildJoint());
    EXPECT_TRUE(bp3.getIsOccluded());
    EXPECT_EQ(0, bp3.getExpectedDistance());

    BodyPart bp4(partID, partName, parentJoint, childJoint, isOccluded, spaceLength);
    EXPECT_EQ(partID, bp4.getPartID());
    EXPECT_EQ(partName, bp4.getPartName());
    EXPECT_EQ(parentJoint, bp4.getParentJoint());
    EXPECT_EQ(childJoint, bp4.getChildJoint());
    EXPECT_TRUE(bp4.getIsOccluded());
    EXPECT_EQ(spaceLength, bp4.getExpectedDistance());

    //Testing constructor "BodyPart()"
    BodyPart bp5(bp4);
    EXPECT_EQ(partID, bp5.getPartID());
    EXPECT_EQ(partName, bp5.getPartName());
    EXPECT_EQ(parentJoint, bp5.getParentJoint());
    EXPECT_EQ(childJoint, bp5.getChildJoint());
    EXPECT_TRUE(bp5.getIsOccluded());
    EXPECT_EQ(spaceLength, bp5.getExpectedDistance());
    EXPECT_FALSE(&bp4 == &bp5);
  }
}
