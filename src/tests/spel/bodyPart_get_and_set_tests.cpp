#include <gtest/gtest.h>
#include <bodyPart.hpp>

namespace SPEL
{
  TEST(bodyPartTest, GetAndSetTest)
  {
    BodyPart bp1;
    int partID = 3;
    string partName = "Part Name";
    int parentJoint = 0;
    int childJoint = 0;
    bool isOccluded = false;
    float expectedDistance = 1.343f;
    POSERECT <Point2f> polygon(Point2f(1.0f, 2.0f), Point2f(2.0f, 3.0f), Point2f(3.0f, 4.0f), Point2f(4.0f, 5.0f));
    float lwRatio = 1.3f;
    float relativeLength = 0.6f;

    float searchRadius = 0.5f;
    float rotationSearchRange = 3.14f;

    bp1.setPartID(partID);
    bp1.setPartName(partName);
    bp1.setParentJoint(parentJoint);
    bp1.setChildJoint(childJoint);
    bp1.setIsOccluded(isOccluded);
    bp1.setExpectedDistance(expectedDistance);
    bp1.setPartPolygon(polygon);
    bp1.setLWRatio(lwRatio);
    bp1.setRelativeLength(relativeLength);
    bp1.setSearchRadius(searchRadius);
    bp1.setRotationSearchRange(rotationSearchRange);


    EXPECT_EQ(partID, bp1.getPartID());
    EXPECT_EQ(partName, bp1.getPartName());
    EXPECT_EQ(parentJoint, bp1.getParentJoint());
    EXPECT_EQ(childJoint, bp1.getChildJoint());
    EXPECT_EQ(isOccluded, bp1.getIsOccluded());
    EXPECT_EQ(expectedDistance, bp1.getExpectedDistance());
    EXPECT_EQ(polygon, bp1.getPartPolygon());
    EXPECT_EQ(lwRatio, bp1.getLWRatio());
    EXPECT_EQ(relativeLength, bp1.getRelativeLength());
    EXPECT_EQ(searchRadius, bp1.getSearchRadius());
    EXPECT_EQ(rotationSearchRange, bp1.getRotationSearchRange());
  }
}
