#include <gtest/gtest.h>
#include <bodyJoint.hpp>

namespace SPEL
{
  TEST(bodyJointTest, OperatorsTest)
  {
    BodyJoint bj1, bj2, bj3;
    const int limbID = 5;
    const string jointName = "Some Name";
    const Point2f imageLocation(2.534f, -1.635f);
    const Point3f spaceLocation(1.231f, -2.0f, -1.5f);
    const bool depthSign = true;
    bj1.setLimbID(limbID);
    bj1.setJointName(jointName);
    bj1.setImageLocation(imageLocation);
    bj1.setSpaceLocation(spaceLocation);
    bj1.setDepthSign(depthSign);

    bj2.setLimbID(limbID);
    bj2.setJointName(jointName);
    bj2.setImageLocation(imageLocation);
    bj2.setSpaceLocation(spaceLocation);
    bj2.setDepthSign(depthSign);

    EXPECT_TRUE(bj1 == bj2);
    EXPECT_FALSE(bj1 != bj2);

    //Testing operator "="
    bj3 = bj2;
    EXPECT_TRUE(bj3 == bj2);
    EXPECT_EQ(jointName, bj3.getJointName());
    EXPECT_EQ(imageLocation, bj3.getImageLocation());
    EXPECT_EQ(spaceLocation, bj3.getSpaceLocation());
    EXPECT_EQ(depthSign, bj3.getDepthSign());

    //Testing operator "="
    const BodyJoint bj4(limbID, jointName, imageLocation, spaceLocation, depthSign);
    BodyJoint bj5 = bj4;
    EXPECT_TRUE(bj5 == bj4);
    EXPECT_EQ(jointName, bj5.getJointName());
    EXPECT_EQ(imageLocation, bj5.getImageLocation());
    EXPECT_EQ(spaceLocation, bj5.getSpaceLocation());
    EXPECT_EQ(depthSign, bj5.getDepthSign());
  }
}