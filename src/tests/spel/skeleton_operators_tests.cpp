#include <gtest/gtest.h>
#include <skeleton.hpp>

namespace SPEL
{
  TEST(skeletonTest, OperatorsTest)
  {
    Skeleton s1;
    string name = "Skeleton Test";
    tree <BodyPart> partTree;
    tree <BodyPart>::iterator pti;
    BodyPart bp;
    pti = partTree.begin();
    partTree.insert(pti, bp);
    tree <BodyJoint> jointTree;
    BodyJoint bj;
    tree <BodyJoint>::iterator jti;
    jti = jointTree.begin();
    jointTree.insert(jti, bj);
    float scale = 0.5;

    s1.setName(name);
    s1.setPartTree(partTree);
    s1.setJointTree(jointTree);
    s1.setScale(scale);

    Skeleton s2 = s1;

    EXPECT_TRUE(s1 == s2);
    EXPECT_FALSE(s1 != s2);

    EXPECT_EQ(s1.getName(), s2.getName());
    tree <BodyPart> pt1 = s1.getPartTree();
    tree <BodyPart> pt2 = s2.getPartTree();
    EXPECT_TRUE(equal(pt1.begin(), pt1.end(), pt2.begin()));
    tree <BodyJoint> jt1 = s1.getJointTree();
    tree <BodyJoint> jt2 = s2.getJointTree();
    EXPECT_TRUE(equal(jt1.begin(), jt1.end(), jt2.begin()));
    EXPECT_EQ(s1.getScale(), s2.getScale());
  }
}
