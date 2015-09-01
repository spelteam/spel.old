#include <gtest/gtest.h>
#include <skeleton.hpp>

namespace SPEL
{
  TEST(skeletonTest, GetAndSetTest)
  {
    Skeleton s;
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

    s.setName(name);
    s.setPartTree(partTree);
    s.setJointTree(jointTree);
    s.setScale(scale);

    EXPECT_EQ(name, s.getName());
    tree <BodyPart> pt1 = s.getPartTree();
    EXPECT_TRUE(equal(pt1.begin(), pt1.end(), partTree.begin()));
    tree <BodyJoint> jt1 = s.getJointTree();
    EXPECT_TRUE(equal(jt1.begin(), jt1.end(), jointTree.begin()));
    EXPECT_EQ(scale, s.getScale());
    EXPECT_EQ(1, s.getPartTreeCount());
    EXPECT_TRUE(bj == *s.getBodyJoint(bj.getLimbID()));
    EXPECT_TRUE(NULL == s.getBodyJoint(-1));
  }
}
