#include <gtest/gtest.h>
#include <spelHelper.hpp>
#include <bodyPart.hpp>
#include <frame.hpp>
#include "TestsFunctions.hpp"

namespace SPEL
{
  class PoseRectTest : public testing::Test{
  protected:
    //init
    virtual void SetUp(){
      //rectangle points
      p[0] = Point2f(1.0, 2.0);
      p[1] = Point2f(2.0, 2.0);
      p[2] = Point2f(2.0, 1.0);
      p[3] = Point2f(1.0, 1.0);
      p[4] = Point2f(-50.0, -10.0);
      p[5] = Point2f(20.0, -10.0);
      p[6] = Point2f(-50.0, 10.0);
      p[7] = Point2f(20.0, 10.0);
      p[8] = Point2f(4.0, 6.0);
      p[9] = Point2f(9.0, 4.0);
      //test points
      p[10] = Point2f(1.5, 1.5);
      p[11] = Point2f(15.0, 10.0);
      p[12] = Point2f(100.0, 100.0);
      p[13] = Point2f(-20.0, 9.28);
      p[14] = Point2f(-15.0, 10.0);
      //first rectangle
      rect1 = POSERECT<Point2f>(p[0], p[1], p[2], p[3]);
      rect3 = POSERECT<Point2f>(p[0], p[1], p[2], p[3]);
      //second rectangle
      rect2 = POSERECT<Point2f>(p[4], p[5], p[6], p[7]);
      rect4 = POSERECT<Point2f>(p[4], p[5], p[6], p[7]);
      //third rectangle
      rect5 = POSERECT<Point2f>(p[6], p[7], p[9], p[8]);

    }

    //clear
    //nothing to clear here
    //virtual void TearDown(){}
  protected:
    Point2f p[15];
    POSERECT<Point2f> rect1, rect2, rect3, rect4, rect5;
  };

  TEST_F(PoseRectTest, ContainsPoint){
    EXPECT_EQ(1, rect1.containsPoint(p[10]));
    EXPECT_EQ(0, rect2.containsPoint(p[11]));
    EXPECT_EQ(-1, rect3.containsPoint(p[12]));
    EXPECT_EQ(1, rect5.containsPoint(p[13]));
    EXPECT_EQ(0, rect5.containsPoint(p[14]));

    EXPECT_EQ(-1, rect1.containsPoint(p[13]));
    EXPECT_EQ(-1, rect3.containsPoint(p[14]));
  }

  TEST_F(PoseRectTest, AsVector){
    std::vector<Point2f> temp = { p[0], p[1], p[2], p[3] };
    EXPECT_EQ(temp, rect1.asVector());
    EXPECT_EQ(4, rect1.asVector().size());
    EXPECT_EQ(temp, rect3.asVector());
    EXPECT_EQ(4, rect3.asVector().size());
    temp = { p[6], p[7], p[9], p[8] };
    EXPECT_EQ(temp, rect5.asVector());
    EXPECT_EQ(4, rect5.asVector().size());
  }

  TEST_F(PoseRectTest, Equality){
    //between rectangles
    EXPECT_TRUE(rect1 == rect3);
    EXPECT_TRUE(rect2 == rect4);
    EXPECT_TRUE(rect1 != rect2);
    EXPECT_TRUE(rect3 != rect4);
    EXPECT_TRUE(rect1 != rect5);
    EXPECT_TRUE(rect2 != rect5);
    EXPECT_TRUE(rect3 != rect5);
    EXPECT_TRUE(rect4 != rect5);
    //between components of rectangle
    EXPECT_EQ(p[0], rect1.point1);
    EXPECT_EQ(p[1], rect1.point2);
    EXPECT_EQ(p[2], rect1.point3);
    EXPECT_EQ(p[3], rect1.point4);

    EXPECT_EQ(p[0], rect3.point1);
    EXPECT_EQ(p[1], rect3.point2);
    EXPECT_EQ(p[2], rect3.point3);
    EXPECT_EQ(p[3], rect3.point4);

    EXPECT_EQ(p[4], rect2.point1);
    EXPECT_EQ(p[5], rect2.point2);
    EXPECT_EQ(p[6], rect2.point3);
    EXPECT_EQ(p[7], rect2.point4);

    EXPECT_EQ(p[4], rect4.point1);
    EXPECT_EQ(p[5], rect4.point2);
    EXPECT_EQ(p[6], rect4.point3);
    EXPECT_EQ(p[7], rect4.point4);

    EXPECT_EQ(p[6], rect5.point1);
    EXPECT_EQ(p[7], rect5.point2);
    EXPECT_EQ(p[9], rect5.point3);
    EXPECT_EQ(p[8], rect5.point4);
  }

  TEST_F(PoseRectTest, GetMinMaxXY)
  {
    float Xmin, Xmax, Ymin, Ymax;
    rect5.GetMinMaxXY(Xmin, Ymin, Xmax, Ymax);
    EXPECT_EQ(p[6].x, Xmin);
    EXPECT_EQ(p[7].x, Xmax);
    EXPECT_EQ(p[9].y, Ymin);
    EXPECT_EQ(p[6].y, Ymax);
  }

  TEST_F(PoseRectTest, GetCenter)
  {
    Point2f center = rect1.GetCenter<Point2f>();
    EXPECT_EQ(center, 0.25*(p[0] + p[1] + p[2] + p[3]));
  }

  TEST_F(PoseRectTest, AssignmentOperator)
  {
    POSERECT<Point2f> rect = rect1;
    EXPECT_EQ(rect1, rect);
  }

  TEST_F(PoseRectTest, RectSize)
  {
    Point2f rectSize = rect1.RectSize<Point2f>();
    EXPECT_EQ(Point2f(1.0, 1.0), rectSize);
  }

  TEST(spelHelperTests_, CopyTree)
  {
    //Loading part tree from existing project
    tree<BodyPart> partTree, copy;
    vector<Frame*> frames = LoadTestProject("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    partTree = frames[0]->getSkeleton().getPartTree();

    spelHelper::copyTree(copy, partTree);

    ASSERT_EQ(partTree.size(), copy.size());
    
    tree<BodyPart>::pre_order_iterator a, b;
    a = partTree.begin();
    b = copy.begin();
    while (a != partTree.end())
    {
      EXPECT_EQ(*a, *b);
      a++;
      b++;
    }

    /* tree<BodyPart>::breadth_first_iterator c, d;
    c = partTree.begin();
    d = copy.begin();
    while (c != partTree.end())
    {
      EXPECT_EQ(*c, *d);
      c++;
      d++;
    }*/
  }

}
