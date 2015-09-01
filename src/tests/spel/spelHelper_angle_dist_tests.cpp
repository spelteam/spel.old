#include <gtest/gtest.h>
#include "spelHelper.hpp"

namespace SPEL
{
  class spelHelperTest : public testing::Test{
  protected:
    //init
    virtual void SetUp(){
      //test data
      p[0] = Point2f(0.0, 0.0);
      p[1] = Point2f(1.0, 1.0);
      p[2] = Point2f(1.0, 0.0);
      p[3] = Point2f(0.0, 1.0);
      p[4] = Point2f(-1.0, 0.0);
      p[5] = Point2f(3.0, 1.0);
      p[6] = Point2f(2.0, 2.732050808);
      p[7] = Point2f(1.0, 1.732050808);
      p[8] = Point2f(4.0, 6.0);
      p[9] = Point2f(9.0, 4.0);
    }

    //clear
    //nothing to clear here
    //virtual void TearDown(){}
  protected:
    Point2f p[10];

  };

  TEST_F(spelHelperTest, DistSquared){
    EXPECT_DOUBLE_EQ(2.0, spelHelper::distSquared(p[0], p[1]));
    EXPECT_DOUBLE_EQ(1.0, spelHelper::distSquared(p[0], p[2]));
    EXPECT_DOUBLE_EQ(1.0, spelHelper::distSquared(p[0], p[3]));
    EXPECT_DOUBLE_EQ(1.0, spelHelper::distSquared(p[0], p[4]));
    EXPECT_DOUBLE_EQ(0.0, spelHelper::distSquared(p[0], p[0]));
    EXPECT_DOUBLE_EQ(0.0, spelHelper::distSquared(p[6], p[6]));
    EXPECT_DOUBLE_EQ(29.0, spelHelper::distSquared(p[8], p[9]));
    //equilateral triangle p[1], p[5], p[6]
    const double error = 0.00001;
    EXPECT_NEAR(4.0, spelHelper::distSquared(p[1], p[5]), error);
    EXPECT_NEAR(4.0, spelHelper::distSquared(p[1], p[6]), error);
    EXPECT_NEAR(4.0, spelHelper::distSquared(p[5], p[6]), error);

  }

  TEST_F(spelHelperTest, Angle2D){
    EXPECT_DOUBLE_EQ(-M_PI / 4, spelHelper::angle2D(p[1].x, p[1].y, p[2].x, p[2].y));
    EXPECT_DOUBLE_EQ(-M_PI / 2, spelHelper::angle2D(p[3].x, p[3].y, p[2].x, p[2].y));
    EXPECT_DOUBLE_EQ(M_PI / 4, spelHelper::angle2D(p[2].x, p[2].y, p[1].x, p[1].y));
    EXPECT_DOUBLE_EQ(M_PI / 2, spelHelper::angle2D(p[2].x, p[2].y, p[3].x, p[3].y));
    const double error = 0.00001;
    EXPECT_NEAR(M_PI / 3, spelHelper::angle2D(p[2].x, p[2].y, p[7].x, p[7].y), error);
    EXPECT_NEAR(-M_PI / 6, spelHelper::angle2D(p[3].x, p[3].y, p[7].x, p[7].y), error);
    float angle32 = 32.3474435 * (M_PI / 180);
    EXPECT_NEAR(-angle32, spelHelper::angle2D(p[8].x, p[8].y, p[9].x, p[9].y), error);
    //testing with null vector
    EXPECT_DOUBLE_EQ(0.0, spelHelper::angle2D(p[0].x, p[0].y, p[1].x, p[1].y));
    EXPECT_DOUBLE_EQ(0.0, spelHelper::angle2D(p[0].x, p[0].y, p[3].x, p[3].y));
    EXPECT_DOUBLE_EQ(0.0, spelHelper::angle2D(p[0].x, p[0].y, p[0].x, p[0].y));
    //testing with itsefl
    EXPECT_DOUBLE_EQ(0.0, spelHelper::angle2D(p[5].x, p[5].y, p[5].x, p[5].y));
    EXPECT_DOUBLE_EQ(0.0, spelHelper::angle2D(p[8].x, p[8].y, p[8].x, p[8].y));
  }

  TEST_F(spelHelperTest, distSquared3D)
  {
    double b = sqrt(1.0 / 3.0);

    Point3d A = Point3d(0.0, 0.0, 0.0);
    Point3d B = Point3d(b, b, b);
    Point3f C = Point3f(-b, -b, -b);

    EXPECT_DOUBLE_EQ(0.0, spelHelper::distSquared3d(A, A));
    EXPECT_DOUBLE_EQ(1.0, spelHelper::distSquared3d(A, B));
    EXPECT_FLOAT_EQ(1.0, (float)spelHelper::distSquared3d(Point3f(0.0, 0.0, 0.0), C));
  }

  TEST_F(spelHelperTest, angleTest)
  {
    double controlAngle45 = 45;
    double controlAngle0 = 0;
    double controlAngle90 = 90;
    double controlAngle180 = 180;

    double angle45 = spelHelper::angle2D(p[1].x, p[1].y, p[3].x, p[3].y) * (180.0 / M_PI);
    double angle0 = spelHelper::angle2D(p[3].x, p[3].y, p[3].x, p[3].y) * (180.0 / M_PI);
    double angle90 = spelHelper::angle2D(p[2].x, p[2].y, p[3].x, p[3].y) * (180.0 / M_PI);
    double angle180 = spelHelper::angle2D(p[2].x, p[2].y, p[4].x, p[4].y) * (180.0 / M_PI);

    EXPECT_DOUBLE_EQ(controlAngle45, angle45);
    EXPECT_DOUBLE_EQ(controlAngle0, angle0);
    EXPECT_DOUBLE_EQ(controlAngle90, angle90);
    EXPECT_DOUBLE_EQ(controlAngle180, angle180);
  }
}
