#include <gtest/gtest.h>
#include <spelHelper.hpp>

namespace SPEL
{
  TEST(spelHelperTest_, rotatepointTest)
  {
    Point2f p1 = Point2f(-1.0, 1.0);
    Point2f p2 = Point2f(1.0, 1.0);
    Point2f p3 = Point2f(1.0, -1.0);
    Point2f p4 = Point2f(-1.0, -1.0);

    float angle90 = 90;
    float angle45 = 45;

    Point2f center0 = Point2f(0.0, 0.0);
    Point2f center11 = Point2f(1.0, 1.0);

    Point2f c1p1 = Point2f(-1.0, -1.0);
    Point2f c1p2 = Point2f(-1.0, 1.0);
    Point2f c1p3 = Point2f(1.0, 1.0);
    Point2f c1p4 = Point2f(1.0, -1.0);

    Point2f t1p1 = spelHelper::rotatePoint2D(p1, center0, angle90);
    Point2f t1p2 = spelHelper::rotatePoint2D(p2, center0, angle90);
    Point2f t1p3 = spelHelper::rotatePoint2D(p3, center0, angle90);
    Point2f t1p4 = spelHelper::rotatePoint2D(p4, center0, angle90);

    EXPECT_FLOAT_EQ(c1p1.x, t1p1.x);
    EXPECT_FLOAT_EQ(c1p1.y, t1p1.y);
    EXPECT_FLOAT_EQ(c1p2.x, t1p2.x);
    EXPECT_FLOAT_EQ(c1p2.y, t1p2.y);
    EXPECT_FLOAT_EQ(c1p3.x, t1p3.x);
    EXPECT_FLOAT_EQ(c1p3.y, t1p3.y);
    EXPECT_FLOAT_EQ(c1p4.x, t1p4.x);
    EXPECT_FLOAT_EQ(c1p4.y, t1p4.y);

    Point2f c2p1 = Point2f(-1.4142135623731, 0.0);
    Point2f c2p2 = Point2f(0.0, 1.4142135623731);
    Point2f c2p3 = Point2f(1.4142135623731, 0.0);
    Point2f c2p4 = Point2f(0.0, -1.4142135623731);

    Point2f t2p1 = spelHelper::rotatePoint2D(p1, center0, angle45);
    Point2f t2p2 = spelHelper::rotatePoint2D(p2, center0, angle45);
    Point2f t2p3 = spelHelper::rotatePoint2D(p3, center0, angle45);
    Point2f t2p4 = spelHelper::rotatePoint2D(p4, center0, angle45);

    EXPECT_FLOAT_EQ(c2p1.x, t2p1.x);
    EXPECT_FLOAT_EQ(c2p1.y, t2p1.y);
    EXPECT_FLOAT_EQ(c2p2.x, t2p2.x);
    EXPECT_FLOAT_EQ(c2p2.y, t2p2.y);
    EXPECT_FLOAT_EQ(c2p3.x, t2p3.x);
    EXPECT_FLOAT_EQ(c2p3.y, t2p3.y);
    EXPECT_FLOAT_EQ(c2p4.x, t2p4.x);
    EXPECT_FLOAT_EQ(c2p4.y, t2p4.y);

    Point2f c3p1 = Point2f(0.0, 1.4142135623731);
    Point2f c3p2 = Point2f(1.4142135623731, 0.0);
    Point2f c3p3 = Point2f(0.0, -1.4142135623731);
    Point2f c3p4 = Point2f(-1.4142135623731, 0.0);

    Point2f t3p1 = spelHelper::rotatePoint2D(p1, center0, angle45 * (-1.0));
    Point2f t3p2 = spelHelper::rotatePoint2D(p2, center0, angle45 * (-1.0));
    Point2f t3p3 = spelHelper::rotatePoint2D(p3, center0, angle45 * (-1.0));
    Point2f t3p4 = spelHelper::rotatePoint2D(p4, center0, angle45 * (-1.0));

    EXPECT_FLOAT_EQ(c3p1.x, t3p1.x);
    EXPECT_FLOAT_EQ(c3p1.y, t3p1.y);
    EXPECT_FLOAT_EQ(c3p2.x, t3p2.x);
    EXPECT_FLOAT_EQ(c3p2.y, t3p2.y);
    EXPECT_FLOAT_EQ(c3p3.x, t3p3.x);
    EXPECT_FLOAT_EQ(c3p3.y, t3p3.y);
    EXPECT_FLOAT_EQ(c3p4.x, t3p4.x);
    EXPECT_FLOAT_EQ(c3p4.y, t3p4.y);

    Point2f c4p1 = Point2f(1.0, 1.0);
    Point2f c4p2 = Point2f(1.0, -1.0);
    Point2f c4p3 = Point2f(-1.0, -1.0);
    Point2f c4p4 = Point2f(-1.0, 1.0);

    Point2f t4p1 = spelHelper::rotatePoint2D(p1, center0, angle90 * (-1.0));
    Point2f t4p2 = spelHelper::rotatePoint2D(p2, center0, angle90 * (-1.0));
    Point2f t4p3 = spelHelper::rotatePoint2D(p3, center0, angle90 * (-1.0));
    Point2f t4p4 = spelHelper::rotatePoint2D(p4, center0, angle90 * (-1.0));

    EXPECT_FLOAT_EQ(c4p1.x, t4p1.x);
    EXPECT_FLOAT_EQ(c4p1.y, t4p1.y);
    EXPECT_FLOAT_EQ(c4p2.x, t4p2.x);
    EXPECT_FLOAT_EQ(c4p2.y, t4p2.y);
    EXPECT_FLOAT_EQ(c4p3.x, t4p3.x);
    EXPECT_FLOAT_EQ(c4p3.y, t4p3.y);
    EXPECT_FLOAT_EQ(c4p4.x, t4p4.x);
    EXPECT_FLOAT_EQ(c4p4.y, t4p4.y);

    Point2f c5p1 = Point2f(1.0, -1.0);
    Point2f c5p2 = Point2f(1.0, 1.0);
    Point2f c5p3 = Point2f(3.0, 1.0);
    Point2f c5p4 = Point2f(3.0, -1.0);

    Point2f t5p1 = spelHelper::rotatePoint2D(p1, center11, angle90);
    Point2f t5p2 = spelHelper::rotatePoint2D(p2, center11, angle90);
    Point2f t5p3 = spelHelper::rotatePoint2D(p3, center11, angle90);
    Point2f t5p4 = spelHelper::rotatePoint2D(p4, center11, angle90);

    EXPECT_FLOAT_EQ(c5p1.x, t5p1.x);
    EXPECT_FLOAT_EQ(c5p1.y, t5p1.y);
    EXPECT_FLOAT_EQ(c5p2.x, t5p2.x);
    EXPECT_FLOAT_EQ(c5p2.y, t5p2.y);
    EXPECT_FLOAT_EQ(c5p3.x, t5p3.x);
    EXPECT_FLOAT_EQ(c5p3.y, t5p3.y);
    EXPECT_FLOAT_EQ(c5p4.x, t5p4.x);
    EXPECT_FLOAT_EQ(c5p4.y, t5p4.y);

    Point2f c6p1 = Point2f(-0.4142135623731, -0.4142135623731);
    Point2f c6p2 = Point2f(1.0, 1.0);
    Point2f c6p3 = Point2f(2.4142135623731, -0.4142135623731);
    Point2f c6p4 = Point2f(1.0, -1.8284271247462);

    Point2f t6p1 = spelHelper::rotatePoint2D(p1, center11, angle45);
    Point2f t6p2 = spelHelper::rotatePoint2D(p2, center11, angle45);
    Point2f t6p3 = spelHelper::rotatePoint2D(p3, center11, angle45);
    Point2f t6p4 = spelHelper::rotatePoint2D(p4, center11, angle45);

    EXPECT_FLOAT_EQ(c6p1.x, t6p1.x);
    EXPECT_FLOAT_EQ(c6p1.y, t6p1.y);
    EXPECT_FLOAT_EQ(c6p2.x, t6p2.x);
    EXPECT_FLOAT_EQ(c6p2.y, t6p2.y);
    EXPECT_FLOAT_EQ(c6p3.x, t6p3.x);
    EXPECT_FLOAT_EQ(c6p3.y, t6p3.y);
    EXPECT_FLOAT_EQ(c6p4.x, t6p4.x);
    EXPECT_FLOAT_EQ(c6p4.y, t6p4.y);

    Point2f c7p1 = Point2f(1.0, 3.0);
    Point2f c7p2 = Point2f(1.0, 1.0);
    Point2f c7p3 = Point2f(-1.0, 1.0);
    Point2f c7p4 = Point2f(-1.0, 3.0);

    Point2f t7p1 = spelHelper::rotatePoint2D(p1, center11, angle90 * (-1.0));
    Point2f t7p2 = spelHelper::rotatePoint2D(p2, center11, angle90 * (-1.0));
    Point2f t7p3 = spelHelper::rotatePoint2D(p3, center11, angle90 * (-1.0));
    Point2f t7p4 = spelHelper::rotatePoint2D(p4, center11, angle90 * (-1.0));

    EXPECT_FLOAT_EQ(c7p1.x, t7p1.x);
    EXPECT_FLOAT_EQ(c7p1.y, t7p1.y);
    EXPECT_FLOAT_EQ(c7p2.x, t7p2.x);
    EXPECT_FLOAT_EQ(c7p2.y, t7p2.y);
    EXPECT_FLOAT_EQ(c7p3.x, t7p3.x);
    EXPECT_FLOAT_EQ(c7p3.y, t7p3.y);
    EXPECT_FLOAT_EQ(c7p4.x, t7p4.x);
    EXPECT_FLOAT_EQ(c7p4.y, t7p4.y);

    Point2f c8p1 = Point2f(-0.4142135623731, 2.4142135623731);
    Point2f c8p2 = Point2f(1.0, 1.0);
    Point2f c8p3 = Point2f(-0.4142135623731, -0.4142135623731);
    Point2f c8p4 = Point2f(-1.8284271247462, 1.0);

    Point2f t8p1 = spelHelper::rotatePoint2D(p1, center11, angle45 * (-1.0));
    Point2f t8p2 = spelHelper::rotatePoint2D(p2, center11, angle45 * (-1.0));
    Point2f t8p3 = spelHelper::rotatePoint2D(p3, center11, angle45 * (-1.0));
    Point2f t8p4 = spelHelper::rotatePoint2D(p4, center11, angle45 * (-1.0));

    EXPECT_FLOAT_EQ(c8p1.x, t8p1.x);
    EXPECT_FLOAT_EQ(c8p1.y, t8p1.y);
    EXPECT_FLOAT_EQ(c8p2.x, t8p2.x);
    EXPECT_FLOAT_EQ(c8p2.y, t8p2.y);
    EXPECT_FLOAT_EQ(c8p3.x, t8p3.x);
    EXPECT_FLOAT_EQ(c8p3.y, t8p3.y);
    EXPECT_FLOAT_EQ(c8p4.x, t8p4.x);
    EXPECT_FLOAT_EQ(c8p4.y, t8p4.y);

    Point2f p5 = Point2f(0.0, 2.5871);
    Point2f p6 = Point2f(18.1097, 2.5871);
    Point2f p7 = Point2f(18.1097, -2.5871);
    Point2f p8 = Point2f(0.0, -2.5871);

    Point2f center90 = Point2f(9.05485, 0.0);
    double angle59 = -59.7705;

    Point2f c1p5 = Point2f(6.7313461216324, 9.126046496688);
    Point2f c1p6 = Point2f(15.848943894977, -6.5210185614465);
    Point2f c1p7 = Point2f(11.378353878368, -9.126046496688);
    Point2f c1p8 = Point2f(2.2607561050225, 6.5210185614465);

    Point2f t1p5 = spelHelper::rotatePoint2D(p5, center90, angle59);
    Point2f t1p6 = spelHelper::rotatePoint2D(p6, center90, angle59);
    Point2f t1p7 = spelHelper::rotatePoint2D(p7, center90, angle59);
    Point2f t1p8 = spelHelper::rotatePoint2D(p8, center90, angle59);

    EXPECT_FLOAT_EQ(c1p5.x, t1p5.x);
    EXPECT_FLOAT_EQ(c1p5.y, t1p5.y);
    EXPECT_FLOAT_EQ(c1p6.x, t1p6.x);
    EXPECT_FLOAT_EQ(c1p6.y, t1p6.y);
    EXPECT_FLOAT_EQ(c1p7.x, t1p7.x);
    EXPECT_FLOAT_EQ(c1p7.y, t1p7.y);
    EXPECT_FLOAT_EQ(c1p8.x, t1p8.x);
    EXPECT_FLOAT_EQ(c1p8.y, t1p8.y);

  }
}
