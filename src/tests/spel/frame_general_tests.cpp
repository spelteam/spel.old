#include <gtest/gtest.h>

//frame and derived from frame interfaces
#include "frame.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"

namespace SPEL
{
  //factory functions for creating implementation
  //for each type

  template<typename T>
  Frame* createFrame();

  template<>
  Frame* createFrame<Keyframe>(){
    return new Keyframe;
  }

  template<>
  Frame* createFrame<Lockframe>(){
    return new Lockframe;
  }

  template<>
  Frame* createFrame<Interpolation>(){
    return new Interpolation;
  }

  //get FRAMETYPE for each derived class
  template<typename T>
  FRAMETYPE getFrametype();

  //declare test fixture
  template<typename T>
  class FrameTest : public testing::Test{
  protected:
    FrameTest() : frame(createFrame<T>()){}
    virtual ~FrameTest() { delete frame; }
  protected:
    //init
    virtual void SetUp(){
      //set id
      id = 7;

      //set image
      Point pt1, pt2;
      pt1.x = 10;
      pt1.y = 10;
      pt2.x = 90;
      pt2.y = 90;

      int icolor = 0x00FFAA11;
      Scalar color_image(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);

      image = Mat::zeros(100, 100, CV_32F);
      line(image, pt1, pt2, color_image, 5, 8);

      //set mask
      pt1.x = 10;
      pt1.y = 90;
      pt2.x = 90;
      pt2.y = 10;

      icolor = 0x00AABBCC;
      Scalar color_mask(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);

      mask = Mat::zeros(100, 100, CV_32F);
      line(mask, pt1, pt2, color_mask, 9, 8);

      //set skeleton
      //nothing

      //set groupPoint
      groundPoint = Point2f(0.0, 0.0);

      //set frametype

      //create frame
      frame->setID(id);
      frame->setImage(image);
      frame->setMask(mask);
      frame->setSkeleton(skeleton);
      frame->setGroundPoint(groundPoint);
    }
    //clear
    //nothing to clear here
    /*virtual void TearDown(){}*/
  protected:
    int id;
    Mat image;
    Mat mask;
    Skeleton skeleton;
    Point2f groundPoint;

    Frame* const frame;
  };

#if GTEST_HAS_TYPED_TEST

  //list of type that want to test
  typedef testing::Types<Keyframe, Lockframe, Interpolation> TypeList;

  TYPED_TEST_CASE(FrameTest, TypeList);

  //declare  general tests

  TYPED_TEST(FrameTest, GetId){
    EXPECT_EQ(this->id, this->frame->getID());
  }

  TYPED_TEST(FrameTest, GetImage){
    EXPECT_EQ(0, cv::countNonZero(this->image != this->frame->getImage()));
    EXPECT_EQ(this->image.rows*this->image.cols, cv::countNonZero(this->image == this->frame->getImage()));
  }

  TYPED_TEST(FrameTest, GetMask){
    EXPECT_EQ(0, cv::countNonZero(this->mask != this->frame->getMask()));
    EXPECT_EQ(this->mask.rows*this->mask.cols, cv::countNonZero(this->mask == this->frame->getMask()));
  }

  TYPED_TEST(FrameTest, GetSkeleton){
    EXPECT_EQ(this->skeleton, this->frame->getSkeleton());
  }

  TYPED_TEST(FrameTest, GetGroundPoint){
    EXPECT_EQ(this->groundPoint, this->frame->getGroundPoint());
  }

#endif
}
