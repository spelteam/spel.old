#include <gtest/gtest.h>
#include <colorHistDetector.hpp>
#include "keyframe.hpp"
#include "bodyPart.hpp"
#include "bodyJoint.hpp"
#include "skeleton.cpp"

namespace SPEL
{
  TEST(colorHistDetectorTest, GetAndSetTest)
  {
    ColorHistDetector chd;
    int id = 3;
    chd.setID(id);

    EXPECT_EQ(id, chd.getID());

    // get a constant
    uint8_t nBins = 8;
    EXPECT_EQ(nBins, chd.getNBins());

    nBins = 7;
    ColorHistDetector chd1(nBins);
    EXPECT_EQ(nBins, chd1.getNBins());

    //TODO (Vitaliy Koshura): get this tests to private methods. Now just comment it.
    /*

    // we can't test setter so test only getter (setter will be automatically tested by other tests. I hope so...)
    // should return zero because of clean class without partHistogram.
    EXPECT_EQ(0, chd.getSizeFG());
    */

    // Testing function "getFrames"

    // Create frames
    int framesCount = 4, rows = 30, cols = 40;
    vector <Frame*> frames;
    for (int i = 0; i < framesCount; i++)
    {
      Mat image = Mat(Size(cols, rows), CV_8UC3, Scalar(i, i, i));
      Frame * frame = new Keyframe();
      frames.push_back(frame);
      frames[i]->setID(i);
      frames[i]->setImage(image);
      frames[i]->setMask(image);
      image.release();
    }

    // Create empty skeleton
    Skeleton skeleton;
    tree<BodyPart> partTree;
    tree<BodyJoint> jointTree;
    skeleton.setPartTree(partTree);
    skeleton.setJointTree(jointTree);

    // Run "train" for setting frames in "chd"
    map <string, float> params;
    chd.train(frames, params);
    frames.clear();

    // Get "chd" frames and compare
    vector <Frame*> actual_frames = chd.getFrames();
    for (int i = 0; i < frames.size(); i++)
      EXPECT_EQ(Vec3b(i, i, i), actual_frames[i]->getImage().at<Vec3b>(0, 0));

  }
}
