#include <gtest/gtest.h>
#include <colorHistDetector.hpp>
/*#include "keyframe.hpp"
#include "bodyPart.hpp"
#include "bodyJoint.hpp"
#include "skeleton.cpp"*/

namespace SPEL
{
  TEST(colorHistDetectorTestO, Operators)
  {
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
    ColorHistDetector chd0;
    map <string, float> params;
    chd0.train(frames, params);
    frames.clear();

    // Using operator "="
    ColorHistDetector copyed_chd = chd0;

    // Get "chd" frames and compare
    vector <Frame*> actual_frames = copyed_chd.getFrames();
    for (int i = 0; i < frames.size(); i++)
      EXPECT_EQ(Vec3b(i, i, i), actual_frames[i]->getImage().at<Vec3b>(0, 0));
  }
}
