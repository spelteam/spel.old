#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include "lockframe.hpp"
#include "keyframe.hpp"
#include "bodyJoint.hpp"
#include "bodyPart.hpp"
#include "skeleton.hpp"
#include "projectLoader.hpp"

namespace SPEL
{
  TEST(FramesTests, Constructors)
  {
    Frame* frame = new Lockframe();
    EXPECT_EQ(LOCKFRAME, frame->getFrametype());
    EXPECT_EQ(-1, frame->getParentFrameID());
    EXPECT_EQ(-1, frame->getID());
    EXPECT_EQ(Point2f(0, 0), frame->getGroundPoint());
    delete frame;

    frame = new Keyframe();
    EXPECT_EQ(KEYFRAME, frame->getFrametype());
    EXPECT_EQ(-1, frame->getParentFrameID());
    EXPECT_EQ(-1, frame->getID());
    EXPECT_EQ(Point2f(0, 0), frame->getGroundPoint());
    delete frame;
  }

  TEST(FramesTests, FramePointerComparer)
  {
    vector<Frame*> frames;
    for (int i = 0; i < 9; i++)
    {
      frames.push_back(new Lockframe());
      frames[i]->setID(rand());
    }
    frames.push_back(new Keyframe());
    sort(frames.begin(), frames.end(), Frame::FramePointerComparer);
    bool FramesIsSorted = true;
    int id = frames[0]->getID();
    for (int i = 0; i < frames.size(); i++)
    {
      if (id > frames[i]->getID())
        FramesIsSorted = false;
      id = frames[i]->getID();
    }
    EXPECT_TRUE(FramesIsSorted);
    for (int i = 0; i < frames.size(); i++)
      delete frames[i];
    frames.clear();
  }

  TEST(FramesTests, GetAndSet)
  {
    // Prepare input data
    int rows = 10, cols = 10;
    Mat image, mask;

    Point2f p0(10, 2), p1(10, 18);
    int LimbLength = p1.y - p0.y;
    POSERECT<Point2f> partPolygon(Point2f(6, 2), Point2f(6, 18), Point2f(14, 18), Point2f(14, 2));

    //Create body joints
    BodyJoint j0(0, "", p0, { 0, 0, 0 }, false);
    BodyJoint j1(1, "", p1, { 0, 0, 0 }, false);
    tree<BodyJoint> jointsTree;
    tree<BodyJoint>::iterator j = jointsTree.begin();
    j = jointsTree.insert(j, j0);
    j = jointsTree.insert(j, j1);
    //Create bodyParts
    int id = 0;
    BodyPart bodyPart(id, "", 0, 1, false, LimbLength);
    bodyPart.setPartPolygon(partPolygon);
    tree<BodyPart> partsTree;
    tree<BodyPart>::iterator p = partsTree.begin();
    p = partsTree.insert(p, bodyPart);
    //Create sketeton
    Skeleton skeleton;
    skeleton.setJointTree(jointsTree);
    skeleton.setPartTree(partsTree);

    //Testing
    Frame* frame = new Lockframe();

    //Lockframe "get-" and "setID"
    frame->setID(1);
    EXPECT_EQ(1, frame->getID());

    //Lockframe "get-" and "setParentFrameID"
    frame->setParentFrameID(0);
    EXPECT_EQ(0, frame->getParentFrameID());

    //Lockframe "get-" and "setGroundPoint"
    frame->setGroundPoint(Point2f(0, 0.4));
    EXPECT_EQ(Point2f(0, 0.4), frame->getGroundPoint());

    //Lockframe "get-" and "setImage"
    image = Mat(Size(cols, rows), CV_8UC3, Scalar(0, 0, 0));
    frame->setImage(image);
    EXPECT_EQ(image.size(), frame->getImage().size());

    //Lockframe "get-" and "setMask"
    mask = Mat(Size(cols, rows), CV_8UC1, Scalar(0));
    frame->setMask(mask);
    EXPECT_EQ(image.size(), frame->getMask().size());

    //Lockframe "get-" and "setSkeleton"
    frame->setSkeleton(skeleton);
    EXPECT_EQ(skeleton, frame->getSkeleton());

    //Lockframe "get-" and "setSkeletonPtr"
    EXPECT_EQ(skeleton, *frame->getSkeletonPtr());

    //Lockframe "getPartPolygon"
    EXPECT_EQ(partPolygon.asVector(), frame->getPartPolygon(id));
    EXPECT_EQ(vector <Point2f>(), frame->getPartPolygon(-1));

    image.release();
    mask.release();
    delete frame;

    frame = new Keyframe();

    //Keyframe "get-" and "setID"
    frame->setID(1);
    EXPECT_EQ(1, frame->getID());

    //Keyframe "get-" and "setParentFrameID"
    frame->setParentFrameID(0);
    EXPECT_EQ(0, frame->getParentFrameID());

    //Keyframe "get-" and "setGroundPoint"
    frame->setGroundPoint(Point2f(0, 0.4));
    EXPECT_EQ(Point2f(0, 0.4), frame->getGroundPoint());

    //Keyframe "get-" and "setImage"
    image = Mat(Size(cols, rows), CV_8UC3, Scalar(0, 0, 0));
    frame->setImage(image);
    EXPECT_EQ(image.size(), frame->getImage().size());

    //Keyframe "get-" and "setMask"
    mask = Mat(Size(cols, rows), CV_8UC1, Scalar(0));
    frame->setMask(mask);
    EXPECT_EQ(image.size(), frame->getMask().size());

    //Keyframe "get-" and "setSkeleton"
    frame->setSkeleton(skeleton);
    EXPECT_EQ(skeleton, frame->getSkeleton());

    //Lockframe "get-" and "setSkeletonPtr"
    EXPECT_EQ(skeleton, *frame->getSkeletonPtr());

    //Keyframe "getPartPolygon"
    EXPECT_EQ(partPolygon.asVector(), frame->getPartPolygon(id));
    EXPECT_EQ(vector <Point2f>(), frame->getPartPolygon(-1));

    image.release();
    mask.release();
    delete frame;
  }

  map<int, Point2f> getImageLocations_ft(Skeleton skeleton)
  {
    map<int, Point2f> Locations;
    tree <BodyJoint> jointTree = skeleton.getJointTree();
    for (tree <BodyJoint>::iterator i = jointTree.begin(); i != jointTree.end(); ++i)
      Locations.emplace(pair<int, Point2f>(i->getLimbID(), i->getImageLocation()));
    return Locations;
  }

  TEST(FramesTests, shiftSkeleton2D)
  {
    String FilePath;
    FilePath = "speltests_TestData/CHDTrainTestData/";

#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/speltests_TestData/CHDTrainTestData/";
#endif

    //Load the input data
    ProjectLoader projectLoader(FilePath);
    projectLoader.Load(FilePath + "trijumpSD_50x41.xml");
    vector<Frame*> frames = projectLoader.getFrames();

    //Copy skeleton from keyframe
    Frame* frame = frames[0];
    Skeleton skeleton = frame->getSkeleton();
    tree<BodyPart> PartTree = skeleton.getPartTree();

    //Create shifted points vector 
    Point2f shift(10, 10);
    map<int, Point2f> locations_expected = getImageLocations_ft(skeleton);
    for (int i = 0; i < locations_expected.size(); i++)
      locations_expected[i] += shift;

    //Run "shiftSkeleton2D"
    frame->shiftSkeleton2D(shift);
    map<int, Point2f> locations_actual = getImageLocations_ft(frame->getSkeleton());

    EXPECT_EQ(locations_expected, locations_actual) << " Skeleton shift error?!\n* Impact of 'skeleton.infer3D' not considered in this test!";
    //Impact of "skeleton.infer3D" in "shiftSkeleton2D" not considered in this test!;
  }
  /**/
  TEST(FramesTests, Resize)
  {
    String FilePath;
    FilePath = "speltests_TestData/CHDTrainTestData/";

#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/speltests_TestData/CHDTrainTestData/";
#endif

    //Load the input data
    ProjectLoader projectLoader(FilePath);
    projectLoader.Load(FilePath + "trijumpSD_50x41.xml");
    vector<Frame*> frames = projectLoader.getFrames();

    //Copy skeleton from keyframe
    Frame* frame = frames[0];
    Skeleton skeleton = frame->getSkeleton();
    tree<BodyPart> PartTree = skeleton.getPartTree();

    // Set resize coefficient and new image height
    float X = 2.0f;
    Mat image = frame->getImage().clone();
    int oldHeight = image.size().height;
    uint32_t newHeight = X*oldHeight;

    //Create scalling points vector 
    Point2f shift(10, 10);
    map<int, Point2f> locations_expected = getImageLocations_ft(skeleton);
    for (int i = 0; i < locations_expected.size(); i++)
      locations_expected[i] *= X;

    //Run "Resize"
    frame->Resize(newHeight);
    map<int, Point2f> locations_actual = getImageLocations_ft(frame->getSkeleton());

    //Compare joints locations
    EXPECT_EQ(locations_expected, locations_actual) << " Skeleton resize error?!\n* Impact of 'skeleton.infer3D' not considered in this test!";
    //Impact of "skeleton.infer3D" in "shiftSkeleton2D" not considered in this test!;

    //Compare image size
    Size oldSize = image.size();
    Size newImageSize = frame->getImage().size();
    Size newMaskSize = frame->getMask().size();
    EXPECT_EQ(oldSize.height*X, newImageSize.height);
    EXPECT_EQ(oldSize.width*X, newImageSize.width);
    EXPECT_EQ(oldSize.height*X, newMaskSize.height);
    EXPECT_EQ(oldSize.width*X, newMaskSize.width);

    //Compare parts search radius
    map <int, float> expected_searchRadius;
    for (tree<BodyPart>::iterator p = PartTree.begin(); p != PartTree.end(); ++p)
      expected_searchRadius.emplace(pair<int, float>(p->getPartID(), p->getSearchRadius()*X));

    map <int, float> actual_searchRadius;
    tree<BodyPart> newPartTree = frame->getSkeleton().getPartTree();
    for (tree<BodyPart>::iterator p = newPartTree.begin(); p != newPartTree.end(); ++p)
      actual_searchRadius.emplace(pair<int, float>(p->getPartID(), p->getSearchRadius()*X));

    EXPECT_EQ(expected_searchRadius, actual_searchRadius);
  }

  TEST(FramesTests, Clone)
  {
    String FilePath;
    FilePath = "speltests_TestData/CHDTrainTestData/";

#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/speltests_TestData/CHDTrainTestData/";
#endif

    //Load the input data
    ProjectLoader projectLoader(FilePath);
    projectLoader.Load(FilePath + "trijumpSD_50x41.xml");
    vector<Frame*> frames = projectLoader.getFrames();

    //Copy keyframe
    Frame* frame0 = frames[0];
    Mat image0 = frame0->getImage();
    Mat mask0 = frame0->getMask();
    imwrite("image_temp.bmp", image0);
    imwrite("mask_temp.bmp", image0);
    image0 = imread("image_temp.bmp");
    mask0 = imread("mask_temp.bmp");

    //Clone "frame0"
    Frame* frame1 = frame0->clone(frame0);

    //Compare 
    EXPECT_EQ(frame0->getID(), frame1->getID());
    EXPECT_EQ(frame0->getGroundPoint(), frame1->getGroundPoint());
    EXPECT_EQ(frame0->getParentFrameID(), frame1->getParentFrameID());
    EXPECT_EQ(frame0->getSkeleton(), frame1->getSkeleton());

    Mat image1 = frame1->getImage();
    Mat mask1 = frame1->getMask();
    bool ImagesIsEqual = true, MasksIsEqual = true;
    Size size = frame0->getImage().size();
    for (int y = 0; y < size.height - 1; y++)
      for (int x = 0; x < size.width - 1; x++)
      {
        ImagesIsEqual = (image0.at<Vec3b>(y, x) == image0.at<Vec3b>(y, x));
        MasksIsEqual = (mask0.at<Vec3b>(y, x) == mask0.at<Vec3b>(y, x));
      }
    EXPECT_TRUE(ImagesIsEqual);
    EXPECT_TRUE(MasksIsEqual);

    image0.release();
    mask0.release();
    image1.release();
    mask1.release();

  }
}
