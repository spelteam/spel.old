#ifndef _LIBPOSE_FRAME_HPP_
#define _LIBPOSE_FRAME_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

#include "skeleton.hpp"

namespace SPEL
{
  using namespace std;
  using namespace cv;

  enum FRAMETYPE
  {
    UNDEFINED = -1,
    KEYFRAME = 0,
    LOCKFRAME = 1,
    INTERPOLATIONFRAME = 2
  };

  class Frame
  {
  public:
    Frame(void);
    Frame(FRAMETYPE _frametype);
    virtual ~Frame(void);
    virtual vector <Point2f> getPartPolygon(int partID) const;
    virtual int getID(void) const;
    virtual void setID(int _id);
    virtual Mat getImage(void) const;
    virtual void setImage(Mat _image);
    virtual Mat getMask(void) const;
    virtual void setMask(Mat _mask);
    virtual Skeleton getSkeleton(void) const;
    //TODO: [!]Refactor getters for direct access
    virtual Skeleton* getSkeletonPtr();
    virtual void shiftSkeleton2D(Point2f shift);
    virtual void setSkeleton(Skeleton _skeleton);
    virtual Point2f getGroundPoint(void) const;
    virtual void setGroundPoint(Point2f _groundPoint);
    virtual FRAMETYPE getFrametype(void) const;
    virtual int getParentFrameID(void) const;
    virtual void setParentFrameID(int _parentFrameID);
    virtual float Resize(uint32_t maxHeight);
    virtual Frame *clone(Frame *dest);
    virtual Size getFrameSize(void) const;
    virtual Size getImageSize(void) const;
    virtual Size getMaskSize(void) const;
    static bool FramePointerComparer(Frame *frame1, Frame *frame2);
  private:
    int id = -1;
    Mat image;
    Mat mask;
    Skeleton skeleton;
    Point2f groundPoint = Point2f(0.0, 0.0);
    int parentFrameID = -1; //the ID of the frame this lockframe was derived from
    Size imageSize = Size(-1, -1);
    Size maskSize = Size(-1, -1);
    FRAMETYPE frametype = UNDEFINED;
  };
}
#endif  // _LIBPOSE_FRAME_HPP_

