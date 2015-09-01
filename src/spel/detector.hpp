#ifndef _LIBPOSE_DETECTOR_HPP_
#define _LIBPOSE_DETECTOR_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif  // WINDOWS
#include <vector>
#include <map>
#include <string>
#include <exception>
#include <functional>

#include "frame.hpp"
#include "limbLabel.hpp"
#include "spelHelper.hpp"
#include "sequence.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"

namespace SPEL
{
  class Detector
  {
  public:
    Detector(void);
    virtual ~Detector(void);
    virtual int getID(void) const = 0;
    virtual void setID(int _id) = 0;
    virtual void train(vector <Frame*> frames, map <string, float> params) = 0;
    virtual map <uint32_t, vector <LimbLabel> > detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels);
    virtual map <uint32_t, vector <LimbLabel>> merge(map <uint32_t, vector <LimbLabel>> first, map <uint32_t, vector <LimbLabel>> second, map <uint32_t, vector <LimbLabel>> secondUnfiltered);
  protected:
    vector <Frame*> frames;
    uint32_t maxFrameHeight;
    uint8_t debugLevelParam = 0;
    virtual Frame *getFrame(uint32_t frameId);
    virtual float getBoneLength(Point2f begin, Point2f end);
    virtual float getBoneWidth(float length, BodyPart bodyPart);
    virtual POSERECT <Point2f> getBodyPartRect(BodyPart bodyPart, Point2f j0, Point2f j1, Size blockSize = Size(0, 0));
    virtual Mat rotateImageToDefault(Mat imgSource, POSERECT <Point2f> &initialRect, float angle, Size size);
    virtual LimbLabel generateLabel(BodyPart bodyPart, Point2f j0, Point2f j1, string detectorName, float _usedet);
    virtual LimbLabel generateLabel(BodyPart bodyPart, Frame *workFrame, Point2f p0, Point2f p1) = 0;
    virtual LimbLabel generateLabel(float boneLength, float rotationAngle, float x, float y, BodyPart bodyPart, Frame *workFrame);
    virtual float compare(void) = 0;
    virtual vector <LimbLabel> filterLimbLabels(vector <LimbLabel> &sortedLabels, float uniqueLocationCandidates, float uniqueAngleCandidates);
  };
}
#endif  // _LIBPOSE_DETECTOR_HPP_
