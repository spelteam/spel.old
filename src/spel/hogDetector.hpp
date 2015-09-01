#ifndef _LIBPOSE_HOGDETECTOR_HPP_
#define _LIBPOSE_HOGDETECTOR_HPP_

// SPEL definitions
#include "predef.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// OpenCV
#include <opencv2/opencv.hpp>

#include "detector.hpp"

namespace SPEL
{
  using namespace std;
  using namespace cv;

  class HogDetector : public Detector
  {
  protected:
    struct PartModel
    {
      POSERECT <Point2f> partModelRect;
      vector <vector <vector <float>>> gradientStrengths;
      Mat partImage;
#ifdef DEBUG
      vector<float> descriptors;
#endif  // DEBUG
    };
  public:
    HogDetector(void);
    virtual ~HogDetector(void);
    virtual int getID(void) const;
    virtual void setID(int _id);
    virtual void train(vector <Frame*> _frames, map <string, float> params);
    virtual map <uint32_t, vector <LimbLabel>> detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels);
    virtual map <uint32_t, map <uint32_t, vector <PartModel>>> getLabelModels(void);
    virtual map <uint32_t, map <uint32_t, PartModel>> getPartModels(void);

    virtual Size getCellSize(void);
    virtual uint8_t getnbins(void);
  private:
#ifdef DEBUG
    FRIEND_TEST(HOGDetectorTests, computeDescriptor);
    FRIEND_TEST(HOGDetectorTests, computeDescriptors);
    FRIEND_TEST(HOGDetectorTests, getMaxBodyPartHeightWidth);
    FRIEND_TEST(HOGDetectorTests, train);
    FRIEND_TEST(HOGDetectorTests, generateLabel);
    FRIEND_TEST(HOGDetectorTests, detect);
    FRIEND_TEST(HOGDetectorTests, compare);
    FRIEND_TEST(HOGDetectorTests, getLabelModels);
    FRIEND_TEST(HOGDetectorTests, getPartModels);
    FRIEND_TEST(HOGDetectorTests, getCellSize);
    FRIEND_TEST(HOGDetectorTests, getNBins);
#endif  // DEBUG
    int id;
  protected:
    const uint8_t nbins = 9;
    map <uint32_t, Size> partSize;
    map <uint32_t, map <uint32_t, PartModel>> partModels;
    map <uint32_t, map <uint32_t, vector <PartModel>>> labelModels;
    bool bGrayImages = false;
    float useHoGdet = 1.0f;
    //TODO(Vitaliy Koshura): Make some of them as detector params
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    Size wndSize = Size(64, 128);
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;
    // Variables for score comparer
    BodyPart *comparer_bodyPart = 0;
    PartModel *comparer_model = 0;

    virtual LimbLabel generateLabel(BodyPart bodyPart, Frame *frame, Point2f j0, Point2f j1);

    virtual map <uint32_t, Size> getMaxBodyPartHeightWidth(vector <Frame*> frames, Size blockSize, float resizeFactor);
    virtual PartModel computeDescriptors(BodyPart bodyPart, Point2f j0, Point2f j1, Mat imgMat, int nbins, Size wndSize, Size blockSize, Size blockStride, Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType);
    virtual map <uint32_t, PartModel> computeDescriptors(Frame *frame, int nbins, Size blockSize, Size blockStride, Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType);
    virtual float compare(BodyPart bodyPart, PartModel partModel, uint8_t nbins);
    virtual float compare(void);
  };
}
#endif  // _LIBPOSE_HOGDETECTOR_HPP_
