#ifndef _LIBPOSE_COLORHISTDETECTOR_HPP_
#define _LIBPOSE_COLORHISTDETECTOR_HPP_

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
  class ColorHistDetector : public Detector
  {
  protected:
    struct PartModel
    {
      PartModel(uint8_t _nBins = 8);
      uint8_t nBins;
      vector <vector <vector <float>>> partHistogram;
      vector <vector <vector <float>>> bgHistogram;
      uint32_t sizeFG;
      uint32_t sizeBG;
      uint32_t fgNumSamples;
      uint32_t bgNumSamples;
      vector <uint32_t> fgSampleSizes;
      vector <uint32_t> bgSampleSizes;
      vector <uint32_t> fgBlankSizes;
      virtual PartModel &operator=(const PartModel &model);
    };
  public:
    ColorHistDetector(uint8_t _nBins = 8);  // default is 8 for 32 bit colourspace
    virtual ~ColorHistDetector(void);
    virtual int getID(void) const;
    virtual void setID(int _id);
    virtual void train(vector <Frame*> _frames, map <string, float> params);
    virtual map <uint32_t, vector <LimbLabel> > detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels);
    virtual uint8_t getNBins(void) const;
    virtual vector <Frame*> getFrames(void) const;
    virtual ColorHistDetector &operator=(const ColorHistDetector &c);
  private:
#ifdef DEBUG
    FRIEND_TEST(colorHistDetectorTest, Constructors);
    FRIEND_TEST(colorHistDetectorTest, computePixelBelongingLikelihood);
    FRIEND_TEST(colorHistDetectorTest, Operators);
    FRIEND_TEST(colorHistDetectorTest, setPartHistogram);
    FRIEND_TEST(colorHistDetectorTest, addpartHistogram);
    FRIEND_TEST(colorHistDetectorTest, getAvgSampleSizeFg);
    FRIEND_TEST(colorHistDetectorTest, getAvgSampleSizeFgBetween);
    FRIEND_TEST(colorHistDetectorTest, matchPartHistogramsED);
    FRIEND_TEST(colorHistDetectorTest, addBackgroundHistogram);
    FRIEND_TEST(colorHistDetectorTest, buildPixelDistributions);
    FRIEND_TEST(colorHistDetectorTest, BuildPixelLabels);
    FRIEND_TEST(colorHistDetectorTest, generateLabel);
    FRIEND_TEST(colorHistDetectorTest, detect);
    FRIEND_TEST(colorHistDetectorTest, Train);
#endif  // DEBUG
    int id;
  protected:
    const uint8_t nBins;
    map <int32_t, PartModel> partModels;
    float useCSdet = 1.0f;
    map <int32_t, Mat> pixelDistributions;
    map <int32_t, Mat> pixelLabels;

    virtual float computePixelBelongingLikelihood(const PartModel &partModel, uint8_t r, uint8_t g, uint8_t b);
    virtual void setPartHistogram(PartModel &partModel, const vector <Point3i> &partColors);
    virtual void addPartHistogram(PartModel &partModel, const vector <Point3i> &partColors, uint32_t nBlankPixels);
    virtual void addBackgroundHistogram(PartModel &partModel, const vector <Point3i> &bgColors);
    virtual float getAvgSampleSizeFg(const PartModel &partModel);
    virtual float getAvgSampleSizeFgBetween(const PartModel &partModel, uint32_t s1, uint32_t s2);
    virtual float matchPartHistogramsED(const PartModel &partModelPrev, const PartModel &partModel);
    virtual map <int32_t, Mat> buildPixelDistributions(Frame *frame);
    virtual map <int32_t, Mat> buildPixelLabels(Frame *frame, map <int32_t, Mat> pixelDistributions);
    virtual LimbLabel generateLabel(BodyPart bodyPart, Frame *frame, Point2f j0, Point2f j1);

    // Variables for score comparer
    BodyPart *comparer_bodyPart = 0;
    Frame **comparer_frame = 0;
    Point2f *comparer_j0 = 0;
    Point2f *comparer_j1 = 0;

    virtual float compare(void);
    virtual float compare(BodyPart bodyPart, Frame *frame, map <int32_t, Mat> pixelDistributions, map <int32_t, Mat> pixelLabels, Point2f j0, Point2f j1);
  };
}
#endif  // _LIBPOSE_COLORHISTDETECTOR_HPP_

