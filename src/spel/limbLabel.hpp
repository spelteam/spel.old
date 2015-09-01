#ifndef _LIBPOSE_LIMBLABEL_HPP_
#define _LIBPOSE_LIMBLABEL_HPP_

// SPEL definitions
#include "predef.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// STL
#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif  // WINDOWS
#include <vector>
#include <string>
#include <cstdint>

// OpenCV
#include <opencv2/opencv.hpp>

#include "score.hpp"

namespace SPEL
{
  using namespace std;
  using namespace cv;

  class LimbLabel
  {
  public:
    LimbLabel();
    LimbLabel(const LimbLabel& ll);
    LimbLabel(int _id, Point2f _centre, float _angle, vector<Point2f> _polygon, vector<Score> _scores, bool _isOccluded = false);
    virtual ~LimbLabel(void);
    /// output labels as printable string
    virtual string toString();
    virtual LimbLabel & operator = (const LimbLabel &ll);
    virtual bool operator == (const LimbLabel &ll) const;
    virtual bool operator != (const LimbLabel &ll) const;
    virtual bool operator < (const LimbLabel &ll) const;
    virtual bool operator > (const LimbLabel &ll) const;
    // compute the endpoints of the limb that this label would produce
    virtual void getEndpoints(Point2f &p0, Point2f &p1) const;
    virtual void addScore(Score detectionScore);
    virtual Point2f getCenter(void) const;
    ///get the label scores
    virtual vector<Score> getScores(void) const;
    virtual void setScores(vector <Score> _scores);
    virtual int getLimbID(void) const;
    virtual float getAngle(void) const;
    virtual vector <Point2f> getPolygon(void) const;
    virtual bool getIsOccluded(void) const;
    virtual float getAvgScore(bool bNegativeToPositive = false) const;
    virtual void Resize(float factor);
    virtual bool containsPoint(Point2f pt);
  private:
#ifdef DEBUG
    FRIEND_TEST(nskpsolverTests, ScoreCostAndJointCost); // Used for set isWeak to "false" in nskpsolver_tests.computeScoreCost
    FRIEND_TEST(nskpsolverTests, evaluateSolution);// Used for set isWeak to "false" in nskpsolver_tests.evaluateSolution
#endif  // DEBUG
    /// Label center 
    Point2f center;
    /// identifier representing what bone this label belongs to
    int limbID;
    /// degrees rotation from y=0 (horizontal line)
    float angle;
    /// detector scores
    vector<Score> scores;
    /// polygon for the label (already rotated, scaled etc. - from which scores are generated)
    vector<Point2f> polygon;
    /// signify label is for an occluded limb
    bool isOccluded;
  };

}

#endif  // _LIBPOSE_LIMBLABEL_HPP_

