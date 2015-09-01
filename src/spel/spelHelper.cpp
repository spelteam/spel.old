#include "spelHelper.hpp"

namespace SPEL
{

  double spelHelper::angle2D(double x1, double y1, double x2, double y2)
  {
    //input has a zero vector
    if ((x1 == 0.0 && y1 == 0.0) || (x2 == 0.0 && y2 == 0.0)){
      //zero vector is both parallel and perpendicular to every vector
      return 0.0;
    }

    double dtheta, theta1, theta2;
    //angle between Ox and first vector
    theta1 = atan2(y1, x1);
    //angle between Ox and second vector
    theta2 = atan2(y2, x2);
    //angle between first and second vector
    dtheta = theta2 - theta1;
    //normalize angle to range [-PI;PI]
    while (dtheta > M_PI)
      dtheta -= (M_PI * 2.0);
    while (dtheta < -M_PI)
      dtheta += (M_PI * 2.0);
    return(dtheta);
  }

  double spelHelper::interpolateFloat(double prevAngle, double nextAngle, int step, int numSteps)
  {
    double t;
    if (numSteps != 0)
      t = (double)step / (double)numSteps;
    else
      t = 0;
    return prevAngle*(1 - t) + nextAngle*t;
  }

  void spelHelper::RecalculateScoreIsWeak(vector <LimbLabel> &labels, string detectorName, float standardDiviationTreshold)
  {

      //@TODO: Ignore this function for now, will be modified before release
    vector <float> scoreValues;
    float min = 1.0f;
    const uint32_t minCount = 600;
    float sum = 0;
    for (vector <LimbLabel>::iterator i = labels.begin(); i != labels.end(); ++i)
    {
      vector <Score> scores = i->getScores();
      for (vector <Score>::iterator j = scores.begin(); j != scores.end(); ++j)
      {
        if (j->getDetName() == detectorName && j->getScore() > 0)
        {
          min = min > j->getScore() ? j->getScore() : min;
          scoreValues.push_back(j->getScore());
          sum += j->getScore();
        }
      }
    }
    bool isWeak = true;
    if (scoreValues.size() > 0)
    {
      float mean = (float)sum / (float)scoreValues.size();
      float sqrSum = 0;
      for (vector <float>::iterator i = scoreValues.begin(); i != scoreValues.end(); ++i)
      {
        sqrSum += pow((*i) - mean, 2);
      }
      float variance = (float)sqrSum / (float)(scoreValues.size());
      float standardDeviation = sqrt(variance);
      float variationCoeff = standardDeviation / (mean - min);
      float dispersionIndex = variance/(mean - min);
      if (scoreValues.size() < minCount)
        variationCoeff = variationCoeff * (1.0 + 1 / (4 * scoreValues.size()));
      isWeak = dispersionIndex < standardDiviationTreshold;
    }
    for (vector <LimbLabel>::iterator i = labels.begin(); i != labels.end(); ++i)
    {
      vector <Score> scores = i->getScores();
      for (vector <Score>::iterator j = scores.begin(); j != scores.end(); ++j)
      {
        if (j->getDetName() == detectorName)
        {
          j->setIsWeak(isWeak);
        }
      }
      i->setScores(scores);
    }
  }

}
