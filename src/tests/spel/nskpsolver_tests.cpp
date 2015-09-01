#include <gtest/gtest.h>
#include "nskpsolver.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "limbLabel.hpp"
#include "bodyJoint.hpp"
#include "bodyPart.hpp"
#include "skeleton.hpp"
#include "spelHelper.hpp"

#include <iostream>

using namespace cv;
namespace SPEL
{
  TEST(nskpsolverTests, findFrameIndexById)
  {
    NSKPSolver S;
    vector<Frame*> frames;
    for (int id = 0; id < 10; id++)
    {
      frames.push_back(new Lockframe());
      frames[id]->setID(id);
    }
    uint32_t id = 6;
    EXPECT_EQ(id, S.findFrameIndexById(id, frames));
  }

  TEST(nskpsolverTests, ScoreCostAndJointCost)
  {
    string hogName = "18500";
    string csName = "4409412";
    string surfName = "21316";
    int id = 0;
    Point2f center = Point2f(10, 10);
    float angle = 0;
    bool isOccluded = false;
    float score1Value = 0.1, score2Value = 0.3;
    float scoreCoeff = 1;
    vector<Point2f> polygon = { Point2f(6, 2), Point2f(6, 18), Point2f(14, 18), Point2f(14, 2) };
    float LimbLength = polygon[1].y - polygon[0].y;
    vector <Score> scores;

    Score score1(score1Value, csName, scoreCoeff);
    Score score2(score2Value, csName, scoreCoeff);

    scores.push_back(score1);
    scores.push_back(score2);

    LimbLabel label1(id, center, angle, polygon, scores, isOccluded);

    map<string, float> params;
    params.emplace("imageCoeff", 1.0);
    params.emplace("useCSdet", 1.0);
    params.emplace("useHoGdet", 0.0);
    params.emplace("useSURFdet", 0.0);
    NSKPSolver S;

    //Testing function "computeScoreCost"
    /*
    //scores[0] = score1Value, isWeak = true, isOccluded = false, 
    EXPECT_EQ(0, S.computeScoreCost(label1, params));

    //scores[0] = score1Value, isWeak = false, isOccluded = false, 
    EXPECT_EQ(score1Value, S.computeScoreCost(label1, params));

    //scores is empty, isWeak = false, isOccluded = false, 
    LimbLabel label2;
    label1.isOccluded = false;
    EXPECT_EQ(0, S.computeScoreCost(label2, params));
    */
    float expected_scoreCost = (scores[0].getScore() + scores[1].getScore());
    EXPECT_EQ(expected_scoreCost, S.computeScoreCost(label1, params)); // ScoreCost == Summ of scores?

    //Testing function "computeJointCost"
    EXPECT_EQ(0, S.computeJointCost(label1, label1, params, false));
    EXPECT_EQ(LimbLength, S.computeJointCost(label1, label1, params, true));

    //Testing function "computeNormJointCost"
    float max = 1;
    params.emplace("jointCoeff", 1.0);
    EXPECT_EQ(0, S.computeNormJointCost(label1, label1, params, max, false));
    EXPECT_EQ(LimbLength, S.computeNormJointCost(label1, label1, params, max, true));

    //Testing function "computePriorCost"
    Point2f p0(10, 2), p1(10, 18);
    LimbLength = p1.y - p0.y;

    BodyJoint j0(0, "", p0, { 0, 0, 0 }, false);
    BodyJoint j1(1, "", p1, { 0, 0, 0 }, false);
    tree<BodyJoint> jointsTree;
    tree<BodyJoint>::iterator j = jointsTree.begin();
    j = jointsTree.insert(j, j0);
    j = jointsTree.insert(j, j1);

    BodyPart bodyPart(0, "", 0, 1, false, LimbLength);
    tree<BodyPart> partsTree;
    tree<BodyPart>::iterator p = partsTree.begin();
    p = partsTree.insert(p, bodyPart);

    Skeleton skeleton;
    skeleton.setJointTree(jointsTree);
    skeleton.setPartTree(partsTree);

    EXPECT_EQ(0, S.computePriorCost(label1, bodyPart, skeleton, params));

    //Testing function "computeNormPriorCost"
    params.emplace("priorCoeff", 1.0);
    EXPECT_EQ(0, S.computeNormPriorCost(label1, bodyPart, skeleton, params, max, max));
  }

  vector<Point2f> shiftPolygon(vector<Point2f> polygon, float dx, float dy)
  {
    vector<Point2f> X = polygon;
    for (int i = 0; i < polygon.size(); i++)
      X[i] += Point2f(dx, dy);
    return X;
  }


  TEST(nskpsolverTests, evaluateSolution)
  {
    int id = 0;
    Point2f center = Point2f(10, 10);
    float angle = 0;
    bool isOccluded = false;
    float score1Value = 0.1, score2Value = 0.3;
    float scoreCoeff = 1;
    vector<Point2f> polygon = { Point2f(6, 2), Point2f(6, 18), Point2f(14, 18), Point2f(14, 2) };
    float LimbLength = polygon[1].y - polygon[0].y;
    float LimbWidth = polygon[2].x - polygon[1].x;

    vector <Score> scores;
    Score score1(score1Value, "", scoreCoeff);
    Score score2(score2Value, "", scoreCoeff);
    scores.push_back(score1);
    scores.push_back(score2);

    //Create labels
    int dx = 60;
    LimbLabel label1(id, center, angle, polygon, scores, isOccluded);
    LimbLabel label2(id + 1, center, angle, shiftPolygon(polygon, dx, 0), scores, isOccluded);

    //Create labels vector
    vector<LimbLabel> labels;
    labels.push_back(label1);
    labels.push_back(label2);

    //Create mask
    int rows = 100, cols = 120;
    Mat mask = Mat(Size(cols, rows), CV_8UC1, Scalar(0));

    for (int i = 0; i < rows; i++)
      for (int k = 0; k < cols; k++)
        if (label1.containsPoint(Point2f(k, i)) || label2.containsPoint(Point2f(k, i)))
          mask.at<uchar>(i, k) = 255;

    imwrite("mask.jpg", mask);

    //Create frame
    Lockframe* frame = new Lockframe();
    frame->setMask(mask);

    //Tesing function "evaluateSolution"
    map<string, float> params;
    double solutionEval = 1;
    NSKPSolver S;

    //All labels pixels in mask
    EXPECT_EQ(1, S.evaluateSolution(frame, labels, params));
    cout << "\n";

    //30% of "labels[1]" not in mask
    float e = 0.3; // Relative shift
    LimbLabel label3(id + 1, center, angle, shiftPolygon(polygon, dx - LimbWidth*e, 0), scores, isOccluded);
    labels[1] = label3;

    double ActualValue = S.evaluateSolution(frame, labels, params);
    double ExpectedValue = (2 - e) / (2 + e); // (2 - e) / ( 2 - e + 2*e);
    float epsilon = 0.02;
    EXPECT_LE(abs(ActualValue - ExpectedValue), epsilon);
    cout << ExpectedValue << " ~ " << ActualValue << "\n\n";

    //90% of "labels[1]" not in mask
    //"labels[1]" is badly lokalised
    e = 0.9; // Relative shift of "label[1]"
    LimbLabel label4(id + 1, center, angle, shiftPolygon(polygon, dx - LimbWidth*e, 0), scores, isOccluded);
    labels[1] = label4;

    ActualValue = S.evaluateSolution(frame, labels, params);
    ExpectedValue = (2 - e) / (2 + e);
    ExpectedValue = ExpectedValue;
    epsilon = 0.04;
    EXPECT_LE(abs(ActualValue - ExpectedValue), epsilon);
    cout << ExpectedValue << " ~ " << ActualValue << "\n";
  }
}
