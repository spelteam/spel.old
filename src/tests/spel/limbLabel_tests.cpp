#include <gtest/gtest.h>
#include <limbLabel.hpp>

namespace SPEL
{
  TEST(limbLabel, ConstructorTest)
  {
    int id = 1;
    Point2f center = Point2f(10, 10);
    float angle = 0.866302;
    bool isOccluded = false;
    float score1Value = 0.1, score2Value = 0.3;
    float scoreCoeff = 1;
    Score score1(score1Value, "", scoreCoeff);
    Score score2(score2Value, "", scoreCoeff);
    vector <Score> scores;
    scores.push_back(score1);
    scores.push_back(score2);
    vector<Point2f> polygon = { Point2f(6, 2), Point2f(6, 18), Point2f(14, 18), Point2f(14, 2) };
    Point2f EndPoint1(10, 2), EndPoint2(10, 18);

    //Testing constructor with field parameters
    LimbLabel label1(id, center, angle, polygon, scores, isOccluded);
    EXPECT_EQ(id, label1.getLimbID());
    EXPECT_EQ(center, label1.getCenter());
    EXPECT_EQ(angle, label1.getAngle());
    EXPECT_EQ(polygon, label1.getPolygon());
    EXPECT_EQ(scores, label1.getScores());
    EXPECT_EQ(isOccluded, label1.getIsOccluded());

    //Testing constructor with the LimbLabel parameter
    //And testing get() functions for fields 
    LimbLabel label2(label1);
    EXPECT_EQ(label1.getLimbID(), label2.getLimbID());
    EXPECT_EQ(label1.getCenter(), label2.getCenter());
    EXPECT_EQ(label1.getAngle(), label2.getAngle());
    EXPECT_EQ(label1.getPolygon(), label2.getPolygon());
    EXPECT_EQ(label1.getScores(), label2.getScores());
    EXPECT_EQ(label1.getIsOccluded(), label2.getIsOccluded());

    //Testing operator "="
    LimbLabel label3;
    label3 = label1;
    EXPECT_EQ(label1.getLimbID(), label3.getLimbID());
    EXPECT_EQ(label1.getCenter(), label3.getCenter());
    EXPECT_EQ(label1.getAngle(), label3.getAngle());
    EXPECT_EQ(label1.getPolygon(), label3.getPolygon());
    EXPECT_EQ(label1.getScores(), label3.getScores());
    EXPECT_EQ(label1.getIsOccluded(), label3.getIsOccluded());

    //Testing function getAvgScore()
    bool bNegativeToPositive = true;
    //float avgScore = (score1Value + score2Value)*scoreCoeff / 2;
    float avgScore = (score1Value + score2Value)*scoreCoeff; // Use sum instead of average
    // Temporary debug messages
    if (avgScore != label1.getAvgScore(bNegativeToPositive))
      cout << "-----------\n   LimbLabel::GetAvgScore():\n" << endl;
    //
    EXPECT_EQ(avgScore, label1.getAvgScore(bNegativeToPositive));
    // Temporary debug messages
    if (avgScore != label1.getAvgScore(bNegativeToPositive))
    {
      cout << endl;
      cout << "----------\n" << endl;
      //cin.get();
    }
    //

    //Testing function AddScore()
    float score3Value = 0.1;
    Score score3(score3Value, "", scoreCoeff);
    scores.push_back(score3);
    label3.addScore(score3);
    EXPECT_EQ(scores, label3.getScores());

    //Testing function getEndPoints()
    Point2f p1, p2;
    label1.getEndpoints(p1, p2);
    EXPECT_EQ(EndPoint1, p1);
    EXPECT_EQ(EndPoint2, p2);
    //    0---3
    //    |   |
    //    1---2

    //Testing function containsPoint()
    bool b = label1.containsPoint(center);
    EXPECT_TRUE(label1.containsPoint(center));
    EXPECT_FALSE(label1.containsPoint(Point2f(-1.0, -1.0)));

  }

  TEST(limbLabel, Operators)
  {
    int id = 1;
    Point2f center = Point2f(10, 10);
    float angle = 0.866302;
    bool isOccluded = false;
    float score1Value = 0.1, score2Value = 0.3;
    float scoreCoeff = 1.0;
    Score score1(score1Value, "", scoreCoeff);
    Score score2(score2Value, "", scoreCoeff);
    vector <Score> scores;
    scores.push_back(score1);
    scores.push_back(score1);
    vector<Point2f> polygon = { Point2f(6, 2), Point2f(6, 18), Point2f(14, 18), Point2f(14, 2) };
    LimbLabel label1(id, center, angle, polygon, scores, isOccluded);
    scores.clear();
    scores.push_back(score2);
    scores.push_back(score2);
    LimbLabel label2(id, center, angle, polygon, scores, isOccluded);

    //Testing operator "="
    LimbLabel label3;
    label3 = label1;
    EXPECT_EQ(label1.getLimbID(), label3.getLimbID());
    EXPECT_EQ(label1.getCenter(), label3.getCenter());
    EXPECT_EQ(label1.getAngle(), label3.getAngle());
    EXPECT_EQ(label1.getPolygon(), label3.getPolygon());
    EXPECT_EQ(label1.getScores(), label3.getScores());
    EXPECT_EQ(label1.getIsOccluded(), label3.getIsOccluded());

    //Testing operator "<"
    EXPECT_TRUE(label1 < label2) << " Operator '<', expected: " << label1.getAvgScore() << " < " << label2.getAvgScore() << " = true" << endl;
    EXPECT_FALSE(label2 < label1) << " Operator '<', expected: " << label2.getAvgScore() << " < " << label1.getAvgScore() << " = false" << endl;
    EXPECT_FALSE(label1 < label1) << " Operator '<', expected: " << label1.getAvgScore() << " < " << label1.getAvgScore() << " = false" << endl;

    //Testing operator ">"
    EXPECT_TRUE(label2 > label1) << " Operator '>', expected: " << label2.getAvgScore() << " > " << label1.getAvgScore() << " = true" << endl;
    EXPECT_FALSE(label1 > label2) << " Operator '>', expected: " << label1.getAvgScore() << " > " << label2.getAvgScore() << " = false" << endl;
    EXPECT_FALSE(label1 > label1) << " Operator '>', expected: " << label1.getAvgScore() << " > " << label1.getAvgScore() << " = false" << endl;
  }
}
