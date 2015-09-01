#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <tree.hh>
#include <string>
#include "bodyPart.hpp"
#include "skeleton.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"
#include "TestsFunctions.hpp"

namespace SPEL
{
  class LimbIDCompare
  {
  public:
    bool operator () (vector<LimbLabel> X, vector<LimbLabel> Y)
    {
      return Y[0].getLimbID() > X[0].getLimbID();
    }
  };

  class TestColorHistDetector : public ColorHistDetector
  {
  public:
    ColorHistDetector::PartModel;
  };

  //IT'S TEMPORARY TESTS

  //Global variables - each next test uses data from previous
  const int  partID = 12;
  int FirstKeyframe;
  vector<Frame*> vFrames;
  Skeleton skeleton;
  tree <BodyPart> partTree;
  BodyPart bodyPart;
  BodyJoint* j0;
  BodyJoint* j1;
  POSERECT<Point2f> rect;
  vector <Point3i> Colors;
  ColorHistDetector detector;
  TestColorHistDetector::PartModel model;
  TestColorHistDetector::PartModel partModel_expected(NBins);
  TestColorHistDetector::PartModel partModel_actual(NBins);
  map <int32_t, Mat> pixelDistributions;
  map<int32_t, Mat> pixelLabels;
  Mat image, mask;
  auto seq = new Sequence();

  TEST(colorHistDetectorTest, Constructors)
  {
    //Testing "PartModel" constructor
    uint8_t _nBins = 10;
    const int maxIndex = _nBins - 1;
    SPEL::ColorHistDetector::PartModel x0(_nBins);
    EXPECT_EQ(0.0f, x0.partHistogram[maxIndex][maxIndex][maxIndex]);
    EXPECT_EQ(x0.nBins, x0.partHistogram[maxIndex][maxIndex].size());
    EXPECT_EQ(0.0f, x0.bgHistogram[maxIndex][maxIndex][maxIndex]);
    EXPECT_EQ(x0.nBins, x0.bgHistogram[maxIndex][maxIndex].size());

    //Testing "ColorHistDetector" constructor with parameter "_nBins"
    ColorHistDetector chd1(_nBins);
    EXPECT_EQ(_nBins, chd1.nBins);
  }

  TEST(colorHistDetectorTest, computePixelBelongingLikelihood)
  {
    //Testing  function "computePixelBelongingLikelihood"
    const uint8_t nBins = 8, outside = 255;
    const uint8_t _factor = static_cast<uint8_t> (ceil(pow(2, 8) / nBins));
    uint8_t i = 70;
    uint8_t t = i / _factor;
    ColorHistDetector chd2(nBins);
    ColorHistDetector::PartModel z(nBins);
    z.partHistogram[t][t][t] = 3.14f;
    EXPECT_EQ(z.partHistogram[t][t][t], chd2.computePixelBelongingLikelihood(z, i, i, i));
    EXPECT_EQ(0.f, chd2.computePixelBelongingLikelihood(z, outside, outside, outside));
  }

  TEST(colorHistDetectorTest, Operators)
  {
    const int nBins = 8;
    ColorHistDetector::PartModel x(nBins);
    x.sizeFG = 100;
    x.sizeBG = 200;
    x.fgNumSamples = 2;
    x.bgNumSamples = 2;
    x.fgSampleSizes.push_back(65);
    x.fgSampleSizes.push_back(45);
    x.bgSampleSizes.push_back(102);
    x.bgSampleSizes.push_back(98);
    x.fgBlankSizes.push_back(30);
    ColorHistDetector::PartModel y = x;
    EXPECT_EQ(x.partHistogram, y.partHistogram);
    EXPECT_EQ(x.bgHistogram, y.bgHistogram);
    EXPECT_EQ(x.sizeFG, y.sizeFG);
    EXPECT_EQ(x.fgNumSamples, y.fgNumSamples);
    EXPECT_EQ(x.bgNumSamples, y.bgNumSamples);
    EXPECT_EQ(x.fgSampleSizes, y.fgSampleSizes);
    EXPECT_EQ(x.bgSampleSizes, y.bgSampleSizes);
    EXPECT_EQ(x.fgBlankSizes, y.fgBlankSizes);
  }

  TEST(colorHistDetectorTest, setPartHistogram)
  {
    //Load the input data
    vFrames = LoadTestProject("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");

    //Setting parameters 
    map <string, float> params = SetParams(vFrames, &seq);
	
    for (auto f : vFrames)
      delete f;
    vFrames.clear();
    vFrames = seq->getFrames();
    
    //Copy image and skeleton from first keyframe
    FirstKeyframe = FirstKeyFrameNum(vFrames);
    image = vFrames[FirstKeyframe]->getImage();
    mask = vFrames[FirstKeyframe]->getMask();
    skeleton = vFrames[FirstKeyframe]->getSkeleton();
    partTree = skeleton.getPartTree();

    //Select body part for testing
    bodyPart = *skeleton.getBodyPart(partID);
    j0 = skeleton.getBodyJoint(bodyPart.getParentJoint());
    j1 = skeleton.getBodyJoint(bodyPart.getChildJoint());
    rect = BuildPartRect(j0, j1, bodyPart.getLWRatio());
      
    //Build part colorset
    Colors = GetPartColors(image, mask, rect);

    //Create expected value	  
    partModel_expected.sizeFG = Colors.size();
    partModel_expected.fgNumSamples = 1;
    partModel_expected.fgSampleSizes.push_back(static_cast <uint32_t> (Colors.size()));
    for (uint32_t i = 0; i < Colors.size(); i++)
      partModel_expected.partHistogram[Colors[i].x / Factor][Colors[i].y / Factor][Colors[i].z / Factor]++;
    for (uint8_t r = 0; r < partModel_expected.nBins; r++)
      for (uint8_t g = 0; g < partModel_expected.nBins; g++)
        for (uint8_t b = 0; b < partModel_expected.nBins; b++)
          partModel_expected.partHistogram[r][g][b] /= Colors.size();

    //Create actual value
    detector.train(vFrames, params);
    model = detector.partModels.at(partID);
    detector.setPartHistogram(partModel_actual, Colors);

    //Compare
    EXPECT_EQ(partModel_expected.partHistogram, partModel_actual.partHistogram);
    EXPECT_EQ(partModel_expected.bgHistogram, partModel_actual.bgHistogram);
    EXPECT_EQ(partModel_expected.sizeFG, partModel_actual.sizeFG);
    EXPECT_EQ(partModel_expected.fgNumSamples, partModel_actual.fgNumSamples);
    EXPECT_EQ(partModel_expected.bgNumSamples, partModel_actual.bgNumSamples);
    EXPECT_EQ(partModel_expected.fgSampleSizes, partModel_actual.fgSampleSizes);
    EXPECT_EQ(partModel_expected.bgSampleSizes, partModel_actual.bgSampleSizes);
    EXPECT_EQ(partModel_expected.fgBlankSizes, partModel_actual.fgBlankSizes);
  }

  TEST(colorHistDetectorTest, addpartHistogram)
  {
    //Create expected value
    uint32_t nBlankPixels = 10;
    for (uint8_t r = 0; r < partModel_expected.nBins; r++)
      for (uint8_t g = 0; g < partModel_expected.nBins; g++)
        for (uint8_t b = 0; b < partModel_expected.nBins; b++)
          partModel_expected.partHistogram[r][g][b] *= partModel_expected.sizeFG;
    partModel_expected.sizeFG += static_cast <uint32_t> (Colors.size());
    partModel_expected.fgNumSamples++;
    partModel_expected.fgSampleSizes.push_back(Colors.size());
    for (uint32_t i = 0; i < Colors.size(); i++)
      partModel_expected.partHistogram[Colors[i].x / Factor][Colors[i].y / Factor][Colors[i].z / Factor]++;
    for (uint8_t r = 0; r < partModel_expected.nBins; r++)
      for (uint8_t g = 0; g < partModel_expected.nBins; g++)
        for (uint8_t b = 0; b < partModel_expected.nBins; b++)
          partModel_expected.partHistogram[r][g][b] /= partModel_expected.sizeFG;
    partModel_expected.fgBlankSizes.push_back(nBlankPixels);

    //Create actual value
    detector.addPartHistogram(partModel_actual, Colors, nBlankPixels);

    //Compare
    EXPECT_EQ(partModel_expected.partHistogram, partModel_actual.partHistogram);
    EXPECT_EQ(partModel_expected.bgHistogram, partModel_actual.bgHistogram);
    EXPECT_EQ(partModel_expected.sizeFG, partModel_actual.sizeFG);
    EXPECT_EQ(partModel_expected.fgNumSamples, partModel_actual.fgNumSamples);
    EXPECT_EQ(partModel_expected.bgNumSamples, partModel_actual.bgNumSamples);
    EXPECT_EQ(partModel_expected.fgSampleSizes, partModel_actual.fgSampleSizes);
    EXPECT_EQ(partModel_expected.bgSampleSizes, partModel_actual.bgSampleSizes);
    EXPECT_EQ(partModel_expected.fgBlankSizes, partModel_actual.fgBlankSizes);
  }

  // Testing function "getAvgSampleSizeFg"
  TEST(colorHistDetectorTest, getAvgSampleSizeFg)
  {
    float Sum = 0;
    for (uint32_t i = 0; i < partModel_expected.fgSampleSizes.size(); i++)
      Sum += partModel_expected.fgSampleSizes[i];
    Sum /= partModel_expected.fgNumSamples;

    EXPECT_EQ(Sum, detector.getAvgSampleSizeFg(partModel_actual));
  }

  // Testing function "getAvgSampleSizeFgBetween"
  TEST(colorHistDetectorTest, getAvgSampleSizeFgBetween)
  {
    uint32_t s1 = 0, s2 = 0;
    float f = (partModel_expected.fgSampleSizes[s1] + partModel_expected.fgSampleSizes[s2]) / 2.0f;
    EXPECT_EQ(f, detector.getAvgSampleSizeFgBetween(partModel_actual, s1, s2));
    EXPECT_EQ(0.f, detector.getAvgSampleSizeFgBetween(partModel_actual, static_cast <uint32_t> (partModel_expected.fgSampleSizes.size()), s2));
  }

  // Testing function "matchPartHistogramsED"
  TEST(colorHistDetectorTest, matchPartHistogramsED)
  {
    float distance = 0;
    for (uint8_t r = 0; r < partModel_expected.nBins; r++)
      for (uint8_t g = 0; g < partModel_expected.nBins; g++)
        for (uint8_t b = 0; b < partModel_expected.nBins; b++)
          distance += pow(partModel_expected.partHistogram[r][g][b] - partModel_expected.partHistogram[r][g][b], 2.0f);
    float f = detector.matchPartHistogramsED(partModel_expected, partModel_expected);
    EXPECT_EQ(sqrt(distance), f);
  }

  // Testing function "addBackgroundHistogram"
  TEST(colorHistDetectorTest, addBackgroundHistogram)
  {
    vector <Point3i> cEmpty;
    for (uint8_t r = 0; r < partModel_expected.nBins; r++)
      for (uint8_t g = 0; g < partModel_expected.nBins; g++)
        for (uint8_t b = 0; b < partModel_expected.nBins; b++)
          partModel_expected.bgHistogram[r][g][b] *= partModel_expected.sizeBG;
    partModel_expected.sizeBG += static_cast <uint32_t> (Colors.size());
    partModel_expected.bgNumSamples++;
    partModel_expected.bgSampleSizes.push_back(static_cast <uint32_t> (Colors.size()));
    for (uint32_t i = 0; i < Colors.size(); i++)
      partModel_expected.bgHistogram[Colors[i].x / Factor][Colors[i].y / Factor][Colors[i].z / Factor]++;
    for (uint8_t r = 0; r < partModel_expected.nBins; r++)
      for (uint8_t g = 0; g < partModel_expected.nBins; g++)
        for (uint8_t b = 0; b < partModel_expected.nBins; b++)
          partModel_expected.bgHistogram[r][g][b] /= (float)partModel_expected.sizeBG;

    detector.addBackgroundHistogram(partModel_actual, cEmpty);
    EXPECT_NE(partModel_expected.bgHistogram, partModel_actual.bgHistogram);
    detector.addBackgroundHistogram(partModel_actual, Colors);
    EXPECT_EQ(partModel_expected.partHistogram, partModel_actual.partHistogram);
    EXPECT_EQ(partModel_expected.bgHistogram, partModel_actual.bgHistogram);
    EXPECT_EQ(partModel_expected.sizeFG, partModel_actual.sizeFG);
    EXPECT_EQ(partModel_expected.fgNumSamples, partModel_actual.fgNumSamples);
    EXPECT_EQ(partModel_expected.bgNumSamples, partModel_actual.bgNumSamples);
    EXPECT_EQ(partModel_expected.fgSampleSizes, partModel_actual.fgSampleSizes);
    EXPECT_EQ(partModel_expected.bgSampleSizes, partModel_actual.bgSampleSizes);
    EXPECT_EQ(partModel_expected.fgBlankSizes, partModel_actual.fgBlankSizes);
  }

  // Testing function buildPixelDistributions
  TEST(colorHistDetectorTest, buildPixelDistributions)
  {
    Mat t(image.rows, image.cols, DataType <float>::type);
    ColorHistDetector::PartModel partModel = detector.partModels[partID];
    for (int x = 0; x < image.cols; x++)
      for (int y = 0; y < image.rows; y++)
      {
        Vec3b intensity = image.at<Vec3b>(y, x);
        uint8_t blue = intensity.val[0];
        uint8_t green = intensity.val[1];
        uint8_t red = intensity.val[2];
        uint8_t mintensity = mask.at<uint8_t>(y, x);
        bool blackPixel = mintensity < 10;

        t.at<float>(y, x) = blackPixel ? 0 : partModel.partHistogram.at(red / Factor).at(green / Factor).at(blue / Factor); // (x, y)  or (y, x) !? ?
        //Matrix "PixelDistributions" - transposed relative to the matrix "Image"
      }

    pixelDistributions = detector.buildPixelDistributions(vFrames[FirstKeyframe]);
    for (int x = 0; x < t.cols; x++)
      for (int y = 0; y < t.rows; y++)
        EXPECT_EQ(t.at<float>(y, x), pixelDistributions[partID].at<float>(y, x));

    // we need to save pixelDistibutions in private class member
    detector.pixelDistributions = pixelDistributions;
  }

  // Testing function "BuildPixelLabels"
  TEST(colorHistDetectorTest, BuildPixelLabels)
  {
    Mat p(image.rows, image.cols, DataType <float>::type);
    for (int x = 0; x < image.cols; x++)
      for (int y = 0; y < image.rows; y++)
      {
        bool blackPixel = mask.at<uint8_t>(y, x) < 10;
        if (!blackPixel)
        {
          float top = 0;
          for (tree <BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
          {
            Mat temp;
            temp = pixelDistributions.at(i->getPartID());
            if (temp.at<float>(y, x) > top) // (x,y)  or (y,x) !?? Matrix "PixelLabels" - transposed relative to the matrix "Image"
              top = temp.at<float>(y, x);
              temp.release(); // (x,y)  or (y,x) !??
          }
          p.at<float>(y, x) = (top == 0) ? 0 : pixelDistributions[partID].at<float>(y, x) / top;
        }
        else
          p.at<float>(y, x) = 0; // (x,y)  or (y,x) !??
       }

    pixelLabels = detector.buildPixelLabels(vFrames[FirstKeyframe], pixelDistributions);

    int q = 0;
    for (int x = 0; x < p.cols; x++)
      for (int y = 0; y < p.rows; y++)
        EXPECT_EQ(p.at<float>(y, x), pixelLabels[partID].at<float>(y, x)) << q++ << ": " << x << ", " << y;

    // we need to save pixelLabels in private class member
    detector.pixelLabels = pixelLabels;
  }
  
  // Testing function generateLabel
  TEST(colorHistDetectorTest, generateLabel)
  {
    vector <Score> s;
    Point2f p0 = j0->getImageLocation(), p1 = j1->getImageLocation();
    Point2f boxCenter = p0 * 0.5 + p1 * 0.5;
    float boneLength = detector.getBoneLength(p0, p1);
    float rot = float(spelHelper::angle2D(1, 0, p1.x - p0.x, p1.y - p0.y) * (180.0 / M_PI));
    uint32_t totalPixels = 0, pixelsInMask = 0, pixelsWithLabel = 0;
    float totalPixelLabelScore = 0, pixDistAvg = 0, pixDistNum = 0;
    stringstream detectorName;
    detectorName << partID;

    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    bool PartContainPoints = false;
    for (int i = xmin; i < xmax; i++)
      for (int j = ymin; j < ymax; j++)
        if (rect.containsPoint(Point2f((float)i, (float)j)) > 0)
        {
          totalPixels++;
          uint8_t mintensity = mask.at<uint8_t>(j, i);
          if (mintensity >= 10)
          {
            pixDistAvg += pixelDistributions.at(partID).at<float>(j, i);
            pixDistNum++;
            if (pixelLabels.at(partID).at<float>(j, i))
            {
              pixelsWithLabel++;
              totalPixelLabelScore += pixelLabels.at(partID).at<float>(j, i);
            }
            pixelsInMask++;
            PartContainPoints = true;
          }
        }
    float supportScore = 0, inMaskSupportScore = 0;
    float inMaskSuppWeight = 0.5;
    LimbLabel limbLabel_e;
    if (PartContainPoints)
    {
      supportScore = (float)totalPixelLabelScore / (float)totalPixels;
      inMaskSupportScore = (float)totalPixelLabelScore / (float)pixelsInMask;
      float score = 1.0f - ((1.0f - inMaskSuppWeight)*supportScore + inMaskSuppWeight*inMaskSupportScore);
      Score sc(score, detectorName.str());
      s.push_back(sc);
      limbLabel_e = LimbLabel(partID, boxCenter, rot, rect.asVector(), s);
    }
    else
    {
      Score sc(1.0, detectorName.str());
      s.push_back(sc);
      limbLabel_e = LimbLabel(partID, boxCenter, rot, rect.asVector(), s);
    }

    LimbLabel limbLabel_a = detector.generateLabel(*skeleton.getBodyPart(partID), vFrames[0], p0, p1);

    EXPECT_EQ(limbLabel_e.getLimbID(), limbLabel_a.getLimbID());
    EXPECT_EQ(limbLabel_e.getCenter(), limbLabel_a.getCenter());
    EXPECT_EQ(limbLabel_e.getCenter(), limbLabel_a.getCenter());
    EXPECT_EQ(limbLabel_e.getAngle(), limbLabel_a.getAngle());

    EXPECT_EQ(limbLabel_e.getScores().size(), limbLabel_a.getScores().size());
    if (limbLabel_e.getScores().size() == limbLabel_a.getScores().size())
    {
      for (int i = 0; i < limbLabel_e.getScores().size(); i++)
      {
        EXPECT_TRUE(fabs(limbLabel_e.getScores().at(i).getScore() - limbLabel_a.getScores().at(i).getScore()) <= 0.005);
      }
    }

    EXPECT_EQ(limbLabel_e.getIsOccluded(), limbLabel_a.getIsOccluded());
    // Temporary debug messages
    if (limbLabel_e.getIsOccluded() != limbLabel_a.getIsOccluded())
    {
      cout << endl << "Function 'colorHistDetector::generateLabel()' IsOccluded() value:" << endl;
      cout << "PartID = " << partID << " ~ BodyPart.IsOccluded = " << limbLabel_e.getIsOccluded() << ",   LimbLabel.IsOccluded = " << limbLabel_a.getIsOccluded() << endl;
      cout << "----------\n";
      //cin.get();
    }
    //
    EXPECT_EQ(limbLabel_e.getPolygon().size(), limbLabel_a.getPolygon().size());
    if (limbLabel_e.getPolygon().size() == limbLabel_a.getPolygon().size())
    {
      for (int i = 0; i < limbLabel_e.getPolygon().size(); i++)
      {
        EXPECT_FLOAT_EQ(limbLabel_e.getPolygon().at(i).x, limbLabel_a.getPolygon().at(i).x);
        EXPECT_FLOAT_EQ(limbLabel_e.getPolygon().at(i).y, limbLabel_a.getPolygon().at(i).y);
      }
    }

    EXPECT_EQ(model.bgHistogram, detector.partModels[partID].bgHistogram);
  }

  // Testing function "detect"
  TEST(colorHistDetectorTest, detect)
  {
    ofstream fout("Output_CHDTest_detect.txt");

    // Copy skeleton from keyframe to frames[1] 
    vFrames[1]->setSkeleton(vFrames[0]->getSkeleton());

    // Run "detect"
    map<uint32_t, vector<LimbLabel>> limbLabels;
    map <string, float> detectParams;
    ASSERT_GT(detector.partModels.size(), 0);
    limbLabels = detector.detect(vFrames[1], detectParams, limbLabels);
    ASSERT_GT(limbLabels.size(), 0);

    // sort "limbLabels" by limb id
    //sort(limbLabels.begin(), limbLabels.end(), LimbIDCompare());

    // Output top of "limbLabels" into text file
    fout << "\nTop Labels, sorted by part id:\n\n";
    for (int i = 0; i < limbLabels.size(); i++) // For all body parts
    {
      for (int k = 0; (k < limbLabels[i].size()) && (k < 4); k++) // For all scores of this bodypart
      {
        Point2f p0, p1;
        limbLabels[i][k].getEndpoints(p0, p1); // Copy the Limblabel points
        fout << "  " << i << ":" << " limbID = " << limbLabels[i][k].getLimbID() << ", Angle = " << limbLabels[i][k].getAngle() << ", Points = {" << p0 << ", " << p1 << "}, AvgScore = " << limbLabels[i][k].getAvgScore() << ", Scores = {";
        vector<Score> scores = limbLabels[i][k].getScores(); // Copy the Label scores
        for (int t = 0; t < scores.size(); t++)
        {
          fout << scores[t].getScore() << ", "; // Put all scores of the Label
        }
        fout << "}\n";
      }
      fout << endl;
    }

    // Copy coordinates of BodyParts from skeleton
    map<int, pair<Point2f, Point2f>> PartLocation = getPartLocations(skeleton);

    // Compare labels with ideal bodyparts from keyframe, and output debug information 
    float TolerableCoordinateError = 7; // Linear error in pixels
    float TolerableAngleError = 0.1; // 10% (not used in this test)
    int TopListLabelsCount = 4; // Size of "labels top list"
    map<int, vector<LimbLabel>> effectiveLabels;
    vector<int> WithoutGoodLabelInTop;
    bool EffectiveLabbelsInTop = true;

    fout << "-------------------------------------\nAll labels, with distance from the ideal body part: \n";

    for (int id = 0; id < limbLabels.size(); id++)
    {
      fout << "\nPartID = " << id << ":\n";
      Point2f l0, l1, p0, p1, delta0, delta1;
      vector<LimbLabel> temp;
      p0 = PartLocation[id].first; // Ideal body part point
      p1 = PartLocation[id].second; // Ideal body part point
      for (int k = 0; k < limbLabels[id].size(); k++)
      {
        limbLabels[id][k].getEndpoints(l0, l1); // Label points
        delta0 = l0 - p0;
        delta1 = l1 - p1;
        float error_A = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
        delta0 = l0 - p1;
        delta1 = l1 - p0;
        float error_B = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
        float error = min(error_A, error_B); // Distance between ideal body part and label
        if (error <= TolerableCoordinateError && limbLabels[id][k].getAvgScore() >= 0) // Label is "effective" if it has small error and of not less than zero  Score  value
          temp.push_back(limbLabels[id][k]); // Copy effective labels
        // Put linear errors for all Labels into text file, copy indexes of a "badly processed parts"
        fout << "    PartID = " << id << ", LabelIndex = " << k << ":    AvgScore = " << limbLabels[id][k].getAvgScore() << ", LinearError = " << error << endl;
        if (k == TopListLabelsCount - 1)
        {
          fout << "    //End of part[" << id << "] top labels list\n";
          if (!(skeleton.getBodyPart(id)->getIsOccluded()))
            if (temp.size() < 1)
            {
              EffectiveLabbelsInTop = false; // false == Present not Occluded bodyparts, but no nave "effective labels" in the top of list
              WithoutGoodLabelInTop.push_back(id); // Copy index of not Occluded parts, which no have "effective labels" in the top of labels list
            }
        }
      }
      effectiveLabels.emplace(pair<int, vector<LimbLabel>>(id, temp));
    }

    //Output top of "effectiveLabels" into text file
    fout << "\n-------------------------------------\n\nTrue Labels:\n\n";
    for (int i = 0; i < effectiveLabels.size(); i++)
    {
      for (int k = 0; k < effectiveLabels[i].size(); k++)
      {
        Point2f p0, p1;
        limbLabels[i][k].getEndpoints(p0, p1);
        fout << "  limbID = " << effectiveLabels[i][k].getLimbID() << ", Angle = " << effectiveLabels[i][k].getAngle() << ", Points = {" << p0 << ", " << p1 << "}, AvgScore = " << effectiveLabels[i][k].getAvgScore() << ", Scores = {";
        vector<Score> scores = effectiveLabels[i][k].getScores();
        for (int t = 0; t < scores.size(); t++)
        {
          fout << scores[t].getScore() << ", ";
        }

        fout << "}\n";
      }
      fout << endl;
    }

    fout.close();
    cout << "\nLimbLabels saved in file: Output_CHDTest_detect.txt\n";

    // Output messages 
    if (!EffectiveLabbelsInTop) cout << endl << " ColorHistDetector_Tests.detect:" << endl;
    EXPECT_TRUE(EffectiveLabbelsInTop);
    if (!EffectiveLabbelsInTop)
    {
      cout << "Body parts with id: ";
      for (int i = 0; i < WithoutGoodLabelInTop.size(); i++)
      {
        cout << WithoutGoodLabelInTop[i];
        if (i != WithoutGoodLabelInTop.size() - 1) cout << ", ";
      }
      cout << " - does not have effective labels in the top of labels list." << endl;
    }
    if (!EffectiveLabbelsInTop) cout << endl;

    for (map <int32_t, Mat>::iterator I = pixelDistributions.begin(); I != pixelDistributions.end(); ++I)
      I->second.release();
    pixelDistributions.clear();
    detector.partModels.clear();
    pixelLabels.clear();
    partTree.clear();
    image.release();
    mask.release();
    Colors.clear();
    for (int i = 0; i < vFrames.size(); i++)
      delete vFrames[i];
    vFrames.clear();
  }
}
