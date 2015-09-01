#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

#include <gtest/gtest.h>
#include <detector.hpp>
#include <hogDetector.hpp>
#include "projectLoader.hpp"
#include "limbLabel.hpp"
#include "spelHelper.hpp"
#include "TestsFunctions.hpp"

namespace SPEL
{
  vector<Frame*> HFrames;

  vector <vector <vector <float>>> decodeDescriptor(vector<float> descriptors, Size wndSize, Size blockSize, Size blockStride, Size cellSize, int nbins)
  {
    vector <vector <vector <float>>> gradientStrengths;
    vector <vector <uint32_t>> counter;

    for (int i = 0; i < wndSize.height; i += cellSize.height)
    {
      gradientStrengths.push_back(vector <vector <float>>());
      counter.push_back(vector <uint32_t>());
      for (int j = 0; j < wndSize.width; j += cellSize.width)
      {
        gradientStrengths.at(i / cellSize.height).push_back(vector <float>());
        counter.at(i / cellSize.height).push_back(0);
        for (int b = 0; b < nbins; b++)
          gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).push_back(0.0f);
      }
    }

    uint32_t d = 0;
    for (uint32_t n = 0; n + blockStride.height < wndSize.height; n += blockStride.height)// window rows
      for (uint32_t k = 0; k + blockStride.width < wndSize.width; k += blockStride.width)// window cols				
        for (uint32_t r = n; r < n + blockSize.height; r += cellSize.height)// block rows					
          for (uint32_t c = k; c < k + blockSize.width; c += cellSize.width)// block cols						
            for (uint32_t b = 0; b < nbins; b++)// nbins
            {
              gradientStrengths.at(r / cellSize.height).at(c / cellSize.width).at(b) += descriptors.at(d);
              if (b == 0) counter.at(r / cellSize.height).at(c / cellSize.width)++;
              d++;
            }

    for (uint32_t i = 0; i < wndSize.height; i += cellSize.height)
      for (uint32_t j = 0; j < wndSize.width; j += cellSize.width)
        for (uint8_t b = 0; b < nbins; b++)
          if (counter.at(i / cellSize.height).at(j / cellSize.width) == 0)
            gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).at(b) = 0;
          else
            gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).at(b) /= counter.at(i / cellSize.height).at(j / cellSize.width);

    return gradientStrengths;
  }

  map<int, Point2f> getImageLocations(Skeleton skeleton)
  {
    map<int, Point2f> Locations;
    tree <BodyJoint> jointTree = skeleton.getJointTree();
    for (tree <BodyJoint>::iterator i = jointTree.begin(); i != jointTree.end(); ++i)
      Locations.emplace(pair<int, Point2f>(i->getLimbID(), i->getImageLocation()));
    return Locations;
  }

  class _LimbIDCompare
  {
  public:
    bool operator () (vector<LimbLabel> X, vector<LimbLabel> Y)
    {
      return Y[0].getLimbID() > X[0].getLimbID();
    }
  };

  // Selecting locations of all body part from skeleton
  map<int, pair<Point2f, Point2f>> _getPartLocations(Skeleton skeleton)
  {
    map<int, pair<Point2f, Point2f>> PartLocations;
    BodyJoint* J0, *J1;
    Point2f p0, p1;
    tree <BodyPart> partTree = skeleton.getPartTree();
    for (tree <BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
    {
      J0 = skeleton.getBodyJoint(i->getChildJoint());
      J1 = skeleton.getBodyJoint(i->getParentJoint());
      p0 = J0->getImageLocation();
      p1 = J1->getImageLocation();
      PartLocations.emplace(pair<int, pair<Point2f, Point2f>>(i->getPartID(), pair<Point2f, Point2f>(p0, p1)));
    }
    return PartLocations;
  }

  TEST(HOGDetectorTests, computeDescriptor)
  {
    //Load the input data
    HFrames = LoadTestProject("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");

    //Counting a keyframes
    int KeyframesCount = 0;
    int FirstKeyframe = 0;

    //Copy image and skeleton from first keyframe
    Mat image = HFrames[FirstKeyframe]->getImage();
    Mat mask = HFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = HFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    //Select body part for testing
    const int  partID = 6;
    //Copy body part
    BodyPart bodyPart = *skeleton.getBodyPart(partID);
    //Copy part joints 
    BodyJoint* j0 = skeleton.getBodyJoint(bodyPart.getParentJoint());
    BodyJoint* j1 = skeleton.getBodyJoint(bodyPart.getChildJoint());

    // Set descriptor parameters
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    Size wndSize = Size(64, 128);
    const int nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;
    bool bGrayImages = false;

    //Calculate actual value
    HogDetector D;
    HogDetector::PartModel partModel;
    partModel = D.computeDescriptors(bodyPart, j0->getImageLocation(), j1->getImageLocation(), image, nbins, wndSize, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType);

    //Calculate expected value
    vector<float> descriptorsValues;
    vector<Point> locations;
    HOGDescriptor d(wndSize, blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);
    d.compute(partModel.partImage, descriptorsValues);

    // Compare
    EXPECT_EQ(descriptorsValues.size(), partModel.descriptors.size());
    for (int i = 0; i < descriptorsValues.size(); i++)
      EXPECT_EQ(descriptorsValues[i], partModel.descriptors[i]);

    vector <vector <vector <float>>> G = decodeDescriptor(descriptorsValues, wndSize, blockSize, blockStride, cellSize, nbins);
    for (int i = 0; i < G.size(); i++)
      for (int k = 0; k < G[i].size(); k++)
        for (int n = 0; n < G[i][k].size(); n++)
          EXPECT_EQ(G[i][k][n], partModel.gradientStrengths[i][k][n]);
  }

  TEST(HOGDetectorTests, computeDescriptors)
  {
    //Counting a keyframes
    int KeyframesCount = 0;
    int FirstKeyframe = 0;

    //Copy image and skeleton from first keyframe
    Mat image = HFrames[FirstKeyframe]->getImage();
    Mat mask = HFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = HFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    // Set descriptor parameters
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    const int nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;
    bool bGrayImages = false;

    //Calculate actual value
    HogDetector D;
    D.partSize = D.getMaxBodyPartHeightWidth(HFrames, blockSize, 1.0f);
    map <uint32_t, HogDetector::PartModel> partModels;
    partModels = D.computeDescriptors(HFrames[FirstKeyframe], nbins, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType);

    //Calculate expected value
    vector<vector<float>> allDescriptors;
    for (int partID = 0; partID < partTree.size(); partID++)
    {
      vector<float> descriptorsValues;
      BodyPart bodyPart = *skeleton.getBodyPart(partID);//Copy body part	
      BodyJoint* j0 = skeleton.getBodyJoint(bodyPart.getParentJoint());//Copy part joints 
      BodyJoint* j1 = skeleton.getBodyJoint(bodyPart.getChildJoint());
      HOGDescriptor d(D.partSize[partID], blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);
      d.compute(partModels[partID].partImage, descriptorsValues);
      allDescriptors.push_back(descriptorsValues);
    }

    //Compare		
    Skeleton S = HFrames[FirstKeyframe]->getSkeleton();
    for (int partID = 0; partID < partTree.size(); partID++)
    {
      EXPECT_EQ(allDescriptors[partID].size(), partModels[partID].descriptors.size());
      for (int i = 0; i < allDescriptors[partID].size(); i++)
        EXPECT_EQ(allDescriptors[partID][i], partModels[partID].descriptors[i]);

      vector <vector <vector <float>>> G = decodeDescriptor(allDescriptors[partID], D.partSize[partID], blockSize, blockStride, cellSize, nbins);
      for (int i = 0; i < G.size(); i++)
        for (int k = 0; k < G[i].size(); k++)
          for (int n = 0; n < G[i][k].size(); n++)
            EXPECT_EQ(G[i][k][n], partModels[partID].gradientStrengths[i][k][n]);

      allDescriptors[partID].clear();

    }
    allDescriptors.clear();
  }

  TEST(HOGDetectorTests, getMaxBodyPartHeightWidth)
  {
    //Counting a keyframes
    int KeyframesCount = 0;
    int FirstKeyframe = 0;

    //Copy image and skeleton from first keyframe
    Mat image = HFrames[FirstKeyframe]->getImage();
    Mat mask = HFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = HFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    //Set blockSize value
    Size blockSize = Size(8, 8);

    //Calculate expected parts size 
    map<int, Point2f> Locations = getImageLocations(skeleton);
    map <uint32_t, Size> partsSize;
    for (tree <BodyPart>::iterator bodyPart = partTree.begin(); bodyPart != partTree.end(); ++bodyPart)
    {
      int partID = bodyPart->getPartID();
      uint32_t parentID = bodyPart->getParentJoint();
      uint32_t childID = bodyPart->getChildJoint();
      float lwRatio = bodyPart->getLWRatio();
      Point2f delta = Locations[parentID] - Locations[childID];
      float length = norm(delta);
      float width = length / lwRatio;
      uint32_t L = ceil(length / blockSize.width)*blockSize.width;
      uint32_t W = ceil(width / blockSize.height)*blockSize.height;
      partsSize.emplace(pair<uint32_t, Size>(partID, Size(L, W)));
    }

    //Calculate actual parts size
    HogDetector D;
    map <uint32_t, Size> partsSize_actual = D.getMaxBodyPartHeightWidth(HFrames, blockSize, 1.0f);
    EXPECT_EQ(partsSize, partsSize_actual);
  }

  TEST(HOGDetectorTests, train)
  {
    //Counting a keyframes
    int KeyframesCount = 0;
    int FirstKeyframe = 0;

    //Copy image and skeleton from first keyframe
    Mat image = HFrames[FirstKeyframe]->getImage();
    Mat mask = HFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = HFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    // Set descriptor parameters
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    const int nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;

    //Calculate actual value
    HogDetector D;
    D.partSize = D.getMaxBodyPartHeightWidth(HFrames, blockSize, 1.0f);
    map<string, float> params;
    D.train(HFrames, params);

    //Calculate expected value
    vector<vector<float>> allDescriptors;
    for (int partID = 0; partID < partTree.size(); partID++)
    {
      vector<float> descriptorsValues;
      BodyPart bodyPart = *skeleton.getBodyPart(partID);//Copy body part	
      BodyJoint* j0 = skeleton.getBodyJoint(bodyPart.getParentJoint());//Copy part joints 
      BodyJoint* j1 = skeleton.getBodyJoint(bodyPart.getChildJoint());
      HOGDescriptor d(D.partSize[partID], blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);
      d.compute(D.partModels[0][partID].partImage, descriptorsValues);
      allDescriptors.push_back(descriptorsValues);
    }

    //Compare	
    for (int partID = 0; partID < partTree.size(); partID++)
    {
      EXPECT_EQ(allDescriptors[partID].size(), D.partModels[0][partID].descriptors.size());
      for (int i = 0; i < allDescriptors[partID].size(); i++)
        EXPECT_EQ(allDescriptors[partID][i], D.partModels[0][partID].descriptors[i]);
      vector <vector <vector <float>>> G = decodeDescriptor(allDescriptors[partID], D.partSize[partID], blockSize, blockStride, cellSize, nbins);
      for (int i = 0; i < G.size(); i++)
        for (int k = 0; k < G[i].size(); k++)
          for (int n = 0; n < G[i][k].size(); n++)
            EXPECT_EQ(G[i][k][n], D.partModels[0][partID].gradientStrengths[i][k][n]);
      allDescriptors[partID].clear();
    }
    allDescriptors.clear();
  }

  TEST(HOGDetectorTests, generateLabel)
  {
    //Counting a keyframes
    int KeyframesCount = 0;
    int FirstKeyframe = 0;

    //Copy image and skeleton from first keyframe
    Mat image = HFrames[FirstKeyframe]->getImage();
    Mat mask = HFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = HFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    //Select body part for testing
    int partID = 7;
    //Copy body part
    BodyPart bodyPart = *skeleton.getBodyPart(partID);
    //Copy part joints 
    BodyJoint* j0 = skeleton.getBodyJoint(bodyPart.getParentJoint());
    BodyJoint* j1 = skeleton.getBodyJoint(bodyPart.getChildJoint());
    //Copy joints locations
    Point2f p0 = j0->getImageLocation();
    Point2f p1 = j1->getImageLocation();

    // Set descriptor parameters
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    Size wndSize = Size(64, 128);
    const int nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;

    //Calculate actual value
    HogDetector D;
    D.partSize = D.getMaxBodyPartHeightWidth(HFrames, blockSize, 1.0f);
    map<string, float> params;
    D.train(HFrames, params);
    bool useHOGDet = true;
    HogDetector::PartModel partModel = D.getPartModels()[0][partID];
    LimbLabel label_actual = D.generateLabel(bodyPart, HFrames[FirstKeyframe], p0, p1);


    //Create expected LimbLabel value
    Point2f center = 0.5*(p0 + p1);
    float angle = bodyPart.getRotationSearchRange();
    POSERECT<Point2f> rect = D.getBodyPartRect(bodyPart, p0, p1);
    vector<Point2f> polygon = rect.asVector();
    Score score;
    vector<Score> scores;
    score.setScore(0);
    scores.push_back(score);

    LimbLabel label_expected(partID, center, angle, polygon, scores);

    //Compare
    EXPECT_EQ(label_expected.getLimbID(), label_actual.getLimbID());
    EXPECT_EQ(label_expected.getCenter(), label_actual.getCenter());
    EXPECT_EQ(label_expected.getPolygon(), label_actual.getPolygon());
    EXPECT_EQ(label_expected.getScores()[0].getScore(), label_actual.getScores()[0].getScore());
    //Skeleton S = frames[FirstKeyframe]->getSkeleton();
    //BodyPart *P = S.getBodyPart(partID);
    //EXPECT_EQ(P->getRotationSearchRange(), label_actual.getAngle());
    float expected_angle = (float)(spelHelper::angle2D(1, 0, p1.x - p0.x, p1.y - p0.y) * 180 / M_PI);
    EXPECT_EQ(expected_angle, label_actual.getAngle());

    //Output debug inf
    cout << endl << "LimbID = " << label_expected.getLimbID() << endl;
    cout << "Angle = " << label_actual.getAngle() << endl;
    cout << "Center = " << label_actual.getCenter() << endl;
    cout << "Polygon = " << label_actual.getPolygon() << endl;

  }

  TEST(HOGDetectorTests, detect)
  {
    //Counting a keyframes
    int KeyframesCount = 0;
    int FirstKeyframe = 0;

    //Copy image and skeleton from first keyframe
    Mat image = HFrames[FirstKeyframe]->getImage();
    Mat mask = HFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = HFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    ofstream fout("Output_HOGTest_detect.txt");

    // Copy skeleton from keyframe to frames[1] 
    HFrames[1]->setSkeleton(HFrames[0]->getSkeleton());

    // Set descriptor parameters
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    Size wndSize = Size(64, 128);
    const int nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;

    // Run "detect"
    HogDetector D;
    D.partSize = D.getMaxBodyPartHeightWidth(HFrames, blockSize, 1.0f);
    map<string, float> params;
    D.train(HFrames, params);
    map<uint32_t, vector<LimbLabel>> limbLabels;
    map <string, float> detectParams;
    limbLabels = D.detect(HFrames[1], detectParams, limbLabels);

    // sort "limbLabels" by limb id
    //sort(limbLabels.begin(), limbLabels.end(), _LimbIDCompare());

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
    map<int, pair<Point2f, Point2f>> PartLocation = _getPartLocations(skeleton);
    
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
      p0 = PartLocation[id].first; // Ideal boby part point
      p1 = PartLocation[id].second; // Ideal boby part point
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
        // Put linear errors for all Lalbels into text file, copy indexes of a "badly processed parts"
        fout << "    PartID = " << id << ", LabelIndex = " << k << ":    AvgScore = " << limbLabels[id][k].getAvgScore() << ", LinearError = " << error << endl;
        if (k == TopListLabelsCount - 1)
        {
          fout << "    //End of part[" << id << "] top labels list\n";
          if (!(skeleton.getBodyPart(id)->getIsOccluded()))
            if (temp.size() < 1)
            {
              EffectiveLabbelsInTop = false; // false == Present not Occluded bodyparts, but no nave "effective labels" in the top of list
              WithoutGoodLabelInTop.push_back(id); // Copy index of not Occluded parts, wich no have "effective labels" in the top of labels list
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
    cout << "\nLimbLabels saved in file: Output_HOGTest_detect.txt\n";

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

    image.release();
    mask.release();
    /*for (int i = 0; i < frames.size(); i++)
        delete frames[i];	*/
  }

  TEST(HOGDetectorTests, compare)
  {
    int DescriptorLength = 3780;
    vector<float> descriptor;
    for (int i = 0; i < DescriptorLength; i++)
      descriptor.push_back(2);

    Size wndSize = Size(64, 128);
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    const int nbins = 9;

    HogDetector::PartModel X;
    X.gradientStrengths = decodeDescriptor(descriptor, wndSize, blockSize, blockStride, cellSize, nbins);

    HogDetector detector;

    map  <uint32_t, HogDetector::PartModel > _partModels;
    _partModels.emplace(pair <uint32_t, HogDetector::PartModel>(0, X));
    map <uint32_t, map <uint32_t, HogDetector::PartModel>> framePartModels;
    framePartModels.emplace(pair<uint32_t, map <uint32_t, HogDetector::PartModel>>(0, _partModels));

    detector.partModels = framePartModels;

    //map <uint32_t, map>
    BodyPart bodyPart;
    bodyPart.setPartID(0);

    float score = detector.compare(bodyPart, X, nbins);
    cout << "Score = " << score << endl;
    EXPECT_EQ(0, score);

    int N = X.gradientStrengths.size()*X.gradientStrengths[0].size()*X.gradientStrengths[0][0].size();

    X.gradientStrengths[0][0][0] += 1;
    score = detector.compare(bodyPart, X, nbins);
    cout << "Score = " << score << endl;
    float x = float(1.0 / (float)N);
    EXPECT_EQ(x, score);

    framePartModels.emplace(pair<uint32_t, map <uint32_t, HogDetector::PartModel>>(0, _partModels));
    score = detector.compare(bodyPart, X, nbins);
    cout << "Score = " << score << endl;
    x = float(1.0 / (float)N);
    EXPECT_EQ(x, score);
  }

  TEST(HOGDetectorTests, getLabelModels)
  {
    // Create "LabelModels"
    Size imageSize = Size(4, 3);
    const int F = 3, P = 10, L = 20, I = 5, K = 6, N = 7;
    map <uint32_t, map <uint32_t, vector <HogDetector::PartModel>>> expected_LabelModels;
    for (uint32_t f = 0; f < F; f++)
    {
      map <uint32_t, vector <HogDetector::PartModel>> temp_Parts;
      for (uint32_t p = 0; p < P; p++)
      {
        vector<HogDetector::PartModel> temp_Labels;
        for (int l = 0; l < L; l++)
        {
          HogDetector::PartModel X;
          X.partModelRect = POSERECT<Point2f>(Point2f(f, f), Point2f(p, p), Point2f(l, l), Point2f(0, 0));
          X.partImage = Mat(imageSize, CV_8UC3, Scalar(f, p, l));
          for (int i = 0; i < I; i++)
          {
            vector <vector <float>> temp_k;
            for (int k = 0; k < K; k++)
            {
              vector <float> temp_n;
              for (int n = 0; n < N; n++)
                temp_n.push_back(p + f + l);
              temp_k.push_back(temp_n);
              temp_n.clear();
            }
            X.gradientStrengths.push_back(temp_k);
            temp_k.clear();
          }
          temp_Labels.push_back(X);
        }
        temp_Parts.emplace(pair<uint32_t, vector <HogDetector::PartModel>>(p, temp_Labels));
        temp_Labels.clear();
      }
      expected_LabelModels.emplace(pair<uint32_t, map <uint32_t, vector <HogDetector::PartModel>>>(f, temp_Parts));
      temp_Parts.clear();
    }

    //Create "HogDetector"
    HogDetector detector;
    detector.labelModels = expected_LabelModels;

    for (uint32_t f = 0; f < F; f++)
      for (uint32_t p = 0; p < P; p++)
        for (int l = 0; l < L; l++)
        {
          detector.labelModels[f][p][l].partImage.release();
          detector.labelModels[f][p][l].partImage = expected_LabelModels[f][p][l].partImage.clone();
        }

    //Get "LabelModels"
    map <uint32_t, map <uint32_t, vector <HogDetector::PartModel>>> actual_LabelModels;
    actual_LabelModels = detector.getLabelModels();

    //Compare
    for (uint32_t f = 0; f < F; f++)
      for (uint32_t p = 0; p < P; p++)
        for (int l = 0; l < L; l++)
        {
          EXPECT_EQ(expected_LabelModels[f][p][l].partModelRect, detector.labelModels[f][p][l].partModelRect);
          EXPECT_EQ(expected_LabelModels[f][p][l].gradientStrengths, detector.labelModels[f][p][l].gradientStrengths);
          Size image_size = expected_LabelModels[f][p][l].partImage.size();
          /* bool ImagesIsEqual = true;
          for (int y = 0; y < image_size.height; y++)
          for (int x = 0; x < image_size.width; x++)
          ImagesIsEqual = (expected_LabelModels[f][p][l].partImage.at<Vec3b>(y, x) == actual_LabelModels[f][p][l].partImage.at<Vec3b>(y, x));

          EXPECT_TRUE(ImagesIsEqual);
          */
          EXPECT_EQ(expected_LabelModels[f][p][l].partImage.at<Vec3b>(0, 0), actual_LabelModels[f][p][l].partImage.at<Vec3b>(0, 0));
        }

    expected_LabelModels.clear();
  }

  TEST(HOGDetectorTests, getPartModels)
  {
    //Create "PartModels"
    Size imageSize = Size(4, 3);
    const int F = 3, P = 10, L = 20, I = 5, K = 6, N = 7;
    HogDetector::PartModel X;
    map <uint32_t, map <uint32_t, HogDetector::PartModel>> expected_PartModels;
    for (uint32_t f = 0; f < F; f++)
    {
      map <uint32_t, HogDetector::PartModel> temp_Parts;
      for (uint32_t p = 0; p < P; p++)
      {
        HogDetector::PartModel X;
        X.partModelRect = POSERECT<Point2f>(Point2f(f, f), Point2f(p, p), Point2f(0, 0), Point2f(0, 0));
        X.partImage = Mat(imageSize, CV_8UC3, Scalar(f, p, 0));
        for (int i = 0; i < I; i++)
        {
          vector <vector <float>> temp_k;
          for (int k = 0; k < K; k++)
          {
            vector <float> temp_n;
            for (int n = 0; n < N; n++)
              temp_n.push_back(p + f + i + k);
            temp_k.push_back(temp_n);
            temp_n.clear();
          }
          X.gradientStrengths.push_back(temp_k);
          temp_k.clear();
        }
        temp_Parts.emplace(pair<uint32_t, HogDetector::PartModel>(p, X));
      }
      expected_PartModels.emplace(pair<uint32_t, map <uint32_t, HogDetector::PartModel>>(f, temp_Parts));
      temp_Parts.clear();
    }

    //Create "HogDetector"
    HogDetector detector;
    detector.partModels = expected_PartModels;

    for (uint32_t f = 0; f < F; f++)
      for (uint32_t p = 0; p < P; p++)
      {
        detector.partModels[f][p].partImage.release();
        detector.partModels[f][p].partImage = expected_PartModels[f][p].partImage.clone();
      }

    //Get "PartModels"
    map <uint32_t, map <uint32_t, HogDetector::PartModel>> actual_PartModels;
    actual_PartModels = detector.getPartModels();

    //Compare
    for (uint32_t f = 0; f < F; f++)
      for (uint32_t p = 0; p < P; p++)
        for (int l = 0; l < L; l++)
        {
          EXPECT_EQ(expected_PartModels[f][p].partModelRect, detector.partModels[f][p].partModelRect);
          EXPECT_EQ(expected_PartModels[f][p].gradientStrengths, detector.partModels[f][p].gradientStrengths);
          Size image_size = expected_PartModels[f][p].partImage.size();
          /*bool ImagesIsEqual = true;
          for (int y = 0; y < image_size.height; y++)
          for (int x = 0; x < image_size.width; x++)
          ImagesIsEqual = (expected_PartModels[f][p].partImage.at<Vec3b>(y, x) == actual_PartModels[f][p].partImage.at<Vec3b>(y, x));
          EXPECT_TRUE(ImagesIsEqual);
          */
          EXPECT_EQ(expected_PartModels[f][p].partImage.at<Vec3b>(0, 0), actual_PartModels[f][p].partImage.at<Vec3b>(0, 0));
        }

    expected_PartModels.clear();
  }

  TEST(HOGDetectorTests, getCellSize)
  {
    HogDetector detector;
    Size size = Size(8, 8);
    detector.cellSize = size;

    EXPECT_EQ(size, detector.getCellSize());
  }

  TEST(HOGDetectorTests, getNBins)
  {
    HogDetector detector;
    uint8_t nBins = 9;

    EXPECT_EQ(nBins, detector.getnbins());
  }
}