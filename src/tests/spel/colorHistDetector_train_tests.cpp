#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif
#include <gtest/gtest.h>
#include <tree.hh>
#include <string>
#include "colorHistDetector.hpp"
#include "bodyPart.hpp"
#include "skeleton.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"
#include "TestsFunctions.hpp"

namespace SPEL
{  
  //Testing function "Train"
  TEST(colorHistDetectorTest, Train)
  {
    //Load the input data
    vector<Frame*> frames = LoadTestProject("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");

    //Setting parameters 
    auto seq = new Sequence();
    map <string, float> params = SetParams(frames, &seq);
	
    for (auto f : frames)
      delete f;
    frames.clear();
    frames = seq->getFrames();
    
    //Counting a keyframes
    int FirstKeyframe = FirstKeyFrameNum(frames);
    int KeyframesCount = keyFramesCount(frames);

    //Copy image and skeleton from keyframe 
    Mat image = frames[FirstKeyframe]->getImage();
    Mat image1;
    image.copyTo(image1);
    Frame *frame = frames[FirstKeyframe];
    Skeleton skeleton = frame->getSkeleton();
    tree<BodyPart> PartTree = skeleton.getPartTree();

    //Build the rectangles for all of bodyparts
    map<int, POSERECT<Point2f>> Rects = SkeletonRects(skeleton);

    //Run "Train()"
    ColorHistDetector detector;
    detector.train(frames, params);

    //Calculate the polygons occlusion
    //Polygons layers:
    map<int, int> depth = { { 0, 2 }, { 1, 1 }, { 2, 3 }, { 3, 2 }, { 4, 4 }, { 5, 4 }, { 6, 1 }, { 7, 3 }, { 8, 2 }, { 9, 0 }, { 10, 4 }, { 11, 1 }, { 12, 3 }, { 13, 0 }, { 14, 4 }, { 15, 1 }, { 16, 3 } };
    //Polygons occlusion:
    vector<vector<pair<int, int>>> Crossings = CrossingsList(Rects, depth);

    //Calculate the parts histograms
    map <int32_t, ColorHistDetector::PartModel> partModels;
    for (int i = 0; i < Rects.size(); i++)
    {
      ColorHistDetector::PartModel Model(8);
      Model.sizeFG = 0;
      float xmin, ymin, xmax, ymax;
      Rects[i].GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
      for (int x = xmin; x < xmax; x++)
      {
        for (int y = ymin; y < ymax; y++)
        {
          bool b = true;
          if (Rects[i].containsPoint(Point2f(x, y)) > 0)
          {
            int k = 0;

            while ((k < Crossings[i].size()) && b)
            {
              if (Rects[Crossings[i][k].first].containsPoint(Point2f(x, y)) > 0)
                b = false;
              k++;
            }
            if (b)
            {
              int c = 50 + i * 10;
              image1.at<Vec3b>(y, x) = Vec3b(c, c, c);
              Vec3b color = image.at<Vec3b>(y, x);
              Model.partHistogram[color[0] / Factor][color[1] / Factor][color[2] / Factor]++;
              Model.sizeFG++;
            }
          }
        }
      }
      partModels.emplace(pair<int32_t, ColorHistDetector::PartModel>(i, Model));
    }

    //Put results
    int nBins = detector.nBins;
    bool AllValuesEqual = true;
    int delta = 2; // tolerable linear error

    ofstream fout("TrainUnitTest_Output.txt");
    fout << "\n--------------------------Don't equal----------------------\n";
    cout << "\nTolerable error: " << delta << endl;
    fout << "Tolerable error: " << delta << endl;
    for (int i = 0; i < partModels.size(); i++)
    {
      for (int r = 0; r < nBins; r++)
        for (int g = 0; g < nBins; g++)
          for (int b = 0; b < nBins; b++)
          {
            int expected = int(partModels[i].partHistogram[b][g][r]);
            int actual = int(detector.partModels[i].partHistogram[r][g][b] * detector.partModels[i].sizeFG / KeyframesCount);
            if (abs(expected - actual) > delta)
            {
              cout << "Part[" << i << "]." << "Histogram[" << r << ", " << g << ", " << b << "]:    Expected = " << expected << ",   Actual = " << actual << endl;
              fout << "Part[" << i << "]." << "Histogram[" << r << ", " << g << ", " << b << "]:    Expected = " << expected << ",   Actual = " << actual << endl;
              if (!(r*g*b == 0)) AllValuesEqual = false;
            }
          }
    }
    if (AllValuesEqual) fout << "none";

    cout << "Output files: TrainUnitTest_Output.txt, UsedPixels.png\n\n";
    EXPECT_TRUE(AllValuesEqual);

    fout << "\n-----------Expected histogram-----------\n";
    fout << "In format:\nHistogramm[r, g, b] = pixelsCount\n";
    for (int i = 0; i < partModels.size(); i++)
    {
      fout << endl << "Rect[" << i << "]:" << endl;
      PutHistogram(fout, partModels[i].partHistogram, 1);
    }

    fout << "\n-----------Actual histogram-----------\n";
    fout << "In format:\nHistogramm[b, g, r] = Histogram[b, g, r]*Part.SizeFG/KeyframesCout\n";
    for (int i = 0; i < detector.partModels.size(); i++)
    {
      fout << endl << "Rect[" << i << "]:" << endl;
      PutHistogram(fout, detector.partModels[i].partHistogram, detector.partModels[i].sizeFG / KeyframesCount);
    }

    fout << "\n------------Occluded polygons-----------\nSorted by layer\n";
    for (int i = 0; i < Crossings.size(); i++)
    {
      fout << "\nPolygon[" << i << "] crossed by polygons: ";
      for (int k = 0; k < Crossings[i].size(); k++)
        fout << Crossings[i][k].first << "; ";
      Crossings[i].clear();
    }
    imwrite("UsedPixels.png", image1);

    fout.close();

    frames.clear();
    params.clear();
    Crossings.clear();
    partModels.clear();

    image.release();
    image1.release();

    delete seq;
  }
}
